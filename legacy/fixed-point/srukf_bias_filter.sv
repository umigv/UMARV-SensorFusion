`timescale 1ns/1ps

// Select this module for the additive dead-wheel bias model. Five state
// lanes use F21/F21/F23/F25/F25. Upper factors are packed row by row,
// omitting entries below the diagonal. Root and noise-factor words use F26.
module srukf_bias_filter #(
    parameter integer CLK_HZ = 50_000_000
)(
    input wire clk, resetn, init,
    input wire [63:0] init_ts,
    input wire [159:0] init_state, process_std,
    input wire [127:0] sample,
    input wire [479:0] init_root,
    input wire [319:0] measurement_root,
    input wire signed [31:0] wheel_offset, wheel_std,
    input wire signed [31:0] wheel_velocity,
    input wire sample_valid,
    input wire [63:0] sample_ts,
    input wire mac_available, cordic_available,
    output wire sample_pop, sample_rejected, predict_start, correct_start,
    output wire predict_done, correct_done, state_valid, fault, deadline_missed,
    output wire [63:0] state_ts,
    output wire [159:0] state,
    output wire [479:0] root,
    output reg overflow, saturation, negative_radicand, invalid_pivot
);
    localparam signed [31:0] ONE = 32'sd134217728;      // 1.0 F27
    localparam signed [31:0] SIGMA_WEIGHT = 32'sd47453133;
    localparam signed [31:0] CENTRAL_WEIGHT = 32'sd177553365; // sqrt(7/4), F27
    localparam [1:0] QR=0, UPDATE=1, DOWNDATE=2;
    typedef enum logic [5:0] {
        IDLE, WHEEL_NOISE_STEP, LEVER, WHEEL_STEP, SIGMA, SIGMA_WRAP, TURN, TRIG, MOVE_X, MOVE_Y,
        MEAN_ADD, MEAN_PUBLISH, DEVIATION, PREDICT_ROW_DONE,
        PROCESS_NOISE, PROCESS_ROW_DONE, WHEEL_NOISE_X, WHEEL_NOISE_Y, WHEEL_NOISE_DONE,
        INNOVATION, INNOVATION_ROW, INNOVATION_DONE,
        FORWARD_SUM, FORWARD_SUB, FORWARD_DIV, BACKWARD_SUM, BACKWARD_DIV,
        CORRECTION_DOT, CORRECTION_WRAP, LOAD_POSTERIOR, REMOVE_COLUMN, REMOVE_DONE,
        FACTOR_PIVOT, FACTOR_COPY, FACTOR_SQUARE, FACTOR_ROOT,
        FACTOR_C, FACTOR_D, FACTOR_U, FACTOR_U_DIV, FACTOR_W, FACTOR_NEXT,
        COMMIT, FAILED
    } phase_t;
    phase_t phase, row_return;
    reg initialized, correcting, second_pass;
    reg [127:0] measurement;
    reg [1:0] row_kind;
    reg [3:0] point_num, row_num;
    reg [2:0] component, pivot, col, row;
    reg [1:0] term;
    reg second_term, pending;
    reg signed [31:0] wheel, distance, angle, cosine, sine;
    reg signed [31:0] offset, velocity_std, lever, wheel_dist_std, central_cos, central_sin;
    reg signed [32:0] pending_heading;
    reg signed [31:0] heading_lift, mean_heading_lift;
    reg signed [31:0] radius, c, d, new_entry;
    reg signed [31:0] committed [0:4], candidate [0:4], point [0:4], residual [0:3];
    reg signed [35:0] mean_sum [0:4];
    reg signed [31:0] committed_root [0:14], factor [0:14];
    reg signed [31:0] q [0:4], r [0:9], u [0:19], gain [0:3], vec [0:4];
    wire [2:0] last_pivot = correcting && row_kind==QR ? 3'd3 : 3'd4;
    localparam signed [72:0] ANGLE_PI=73'sd26353589, ANGLE_TAU=73'sd52707179;
    wire signed [31:0] dt;
    wire phase_fault = phase == FAILED;
    assign predict_done = phase == COMMIT && !correcting && !fault;
    assign correct_done = phase == COMMIT && correcting && !fault;

    srukf_400hz_scheduler #(.CLK_HZ(CLK_HZ)) scheduler (
        .clk, .resetn, .init, .init_ts, .sample_valid, .sample_ts,
        .sample_pop, .sample_rejected, .predict_start, .correct_start,
        .dt, .prediction_ts(), .predict_done, .correct_done, .phase_fault,
        .state_ts, .state_valid, .deadline_missed, .fault
    );

    for (genvar i=0; i<5; i=i+1) begin : state_lanes
        assign state[32*i+:32] = committed[i];
    end
    for (genvar i=0; i<15; i=i+1) begin : root_lanes
        assign root[32*i+:32] = committed_root[i];
    end

    // Transpose the external upper root into a logical lower root.
    // Column-packed lower storage keeps only the fifteen live words.
    function automatic [3:0] cell_index(input [2:0] i, j);
        case (j)
            0: cell_index=4'(i);
            1: cell_index=4'(i)+4'd4;
            2: cell_index=4'(i)+4'd7;
            3: cell_index=4'(i)+4'd9;
            default: cell_index=4'd14;
        endcase
    endfunction
    function automatic [3:0] measurement_index(input [1:0] i,j);
        case (j)
            0: measurement_index=4'(i);
            1: measurement_index=4'(i)+4'd3;
            2: measurement_index=4'(i)+4'd5;
            default: measurement_index=4'd9;
        endcase
    endfunction
    function automatic signed [72:0] wrap_once(input signed [72:0] value);
        if (value>ANGLE_PI) wrap_once=value-ANGLE_TAU;
        else if (value< -ANGLE_PI) wrap_once=value+ANGLE_TAU;
        else wrap_once=value;
    endfunction
    function automatic angle_valid(input signed [72:0] value);
        angle_valid=value>= -ANGLE_PI && value<=ANGLE_PI;
    endfunction
    function automatic [2:0] state_alignment(input [2:0] i);
        // Difference between F25 and the component's binary point.
        case (i)
            0, 1: state_alignment=4;
            2: state_alignment=2;
            default: state_alignment=0;
        endcase
    endfunction
    function automatic signed [72:0] round_shift(input signed [72:0] value,
                                                  input [5:0] shift);
        reg [72:0] magnitude, rounded;
        begin
            magnitude = value[72] ? -value : value;
            rounded = magnitude >> shift;
            if (shift != 0 && magnitude[shift-1'b1]) rounded = rounded+1'b1;
            round_shift = value[72] ? -rounded : rounded;
        end
    endfunction
    function automatic fits_word(input signed [72:0] value);
        fits_word = value[72:31] == {42{value[31]}};
    endfunction
    task automatic fail_range;
        begin overflow <= 1; saturation <= 1; phase <= FAILED; end
    endtask
    task automatic fail_pivot;
        begin invalid_pivot <= 1; phase <= FAILED; end
    endtask
    task automatic begin_factor_row(input [1:0] kind, input phase_t resume);
        begin
            row_kind <= kind; row_return <= resume;
            pivot <= 0; second_term <= 0; phase <= FACTOR_PIVOT;
        end
    endtask

    reg mac_op, mac_clear, mac_subtract;
    reg [2:0] mac_align;
    reg signed [31:0] mac_a, mac_b;
    wire [2:0] sigma_col = point_num>5 ? 3'(point_num-6) : 3'(point_num-1);
    wire signed [31:0] old_pivot = factor[cell_index(pivot,pivot)];
    wire signed [31:0] old_entry = factor[cell_index(col,pivot)];
    wire signed [71:0] mac_result;
    wire mac_done, mac_overflow;
    wire mac_start = mac_op && !pending && !fault && mac_available;
    srukf_mac mac(.clk,.resetn,.start(mac_start),.clear(mac_clear),.subtract(mac_subtract),
                  .a(mac_a),.b(mac_b),.align(mac_align),.result(mac_result),
                  .done(mac_done),.overflow(mac_overflow));

    reg div_op;
    reg signed [71:0] numerator;
    reg signed [31:0] denominator;
    wire signed [31:0] div_result;
    wire div_done, div_zero, div_overflow;
    wire div_start = div_op && !pending && !fault;
    srukf_full_div divider(.clk,.resetn,.start(div_start),.numerator,.denominator,
                           .result(div_result),.done(div_done),.zero(div_zero),
                           .overflow(div_overflow));
    wire root_start = phase == FACTOR_ROOT && !pending && !fault;
    wire signed [31:0] root_result;
    wire root_done, root_negative, root_overflow;
    srukf_full_root square_root(.clk,.resetn,.start(root_start),.radicand(mac_result),
                               .result(root_result),.done(root_done),
                               .negative(root_negative),.overflow(root_overflow));

    wire trig_start = phase == TRIG && !pending && !fault && cordic_available;
    wire signed [31:0] trig_sin, trig_cos;
    wire trig_done, trig_overflow, trig_error;
    cordic #(.FRAC(23)) trig(.clk,.resetn,.start(trig_start),.mode(2'd0),.theta(angle),
                            .sin_theta(trig_sin),.cos_theta(trig_cos),.busy(),
                            .done(trig_done),.overflow(trig_overflow),.error(trig_error));

    wire signed [72:0] raw_deviation = 73'(point[component])-73'(candidate[component]);
    wire signed [72:0] deviation = component==2 ? 73'(heading_lift)-73'(mean_heading_lift) : raw_deviation;
    wire signed [72:0] raw_innovation =
        73'($signed(measurement[32*component+:32]))-73'(committed[component]);
    wire signed [72:0] innovation = component==2 ? wrap_once(raw_innovation) : raw_innovation;
    wire signed [72:0] sigma_twice =
        component<sigma_col ? 73'sd0 : 73'(committed_root[cell_index(component,sigma_col)]) <<< 1;
    wire signed [72:0] sigma_span = round_shift(sigma_twice,6'd1+{3'd0,state_alignment(component)});
    wire signed [72:0] sigma_offset = point_num>5 ? -sigma_span : sigma_span;
    wire signed [72:0] raw_sigma = 73'(committed[component])+sigma_offset;
    wire signed [72:0] sigma_value = component==2 ? wrap_once(raw_sigma) : raw_sigma;
    wire signed [72:0] speed_without_bias = 73'(wheel)-round_shift(73'(point[4]),6'd4);
    wire signed [72:0] center_velocity = speed_without_bias+73'(lever);
    wire signed [72:0] signed_row_entry =
        vec[pivot][31] ? -73'(vec[col]) : 73'(vec[col]);

    reg [5:0] output_shift;
    always @* begin
        case (phase)
            WHEEL_STEP: output_shift=31;
            LEVER: output_shift=25;
            WHEEL_NOISE_STEP: output_shift=26;
            WHEEL_NOISE_X, WHEEL_NOISE_Y: output_shift=23;
            TURN: output_shift=33;
            MOVE_X, MOVE_Y: output_shift=23;
            DEVIATION: output_shift=26-{3'd0,state_alignment(component)};
            FACTOR_U, FACTOR_W: output_shift=27;
            CORRECTION_DOT: output_shift=27+{3'd0,state_alignment(row)};
            default: output_shift=0;
        endcase
    end
    wire signed [72:0] narrowed = round_shift(73'(mac_result),output_shift);
    wire signed [72:0] midpoint = wrap_once(73'(point[2])+round_shift(73'(mac_result),6'd34));
    wire signed [72:0] turn_heading = wrap_once(73'(point[2])+narrowed);
    wire signed [72:0] turned_lift = 73'(heading_lift)+narrowed;
    wire signed [72:0] moved_x = 73'(point[0])+narrowed;
    wire signed [72:0] moved_y = 73'(point[1])+narrowed;
    wire signed [72:0] corrected = 73'(committed[row])+narrowed;
    wire signed [72:0] raw_mean = round_shift(73'(mean_sum[component]),6'd3);
    wire signed [72:0] mean_value = component==2 ? wrap_once(73'(committed[2])+raw_mean) : raw_mean;
    wire signed [72:0] wrapped_pending = wrap_once(73'(pending_heading));

    always @* begin
        mac_op=1; mac_clear=1; mac_subtract=0; mac_align=0; mac_a=0; mac_b=0;
        case (phase)
            WHEEL_NOISE_STEP: begin mac_a=velocity_std; mac_b=dt; end
            LEVER: begin mac_a=point[3]; mac_b=offset; end
            WHEEL_STEP: begin
                mac_a=center_velocity[31:0]; mac_b=dt;
                mac_op=fits_word(speed_without_bias) && fits_word(center_velocity);
            end
            WHEEL_NOISE_X: begin mac_a=wheel_dist_std; mac_b=central_cos; end
            WHEEL_NOISE_Y: begin mac_a=wheel_dist_std; mac_b=central_sin; end
            TURN: begin mac_a=point[3]; mac_b=dt; end
            MOVE_X: begin mac_a=distance; mac_b=cosine; end
            MOVE_Y: begin mac_a=distance; mac_b=sine; end
            DEVIATION: begin
                mac_a=deviation[31:0]; mac_b=point_num==0 ? CENTRAL_WEIGHT : SIGMA_WEIGHT;
                mac_op=fits_word(deviation);
            end
            FACTOR_SQUARE: begin
                mac_clear=!second_term;
                mac_a=second_term ? vec[pivot] : old_pivot;
                mac_b=mac_a;
                mac_subtract=second_term && row_kind==DOWNDATE;
            end
            FACTOR_U: begin
                mac_clear=!second_term;
                mac_a=second_term ? d : (row_kind==QR ? c : ONE);
                mac_b=second_term ? vec[col] : old_entry;
                mac_subtract=second_term && row_kind==DOWNDATE;
            end
            FACTOR_W: begin
                mac_clear=!second_term;
                mac_a=second_term ? d : c;
                mac_b=second_term ? (row_kind==QR ? old_entry : new_entry) : vec[col];
                mac_subtract=second_term;
            end
            FORWARD_SUM: begin
                mac_clear=term==0;
                mac_a=term>row ? 32'sd0 : committed_root[cell_index(row,{1'b0,term})];
                mac_b=term>col ? 32'sd0 : committed_root[cell_index(col,{1'b0,term})];
            end
            FORWARD_SUB: begin
                mac_clear=0; mac_subtract=1;
                mac_a=factor[cell_index(col,{1'b0,term})]; mac_b=u[{row,term}];
            end
            BACKWARD_SUM: begin
                if (term==col) begin mac_a=u[{row,col[1:0]}]; mac_b=ONE; end
                else begin
                    mac_clear=0; mac_subtract=1;
                    mac_a=factor[cell_index({1'b0,term},col)]; mac_b=gain[term];
                end
            end
            CORRECTION_DOT: begin
                mac_clear=term==0; mac_align=state_alignment(term);
                mac_a=gain[term]; mac_b=residual[term];
            end
            default: mac_op=0;
        endcase
    end
    always @* begin
        div_op=1; numerator=mac_result; denominator=factor[cell_index(col,col)];
        case (phase)
            FACTOR_C: begin
                numerator=72'(row_kind==QR ? old_pivot : radius) <<< 27;
                denominator=row_kind==QR ? radius : old_pivot;
            end
            FACTOR_D: begin
                numerator=72'(vec[pivot]) <<< 27;
                denominator=row_kind==QR ? radius : old_pivot;
            end
            FACTOR_U_DIV: begin numerator=72'(new_entry) <<< 27; denominator=c; end
            FORWARD_DIV, BACKWARD_DIV: begin end
            default: div_op=0;
        endcase
    end

    always @(posedge clk) begin
        if (!resetn) begin
            phase <= IDLE; row_return <= IDLE;
            initialized <= 0; correcting <= 0; second_pass <= 0;
            row_kind <= QR; point_num <= 0; row_num <= 0;
            component <= 0; pivot <= 0; col <= 0; row <= 0; term <= 0;
            second_term <= 0; pending <= 0;
            offset <= 0; velocity_std <= 0; lever <= 0; wheel_dist_std <= 0;
            central_cos <= 0; central_sin <= 0; pending_heading <= 0;
            heading_lift <= 0; mean_heading_lift <= 0;
            wheel <= 0; distance <= 0; angle <= 0; cosine <= 0; sine <= 0;
            radius <= 0; c <= 0; d <= 0; new_entry <= 0; measurement <= 0;
            overflow <= 0; saturation <= 0; negative_radicand <= 0; invalid_pivot <= 0;
            for (integer i=0;i<5;i=i+1) begin
                committed[i] <= 0; candidate[i] <= 0; point[i] <= 0;
                mean_sum[i] <= 0; vec[i] <= 0; q[i] <= 0;
            end
            for (integer i=0;i<4;i=i+1) begin residual[i] <= 0; gain[i] <= 0; end
            for (integer i=0;i<15;i=i+1) begin committed_root[i] <= 0; factor[i] <= 0; end
            for (integer i=0;i<10;i=i+1) r[i] <= 0;
            for (integer i=0;i<20;i=i+1) u[i] <= 0;
        end else if (fault) begin
            phase <= FAILED;
            pending <= 0;
        end else begin
            if (mac_start || div_start || root_start || trig_start) pending <= 1;
            if (mac_done || div_done || root_done || trig_done) pending <= 0;
            if (mac_done && mac_overflow) begin
                overflow <= 1; phase <= FAILED;
            end else if ((root_done && root_overflow) || (div_done && div_overflow) ||
                         (trig_done && trig_overflow)) fail_range();
            else if (root_done && root_negative) begin
                negative_radicand <= 1; fail_pivot();
            end else if ((div_done && div_zero) || (trig_done && trig_error)) fail_pivot();
            else case (phase)
                IDLE: if (init && !initialized) begin
                    initialized <= 1;
                    offset <= wheel_offset; velocity_std <= wheel_std;
                    for (integer i=0;i<5;i=i+1) begin
                        committed[i] <= init_state[32*i+:32];
                        q[i] <= process_std[32*i+:32];
                        if ($signed(init_root[32*cell_index(i[2:0],i[2:0])+:32])<=0 ||
                            $signed(process_std[32*i+:32])<0) fail_pivot();
                    end
                    for (integer i=0;i<15;i=i+1) committed_root[i] <= init_root[32*i+:32];
                    for (integer i=0;i<10;i=i+1) r[i] <= measurement_root[32*i+:32];
                    if ($signed(process_std[128+:32])<=0 || wheel_std<0 ||
                        !angle_valid(73'($signed(init_state[64+:32])))) fail_pivot();
                end else if (predict_start) begin
                    correcting <= 0; second_pass <= 0; point_num <= 0; component <= 0;
                    wheel <= wheel_velocity;
                    for (integer i=0;i<5;i=i+1) mean_sum[i] <= 0;
                    phase <= WHEEL_NOISE_STEP;
                end else if (correct_start) begin
                    correcting <= 1; component <= 0; row_num <= 0;
                    measurement <= sample;
                    for (integer i=0;i<15;i=i+1) factor[i] <= 0;
                    phase <= INNOVATION;
                end
                WHEEL_NOISE_STEP: if (mac_done) begin
                    if (!fits_word(narrowed)) fail_range();
                    else begin wheel_dist_std <= narrowed[31:0]; phase <= SIGMA; end
                end
                LEVER: if (mac_done) begin
                    if (!fits_word(narrowed)) fail_range();
                    else begin lever <= narrowed[31:0]; phase <= WHEEL_STEP; end
                end
                WHEEL_STEP: begin
                    if (!fits_word(speed_without_bias) || !fits_word(center_velocity)) fail_range();
                    else if (mac_done) begin
                        if (!fits_word(narrowed)) fail_range();
                        else begin distance <= narrowed[31:0]; phase <= TURN; end
                    end
                end
                SIGMA: begin
                    if (point_num!=0 && (!fits_word(sigma_span) || !fits_word(sigma_offset)))
                        fail_range();
                    else if (component==2 && point_num!=0 && !angle_valid(sigma_value)) begin
                        heading_lift <= sigma_offset[31:0];
                        pending_heading <= sigma_value[32:0];
                        phase <= SIGMA_WRAP;
                    end else if (point_num!=0 && !fits_word(sigma_value)) fail_range();
                    else begin
                        point[component] <= point_num==0 ? committed[component] : sigma_value[31:0];
                        if (component==2) heading_lift <= point_num==0 ? 32'sd0 : sigma_offset[31:0];
                        if (component==4) begin component <= 0; phase <= LEVER; end
                        else component <= component+1'b1;
                    end
                end
                SIGMA_WRAP: begin
                    if (angle_valid(wrapped_pending)) begin
                        point[2] <= wrapped_pending[31:0];
                        component <= 3; phase <= SIGMA;
                    end else pending_heading <= wrapped_pending[32:0];
                end
                TURN: if (mac_done) begin
                    if (!fits_word(midpoint) || !fits_word(turn_heading) ||
                        !fits_word(narrowed) || !fits_word(turned_lift)) fail_range();
                    else begin
                        angle <= midpoint[31:0];
                        point[2] <= turn_heading[31:0];
                        heading_lift <= turned_lift[31:0];
                        phase <= TRIG;
                    end
                end
                TRIG: if (trig_done) begin
                    cosine <= trig_cos; sine <= trig_sin; phase <= MOVE_X;
                    if (point_num==0 && !second_pass) begin
                        central_cos <= trig_cos; central_sin <= trig_sin;
                    end
                end
                MOVE_X: if (mac_done) begin
                    if (!fits_word(narrowed) || !fits_word(moved_x)) fail_range();
                    else begin point[0] <= moved_x[31:0]; phase <= MOVE_Y; end
                end
                MOVE_Y: if (mac_done) begin
                    if (!fits_word(narrowed) || !fits_word(moved_y)) fail_range();
                    else begin
                        point[1] <= moved_y[31:0];
                        phase <= second_pass ? DEVIATION : MEAN_ADD;
                    end
                end
                MEAN_ADD: begin
                    // 8*W0=-2 and 8*Wi=1, using the signed local heading lift.
                    for (integer i=0;i<5;i=i+1) begin
                        if (i==2)
                            mean_sum[i] <= point_num==0 ? -(36'(heading_lift) <<< 1) :
                                         mean_sum[i]+36'(heading_lift);
                        else mean_sum[i] <= point_num==0 ? -(36'(point[i]) <<< 1) : mean_sum[i]+36'(point[i]);
                    end
                    if (point_num==10) phase <= MEAN_PUBLISH;
                    else begin point_num <= point_num+1'b1; phase <= SIGMA; end
                end
                MEAN_PUBLISH: begin
                    if (!fits_word(mean_value)) fail_range();
                    else if (component==2 && !angle_valid(mean_value)) fail_pivot();
                    else begin
                        candidate[component] <= mean_value[31:0];
                        if (component==2) mean_heading_lift <= raw_mean[31:0];
                        if (component==4) begin
                            component <= 0; second_pass <= 1; point_num <= 1;
                            for (integer i=0;i<15;i=i+1) factor[i] <= 0;
                            phase <= SIGMA;
                        end else component <= component+1'b1;
                    end
                end
                DEVIATION: begin
                    if (!fits_word(deviation)) fail_range();
                    else if (mac_done) begin
                        if (!fits_word(narrowed)) fail_range();
                        else begin
                            vec[component] <= narrowed[31:0];
                            if (component==4) begin
                                component <= 0;
                                if (point_num==0) begin_factor_row(UPDATE,COMMIT);
                                else begin_factor_row(QR,PREDICT_ROW_DONE);
                            end else component <= component+1'b1;
                        end
                    end
                end
                PREDICT_ROW_DONE: begin
                    if (point_num==10) begin row_num <= 0; phase <= PROCESS_NOISE; end
                    else begin point_num <= point_num+1'b1; phase <= SIGMA; end
                end
                PROCESS_NOISE: begin
                    for (integer i=0;i<5;i=i+1) vec[i] <= i==int'(row_num) ? q[i] : 0;
                    begin_factor_row(QR,PROCESS_ROW_DONE);
                end
                PROCESS_ROW_DONE: begin
                    if (row_num==4) phase <= WHEEL_NOISE_X;
                    else begin row_num <= row_num+1'b1; phase <= PROCESS_NOISE; end
                end
                WHEEL_NOISE_X: if (mac_done) begin
                    if (!fits_word(narrowed)) fail_range();
                    else begin vec[0] <= narrowed[31:0]; phase <= WHEEL_NOISE_Y; end
                end
                WHEEL_NOISE_Y: if (mac_done) begin
                    if (!fits_word(narrowed)) fail_range();
                    else begin
                        vec[1] <= narrowed[31:0]; vec[2] <= 0; vec[3] <= 0; vec[4] <= 0;
                        begin_factor_row(QR,WHEEL_NOISE_DONE);
                    end
                end
                WHEEL_NOISE_DONE: begin
                    if (factor[0]<=0 || factor[5]<=0 || factor[9]<=0 || factor[12]<=0 || factor[14]<=0)
                        fail_pivot();
                    else begin point_num <= 0; phase <= SIGMA; end
                end
                INNOVATION: begin
                    if (component==2 && !angle_valid(73'($signed(measurement[64+:32])))) fail_pivot();
                    else if (!fits_word(innovation)) fail_range();
                    else begin
                        residual[component[1:0]] <= innovation[31:0];
                        if (component==3) begin component <= 0; phase <= INNOVATION_ROW; end
                        else component <= component+1'b1;
                    end
                end
                INNOVATION_ROW: begin
                    for (integer i=0;i<4;i=i+1)
                        vec[i] <= i[1:0]<row_num[1:0] ? 32'sd0 :
                                  row_num[2] ? r[measurement_index(i[1:0],row_num[1:0])] :
                                               committed_root[cell_index(i[2:0],{1'b0,row_num[1:0]})];
                    vec[4] <= 0;
                    begin_factor_row(QR,INNOVATION_DONE);
                end
                INNOVATION_DONE: begin
                    if (row_num==7) begin
                        if (factor[0]<=0 || factor[5]<=0 || factor[9]<=0 || factor[12]<=0) fail_pivot();
                        else begin row <= 0; col <= 0; term <= 0; phase <= FORWARD_SUM; end
                    end else begin row_num <= row_num+1'b1; phase <= INNOVATION_ROW; end
                end
                FORWARD_SUM: if (mac_done) begin
                    term <= term+1'b1;
                    if (term==3) phase <= col==0 ? FORWARD_DIV : FORWARD_SUB;
                end
                FORWARD_SUB: if (mac_done) begin
                    term <= term+1'b1;
                    if ({1'b0,term}+3'd1==col) phase <= FORWARD_DIV;
                end
                FORWARD_DIV: if (div_done) begin
                    u[{row,col[1:0]}] <= div_result;
                    term <= col==3 ? 2'd3 : 2'd0;
                    if (col==3) phase <= BACKWARD_SUM;
                    else begin col <= col+1'b1; phase <= FORWARD_SUM; end
                end
                BACKWARD_SUM: if (mac_done) begin
                    term <= term+1'b1;
                    if (term==3) phase <= BACKWARD_DIV;
                end
                BACKWARD_DIV: if (div_done) begin
                    gain[col[1:0]] <= div_result;
                    if (col==0) begin term <= 0; phase <= CORRECTION_DOT; end
                    else begin col <= col-1'b1; term <= col-1'b1; phase <= BACKWARD_SUM; end
                end
                CORRECTION_DOT: if (mac_done) begin
                    if (term!=3) term <= term+1'b1;
                    else if (!fits_word(narrowed) || (row!=2 && !fits_word(corrected))) fail_range();
                    else if (row==2) begin pending_heading <= corrected[32:0]; phase <= CORRECTION_WRAP; end
                    else begin
                        candidate[row] <= corrected[31:0];
                        if (row==4) phase <= LOAD_POSTERIOR;
                        else begin row <= row+1'b1; col <= 0; term <= 0; phase <= FORWARD_SUM; end
                    end
                end
                CORRECTION_WRAP: begin
                    if (angle_valid(wrapped_pending)) begin
                        candidate[2] <= wrapped_pending[31:0];
                        row <= 3; col <= 0; term <= 0; phase <= FORWARD_SUM;
                    end else pending_heading <= wrapped_pending[32:0];
                end
                LOAD_POSTERIOR: begin
                    for (integer i=0;i<15;i=i+1) factor[i] <= committed_root[i];
                    row_num <= 0; phase <= REMOVE_COLUMN;
                end
                REMOVE_COLUMN: begin
                    for (integer i=0;i<5;i=i+1) vec[i] <= u[{i[2:0],row_num[1:0]}];
                    begin_factor_row(DOWNDATE,REMOVE_DONE);
                end
                REMOVE_DONE: begin
                    if (row_num==3) phase <= COMMIT;
                    else begin row_num <= row_num+1'b1; phase <= REMOVE_COLUMN; end
                end
                FACTOR_PIVOT: begin
                    if (old_pivot<0 || (row_kind!=QR && old_pivot==0)) fail_pivot();
                    else if (vec[pivot]==0) phase <= FACTOR_NEXT;
                    else if (old_pivot==0) begin col <= pivot; phase <= FACTOR_COPY; end
                    else begin second_term <= 0; phase <= FACTOR_SQUARE; end
                end
                FACTOR_COPY: begin
                    if (!fits_word(signed_row_entry)) fail_range();
                    else begin
                        factor[cell_index(col,pivot)] <= signed_row_entry[31:0];
                        col <= col+1'b1;
                        if (col==last_pivot) phase <= row_return;
                    end
                end
                FACTOR_SQUARE: if (mac_done) begin
                    if (second_term) phase <= FACTOR_ROOT;
                    second_term <= !second_term;
                end
                FACTOR_ROOT: if (root_done) begin
                    if (root_result==0) fail_pivot();
                    else begin radius <= root_result; phase <= FACTOR_C; end
                end
                FACTOR_C: if (div_done) begin
                    if (row_kind!=QR && div_result==0) fail_pivot();
                    else begin c <= div_result; phase <= FACTOR_D; end
                end
                FACTOR_D: if (div_done) begin
                    d <= div_result; factor[cell_index(pivot,pivot)] <= radius;
                    col <= pivot+1'b1; phase <= pivot==last_pivot ? FACTOR_NEXT : FACTOR_U;
                end
                FACTOR_U: if (mac_done) begin
                    second_term <= !second_term;
                    if (second_term) begin
                        if (!fits_word(narrowed)) fail_range();
                        else begin new_entry <= narrowed[31:0]; phase <= row_kind==QR ? FACTOR_W : FACTOR_U_DIV; end
                    end
                end
                FACTOR_U_DIV: if (div_done) begin new_entry <= div_result; phase <= FACTOR_W; end
                FACTOR_W: if (mac_done) begin
                    second_term <= !second_term;
                    if (second_term) begin
                        if (!fits_word(narrowed)) fail_range();
                        else begin
                            factor[cell_index(col,pivot)] <= new_entry;
                            vec[col] <= narrowed[31:0]; col <= col+1'b1;
                            phase <= col==last_pivot ? FACTOR_NEXT : FACTOR_U;
                        end
                    end
                end
                FACTOR_NEXT: begin
                    pivot <= pivot+1'b1;
                    phase <= pivot==last_pivot ? row_return : FACTOR_PIVOT;
                end
                COMMIT: begin
                    for (integer i=0;i<5;i=i+1) committed[i] <= candidate[i];
                    for (integer i=0;i<15;i=i+1) committed_root[i] <= factor[i];
                    phase <= IDLE;
                end
                FAILED: begin end
                default: phase <= FAILED;
            endcase
        end
    end
endmodule
