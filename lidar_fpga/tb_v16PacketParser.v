`timescale 1ns/1ps
module tb_v16PacketParser;
    reg clk=0;
    always #5 clk=~clk;
    reg resetn=0, valid=0, last=0;
    reg [7:0] data=0;
    wire ready, out_valid;
    wire [15:0] az, dist;
    wire [7:0] refl;
    integer count=0, b, p, i;
    reg [15:0] expected_az, expected_dist;
    reg [7:0] expected_refl;

    v16PacketParser #(.HEADER_BYTES(42)) dut (
        .clk(clk), .resetn(resetn),
        .s_AXIstream_tdata(data), .s_AXIstream_tvalid(valid),
        .s_AXIstream_tlast(last), .s_AXIstream_tready(ready),
        .out_azimuth(az), .out_distance(dist),
        .out_reflectivity(refl), .out_data_valid(out_valid)
    );

    task push;
        input [7:0] value;
        input finish;
        input expected_pulse;
        begin
            @(negedge clk); data=value; valid=1; last=finish;
            @(posedge clk); #1;
            if (!ready) $fatal(1,"Unexpected backpressure");
            if (out_valid !== expected_pulse)
                $fatal(1,"Unexpected valid pulse: b=%0d p=%0d got=%b expected=%b", b,p,out_valid,expected_pulse);
            if (expected_pulse) begin
                if (az !== expected_az || dist !== expected_dist || refl !== expected_refl)
                    $fatal(1,"Wrong point: az=%h dist=%h refl=%h; expected %h %h %h",
                           az,dist,refl,expected_az,expected_dist,expected_refl);
                count=count+1;
            end
        end
    endtask

    task pause_stream;
        begin
            @(negedge clk); valid=0; last=0;
            repeat(3) begin
                @(posedge clk); #1;
                if (out_valid !== 0) $fatal(1,"Unexpected pulse while paused");
            end
        end
    endtask

    initial begin
        repeat(3) @(negedge clk);
        resetn=1;
        // 42 header bytes
        for(i=0;i<42;i=i+1) push(8'h00,0,0);
        for(b=0;b<12;b=b+1) begin
            // Corrupt block 5 to test fault suppression.
            push(b==5 ? 8'h00 : 8'hFF,0,0);
            push(8'hEE,0,0);
            push(b[7:0],0,0); // azimuth low
            push(8'h12,0,0);  // azimuth high
            expected_az={8'h12,b[7:0]};
            for(p=0;p<32;p=p+1) begin
                expected_dist={b[7:0],p[7:0]};
                expected_refl=p[7:0]+8'd10;
                push(p[7:0],0,0);
                if(b==0 && p==0) pause_stream();
                push(b[7:0],0,0);
                push(p[7:0]+8'd10,0,b!=5);
            end
        end
        // timestamp (4 bytes) and factory info (2 bytes)
        for(i=0;i<6;i=i+1) push(8'h00,i==5,0);
        if(count!=352) $fatal(1,"Expected 352 good points, observed %0d",count);
        // Confirm tlast aligned the start of the next frame.
        push(8'h00,1,0);
        $display("PASS: %0d outputs, bad block suppressed, pauses/tlast tested", count);
        $finish;
    end
endmodule
