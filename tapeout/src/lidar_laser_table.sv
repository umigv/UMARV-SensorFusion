// Per-laser geometry of the Velodyne lidars that have 16 or 32 lasers and use the 1206-byte packet.
//
// Values are taken from the manufacturer's manuals:
//   VLP-16, Puck LITE, Puck Hi-Res   VLP-16 User Manual 63-9243 Rev. F, Table 9-1
//   VLP-32C                          VLP-32C User Manual 63-9325 Rev. B, Table 9-2 and section 8.1
//   HDL-32E                          HDL-32E User Manual 63-9113 Rev. M, Table 9-2 and section 8.1
//
// "laser" is the position of a return within one firing, which is how the packet orders them.
//
//   model  sensor                   lasers  distance unit  one firing takes
//     0    VLP-16, Puck LITE          16        2 mm          55.296 us
//     1    Puck Hi-Res                16        2 mm          55.296 us
//     2    VLP-32C                    32        4 mm          55.296 us
//     3    HDL-32E                    32        2 mm          46.080 us
//
// sin_el and cos_el are the sine and cosine of the laser's fixed vertical angle, signed with
// 15 fractional bits. az_offset is the laser's fixed horizontal offset in units of 0.01 degree,
// to be added to the block's azimuth (only the VLP-32C has any).
//
// The last byte of every data packet is the sensor's product ID; model_from_id decodes it.
//
// Not included: the per-laser vertical position corrections (up to about 11 mm), dual-return mode.
module lidar_laser_table (
    input  logic        [1:0]  model,
    input  logic        [4:0]  laser,
    output logic signed [15:0] sin_el,
    output logic signed [15:0] cos_el,
    output logic signed [9:0]  az_offset,

    output logic               lasers_32,           // 1: 32 lasers and one firing per data block. 0: 16 lasers, two firings per block
    output logic               dist_4mm,            // 1: distances are in 4 mm steps. 0: 2 mm
    output logic        [15:0] firing_period_ns,    // time for one firing of all lasers

    input  logic        [7:0]  product_id,          // last byte of a data packet
    output logic        [1:0]  model_from_id,
    output logic               model_id_known
);

    always_comb begin
        case (model)
            2'd0:    begin lasers_32 = 1'b0; dist_4mm = 1'b0; firing_period_ns = 16'd55296; end
            2'd1:    begin lasers_32 = 1'b0; dist_4mm = 1'b0; firing_period_ns = 16'd55296; end
            2'd2:    begin lasers_32 = 1'b1; dist_4mm = 1'b1; firing_period_ns = 16'd55296; end
            default: begin lasers_32 = 1'b1; dist_4mm = 1'b0; firing_period_ns = 16'd46080; end
        endcase
    end

    always_comb begin
        model_id_known = 1'b1;
        case (product_id)
            8'h22:   model_from_id = 2'd0;      // VLP-16, Puck LITE
            8'h24:   model_from_id = 2'd1;      // Puck Hi-Res
            8'h28:   model_from_id = 2'd2;      // VLP-32C
            8'h21:   model_from_id = 2'd3;      // HDL-32E
            default: begin model_from_id = 2'd0; model_id_known = 1'b0; end
        endcase
    end

    always_comb begin
        sin_el = 16'sd0;
        cos_el = 16'sd32767;
        az_offset = 10'sd0;
        case ({model, laser})
            // VLP-16 and Puck LITE
            {2'd0, 5'd0 }: begin sin_el = -16'sd8481; cos_el = 16'sd31651; az_offset = 10'sd0;   end   //  -15.000 deg
            {2'd0, 5'd1 }: begin sin_el = 16'sd572;   cos_el = 16'sd32763; az_offset = 10'sd0;   end   //    1.000 deg
            {2'd0, 5'd2 }: begin sin_el = -16'sd7371; cos_el = 16'sd31928; az_offset = 10'sd0;   end   //  -13.000 deg
            {2'd0, 5'd3 }: begin sin_el = 16'sd1715;  cos_el = 16'sd32723; az_offset = 10'sd0;   end   //    3.000 deg
            {2'd0, 5'd4 }: begin sin_el = -16'sd6252; cos_el = 16'sd32166; az_offset = 10'sd0;   end   //  -11.000 deg
            {2'd0, 5'd5 }: begin sin_el = 16'sd2856;  cos_el = 16'sd32643; az_offset = 10'sd0;   end   //    5.000 deg
            {2'd0, 5'd6 }: begin sin_el = -16'sd5126; cos_el = 16'sd32365; az_offset = 10'sd0;   end   //   -9.000 deg
            {2'd0, 5'd7 }: begin sin_el = 16'sd3993;  cos_el = 16'sd32524; az_offset = 10'sd0;   end   //    7.000 deg
            {2'd0, 5'd8 }: begin sin_el = -16'sd3993; cos_el = 16'sd32524; az_offset = 10'sd0;   end   //   -7.000 deg
            {2'd0, 5'd9 }: begin sin_el = 16'sd5126;  cos_el = 16'sd32365; az_offset = 10'sd0;   end   //    9.000 deg
            {2'd0, 5'd10}: begin sin_el = -16'sd2856; cos_el = 16'sd32643; az_offset = 10'sd0;   end   //   -5.000 deg
            {2'd0, 5'd11}: begin sin_el = 16'sd6252;  cos_el = 16'sd32166; az_offset = 10'sd0;   end   //   11.000 deg
            {2'd0, 5'd12}: begin sin_el = -16'sd1715; cos_el = 16'sd32723; az_offset = 10'sd0;   end   //   -3.000 deg
            {2'd0, 5'd13}: begin sin_el = 16'sd7371;  cos_el = 16'sd31928; az_offset = 10'sd0;   end   //   13.000 deg
            {2'd0, 5'd14}: begin sin_el = -16'sd572;  cos_el = 16'sd32763; az_offset = 10'sd0;   end   //   -1.000 deg
            {2'd0, 5'd15}: begin sin_el = 16'sd8481;  cos_el = 16'sd31651; az_offset = 10'sd0;   end   //   15.000 deg
            // Puck Hi-Res
            {2'd1, 5'd0 }: begin sin_el = -16'sd5690; cos_el = 16'sd32270; az_offset = 10'sd0;   end   //  -10.000 deg
            {2'd1, 5'd1 }: begin sin_el = 16'sd383;   cos_el = 16'sd32766; az_offset = 10'sd0;   end   //    0.670 deg
            {2'd1, 5'd2 }: begin sin_el = -16'sd4940; cos_el = 16'sd32394; az_offset = 10'sd0;   end   //   -8.670 deg
            {2'd1, 5'd3 }: begin sin_el = 16'sd1144;  cos_el = 16'sd32748; az_offset = 10'sd0;   end   //    2.000 deg
            {2'd1, 5'd4 }: begin sin_el = -16'sd4181; cos_el = 16'sd32500; az_offset = 10'sd0;   end   //   -7.330 deg
            {2'd1, 5'd5 }: begin sin_el = 16'sd1903;  cos_el = 16'sd32713; az_offset = 10'sd0;   end   //    3.330 deg
            {2'd1, 5'd6 }: begin sin_el = -16'sd3425; cos_el = 16'sd32588; az_offset = 10'sd0;   end   //   -6.000 deg
            {2'd1, 5'd7 }: begin sin_el = 16'sd2668;  cos_el = 16'sd32659; az_offset = 10'sd0;   end   //    4.670 deg
            {2'd1, 5'd8 }: begin sin_el = -16'sd2668; cos_el = 16'sd32659; az_offset = 10'sd0;   end   //   -4.670 deg
            {2'd1, 5'd9 }: begin sin_el = 16'sd3425;  cos_el = 16'sd32588; az_offset = 10'sd0;   end   //    6.000 deg
            {2'd1, 5'd10}: begin sin_el = -16'sd1903; cos_el = 16'sd32713; az_offset = 10'sd0;   end   //   -3.330 deg
            {2'd1, 5'd11}: begin sin_el = 16'sd4181;  cos_el = 16'sd32500; az_offset = 10'sd0;   end   //    7.330 deg
            {2'd1, 5'd12}: begin sin_el = -16'sd1144; cos_el = 16'sd32748; az_offset = 10'sd0;   end   //   -2.000 deg
            {2'd1, 5'd13}: begin sin_el = 16'sd4940;  cos_el = 16'sd32394; az_offset = 10'sd0;   end   //    8.670 deg
            {2'd1, 5'd14}: begin sin_el = -16'sd383;  cos_el = 16'sd32766; az_offset = 10'sd0;   end   //   -0.670 deg
            {2'd1, 5'd15}: begin sin_el = 16'sd5690;  cos_el = 16'sd32270; az_offset = 10'sd0;   end   //   10.000 deg
            // VLP-32C
            {2'd2, 5'd0 }: begin sin_el = -16'sd13848; cos_el = 16'sd29698; az_offset = 10'sd140; end   //  -25.000 deg, offset +1.4 deg
            {2'd2, 5'd1 }: begin sin_el = -16'sd572;  cos_el = 16'sd32763; az_offset = -10'sd420; end   //   -1.000 deg, offset -4.2 deg
            {2'd2, 5'd2 }: begin sin_el = -16'sd953;  cos_el = 16'sd32754; az_offset = 10'sd140; end   //   -1.667 deg, offset +1.4 deg
            {2'd2, 5'd3 }: begin sin_el = -16'sd8833; cos_el = 16'sd31555; az_offset = -10'sd140; end   //  -15.639 deg, offset -1.4 deg
            {2'd2, 5'd4 }: begin sin_el = -16'sd6426; cos_el = 16'sd32132; az_offset = 10'sd140; end   //  -11.310 deg, offset +1.4 deg
            {2'd2, 5'd5 }: begin sin_el = 16'sd0;     cos_el = 16'sd32767; az_offset = -10'sd140; end   //    0.000 deg, offset -1.4 deg
            {2'd2, 5'd6 }: begin sin_el = -16'sd381;  cos_el = 16'sd32766; az_offset = 10'sd420; end   //   -0.667 deg, offset +4.2 deg
            {2'd2, 5'd7 }: begin sin_el = -16'sd5037; cos_el = 16'sd32378; az_offset = -10'sd140; end   //   -8.843 deg, offset -1.4 deg
            {2'd2, 5'd8 }: begin sin_el = -16'sd4138; cos_el = 16'sd32506; az_offset = 10'sd140; end   //   -7.254 deg, offset +1.4 deg
            {2'd2, 5'd9 }: begin sin_el = 16'sd190;   cos_el = 16'sd32767; az_offset = -10'sd420; end   //    0.333 deg, offset -4.2 deg
            {2'd2, 5'd10}: begin sin_el = -16'sd190;  cos_el = 16'sd32767; az_offset = 10'sd140; end   //   -0.333 deg, offset +1.4 deg
            {2'd2, 5'd11}: begin sin_el = -16'sd3509; cos_el = 16'sd32580; az_offset = -10'sd140; end   //   -6.148 deg, offset -1.4 deg
            {2'd2, 5'd12}: begin sin_el = -16'sd3046; cos_el = 16'sd32626; az_offset = 10'sd420; end   //   -5.333 deg, offset +4.2 deg
            {2'd2, 5'd13}: begin sin_el = 16'sd762;   cos_el = 16'sd32759; az_offset = -10'sd140; end   //    1.333 deg, offset -1.4 deg
            {2'd2, 5'd14}: begin sin_el = 16'sd381;   cos_el = 16'sd32766; az_offset = 10'sd420; end   //    0.667 deg, offset +4.2 deg
            {2'd2, 5'd15}: begin sin_el = -16'sd2286; cos_el = 16'sd32688; az_offset = -10'sd140; end   //   -4.000 deg, offset -1.4 deg
            {2'd2, 5'd16}: begin sin_el = -16'sd2666; cos_el = 16'sd32659; az_offset = 10'sd140; end   //   -4.667 deg, offset +1.4 deg
            {2'd2, 5'd17}: begin sin_el = 16'sd953;   cos_el = 16'sd32754; az_offset = -10'sd420; end   //    1.667 deg, offset -4.2 deg
            {2'd2, 5'd18}: begin sin_el = 16'sd572;   cos_el = 16'sd32763; az_offset = 10'sd140; end   //    1.000 deg, offset +1.4 deg
            {2'd2, 5'd19}: begin sin_el = -16'sd2096; cos_el = 16'sd32701; az_offset = -10'sd420; end   //   -3.667 deg, offset -4.2 deg
            {2'd2, 5'd20}: begin sin_el = -16'sd1905; cos_el = 16'sd32713; az_offset = 10'sd420; end   //   -3.333 deg, offset +4.2 deg
            {2'd2, 5'd21}: begin sin_el = 16'sd1905;  cos_el = 16'sd32713; az_offset = -10'sd140; end   //    3.333 deg, offset -1.4 deg
            {2'd2, 5'd22}: begin sin_el = 16'sd1334;  cos_el = 16'sd32741; az_offset = 10'sd140; end   //    2.333 deg, offset +1.4 deg
            {2'd2, 5'd23}: begin sin_el = -16'sd1525; cos_el = 16'sd32733; az_offset = -10'sd140; end   //   -2.667 deg, offset -1.4 deg
            {2'd2, 5'd24}: begin sin_el = -16'sd1715; cos_el = 16'sd32723; az_offset = 10'sd140; end   //   -3.000 deg, offset +1.4 deg
            {2'd2, 5'd25}: begin sin_el = 16'sd3993;  cos_el = 16'sd32524; az_offset = -10'sd140; end   //    7.000 deg, offset -1.4 deg
            {2'd2, 5'd26}: begin sin_el = 16'sd2666;  cos_el = 16'sd32659; az_offset = 10'sd140; end   //    4.667 deg, offset +1.4 deg
            {2'd2, 5'd27}: begin sin_el = -16'sd1334; cos_el = 16'sd32741; az_offset = -10'sd420; end   //   -2.333 deg, offset -4.2 deg
            {2'd2, 5'd28}: begin sin_el = -16'sd1144; cos_el = 16'sd32748; az_offset = 10'sd420; end   //   -2.000 deg, offset +4.2 deg
            {2'd2, 5'd29}: begin sin_el = 16'sd8481;  cos_el = 16'sd31651; az_offset = -10'sd140; end   //   15.000 deg, offset -1.4 deg
            {2'd2, 5'd30}: begin sin_el = 16'sd5878;  cos_el = 16'sd32237; az_offset = 10'sd140; end   //   10.333 deg, offset +1.4 deg
            {2'd2, 5'd31}: begin sin_el = -16'sd762;  cos_el = 16'sd32759; az_offset = -10'sd140; end   //   -1.333 deg, offset -1.4 deg
            // HDL-32E
            {2'd3, 5'd0 }: begin sin_el = -16'sd16715; cos_el = 16'sd28184; az_offset = 10'sd0;   end   //  -30.670 deg
            {2'd3, 5'd1 }: begin sin_el = -16'sd5312; cos_el = 16'sd32335; az_offset = 10'sd0;   end   //   -9.330 deg
            {2'd3, 5'd2 }: begin sin_el = -16'sd16051; cos_el = 16'sd28568; az_offset = 10'sd0;   end   //  -29.330 deg
            {2'd3, 5'd3 }: begin sin_el = -16'sd4560; cos_el = 16'sd32449; az_offset = 10'sd0;   end   //   -8.000 deg
            {2'd3, 5'd4 }: begin sin_el = -16'sd15384; cos_el = 16'sd28932; az_offset = 10'sd0;   end   //  -28.000 deg
            {2'd3, 5'd5 }: begin sin_el = -16'sd3806; cos_el = 16'sd32546; az_offset = 10'sd0;   end   //   -6.670 deg
            {2'd3, 5'd6 }: begin sin_el = -16'sd14708; cos_el = 16'sd29282; az_offset = 10'sd0;   end   //  -26.670 deg
            {2'd3, 5'd7 }: begin sin_el = -16'sd3044; cos_el = 16'sd32626; az_offset = 10'sd0;   end   //   -5.330 deg
            {2'd3, 5'd8 }: begin sin_el = -16'sd14019; cos_el = 16'sd29618; az_offset = 10'sd0;   end   //  -25.330 deg
            {2'd3, 5'd9 }: begin sin_el = -16'sd2286; cos_el = 16'sd32688; az_offset = 10'sd0;   end   //   -4.000 deg
            {2'd3, 5'd10}: begin sin_el = -16'sd13328; cos_el = 16'sd29935; az_offset = 10'sd0;   end   //  -24.000 deg
            {2'd3, 5'd11}: begin sin_el = -16'sd1526; cos_el = 16'sd32732; az_offset = 10'sd0;   end   //   -2.670 deg
            {2'd3, 5'd12}: begin sin_el = -16'sd12630; cos_el = 16'sd30236; az_offset = 10'sd0;   end   //  -22.670 deg
            {2'd3, 5'd13}: begin sin_el = -16'sd761;  cos_el = 16'sd32759; az_offset = 10'sd0;   end   //   -1.330 deg
            {2'd3, 5'd14}: begin sin_el = -16'sd11919; cos_el = 16'sd30523; az_offset = 10'sd0;   end   //  -21.330 deg
            {2'd3, 5'd15}: begin sin_el = 16'sd0;     cos_el = 16'sd32767; az_offset = 10'sd0;   end   //    0.000 deg
            {2'd3, 5'd16}: begin sin_el = -16'sd11207; cos_el = 16'sd30792; az_offset = 10'sd0;   end   //  -20.000 deg
            {2'd3, 5'd17}: begin sin_el = 16'sd761;   cos_el = 16'sd32759; az_offset = 10'sd0;   end   //    1.330 deg
            {2'd3, 5'd18}: begin sin_el = -16'sd10490; cos_el = 16'sd31044; az_offset = 10'sd0;   end   //  -18.670 deg
            {2'd3, 5'd19}: begin sin_el = 16'sd1526;  cos_el = 16'sd32732; az_offset = 10'sd0;   end   //    2.670 deg
            {2'd3, 5'd20}: begin sin_el = -16'sd9761; cos_el = 16'sd31280; az_offset = 10'sd0;   end   //  -17.330 deg
            {2'd3, 5'd21}: begin sin_el = 16'sd2286;  cos_el = 16'sd32688; az_offset = 10'sd0;   end   //    4.000 deg
            {2'd3, 5'd22}: begin sin_el = -16'sd9032; cos_el = 16'sd31499; az_offset = 10'sd0;   end   //  -16.000 deg
            {2'd3, 5'd23}: begin sin_el = 16'sd3044;  cos_el = 16'sd32626; az_offset = 10'sd0;   end   //    5.330 deg
            {2'd3, 5'd24}: begin sin_el = -16'sd8299; cos_el = 16'sd31700; az_offset = 10'sd0;   end   //  -14.670 deg
            {2'd3, 5'd25}: begin sin_el = 16'sd3806;  cos_el = 16'sd32546; az_offset = 10'sd0;   end   //    6.670 deg
            {2'd3, 5'd26}: begin sin_el = -16'sd7555; cos_el = 16'sd31885; az_offset = 10'sd0;   end   //  -13.330 deg
            {2'd3, 5'd27}: begin sin_el = 16'sd4560;  cos_el = 16'sd32449; az_offset = 10'sd0;   end   //    8.000 deg
            {2'd3, 5'd28}: begin sin_el = -16'sd6813; cos_el = 16'sd32052; az_offset = 10'sd0;   end   //  -12.000 deg
            {2'd3, 5'd29}: begin sin_el = 16'sd5312;  cos_el = 16'sd32335; az_offset = 10'sd0;   end   //    9.330 deg
            {2'd3, 5'd30}: begin sin_el = -16'sd6067; cos_el = 16'sd32201; az_offset = 10'sd0;   end   //  -10.670 deg
            {2'd3, 5'd31}: begin sin_el = 16'sd6067;  cos_el = 16'sd32201; az_offset = 10'sd0;   end   //   10.670 deg
            default: begin end                  // lasers 16..31 of a 16-laser model
        endcase
    end

endmodule
