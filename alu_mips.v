//====================================================================
// alu_mips.v   (MIPS-compatible cores, step 4)  -  ALU with the shifter (Execute stage)
//
//   Harris & Harris MIPS ALU (section 5.2.4: ADD/SUB with one adder, AND, OR, SLT) extended with the operations of
//   the 52-instruction subset (mips_p_spec.md section 6). ctl:
//     0 ADD   1 SUB   2 AND   3 OR   4 XOR   5 NOR   6 SLT   7 SLTU
//     8 SLL   9 SRL  10 SRA  11 pass B (LUI: B = imm << 16)
//   One adder/subtractor serves ADD, SUB, SLT and SLTU (SLT = sign of a - b corrected for overflow, SLTU = borrow).
//   Shifts move b (= rt) by 'shamt' (shamt field or rs[4:0], chosen by the datapath).
//   SH_EN = 0 removes the shifter (FEATURES[2] = 0 build).
//====================================================================
module alu_mips #(
    parameter SH_EN = 1
) (
    input  [31:0] a,
    input  [31:0] b,
    input  [4:0]  shamt,
    input  [3:0]  ctl,
    output reg [31:0] y
);
    wire        sub  = (ctl == 4'd1) | (ctl == 4'd6) | (ctl == 4'd7);
    wire [32:0] sum  = {1'b0, a} + {1'b0, sub ? ~b : b} + {32'd0, sub};
    wire        ovf  = (a[31] ^ b[31]) & (a[31] ^ sum[31]);          // overflow of a - b
    wire        slt  = sum[31] ^ ovf;
    wire        sltu = ~sum[32];                                     // borrow

    wire [31:0] shl, shr;
    generate
        if (SH_EN) begin : g_sh
            wire [32:0] shr33 = $signed({ctl == 4'd10 ? b[31] : 1'b0, b}) >>> shamt;   // SRA fills with b[31], SRL with 0
            assign shl = b << shamt;
            assign shr = shr33[31:0];
        end else begin : g_nosh
            assign shl = 32'd0;
            assign shr = 32'd0;
        end
    endgenerate

    always @(*) begin
        case (ctl)
            4'd0, 4'd1: y = sum[31:0];
            4'd2:       y = a & b;
            4'd3:       y = a | b;
            4'd4:       y = a ^ b;
            4'd5:       y = ~(a | b);
            4'd6:       y = {31'd0, slt};
            4'd7:       y = {31'd0, sltu};
            4'd8:       y = shl;
            4'd9, 4'd10: y = shr;
            default:    y = b;
        endcase
    end
endmodule
