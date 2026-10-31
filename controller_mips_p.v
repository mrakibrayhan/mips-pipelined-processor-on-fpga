//====================================================================
// controller_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, step 5)  -  control unit (Decode stage)
//
//   Harris & Harris section 7.3.2 / 7.5.2: a MAIN DECODER (opcode) and an ALU DECODER (funct), both in Decode; their
//   outputs travel down the pipeline with the instruction (D/E, E/M, M/W registers in datapath_mips_p.v).
//   Extended to the 52-instruction subset (mips_p_spec.md sections 2, 6). Decoding uses opcode / funct only (plus rt for
//   REGIMM, funct for SPECIAL2); the other fields are don't-care (same as mips_iss.py).
//
//   FEATURES (= FEATURES & BUILT of the core): a class whose bit is 0 decodes as ILLEGAL (FaultD).
//     0 BASE  ADD SUB AND OR SLT ADDI LW SW BEQ J LUI (+ NOP = 0x00000000, always legal)
//     1 ALU   ADDU SUBU XOR NOR SLTU ADDIU SLTI SLTIU ANDI ORI XORI
//     2 SHIFT SLL SRL SRA SLLV SRLV SRAV            3 BR   BNE BLEZ BGTZ BLTZ BGEZ
//     4 JAL   JAL JR JALR                           5 MEMX LB LBU LH LHU SB SH
//     6 MUL   MULT MULTU MFHI MFLO MTHI MTLO MUL    7 STOP SYSCALL BREAK
//   STOP: "b ." (BEQ rs, rs, -1) -> StopD + BselfD (not a branch, reads nothing); SYSCALL / BREAK -> StopD.
//   All outputs are 0 for a bubble (validD = 0).
//   useRsD / useRtD: the instruction really reads rs / rt and it is not $0 (used only for the stall equations).
//====================================================================
module controller_mips_p #(
    parameter [7:0] FEATURES = 8'hFF
) (
    input  [31:0] InstrD,
    input         validD,
    output        RegWriteD, MemtoRegD, MemWriteD, MemSignD, ALUSrcD, ShVarD,
    output [1:0]  RegDstD,                 // 0 rt, 1 rd, 2 $31
    output [1:0]  ImmSrcD,                 // 0 sign, 1 zero, 2 LUI
    output [1:0]  MemSizeD,                // 0 word, 1 half, 2 byte
    output [3:0]  ALUControlD,
    output [2:0]  ResSelD,                 // 0 ALU, 1 HI, 2 LO, 3 product, 4 link
    output [2:0]  HiLoOpD,                 // 0 none, 1 MULT, 2 MULTU, 3 MTHI, 4 MTLO
    output [2:0]  BrTypeD,                 // 0 BEQ, 1 BNE, 2 BLEZ, 3 BGTZ, 4 BLTZ, 5 BGEZ
    output        BranchD, JumpD, JrD,
    output        StopD, FaultD, BselfD,
    output        useRsD, useRtD
);
    wire [5:0] op = InstrD[31:26];
    wire [5:0] fn = InstrD[5:0];
    wire [4:0] rs = InstrD[25:21];
    wire [4:0] rt = InstrD[20:16];

    // ---------------- main decoder + ALU decoder ----------------
    reg       rw, mtr, mw, msg, asrc, shv, br, jmp, jr, stop, ur, ut;
    reg [1:0] rdst, isrc, msz;
    reg [3:0] aluc;
    reg [2:0] rsel, hlo, brt;
    reg [2:0] cls;                         // feature bit of the instruction
    reg       known;
    always @(*) begin
        rw = 1'b0; mtr = 1'b0; mw = 1'b0; msg = 1'b0; asrc = 1'b0; shv = 1'b0; br = 1'b0; jmp = 1'b0; jr = 1'b0;
        stop = 1'b0; ur = 1'b0; ut = 1'b0; rdst = 2'd0; isrc = 2'd0; msz = 2'd0; aluc = 4'd0; rsel = 3'd0; hlo = 3'd0;
        brt = 3'd0; cls = 3'd0; known = 1'b1;
        case (op)
            6'h00: begin                                        // SPECIAL: ALU decoder on funct
                rdst = 2'd1;
                case (fn)
                    6'h20: begin rw = 1; ur = 1; ut = 1; aluc = 4'd0; cls = 3'd0; end   // ADD
                    6'h21: begin rw = 1; ur = 1; ut = 1; aluc = 4'd0; cls = 3'd1; end   // ADDU
                    6'h22: begin rw = 1; ur = 1; ut = 1; aluc = 4'd1; cls = 3'd0; end   // SUB
                    6'h23: begin rw = 1; ur = 1; ut = 1; aluc = 4'd1; cls = 3'd1; end   // SUBU
                    6'h24: begin rw = 1; ur = 1; ut = 1; aluc = 4'd2; cls = 3'd0; end   // AND
                    6'h25: begin rw = 1; ur = 1; ut = 1; aluc = 4'd3; cls = 3'd0; end   // OR
                    6'h26: begin rw = 1; ur = 1; ut = 1; aluc = 4'd4; cls = 3'd1; end   // XOR
                    6'h27: begin rw = 1; ur = 1; ut = 1; aluc = 4'd5; cls = 3'd1; end   // NOR
                    6'h2A: begin rw = 1; ur = 1; ut = 1; aluc = 4'd6; cls = 3'd0; end   // SLT
                    6'h2B: begin rw = 1; ur = 1; ut = 1; aluc = 4'd7; cls = 3'd1; end   // SLTU
                    6'h00: begin rw = 1; ut = 1; aluc = 4'd8;  cls = 3'd2; end          // SLL
                    6'h02: begin rw = 1; ut = 1; aluc = 4'd9;  cls = 3'd2; end          // SRL
                    6'h03: begin rw = 1; ut = 1; aluc = 4'd10; cls = 3'd2; end          // SRA
                    6'h04: begin rw = 1; ur = 1; ut = 1; shv = 1; aluc = 4'd8;  cls = 3'd2; end   // SLLV
                    6'h06: begin rw = 1; ur = 1; ut = 1; shv = 1; aluc = 4'd9;  cls = 3'd2; end   // SRLV
                    6'h07: begin rw = 1; ur = 1; ut = 1; shv = 1; aluc = 4'd10; cls = 3'd2; end   // SRAV
                    6'h08: begin jr = 1; ur = 1; cls = 3'd4; end                                  // JR
                    6'h09: begin jr = 1; ur = 1; rw = 1; rsel = 3'd4; cls = 3'd4; end             // JALR
                    6'h0C, 6'h0D: begin stop = 1; cls = 3'd7; end                                 // SYSCALL, BREAK
                    6'h10: begin rw = 1; rsel = 3'd1; cls = 3'd6; end                             // MFHI
                    6'h12: begin rw = 1; rsel = 3'd2; cls = 3'd6; end                             // MFLO
                    6'h11: begin ur = 1; hlo = 3'd3; cls = 3'd6; end                              // MTHI
                    6'h13: begin ur = 1; hlo = 3'd4; cls = 3'd6; end                              // MTLO
                    6'h18: begin ur = 1; ut = 1; hlo = 3'd1; cls = 3'd6; end                      // MULT
                    6'h19: begin ur = 1; ut = 1; hlo = 3'd2; cls = 3'd6; end                      // MULTU
                    default: known = 1'b0;
                endcase
            end
            6'h01: begin                                        // REGIMM: BLTZ / BGEZ
                br = 1; ur = 1; cls = 3'd3;
                if (rt == 5'd0)      brt = 3'd4;
                else if (rt == 5'd1) brt = 3'd5;
                else                 known = 1'b0;
            end
            6'h02: begin jmp = 1; cls = 3'd0; end                                         // J
            6'h03: begin jmp = 1; rw = 1; rdst = 2'd2; rsel = 3'd4; cls = 3'd4; end        // JAL
            6'h04: begin br = 1; ur = 1; ut = 1; brt = 3'd0; cls = 3'd0; end               // BEQ
            6'h05: begin br = 1; ur = 1; ut = 1; brt = 3'd1; cls = 3'd3; end               // BNE
            6'h06: begin br = 1; ur = 1; brt = 3'd2; cls = 3'd3; end                       // BLEZ
            6'h07: begin br = 1; ur = 1; brt = 3'd3; cls = 3'd3; end                       // BGTZ
            6'h08: begin rw = 1; ur = 1; asrc = 1; aluc = 4'd0; cls = 3'd0; end            // ADDI
            6'h09: begin rw = 1; ur = 1; asrc = 1; aluc = 4'd0; cls = 3'd1; end            // ADDIU
            6'h0A: begin rw = 1; ur = 1; asrc = 1; aluc = 4'd6; cls = 3'd1; end            // SLTI
            6'h0B: begin rw = 1; ur = 1; asrc = 1; aluc = 4'd7; cls = 3'd1; end            // SLTIU
            6'h0C: begin rw = 1; ur = 1; asrc = 1; isrc = 2'd1; aluc = 4'd2; cls = 3'd1; end   // ANDI
            6'h0D: begin rw = 1; ur = 1; asrc = 1; isrc = 2'd1; aluc = 4'd3; cls = 3'd1; end   // ORI
            6'h0E: begin rw = 1; ur = 1; asrc = 1; isrc = 2'd1; aluc = 4'd4; cls = 3'd1; end   // XORI
            6'h0F: begin rw = 1; asrc = 1; isrc = 2'd2; aluc = 4'd11; cls = 3'd0; end         // LUI
            6'h1C: begin                                                                      // SPECIAL2: MUL
                if (fn == 6'h02) begin rw = 1; rdst = 2'd1; ur = 1; ut = 1; rsel = 3'd3; cls = 3'd6; end
                else known = 1'b0;
            end
            6'h20: begin rw = 1; mtr = 1; ur = 1; asrc = 1; msz = 2'd2; msg = 1; cls = 3'd5; end   // LB
            6'h21: begin rw = 1; mtr = 1; ur = 1; asrc = 1; msz = 2'd1; msg = 1; cls = 3'd5; end   // LH
            6'h23: begin rw = 1; mtr = 1; ur = 1; asrc = 1; msz = 2'd0; cls = 3'd0; end            // LW
            6'h24: begin rw = 1; mtr = 1; ur = 1; asrc = 1; msz = 2'd2; cls = 3'd5; end            // LBU
            6'h25: begin rw = 1; mtr = 1; ur = 1; asrc = 1; msz = 2'd1; cls = 3'd5; end            // LHU
            6'h28: begin mw = 1; ur = 1; ut = 1; asrc = 1; msz = 2'd2; cls = 3'd5; end             // SB
            6'h29: begin mw = 1; ur = 1; ut = 1; asrc = 1; msz = 2'd1; cls = 3'd5; end             // SH
            6'h2B: begin mw = 1; ur = 1; ut = 1; asrc = 1; msz = 2'd0; cls = 3'd0; end             // SW
            default: known = 1'b0;
        endcase
    end

    // ---------------- legality, NOP, "b ." ----------------
    wire nop   = (InstrD == 32'd0);
    wire legal = nop | (known & FEATURES[cls]);
    wire bself = (op == 6'h04) & (rs == rt) & (InstrD[15:0] == 16'hFFFF);
    wire ok    = validD & legal & ~nop;              // a NOP has no effect at all

    assign RegWriteD   = ok & rw;
    assign MemtoRegD   = ok & mtr;
    assign MemWriteD   = ok & mw;
    assign MemSignD    = msg;
    assign ALUSrcD     = asrc;
    assign ShVarD      = shv;
    assign RegDstD     = rdst;
    assign ImmSrcD     = isrc;
    assign MemSizeD    = msz;
    assign ALUControlD = aluc;
    assign ResSelD     = rsel;
    assign HiLoOpD     = ok ? hlo : 3'd0;
    assign BrTypeD     = brt;
    assign BranchD     = ok & br & ~bself;
    assign JumpD       = ok & jmp;
    assign JrD         = ok & jr;
    assign BselfD      = ok & bself;
    assign StopD       = ok & (stop | bself);
    assign FaultD      = validD & ~legal;
    assign useRsD      = ok & ur & ~bself & (rs != 5'd0);
    assign useRtD      = ok & ut & ~bself & (rt != 5'd0);
endmodule
