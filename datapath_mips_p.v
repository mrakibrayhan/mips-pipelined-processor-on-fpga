//====================================================================
// datapath_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, step 4)  -  pipelined datapath
//
//   Harris & Harris section 7.5 (full hazard-handling pipeline, section 7.5.3), mips_p_spec.md sections 4-7:
//     F : PCNext mux (RedirD ? target : PCPlus4F), PC register (pc_en), ROM port A (outside, in mem_mips_p) -> InstrF
//     D : F/D register (InstrD, PCPlus4D; EN = ~StallD), register file (2 M9K, read in D), ForwardAD/BD muxes from
//         ALUOutM, branch comparator, sign/zero/LUI extension, branch target, jump target, JR target
//     E : D/E register (CLR = FlushE), ForwardAE/BE muxes (ALUOutM / ResultW), ALUSrc mux, ALU + shifter, multiplier,
//         HI / LO (written at the end of E), result select (ALU / HI / LO / product / link), WriteReg mux (rt / rd / 31)
//     M : E/M register; the data memory (outside) latches ALUResE / WriteDataE at the E->M edge -> ReadDataM
//     W : M/W register, ResultW mux (MemtoRegW), register-file write port
//   'valid' bits mark bubbles (validD is kept in the core). Control signals arriving here from the controller are already
//   0 for a bubble in D; FlushE clears the D/E register (validE = 0, all controls 0).
//   PCxE: the 32-bit PC value carried to E = PCPlus4D (link value of JAL / JALR, stop PC of SYSCALL / BREAK / illegal),
//         or the branch target for "b ." (= its own address, its stop PC).
//====================================================================
module datapath_mips_p #(
    parameter SH_EN  = 1,
    parameter MUL_EN = 1
) (
    input             clk,
    input             rst,
    // ---------------- Fetch ----------------
    input             pc_en,
    output reg [31:0] PCF,
    output     [31:0] PCNext,
    input      [31:0] InstrF,
    // ---------------- Decode ----------------
    input             StallD,
    input             validD,
    output reg [31:0] InstrD,
    // control from the controller (Decode)
    input             RegWriteD, MemtoRegD, MemWriteD, MemSignD, ALUSrcD, ShVarD,
    input      [1:0]  RegDstD, ImmSrcD, MemSizeD,
    input      [3:0]  ALUControlD,
    input      [2:0]  ResSelD, HiLoOpD, BrTypeD,
    input             BranchD, JumpD, JrD, StopD, FaultD, BselfD,
    // hazard unit
    input             ForwardAD, ForwardBD,
    input      [1:0]  ForwardAE, ForwardBE,
    input             FlushE,
    output            RedirD,
    // register-file view (board)
    input             view,
    input      [4:0]  view_ra,
    output     [31:0] RD1D_raw,
    // ---------------- Execute ----------------
    output reg        validE,
    output reg [4:0]  RsE, RtE,
    output     [4:0]  WriteRegE,
    output reg        RegWriteE, MemtoRegE, MemWriteE, MemSignE,
    output reg [1:0]  MemSizeE,
    output reg        StopE, FaultE,
    output     [31:0] ALUResE,                 // data-memory address
    output     [31:0] WriteDataE,              // store data
    output reg [31:0] PCxE,
    output reg [31:0] HI, LO,
    // ---------------- Memory ----------------
    output reg        validM,
    output reg [4:0]  WriteRegM,
    output reg        RegWriteM, MemtoRegM,
    output reg [31:0] ALUOutM,
    input      [31:0] ReadDataM,
    // ---------------- Writeback ----------------
    output reg        validW,
    output reg [4:0]  WriteRegW,
    output reg        RegWriteW,
    output reg        StopW, FaultW,
    output     [31:0] ResultW
);
    // =============== Fetch ===============
    wire [31:0] PCPlus4F = PCF + 32'd4;
    always @(posedge clk) begin
        if (rst)        PCF <= 32'd0;
        else if (pc_en) PCF <= PCNext;
    end

    // =============== F/D register ===============
    reg [31:0] PCPlus4D;
    initial begin InstrD = 32'd0; PCPlus4D = 32'd0; PCF = 32'd0; end
    always @(posedge clk) begin
        if (!StallD) begin
            InstrD   <= InstrF;
            PCPlus4D <= PCPlus4F;
        end
    end

    // =============== Decode ===============
    wire [4:0]  rsD = InstrD[25:21], rtD = InstrD[20:16], rdD = InstrD[15:11], shD = InstrD[10:6];
    wire [31:0] RD1D, RD2D;
    regfile_mips_p rf (.clk(clk), .rst(rst), .ra1F(InstrF[25:21]), .ra2F(InstrF[20:16]), .hold(StallD),
                       .view(view), .ra_view(view_ra), .rd1(RD1D), .rd2(RD2D),
                       .we(RegWriteW), .wa(WriteRegW), .wd(ResultW));
    assign RD1D_raw = RD1D;

    // ForwardAD / ForwardBD (from ALUOutM) and the branch comparator
    wire [31:0] CmpA = ForwardAD ? ALUOutM : RD1D;
    wire [31:0] CmpB = ForwardBD ? ALUOutM : RD2D;
    wire        EqualD = (CmpA == CmpB);
    wire        ZeroA  = (CmpA == 32'd0);
    reg         CondD;
    always @(*) begin
        case (BrTypeD)
            3'd0:    CondD = EqualD;                 // BEQ
            3'd1:    CondD = ~EqualD;                // BNE
            3'd2:    CondD = CmpA[31] | ZeroA;       // BLEZ
            3'd3:    CondD = ~CmpA[31] & ~ZeroA;     // BGTZ
            3'd4:    CondD = CmpA[31];               // BLTZ
            default: CondD = ~CmpA[31];              // BGEZ
        endcase
    end
    wire PCSrcD = BranchD & CondD;

    // extension and targets
    wire [15:0] imm = InstrD[15:0];
    wire [31:0] SignImmD = {{16{imm[15]}}, imm};
    wire [31:0] ExtImmD  = (ImmSrcD == 2'd1) ? {16'd0, imm} : (ImmSrcD == 2'd2) ? {imm, 16'd0} : SignImmD;
    wire [31:0] PCBranchD = PCPlus4D + {SignImmD[29:0], 2'b00};
    wire [31:0] PCJumpD   = {PCPlus4D[31:28], InstrD[25:0], 2'b00};

    assign RedirD = validD & ~StallD & (PCSrcD | JumpD | JrD);
    assign PCNext = RedirD ? (JumpD ? PCJumpD : JrD ? CmpA : PCBranchD) : PCPlus4F;

    // =============== D/E register ===============
    reg  [31:0] RD1E, RD2E, ExtImmE;
    reg  [4:0]  RdE, ShamtE;
    reg  [3:0]  ALUControlE;
    reg  [2:0]  ResSelE, HiLoOpE;
    reg  [1:0]  RegDstE;
    reg         ALUSrcE, ShVarE;
    initial begin
        validE = 1'b0; RegWriteE = 1'b0; MemtoRegE = 1'b0; MemWriteE = 1'b0; StopE = 1'b0; FaultE = 1'b0;
        HiLoOpE = 3'd0; HI = 32'd0; LO = 32'd0;
    end
    always @(posedge clk) begin
        if (rst | FlushE) begin
            validE <= 1'b0; RegWriteE <= 1'b0; MemtoRegE <= 1'b0; MemWriteE <= 1'b0; StopE <= 1'b0; FaultE <= 1'b0;
            HiLoOpE <= 3'd0;
        end else begin
            validE <= validD; RegWriteE <= RegWriteD; MemtoRegE <= MemtoRegD; MemWriteE <= MemWriteD;
            StopE <= StopD; FaultE <= FaultD; HiLoOpE <= HiLoOpD;
        end
        // data part (no clear needed: a bubble has no controls)
        RD1E <= RD1D; RD2E <= RD2D; ExtImmE <= ExtImmD;
        RsE <= rsD; RtE <= rtD; RdE <= rdD; ShamtE <= shD;
        PCxE <= BselfD ? PCBranchD : PCPlus4D;
        ALUControlE <= ALUControlD; ResSelE <= ResSelD; RegDstE <= RegDstD; ALUSrcE <= ALUSrcD; ShVarE <= ShVarD;
        MemSizeE <= MemSizeD; MemSignE <= MemSignD;
    end

    // =============== Execute ===============
    wire [31:0] SrcAE = (ForwardAE == 2'b10) ? ALUOutM : (ForwardAE == 2'b01) ? ResultW : RD1E;
    assign WriteDataE = (ForwardBE == 2'b10) ? ALUOutM : (ForwardBE == 2'b01) ? ResultW : RD2E;
    wire [31:0] SrcBE = ALUSrcE ? ExtImmE : WriteDataE;
    wire [4:0]  ShE   = ShVarE ? SrcAE[4:0] : ShamtE;
    alu_mips #(.SH_EN(SH_EN)) alu (.a(SrcAE), .b(SrcBE), .shamt(ShE), .ctl(ALUControlE), .y(ALUResE));

    wire [63:0] ProdE;
    mul_mips_p #(.MUL_EN(MUL_EN), .LONG_EN(1)) mul (.a(SrcAE), .b(WriteDataE), .sgn(HiLoOpE == 3'd1), .p(ProdE));

    // HI / LO written at the end of E: 1 MULT, 2 MULTU, 3 MTHI, 4 MTLO (no exceptions, so this is safe)
    always @(posedge clk) begin
        if (rst) begin
            HI <= 32'd0; LO <= 32'd0;
        end else if (MUL_EN) begin
            case (HiLoOpE)
                3'd1, 3'd2: begin HI <= ProdE[63:32]; LO <= ProdE[31:0]; end
                3'd3:       HI <= SrcAE;
                3'd4:       LO <= SrcAE;
                default: ;
            endcase
        end
    end

    reg [31:0] ALUOutE;
    always @(*) begin
        case (ResSelE)
            3'd1:    ALUOutE = MUL_EN ? HI : 32'd0;
            3'd2:    ALUOutE = MUL_EN ? LO : 32'd0;
            3'd3:    ALUOutE = ProdE[31:0];
            3'd4:    ALUOutE = PCxE + 32'd4;          // link = PC + 8 (MIPS convention, same as MIPS-SC/MC; Step E G6). PCxE = PCPlus4E
            default: ALUOutE = ALUResE;
        endcase
    end
    assign WriteRegE = (RegDstE == 2'd2) ? 5'd31 : (RegDstE == 2'd1) ? RdE : RtE;

    // =============== E/M register ===============
    initial begin validM = 1'b0; RegWriteM = 1'b0; MemtoRegM = 1'b0; end
    reg StopM, FaultM;
    initial begin StopM = 1'b0; FaultM = 1'b0; end
    always @(posedge clk) begin
        if (rst) begin
            validM <= 1'b0; RegWriteM <= 1'b0; MemtoRegM <= 1'b0; StopM <= 1'b0; FaultM <= 1'b0;
        end else begin
            validM <= validE; RegWriteM <= RegWriteE; MemtoRegM <= MemtoRegE; StopM <= StopE; FaultM <= FaultE;
        end
        ALUOutM <= ALUOutE; WriteRegM <= WriteRegE;
    end

    // =============== M/W register ===============
    reg [31:0] ALUOutW, ReadDataW;
    reg        MemtoRegW;
    initial begin validW = 1'b0; RegWriteW = 1'b0; StopW = 1'b0; FaultW = 1'b0; MemtoRegW = 1'b0; end
    always @(posedge clk) begin
        if (rst) begin
            validW <= 1'b0; RegWriteW <= 1'b0; MemtoRegW <= 1'b0; StopW <= 1'b0; FaultW <= 1'b0;
        end else begin
            validW <= validM; RegWriteW <= RegWriteM; MemtoRegW <= MemtoRegM; StopW <= StopM; FaultW <= FaultM;
        end
        ALUOutW <= ALUOutM; ReadDataW <= ReadDataM; WriteRegW <= WriteRegM;
    end

    // =============== Writeback ===============
    assign ResultW = MemtoRegW ? ReadDataW : ALUOutW;
endmodule
