//====================================================================
// cpu_core_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, step 5)  -  processor core
//
//   controller_mips_p (Decode control) + datapath_mips_p (pipeline registers, register file, E-stage units, W) +
//   hazard_mips_p + mem_mips_p (synchronous-read M9K: dual-port ROM + byte-enable RAM). Harris & Harris section 7.5.
//
//   F/D valid bit 'validD' (0 = bubble):  rst | KillD -> 0     StallD -> hold     FlushD -> 0     else 1
//   BUILT : classes whose hardware is enabled in this build step (FEATURES bits, controller_mips_p.v). FEATURES is ANDed
//           with BUILT, so a class that is not built yet decodes as illegal.
//           Step 6: 0x01   step 7: 0x07   step 8: 0x1F   step 9: 0x3F   step 10: 0x7F   step 11: 0xFF
//   Run control: the stop instruction ("b .", SYSCALL, BREAK, an illegal word) kills everything younger when it is in
//   Execute and sets 'halted' / 'fault' when it reaches Writeback (the end of its W cycle). Its stop PC (b . -> its own
//   address, the others -> address + 4) is captured in E. rst returns to PC 0 and clears $1..$31 (hold rst >= 32 cycles).
//
//   Board view (DBG_EN = 1): 'freeze' stops fetching; when the pipeline is empty (frozen, halted or faulted) 'view_rdy'
//   = 1 and the view borrows register-file port 1 and the data port: dbg_idx 0..31 = $0..$31, 32..63 = RAM word
//   (idx - 32). SYNCHRONOUS read: dbg_val belongs to the dbg_idx of the PREVIOUS cycle (the snapshot unit allows for it).
//   PC shown = the stop PC after a halt / fault, otherwise PCF (= the next instruction to fetch).
//====================================================================
module cpu_core_mips_p #(
    parameter [7:0] FEATURES  = 8'hFF,
    parameter       IMEM_AW   = 7,
    parameter       DMEM_AW   = 5,
    parameter       INIT_FILE = "program.hex",
    parameter       DATA_FILE = "data.hex",
    parameter [7:0] BUILT     = 8'hFF,
    parameter       DBG_EN    = 0
) (
    input         clk,
    input         rst,
    output reg    halted,
    output reg    fault,
    output        retire,           // 1 when an instruction is in Writeback (the stop instruction included)
    output [31:0] pc,               // PC shown on the board (see above)
    output [31:0] instr,            // instruction in Decode (InstrD)
    output [31:0] hi, lo,
    output [4:0]  pstat,            // pipe status {stopping, validD, validE, validM, validW}
    // board view
    input         freeze,
    output        view_rdy,
    input  [5:0]  dbg_idx,
    output [31:0] dbg_val,
    // trace (testbench)
    output        tr_regw,  output [4:0] tr_wa,  output [31:0] tr_wd,
    output        tr_memw,  output [31:0] tr_adr, output [31:0] tr_mdata, output [1:0] tr_msize
);
    localparam [7:0] FEAT = FEATURES & BUILT;

    // ---------------- wires ----------------
    wire [31:0] PCF, PCNext, InstrF, InstrD, ALUResE, WriteDataE, PCxE, ReadDataM, ResultW, RD1D_raw, d_q;
    wire        RegWriteD, MemtoRegD, MemWriteD, MemSignD, ALUSrcD, ShVarD, BranchD, JumpD, JrD, StopD, FaultD, BselfD;
    wire        useRsD, useRtD;
    wire [1:0]  RegDstD, ImmSrcD, MemSizeD, MemSizeE, ForwardAE, ForwardBE;
    wire [3:0]  ALUControlD;
    wire [2:0]  ResSelD, HiLoOpD, BrTypeD;
    wire        ForwardAD, ForwardBD, FlushE, RedirD, lwstall, branchstall, StallF, StallD, FlushD, KillD, pc_en;
    wire        validE, validM, validW, RegWriteE, MemtoRegE, MemWriteE, MemSignE, StopE, FaultE;
    wire        RegWriteM, MemtoRegM, RegWriteW, StopW, FaultW;
    wire [4:0]  RsE, RtE, WriteRegE, WriteRegM, WriteRegW;

    // ---------------- F/D valid bit, stop state ----------------
    reg validD = 1'b0, stopping = 1'b0;
    reg [31:0] stop_pc = 32'd0;
    initial begin halted = 1'b0; fault = 1'b0; end
    wire StopAnyE = StopE | FaultE;               // (only set for a valid instruction)
    always @(posedge clk) begin
        if (rst | KillD)  validD <= 1'b0;
        else if (StallD)  validD <= validD;
        else              validD <= ~FlushD;
        if (rst)           stopping <= 1'b0;
        else if (StopAnyE) stopping <= 1'b1;
        if (StopAnyE)      stop_pc <= PCxE;
        if (rst) begin
            halted <= 1'b0; fault <= 1'b0;
        end else if (validW) begin
            if (StopW)  halted <= 1'b1;
            if (FaultW) fault  <= 1'b1;
        end
    end

    wire frz = DBG_EN ? freeze : 1'b0;

    // ---------------- control ----------------
    controller_mips_p #(.FEATURES(FEAT)) ctl (
        .InstrD(InstrD), .validD(validD), .RegWriteD(RegWriteD), .MemtoRegD(MemtoRegD), .MemWriteD(MemWriteD),
        .MemSignD(MemSignD), .ALUSrcD(ALUSrcD), .ShVarD(ShVarD), .RegDstD(RegDstD), .ImmSrcD(ImmSrcD),
        .MemSizeD(MemSizeD), .ALUControlD(ALUControlD), .ResSelD(ResSelD), .HiLoOpD(HiLoOpD), .BrTypeD(BrTypeD),
        .BranchD(BranchD), .JumpD(JumpD), .JrD(JrD), .StopD(StopD), .FaultD(FaultD), .BselfD(BselfD),
        .useRsD(useRsD), .useRtD(useRtD));

    hazard_mips_p hz (
        .validD(validD), .RsD(InstrD[25:21]), .RtD(InstrD[20:16]), .useRsD(useRsD), .useRtD(useRtD),
        .BranchD(BranchD), .JrD(JrD), .RedirD(RedirD), .RsE(RsE), .RtE(RtE), .WriteRegE(WriteRegE),
        .RegWriteE(RegWriteE), .MemtoRegE(MemtoRegE), .StopE(StopAnyE), .WriteRegM(WriteRegM), .WriteRegW(WriteRegW),
        .RegWriteM(RegWriteM), .MemtoRegM(MemtoRegM), .RegWriteW(RegWriteW), .freeze(frz), .stopping(stopping),
        .ForwardAD(ForwardAD), .ForwardBD(ForwardBD), .ForwardAE(ForwardAE), .ForwardBE(ForwardBE),
        .lwstall(lwstall), .branchstall(branchstall), .StallF(StallF), .StallD(StallD), .FlushD(FlushD),
        .KillD(KillD), .FlushE(FlushE), .pc_en(pc_en));

    // ---------------- board view ----------------
    wire empty = ~validD & ~validE & ~validM & ~validW;
    assign view_rdy = DBG_EN & empty & (frz | halted | fault);
    wire        view    = view_rdy;
    wire [31:0] dbg_adr = {4'h1, 21'd0, dbg_idx[4:0], 2'b00};
    reg  [5:0]  idx_q = 6'd0;
    always @(posedge clk) idx_q <= dbg_idx;
    generate
        if (DBG_EN) begin : g_dbg
            assign dbg_val = (idx_q < 6'd32) ? RD1D_raw : ({1'b0, idx_q[4:0]} < (6'd1 << DMEM_AW)) ? d_q : 32'd0;
        end else begin : g_nodbg
            assign dbg_val = 32'd0;
        end
    endgenerate
    wire dview = view & dbg_idx[5];

    // ---------------- datapath ----------------
    datapath_mips_p #(.SH_EN(FEAT[2]), .MUL_EN(FEAT[6])) dp (
        .clk(clk), .rst(rst), .pc_en(pc_en), .PCF(PCF), .PCNext(PCNext), .InstrF(InstrF),
        .StallD(StallD), .validD(validD), .InstrD(InstrD),
        .RegWriteD(RegWriteD), .MemtoRegD(MemtoRegD), .MemWriteD(MemWriteD), .MemSignD(MemSignD), .ALUSrcD(ALUSrcD),
        .ShVarD(ShVarD), .RegDstD(RegDstD), .ImmSrcD(ImmSrcD), .MemSizeD(MemSizeD), .ALUControlD(ALUControlD),
        .ResSelD(ResSelD), .HiLoOpD(HiLoOpD), .BrTypeD(BrTypeD), .BranchD(BranchD), .JumpD(JumpD), .JrD(JrD),
        .StopD(StopD), .FaultD(FaultD), .BselfD(BselfD),
        .ForwardAD(ForwardAD), .ForwardBD(ForwardBD), .ForwardAE(ForwardAE), .ForwardBE(ForwardBE), .FlushE(FlushE),
        .RedirD(RedirD), .view(view), .view_ra(dbg_idx[4:0]), .RD1D_raw(RD1D_raw),
        .validE(validE), .RsE(RsE), .RtE(RtE), .WriteRegE(WriteRegE), .RegWriteE(RegWriteE), .MemtoRegE(MemtoRegE),
        .MemWriteE(MemWriteE), .MemSignE(MemSignE), .MemSizeE(MemSizeE), .StopE(StopE), .FaultE(FaultE),
        .ALUResE(ALUResE), .WriteDataE(WriteDataE), .PCxE(PCxE), .HI(hi), .LO(lo),
        .validM(validM), .WriteRegM(WriteRegM), .RegWriteM(RegWriteM), .MemtoRegM(MemtoRegM), .ALUOutM(),
        .ReadDataM(ReadDataM), .validW(validW), .WriteRegW(WriteRegW), .RegWriteW(RegWriteW), .StopW(StopW),
        .FaultW(FaultW), .ResultW(ResultW));

    // ---------------- memories ----------------
    mem_mips_p #(.IMEM_AW(IMEM_AW), .DMEM_AW(DMEM_AW), .INIT_FILE(INIT_FILE), .DATA_FILE(DATA_FILE)) mem (
        .clk(clk), .rst(rst), .ia_next(PCNext), .ia_hold(~pc_en), .instr(InstrF),
        .d_adr(dview ? dbg_adr : ALUResE), .d_we(MemWriteE & ~view), .d_size(view ? 2'd0 : MemSizeE),
        .d_sgn(view ? 1'b0 : MemSignE), .d_wdata(WriteDataE), .d_q(d_q), .d_rdata(ReadDataM));

    assign retire = validW;
    assign pc     = (halted | fault) ? stop_pc : PCF;
    assign instr  = InstrD;
    assign pstat  = {stopping, validD, validE, validM, validW};

    // ---------------- trace ----------------
    assign tr_regw = RegWriteW & (WriteRegW != 5'd0); assign tr_wa = WriteRegW; assign tr_wd = ResultW;
    assign tr_memw = MemWriteE; assign tr_adr = ALUResE; assign tr_mdata = WriteDataE; assign tr_msize = MemSizeE;
endmodule
