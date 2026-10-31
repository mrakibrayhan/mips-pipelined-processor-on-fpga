//====================================================================
// hazard_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, step 5)  -  hazard unit (mips_p_spec.md section 7)
//
//   Harris & Harris section 7.5.3 (full hazard handling), equations kept:
//     ForwardAE = 10 if RsE != 0 & RsE == WriteRegM & RegWriteM, else 01 if RsE != 0 & RsE == WriteRegW & RegWriteW
//     ForwardBE = same with RtE
//     ForwardAD = RsD != 0 & RsD == WriteRegM & RegWriteM          ForwardBD = same with RtD
//     lwstall     = MemtoRegE & (RsD == WriteRegE | RtD == WriteRegE)
//     branchstall = BranchD & (RegWriteE & (WriteRegE == RsD | WriteRegE == RtD)
//                            | MemtoRegM & (WriteRegM == RsD | WriteRegM == RtD))
//     StallF = StallD = FlushE = lwstall | branchstall
//   Refinements (spec section 7): the register compares in the two stall equations are qualified by the source-use bits
//   (useRsD / useRtD: really read, never $0) so there are no false stalls; JR / JALR (resolved in D like a branch) are
//   included in branchstall.
//   Control / run control:
//     RedirD (taken branch / J / JAL / JR / JALR resolved in D) -> the instruction entering D is a bubble (FlushD)
//     StopE  (b . / SYSCALL / BREAK / illegal in E)              -> kill the instruction entering D (KillD) and the one
//                                                                   in D (FlushE); then 'stopping' stops fetching
//     freeze (board snapshot)                                    -> stop fetching, bubbles into D, the pipeline drains
//   Priority in the F/D register: KillD > StallD > FlushD (as in [HH] the clear of the F/D register needs its enable).
//   pc_en = ~StallF | RedirD (a taken branch updates the PC even while fetching is frozen).
//====================================================================
module hazard_mips_p (
    // Decode
    input        validD,
    input  [4:0] RsD, RtD,
    input        useRsD, useRtD,
    input        BranchD, JrD,
    input        RedirD,
    // Execute
    input  [4:0] RsE, RtE, WriteRegE,
    input        RegWriteE, MemtoRegE,
    input        StopE,              // stop or fault in E (valid)
    // Memory, Writeback
    input  [4:0] WriteRegM, WriteRegW,
    input        RegWriteM, MemtoRegM, RegWriteW,
    // run control
    input        freeze,
    input        stopping,
    // outputs
    output       ForwardAD, ForwardBD,
    output [1:0] ForwardAE, ForwardBE,
    output       lwstall, branchstall,
    output       StallF, StallD, FlushD, KillD, FlushE,
    output       pc_en
);
    assign ForwardAE = ((RsE != 5'd0) & (RsE == WriteRegM) & RegWriteM) ? 2'b10 :
                       ((RsE != 5'd0) & (RsE == WriteRegW) & RegWriteW) ? 2'b01 : 2'b00;
    assign ForwardBE = ((RtE != 5'd0) & (RtE == WriteRegM) & RegWriteM) ? 2'b10 :
                       ((RtE != 5'd0) & (RtE == WriteRegW) & RegWriteW) ? 2'b01 : 2'b00;
    assign ForwardAD = (RsD != 5'd0) & (RsD == WriteRegM) & RegWriteM;
    assign ForwardBD = (RtD != 5'd0) & (RtD == WriteRegM) & RegWriteM;

    wire matchE = (useRsD & (RsD == WriteRegE)) | (useRtD & (RtD == WriteRegE));
    wire matchM = (useRsD & (RsD == WriteRegM)) | (useRtD & (RtD == WriteRegM));
    assign lwstall     = MemtoRegE & matchE;
    assign branchstall = (BranchD | JrD) & ((RegWriteE & matchE) | (MemtoRegM & matchM));

    assign StallD = lwstall | branchstall;
    assign StallF = StallD | freeze | stopping;
    assign FlushD = RedirD | freeze | stopping;
    assign KillD  = StopE;
    assign FlushE = StallD | StopE | ~validD;
    assign pc_en  = ~StallF | RedirD;
endmodule
