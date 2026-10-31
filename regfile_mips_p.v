//====================================================================
// regfile_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, step 3)  -  register file with SYNCHRONOUS read (two M9K copies)
//
//   Harris & Harris pipelined datapath (section 7.5.1): the register file is READ IN DECODE (RD1D, RD2D feed the
//   branch comparator in D) and WRITTEN IN WRITEBACK on the falling clock edge (first half of the cycle), so an
//   instruction in Decode sees the value written by Writeback in the same cycle.
//
//   M9K mapping (mips_p_spec.md section 4): each read port is an M9K copy (port 1 = rs, port 2 = rt); both copies share the
//   one write port, so they always hold the same $1..$31. The read ADDRESS is the rs / rt field of InstrF, latched at the
//   F->D edge (the same edge that loads InstrD), so the M9K outputs are valid during D, as in [HH].
//     hold = StallD : the latched address is presented again (address-hold mux), like the F/D register.
//     view = 1      : port 1 reads the board-view index (the pipeline is empty then).
//   The [HH] first-half write is replaced by two bypasses (cycle-equivalent):
//     (a) registered write-first bypass: a write on the SAME edge that latches the read address makes the M9K return the
//         OLD word; the hit is remembered with the written value and selected.
//     (b) combinational W->D bypass: the instruction in W writes at the END of this cycle; if it writes the register
//         read in D, the value (ResultW) is selected now (= [HH] "write in the first half, read in the second half").
//   $0 is never written, so it always reads 0.
//   RESET: rst = 1 clears $1..$31 through the write port, one register per cycle (rst must be held >= 32 cycles; the
//   board holds it for milliseconds, the testbenches for 40 cycles).
//====================================================================
module regfile_mips_p (
    input         clk,
    input         rst,
    // read addresses (from InstrF), data valid in the next cycle (Decode)
    input  [4:0]  ra1F,         // InstrF[25:21] (rs)
    input  [4:0]  ra2F,         // InstrF[20:16] (rt)
    input         hold,         // StallD
    input         view,         // board view (port 1)
    input  [4:0]  ra_view,
    output [31:0] rd1,          // RD1D
    output [31:0] rd2,          // RD2D
    // write port (Writeback)
    input         we,           // RegWriteW
    input  [4:0]  wa,           // WriteRegW
    input  [31:0] wd            // ResultW
);
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf1 [0:31];
    (* ramstyle = "M9K, no_rw_check" *) reg [31:0] rf2 [0:31];
    integer i;
    initial for (i = 0; i < 32; i = i + 1) begin rf1[i] = 32'd0; rf2[i] = 32'd0; end

    reg  [4:0]  clr = 5'd0;
    wire        wen = rst | (we & (wa != 5'd0));
    wire [4:0]  wad = rst ? clr : wa;
    wire [31:0] wdd = rst ? 32'd0 : wd;

    // read addresses (the M9K address registers hold a copy: a1, a2)
    reg  [4:0] a1 = 5'd0, a2 = 5'd0;
    wire [4:0] n1 = view ? ra_view : hold ? a1 : ra1F;
    wire [4:0] n2 = hold ? a2 : ra2F;

    reg [31:0] q1, q2;                                   // M9K read registers: no power-up value
    always @(posedge clk) begin
        if (wen) rf1[wad] <= wdd;
        q1 <= rf1[n1];
    end
    always @(posedge clk) begin
        if (wen) rf2[wad] <= wdd;
        q2 <= rf2[n2];
    end

    // (a) registered write-first bypass
    reg [31:0] wq = 32'd0;
    reg        h1 = 1'b0, h2 = 1'b0;
    always @(posedge clk) begin
        clr <= rst ? clr + 5'd1 : 5'd0;
        a1  <= n1;
        a2  <= n2;
        wq  <= wdd;
        h1  <= wen & (wad == n1);
        h2  <= wen & (wad == n2);
    end
    // (b) W->D bypass (the write port signals of this cycle; never $0, never during rst)
    wire b1 = ~rst & we & (wa != 5'd0) & (wa == a1);
    wire b2 = ~rst & we & (wa != 5'd0) & (wa == a2);
    assign rd1 = b1 ? wd : h1 ? wq : q1;
    assign rd2 = b2 ? wd : h2 ? wq : q2;
endmodule
