//====================================================================
// ** Step E (debug unit alignment): S00 removed. FOUR RAMs R00 R16 D00 D16, sweep idx 0..63 (65 cycles), exactly
//    like the ARM / RISC-V / MIPS-SC boards. PC, IR, counters, halted/fault/started are read on the ISSP. **
// snapshot_mips_p.v   (MIPS-compatible 5-stage pipelined CPU, board view, step 13)  -  register / RAM / status snapshot
//                                                                                  for the In-System Memory Content Editor
//   Same unit as snapshot_arm_p.v of ARM-P, with FIVE RAMs (MIPS has 32 registers):
//        R00 = $0  .. $15          R16 = $16 .. $31
//        D00 = RAM word  0 .. 15   (0x10000000 .. 0x1000003C)      D16 = RAM word 16 .. 31 (0x10000040 .. 0x1000007C)
//        S00 = status words: 0 HI   1 PC   2 instruction word (InstrD)   3 cycle counter   4 instruction counter
//                            5 {29'b0, fault, halted, started}   6 pipe status {stopping, validD, validE, validM, validW}
//                            7 LO   8..15 = 0          (words 1..6 = the same as ARM-P R16)
//   WHEN: the program halts or faults (trigger_auto) or the ISSP source bit is set (trigger_src, works at any time).
//   The CPU stops only when its pipeline is EMPTY: 'freeze' stops fetching, the instructions already in the pipeline
//   finish, then the core raises 'view_rdy'; the unit waits for it (state WAIT).
//   The register file and the RAM are synchronous: the value for index i comes one cycle after i is presented, so the
//   sweep presents idx 0..79 and writes word (idx - 1) one cycle later (81 cycles). idx 0..31 = registers, 32..63 = RAM
//   (core view ports), 64..79 = status words (extra_val from the top, selected with widx[3:0]).
//   IDLE (CPU runs) -> WAIT (freeze, until view_rdy) -> SNAP (81 cycles) -> DONE (frozen while the trigger stays true) -> IDLE.
//   RESET CLEAR: while rst = 1 the five RAMs are written with 0, word by word (all in parallel, 16 cycles).
//====================================================================
module snapshot_mips_p (
    input         clk,           // cpu_clk
    input         rst,           // debounced reset (level)
    input         started,
    input         trigger_auto,  // halted | fault
    input         trigger_src,   // ISSP source bit (JTAG clock domain)
    input         view_rdy,      // the core is stopped and its view ports are free
    input  [31:0] view_val,      // core dbg_val (belongs to the idx of the previous cycle)
    input  [31:0] extra_val,     // not used any more (S00 removed, 2026-10-29); kept so the board top is unchanged
    output        freeze,
    output        done,
    output [6:0]  idx,           // view index presented to the core (bits 5:0 used by the core)
    output [6:0]  widx           // index of the word written in this cycle
);
    reg s1 = 1'b0, s2 = 1'b0;
    always @(posedge clk) begin
        s1 <= trigger_src;
        s2 <= s1;
    end
    wire trigger = trigger_auto | s2;

    localparam IDLE = 2'd0, WAIT = 2'd1, SNAP = 2'd2, DONE = 2'd3;
    reg [1:0] state = IDLE;
    reg [6:0] cnt   = 7'd0;
    always @(posedge clk) begin
        if (rst) begin
            state <= IDLE;
            cnt   <= 7'd0;
        end else begin
            case (state)
                IDLE: if (started && trigger) state <= WAIT;
                WAIT: if (view_rdy) begin state <= SNAP; cnt <= 7'd0; end
                SNAP: if (cnt == 7'd64) state <= DONE; else cnt <= cnt + 7'd1;   // 4 RAMs: idx 0..63 (65 cycles)
                DONE: if (!trigger) state <= IDLE;
                default: state <= IDLE;
            endcase
        end
    end
    assign freeze = (state != IDLE);
    assign done   = (state == DONE);
    assign idx    = cnt;
    assign widx   = cnt - 7'd1;
    wire   wr     = (state == SNAP) && (cnt != 7'd0);

    // reset clear: a 4-bit word counter that runs while rst = 1
    reg  [3:0]  clr = 4'd0;
    always @(posedge clk) clr <= rst ? clr + 4'd1 : 4'd0;

    wire [3:0]  word_in   = rst ? clr : widx[3:0];
    wire [31:0] snap_data = rst ? 32'd0 : view_val;            // S00 removed (status is on the ISSP)
    wire [31:0] q_R00_unused, q_R16_unused, q_D00_unused, q_D16_unused;

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R00"), .lpm_type ("altsyncram")
    ) snap_ram_R00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[6:4] == 3'd0)), .q_a (q_R00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=R16"), .lpm_type ("altsyncram")
    ) snap_ram_R16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[6:4] == 3'd1)), .q_a (q_R16_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D00"), .lpm_type ("altsyncram")
    ) snap_ram_D00 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[6:4] == 3'd2)), .q_a (q_D00_unused));

    altsyncram #(
        .operation_mode ("SINGLE_PORT"), .width_a (32), .widthad_a (4), .numwords_a (16),
        .outdata_reg_a ("UNREGISTERED"), .clock_enable_input_a ("BYPASS"), .clock_enable_output_a ("BYPASS"),
        .power_up_uninitialized ("FALSE"), .read_during_write_mode_port_a ("NEW_DATA_NO_NBE_READ"),
        .ram_block_type ("M9K"), .intended_device_family ("Cyclone IV E"),
        .lpm_hint ("ENABLE_RUNTIME_MOD=YES,INSTANCE_NAME=D16"), .lpm_type ("altsyncram")
    ) snap_ram_D16 (.address_a (word_in), .clock0 (clk), .data_a (snap_data), .wren_a (rst | (wr && widx[6:4] == 3'd3)), .q_a (q_D16_unused));

endmodule
