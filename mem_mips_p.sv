//====================================================================
// mem_mips_p.sv  (MIPS-compatible 5-stage pipelined CPU, step 2; SystemVerilog file because of the byte-enable RAM)
//                instruction + data memories with SYNCHRONOUS-read M9K blocks (mips_p_spec.md sections 3, 4)
//
//   Harris & Harris pipelined processor (section 7.5): instruction memory read in Fetch, data memory accessed in Memory.
//     ROM 0x00000000 : 2**IMEM_AW words, ONE M9K used as a DUAL-PORT ROM ($readmemh(INIT_FILE)):
//                      port A = instruction fetch, port B = data loads (constants / .rodata in the ROM)
//     RAM 0x10000000 : 2**DMEM_AW words, ONE M9K with byte enables ($readmemh(DATA_FILE)) (same template as ARM-P)
//     other          : data reads 0, writes ignored.   Instruction fetch always reads the ROM (the index wraps).
//
//   SYNCHRONOUS READ = the M9K input registers ARE [HH] registers:
//     port A : the M9K ADDRESS REGISTER IS THE PC REGISTER. Its input is PCNext; when the PC is not enabled
//              (ia_hold = ~pc_en) the address-hold mux re-presents the latched address; during rst it is 0.
//              So 'instr' = rom[PCF] = InstrF, valid in Fetch exactly like [HH]'s combinational instruction memory.
//     data   : address / store data / write enable / size / sign are latched at the E->M edge (= ALUOutM, WriteDataM,
//              MemWriteM of [HH]); the load data is valid during M on 'd_rdata' (after the aligner) = ReadDataM, and the
//              core registers it into ReadDataW, exactly as in [HH]. The store is written at that same edge.
//   LITTLE-ENDIAN. STORE: the data is copied onto every lane (SB 4 x byte, SH 2 x half) and the byte enables pick the
//          lane(s). size 0 word, 1 halfword, 2 byte. Word accesses ignore adr[1:0], halfword accesses ignore adr[0].
//   One data access per cycle, so read-during-write on the data port never happens (no_rw_check).
//   The RAM keeps its contents through a reset (KEY3); program the FPGA again to restore data.hex.
//====================================================================
module mem_mips_p #(
    parameter IMEM_AW   = 7,                  // 128 words of ROM (board build)
    parameter DMEM_AW   = 5,                  // 32 words of RAM (board build)
    parameter INIT_FILE = "program.hex",
    parameter DATA_FILE = "data.hex"
) (
    input         clk,
    input         rst,
    // port A: instruction fetch
    input  [31:0] ia_next,                    // PCNext
    input         ia_hold,                    // ~pc_en: keep the PC (re-read the same word)
    output [31:0] instr,                      // InstrF = rom[PCF]
    // data port (E -> M)
    input  [31:0] d_adr,                      // ALUOutE (or the board-view address)
    input         d_we,                       // MemWriteE
    input  [1:0]  d_size,                     // 0 word, 1 half, 2 byte
    input         d_sgn,                      // LB / LH
    input  [31:0] d_wdata,                    // WriteDataE (forwarded rt)
    output [31:0] d_q,                        // raw word in M (board view)
    output [31:0] d_rdata                     // aligned / extended load data in M (ReadDataM)
);
    // ---------------- ROM: one M9K, dual port ----------------
    (* romstyle = "M9K" *) reg [31:0] rom [0:(1 << IMEM_AW) - 1];
    initial $readmemh(INIT_FILE, rom);

    reg  [IMEM_AW-1:0] ia_q = {IMEM_AW{1'b0}};
    wire [IMEM_AW-1:0] ia_idx = rst ? {IMEM_AW{1'b0}} : ia_hold ? ia_q : ia_next[IMEM_AW+1:2];
    reg  [31:0] rom_qa, rom_qb;                            // M9K read registers: no power-up value
    always @(posedge clk) begin
        ia_q   <= ia_idx;
        rom_qa <= rom[ia_idx];
    end
    assign instr = rom_qa;

    wire [IMEM_AW-1:0] rb_idx = d_adr[IMEM_AW+1:2];
    always @(posedge clk) rom_qb <= rom[rb_idx];

    // ---------------- store lanes and byte enables ----------------
    wire sel_rom = (d_adr[31:28] == 4'h0);
    wire sel_ram = (d_adr[31:28] == 4'h1);
    wire [DMEM_AW-1:0] widx = d_adr[DMEM_AW+1:2];
    reg [31:0] sdata;
    reg [3:0]  be;
    always @(*) begin
        case (d_size)
            2'd2:    begin sdata = {4{d_wdata[7:0]}};  be = 4'b0001 << d_adr[1:0];        end   // byte
            2'd1:    begin sdata = {2{d_wdata[15:0]}}; be = d_adr[1] ? 4'b1100 : 4'b0011; end   // halfword
            default: begin sdata = d_wdata;            be = 4'b1111;                        end   // word
        endcase
    end
    wire wram = d_we & sel_ram;

    // ---------------- RAM: one 32-bit M9K with byte enables (Intel SystemVerilog byte-enable RAM template) ----------------
    (* ramstyle = "M9K, no_rw_check" *) logic [3:0][7:0] ram [0:(1 << DMEM_AW) - 1];
    initial $readmemh(DATA_FILE, ram);
    logic [31:0] ram_q;
    always @(posedge clk) begin
        if (wram) begin
            if (be[0]) ram[widx][0] <= sdata[7:0];
            if (be[1]) ram[widx][1] <= sdata[15:8];
            if (be[2]) ram[widx][2] <= sdata[23:16];
            if (be[3]) ram[widx][3] <= sdata[31:24];
        end
        ram_q <= ram[widx];
    end

    // ---------------- registered region / byte offset / size / sign (they belong to the word on the outputs) ----------------
    reg rom_r = 1'b0, ram_r = 1'b0, sgn_r = 1'b0;
    reg [1:0] a10_r = 2'd0, size_r = 2'd0;
    always @(posedge clk) begin
        rom_r  <= sel_rom;
        ram_r  <= sel_ram;
        a10_r  <= d_adr[1:0];
        size_r <= d_size;
        sgn_r  <= d_sgn;
    end
    assign d_q = ram_r ? ram_q : rom_r ? rom_qb : 32'd0;

    // ---------------- load aligner (ReadDataM) ----------------
    reg [7:0] b;
    always @(*) begin
        case (a10_r)
            2'd0:    b = d_q[7:0];
            2'd1:    b = d_q[15:8];
            2'd2:    b = d_q[23:16];
            default: b = d_q[31:24];
        endcase
    end
    wire [15:0] h = a10_r[1] ? d_q[31:16] : d_q[15:0];
    assign d_rdata = (size_r == 2'd2) ? {{24{sgn_r & b[7]}}, b} :
                     (size_r == 2'd1) ? {{16{sgn_r & h[15]}}, h} : d_q;
endmodule
