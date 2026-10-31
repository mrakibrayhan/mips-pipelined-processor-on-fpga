# MIPS-compatible 5-stage pipelined processor on FPGA (MIPS-P)

A MIPS-compatible 5-stage pipelined processor implemented in Verilog and tested on the RZ-EasyFPGA A2.2 board (Intel Cyclone IV E EP4CE6E22C8).

> **Status:** complete and tested on hardware. Compiles with Intel Quartus Prime Lite.

## At a glance

| Item | Value |
|---|---|
| ISA | MIPS-compatible (MIPS32 integer subset: ALU, shifts, loads/stores, branches/jumps, MUL and HI/LO multiply; no delay slot; `FEATURES` parameter selects instruction groups) |
| Microarchitecture | 5-stage pipelined: IF, ID, EX, MEM, WB with hazard unit, forwarding and stalls. |
| Language | Verilog / SystemVerilog |
| Board | RZ-EasyFPGA A2.2 (Intel Cyclone IV E EP4CE6E22C8, 6,272 LEs, 30 M9K) |
| Tool | Intel Quartus Prime Lite (built with 25.1std) |
| Top-level entity | `top_cpu_mips_p` |
| Board clock | 50 MHz input divided by 4 = 12.5 MHz CPU clock |

## Memory system
Synchronous-read M9K memories: dual-port ROM (1 M9K, fetch + loads), byte-enable RAM (1 M9K), register file (2 M9K copies).

## Repository contents
- `alu_mips.v`
- `controller_mips_p.v`
- `cpu_core_mips_p.v`
- `datapath_mips_p.v`
- `debounce.v`
- `hazard_mips_p.v`
- `mem_mips_p.sv`
- `mul_mips_p.v`
- `regfile_mips_p.v`
- `seg7_decoder.v`
- `seg7_display.v`
- `snapshot_mips_p.v`
- `top_cpu_mips_p.v`
- `timing_constraints.sdc`
- `program.hex`
- `data.hex`
- `MIPS_Pipelined_Cpu.qpf`
- `MIPS_Pipelined_Cpu.qsf`
- `MIPS_Pipelined_Cpu.spf`

- `LICENSE`, `.gitignore`, `README.md`

## Build (Quartus)
1. Install Intel Quartus Prime Lite (the Cyclone IV E family is included).
2. Open `MIPS_Pipelined_Cpu.qpf`.
3. Processing > Start Compilation.
4. Programmer: load `output_files/MIPS_Pipelined_Cpu.sof` (not stored in the repository; see Releases for a ready-made `.sof`).

The `.qsf` already contains the device (`EP4CE6E22C8`), the pin assignments and the file list.

## Run on the board
- **KEY[3]** = reset (the CPU also stays in reset until KEY[2] is pressed once). **KEY[2]** = run.
- LEDG (active low = lit): [0] fault, [1] started, [2] snapshot done, [3] halted.
- Stop rule: `syscall`, `break`, or `j .` halts; illegal instruction = fault.
- The program is loaded from `program.hex`, the initial RAM from `data.hex`, both when the FPGA is programmed. Reset does not reload the RAM; program the FPGA again to restore it.
- **Snapshot / debug:** when the program halts, or when the ISSP source bit is set, the CPU is frozen and registers and RAM words are copied into small RAMs. Read them in Quartus with *Tools > In-System Memory Content Editor*; read counters and status with *Tools > In-System Sources and Probes Editor*. CPI = cycle counter / instruction counter.

## Limitations
- Subset ISA, no interrupts, no caches, no MMU, no floating point.
{c['lim']}

## Related repositories
This is one of 9 individual processor projects on FPGA: MIPS-compatible, RISC-V and ARMv4-compatible, each as single-cycle, multi-cycle and pipelined. <!-- add links when ready -->

## License
MIT, see `LICENSE`.

## Trademarks
MIPS is a trademark of MIPS Technologies, Inc.; this project is an independent, educational, MIPS-*compatible* design and is not affiliated with or endorsed by them.

## Author
Md. Rakib Rayhan, Dept. of ECE, North South University. Research Assistant, Microprocessor, Embedded Systems and IoT Lab (MESIL).
