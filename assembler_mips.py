#!/usr/bin/env python3
"""
assembler_mips.py - two-pass assembler for the MIPS-compatible CPUs (MIPS-SC, MIPS-MC, MIPS-P).

    python3 assembler_mips.py prog.s [-o program.hex] [--depth 128] [-d data.hex] [--data-depth 32]
                              [--core sc|mc|p] [--listing prog.lst] [--auto-nop]

Input : GNU-style MIPS32 assembly (the integer subset the cores run).
Output: program.hex = one 8-digit hex word per line, exactly --depth lines (instruction ROM, read with $readmemh);
        unused words are filled with 00000000 (nop).  An error is reported if the program does not fit.
        data.hex    = (only if the program has a .data section, or with -d) the starting values of the data RAM,
                      one 32-bit word per line (little-endian bytes inside a word), exactly --data-depth lines.

Instructions
  R-type   add addu sub subu and or xor nor slt sltu           rd, rs, rt
           sll srl sra                                          rd, rt, shamt
           sllv srlv srav                                       rd, rt, rs
           jr rs      jalr rs | jalr rd, rs      syscall   break
           mult multu                                           rs, rt
           div divu                                             rs, rt     (MIPS-MC only: --core mc)
           mfhi mflo rd      mthi mtlo rs
           mul rd, rs, rt                                       (MIPS32 SPECIAL2, rd = low 32 bits of rs x rt)
  I-type   addi addiu slti sltiu andi ori xori                  rt, rs, imm
           lui rt, imm
           lw lh lhu lb lbu sw sh sb                            rt, offset(rs)
           beq bne rs, rt, label      blez bgtz bltz bgez rs, label
  J-type   j label      jal label
Pseudo     nop  move  li  la  b  beqz  bnez  not  neg negu  clear  blt bgt ble bge (and the unsigned ...u forms; they use $at)
Registers  $0..$31 and the names $zero $at $v0-$v1 $a0-$a3 $t0-$t9 $s0-$s7 $k0-$k1 $gp $sp $fp $s8 $ra
Directives .text .data .globl .global .set .ent .end .type .size (ignored)   .align n   .word .half .byte .space .ascii .asciiz
           .equ name, expr   (and  .set name, expr)
Syntax     labels  name:   numeric local labels  1:  with 1b / 1f ; comments  #  or  // ; several statements per line with  ;
           expressions: + - * / % << >> & | ^ ~ ( ), decimal / 0x / 0b / 'c', labels, '.' (current address), %hi(x) %lo(x)

THESE CORES HAVE NO BRANCH DELAY SLOT.  The instruction after a branch or jump is NOT executed, and jal / jalr put
PC + 8 in the link register.  So write a  nop  after every  jal / jalr  (or use --auto-nop, which inserts it for you);
a branch or jump to the next-but-one word is where a function returns to.
Memory map: .text = instruction ROM at 0x00000000;  .data = data RAM at 0x10000000.
Stop rule of the cores: syscall, break, or  j .  (a jump to itself) halts the CPU.
"""
import sys, re, argparse

M32 = 0xFFFFFFFF
RAM_BASE = 0x10000000

REGS = {"zero": 0, "at": 1, "v0": 2, "v1": 3, "a0": 4, "a1": 5, "a2": 6, "a3": 7,
        "t0": 8, "t1": 9, "t2": 10, "t3": 11, "t4": 12, "t5": 13, "t6": 14, "t7": 15,
        "s0": 16, "s1": 17, "s2": 18, "s3": 19, "s4": 20, "s5": 21, "s6": 22, "s7": 23,
        "t8": 24, "t9": 25, "k0": 26, "k1": 27, "gp": 28, "sp": 29, "fp": 30, "s8": 30, "ra": 31}
for _i in range(32):
    REGS[str(_i)] = _i

# name -> (format, opcode/funct, ...)
RTYPE3 = {"add": 0x20, "addu": 0x21, "sub": 0x22, "subu": 0x23, "and": 0x24, "or": 0x25, "xor": 0x26,
          "nor": 0x27, "slt": 0x2A, "sltu": 0x2B}
SHIFT_IMM = {"sll": 0x00, "srl": 0x02, "sra": 0x03}
SHIFT_VAR = {"sllv": 0x04, "srlv": 0x06, "srav": 0x07}
MULDIV = {"mult": 0x18, "multu": 0x19, "div": 0x1A, "divu": 0x1B}
ITYPE_ALU = {"addi": 0x08, "addiu": 0x09, "slti": 0x0A, "sltiu": 0x0B, "andi": 0x0C, "ori": 0x0D, "xori": 0x0E}
LOADS = {"lb": 0x20, "lh": 0x21, "lw": 0x23, "lbu": 0x24, "lhu": 0x25}
STORES = {"sb": 0x28, "sh": 0x29, "sw": 0x2B}
BR2 = {"beq": 0x04, "bne": 0x05}
BR1 = {"blez": (0x06, 0), "bgtz": (0x07, 0), "bltz": (0x01, 0), "bgez": (0x01, 1)}
PSEUDO = {"nop", "move", "li", "la", "b", "beqz", "bnez", "not", "neg", "negu", "clear",
          "blt", "bgt", "ble", "bge", "bltu", "bgtu", "bleu", "bgeu"}
IGNORED_DIRS = {".globl", ".global", ".ent", ".end", ".type", ".size", ".file", ".section", ".option",
                ".abicalls", ".nan", ".module", ".ident", ".frame", ".mask", ".fmask", ".cpload", ".cprestore"}


class AsmError(Exception):
    pass


def reg(tok):
    t = tok.strip()
    if not t.startswith("$"):
        raise AsmError("register expected (like $t0 or $8), got '%s'" % tok)
    n = REGS.get(t[1:].lower())
    if n is None:
        raise AsmError("unknown register '%s'" % tok)
    return n


def split_ops(s):
    """split the operand string at top-level commas"""
    out, depth, cur = [], 0, ""
    for ch in s:
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
        if ch == "," and depth == 0:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def strip_comment(line):
    out, q = "", None
    i = 0
    while i < len(line):
        ch = line[i]
        if q:
            out += ch
            if ch == "\\" and i + 1 < len(line):
                out += line[i + 1]
                i += 1
            elif ch == q:
                q = None
        elif ch in "\"'":
            q = ch
            out += ch
        elif ch == "#" or line.startswith("//", i):
            break
        else:
            out += ch
        i += 1
    return out


def split_statements(line):
    out, cur, q = [], "", None
    for ch in line:
        if q:
            cur += ch
            if ch == q:
                q = None
        elif ch in "\"'":
            q = ch
            cur += ch
        elif ch == ";":
            out.append(cur)
            cur = ""
        else:
            cur += ch
    out.append(cur)
    return [s.strip() for s in out if s.strip()]


class Asm:
    def __init__(self, core="mc", auto_nop=False):
        self.core = core
        self.auto_nop = auto_nop
        self.syms = {}
        self.text = []      # (addr, mnemonic, operands, srcline)
        self.data = bytearray()
        self.sect = ".text"
        self.pc = 0
        self.numeric = {}   # numeric local labels: n -> list of addresses
        self.pending = []

    # ---------------- expression evaluation
    def ev(self, expr, here=None, final=True):
        e = expr.strip()
        e = re.sub(r"%hi\(", "_HI(", e)
        e = re.sub(r"%lo\(", "_LO(", e)

        def num_label(m):
            n, d = int(m.group(1)), m.group(2)
            addrs = self.numeric.get(n, [])
            h = here if here is not None else self.cur_addr()
            if d == "b":
                c = [a for a in addrs if a <= h and a is not None]
                if not c:
                    raise AsmError("no earlier label %db" % n)
                return str(c[-1])
            c = [a for a in addrs if a > h]
            if not c:
                if final:
                    raise AsmError("no later label %df" % n)
                return "0"
            return str(c[0])
        e = re.sub(r"\b(\d+)([bf])\b(?![0-9a-fA-Fx])", num_label, e)

        def sym(m):
            w = m.group(0)
            if w in ("_HI", "_LO"):
                return w
            if re.match(r"^(0[xX][0-9a-fA-F]+|0[bB][01]+|\d+)$", w):
                return w
            if w == ".":
                return str(here if here is not None else self.cur_addr())
            if w in self.syms:
                return str(self.syms[w])
            if not final:
                return "0"
            raise AsmError("undefined symbol '%s'" % w)
        e = re.sub(r"'(\\?.)'", lambda m: str(ord(m.group(1)[-1]) if len(m.group(1)) == 1 else {"n": 10, "t": 9, "0": 0, "r": 13}.get(m.group(1)[1], ord(m.group(1)[1]))), e)
        e = re.sub(r"\.(?![A-Za-z0-9_])|[A-Za-z_][A-Za-z0-9_$.]*|0[xX][0-9a-fA-F]+|0[bB][01]+|\d+", sym, e)
        e = e.replace("/", "//")
        try:
            v = eval(e, {"__builtins__": {}}, {"_HI": lambda x: ((x + 0x8000) >> 16) & 0xFFFF,
                                               "_LO": lambda x: ((x & 0xFFFF) ^ 0x8000) - 0x8000})
        except Exception:
            raise AsmError("bad expression '%s'" % expr)
        return int(v)

    def cur_addr(self):
        return self.pc if self.sect == ".text" else RAM_BASE + len(self.data)

    # ---------------- pass 1
    def define(self, name, val):
        if re.match(r"^\d+$", name):
            self.numeric.setdefault(int(name), []).append(val)
            return
        if name in self.syms:
            raise AsmError("label '%s' defined twice" % name)
        self.syms[name] = val

    def pass1(self, src):
        for ln, raw in enumerate(src.splitlines(), 1):
            line = strip_comment(raw).strip()
            if not line:
                continue
            for stmt in split_statements(line):
                try:
                    self.stmt1(stmt, raw.strip())
                except AsmError as e:
                    raise AsmError("line %d: %s   [%s]" % (ln, e, raw.strip()))

    def stmt1(self, stmt, raw):
        while True:
            m = re.match(r"^([A-Za-z_.$][A-Za-z0-9_.$]*|\d+)\s*:\s*(.*)$", stmt)
            if not m:
                break
            self.define(m.group(1), self.cur_addr())
            stmt = m.group(2).strip()
        if not stmt:
            return
        parts = stmt.split(None, 1)
        op = parts[0].lower()
        args = parts[1] if len(parts) > 1 else ""
        if op.startswith("."):
            self.directive(op, args)
            return
        if self.sect != ".text":
            raise AsmError("instruction in the .data section")
        for (mn, ops) in self.expand(op, args, size_only=True):
            self.text.append([self.pc, mn, ops, raw])
            self.pc += 4
            if self.auto_nop and mn in ("jal", "jalr"):
                self.text.append([self.pc, "nop", "", "(auto-nop)"])
                self.pc += 4

    def directive(self, op, args):
        if op in (".text",):
            self.sect = ".text"
        elif op in (".data", ".bss"):
            self.sect = ".data"
        elif op in IGNORED_DIRS:
            pass
        elif op == ".set":
            a = split_ops(args)
            if len(a) == 2:
                self.syms[a[0]] = self.ev(a[1], final=True)
        elif op in (".equ", ".equiv"):
            a = split_ops(args)
            if len(a) != 2:
                raise AsmError(".equ name, value")
            self.syms[a[0]] = self.ev(a[1], final=True)
        elif op in (".align", ".balign", ".p2align"):
            n = self.ev(args)
            al = (1 << n) if op != ".balign" else n
            if self.sect == ".text":
                while self.pc % al:
                    self.text.append([self.pc, "nop", "", "(align)"])
                    self.pc += 4
            else:
                while len(self.data) % al:
                    self.data.append(0)
        elif op in (".word", ".long", ".int", ".half", ".hword", ".short", ".byte", ".space", ".skip", ".ascii", ".asciiz", ".string"):
            if self.sect == ".text":
                if op in (".word", ".long", ".int"):
                    for a in split_ops(args):
                        self.text.append([self.pc, ".word", a, ".word " + a])
                        self.pc += 4
                    return
                raise AsmError("%s is only allowed in the .data section" % op)
            self.data_item(op, args)
        else:
            raise AsmError("unknown directive '%s'" % op)

    def data_item(self, op, args):
        d = self.data
        if op in (".word", ".long", ".int"):
            while len(d) % 4:
                d.append(0)
            for a in split_ops(args):
                self.pending.append(("w", len(d), a)); d += b"\0\0\0\0"
        elif op in (".half", ".hword", ".short"):
            while len(d) % 2:
                d.append(0)
            for a in split_ops(args):
                self.pending.append(("h", len(d), a)); d += b"\0\0"
        elif op == ".byte":
            for a in split_ops(args):
                self.pending.append(("b", len(d), a)); d += b"\0"
        elif op in (".space", ".skip"):
            d += bytes(self.ev(split_ops(args)[0]))
        else:
            m = re.match(r'^\s*"((?:[^"\\]|\\.)*)"\s*$', args)
            if not m:
                raise AsmError("string expected")
            s = bytes(m.group(1), "utf-8").decode("unicode_escape").encode("latin-1")
            d += s
            if op in (".asciiz", ".string"):
                d.append(0)

    # ---------------- pseudo expansion (sizes are fixed in pass 1)
    def expand(self, op, args, size_only=False):
        o = split_ops(args)
        if op not in PSEUDO:
            return [(op, args)]
        if op == "nop":
            return [("sll", "$0, $0, 0")]
        if op == "move":
            return [("or", "%s, %s, $0" % (o[0], o[1]))]
        if op == "clear":
            return [("or", "%s, $0, $0" % o[0])]
        if op == "not":
            return [("nor", "%s, %s, $0" % (o[0], o[1]))]
        if op in ("neg", "negu"):
            return [("sub" if op == "neg" else "subu", "%s, $0, %s" % (o[0], o[1]))]
        if op == "b":
            return [("beq", "$0, $0, %s" % o[0])]
        if op == "beqz":
            return [("beq", "%s, $0, %s" % (o[0], o[1]))]
        if op == "bnez":
            return [("bne", "%s, $0, %s" % (o[0], o[1]))]
        if op == "li":
            if size_only:
                try:
                    v = self.ev(o[1], final=True)
                except AsmError:            # value not known yet (forward symbol): two words, evaluated in pass 2
                    return [("lui", "%s, ((%s)>>16)&0xFFFF" % (o[0], o[1])),
                            ("ori", "%s, %s, (%s)&0xFFFF" % (o[0], o[0], o[1]))]
            else:
                v = self.ev(o[1])
            v &= M32
            sv = v - (1 << 32) if v >> 31 else v
            if -32768 <= sv <= 32767:
                return [("addiu", "%s, $0, %d" % (o[0], sv))]
            if v <= 0xFFFF:
                return [("ori", "%s, $0, %d" % (o[0], v))]
            if (v & 0xFFFF) == 0:
                return [("lui", "%s, %d" % (o[0], v >> 16))]
            return [("lui", "%s, %d" % (o[0], v >> 16)), ("ori", "%s, %s, %d" % (o[0], o[0], v & 0xFFFF))]
        if op == "la":
            return [("lui", "%s, %%hi(%s)" % (o[0], o[1])), ("addiu", "%s, %s, %%lo(%s)" % (o[0], o[0], o[1]))]
        if op in ("blt", "bgt", "ble", "bge", "bltu", "bgtu", "bleu", "bgeu"):
            u = "u" if op.endswith("u") else ""
            base = op[:3] if u == "" else op[:3]
            slt = "sltu" if u else "slt"
            a, b, lab = o
            if base == "blt":
                return [(slt, "$1, %s, %s" % (a, b)), ("bne", "$1, $0, %s" % lab)]
            if base == "bgt":
                return [(slt, "$1, %s, %s" % (b, a)), ("bne", "$1, $0, %s" % lab)]
            if base == "ble":
                return [(slt, "$1, %s, %s" % (b, a)), ("beq", "$1, $0, %s" % lab)]
            return [(slt, "$1, %s, %s" % (a, b)), ("beq", "$1, $0, %s" % lab)]
        raise AsmError("bad pseudo-instruction")

    # ---------------- pass 2
    def finish_data(self):
        for kind, pos, expr in self.pending:
            v = self.ev(expr, here=RAM_BASE + pos)
            if kind == "w":
                self.data[pos:pos + 4] = (v & M32).to_bytes(4, "little")
            elif kind == "h":
                self.data[pos:pos + 2] = (v & 0xFFFF).to_bytes(2, "little")
            else:
                self.data[pos] = v & 0xFF

    def encode_all(self):
        words = []
        # re-expand 'li' with final values: sizes were fixed in pass 1, so only values change here
        for addr, mn, ops, raw in self.text:
            try:
                words.append(self.encode(addr, mn, ops))
            except AsmError as e:
                raise AsmError("%s   [%s]" % (e, raw))
        return words

    def encode(self, addr, mn, args):
        if mn == ".word":
            return self.ev(args, here=addr) & M32
        if mn == "nop":
            return 0
        o = split_ops(args)
        n = lambda k: self._need(o, k, mn)
        ev = lambda x: self.ev(x, here=addr)

        def R(rs, rt, rd, sh, fn, op=0):
            return (op << 26) | (rs << 21) | (rt << 16) | (rd << 11) | (sh << 6) | fn

        def I(op, rs, rt, imm):
            return (op << 26) | (rs << 21) | (rt << 16) | (imm & 0xFFFF)
        if mn in RTYPE3:
            n(3); return R(reg(o[1]), reg(o[2]), reg(o[0]), 0, RTYPE3[mn])
        if mn in SHIFT_IMM:
            n(3); sa = ev(o[2])
            if not 0 <= sa <= 31:
                raise AsmError("shift amount %d out of range 0..31" % sa)
            return R(0, reg(o[1]), reg(o[0]), sa, SHIFT_IMM[mn])
        if mn in SHIFT_VAR:
            n(3); return R(reg(o[2]), reg(o[1]), reg(o[0]), 0, SHIFT_VAR[mn])
        if mn in MULDIV:
            n(2)
            if mn in ("div", "divu") and self.core != "mc":
                raise AsmError("'%s' exists only in the multi-cycle core (use --core mc)" % mn)
            return R(reg(o[0]), reg(o[1]), 0, 0, MULDIV[mn])
        if mn == "mul":
            n(3); return R(reg(o[1]), reg(o[2]), reg(o[0]), 0, 0x02, op=0x1C)
        if mn in ("mfhi", "mflo"):
            n(1); return R(0, 0, reg(o[0]), 0, 0x10 if mn == "mfhi" else 0x12)
        if mn in ("mthi", "mtlo"):
            n(1); return R(reg(o[0]), 0, 0, 0, 0x11 if mn == "mthi" else 0x13)
        if mn == "jr":
            n(1); return R(reg(o[0]), 0, 0, 0, 0x08)
        if mn == "jalr":
            if len(o) == 1:
                return R(reg(o[0]), 0, 31, 0, 0x09)
            n(2); return R(reg(o[1]), 0, reg(o[0]), 0, 0x09)
        if mn in ("syscall", "break"):
            code = ev(o[0]) if o else 0
            return ((code & 0xFFFFF) << 6) | (0x0C if mn == "syscall" else 0x0D)
        if mn in ITYPE_ALU:
            n(3); imm = ev(o[2])
            if mn in ("andi", "ori", "xori"):
                if not 0 <= imm <= 0xFFFF:
                    raise AsmError("%s immediate %d must be 0..65535" % (mn, imm))
            elif not -32768 <= imm <= 32767:
                raise AsmError("%s immediate %d must be -32768..32767 (use li for bigger values)" % (mn, imm))
            return I(ITYPE_ALU[mn], reg(o[1]), reg(o[0]), imm)
        if mn == "lui":
            n(2); imm = ev(o[1])
            if not (-32768 <= imm <= 65535):
                raise AsmError("lui immediate %d does not fit 16 bits" % imm)
            return I(0x0F, 0, reg(o[0]), imm)
        if mn in LOADS or mn in STORES:
            n(2)
            m = re.match(r"^(.*)\(\s*(\$\w+)\s*\)\s*$", o[1])
            if not m:
                raise AsmError("memory operand must be offset($reg)")
            off = ev(m.group(1)) if m.group(1).strip() else 0
            if not -32768 <= off <= 32767:
                raise AsmError("offset %d does not fit 16 bits" % off)
            return I((LOADS.get(mn) or STORES[mn]), reg(m.group(2)), reg(o[0]), off)
        if mn in BR2 or mn in BR1:
            if mn in BR2:
                n(3); rs, rt, tgt = reg(o[0]), reg(o[1]), o[2]; opc = BR2[mn]
            else:
                n(2); rs, tgt = reg(o[0]), o[1]; opc, rt = BR1[mn]
            t = ev(tgt)
            if t % 4:
                raise AsmError("branch target 0x%x is not word aligned" % t)
            off = (t - (addr + 4)) // 4
            if not -32768 <= off <= 32767:
                raise AsmError("branch target too far")
            return I(opc, rs, rt, off)
        if mn in ("j", "jal"):
            n(1); t = ev(o[0])
            if t % 4:
                raise AsmError("jump target 0x%x is not word aligned" % t)
            if (t >> 28) != ((addr + 4) >> 28):
                raise AsmError("jump target outside the 256 MB region")
            return ((0x02 if mn == "j" else 0x03) << 26) | ((t >> 2) & 0x3FFFFFF)
        raise AsmError("unknown instruction '%s'" % mn)

    @staticmethod
    def _need(o, k, mn):
        if len(o) != k:
            raise AsmError("'%s' needs %d operand(s), got %d" % (mn, k, len(o)))


def main():
    ap = argparse.ArgumentParser(description="assembler for the MIPS-compatible CPUs")
    ap.add_argument("source")
    ap.add_argument("-o", "--output", default="program.hex")
    ap.add_argument("--depth", type=int, default=128, help="instruction ROM words (default 128)")
    ap.add_argument("-d", "--data", default=None, help="data.hex file name (default: written if there is a .data section)")
    ap.add_argument("--data-depth", type=int, default=32, help="data RAM words (default 32)")
    ap.add_argument("--core", choices=["sc", "mc", "p"], default="mc",
                    help="MIPS-SC / MIPS-MC / MIPS-P (div and divu exist only in the multi-cycle core)")
    ap.add_argument("--listing", default=None)
    ap.add_argument("--auto-nop", action="store_true", help="insert a nop after every jal / jalr")
    a = ap.parse_args()
    src = open(a.source).read()
    asm = Asm(core=a.core, auto_nop=a.auto_nop)
    try:
        asm.pass1(src)
        asm.finish_data()
        words = asm.encode_all()
    except AsmError as e:
        print("ASSEMBLY ERROR:", e)
        sys.exit(1)
    if len(words) > a.depth:
        print("ERROR: the program has %d words but the ROM has only %d (use --depth)" % (len(words), a.depth))
        sys.exit(1)
    with open(a.output, "w") as f:
        for w in words + [0] * (a.depth - len(words)):
            f.write("%08x\n" % w)
    print("%s: %d instruction words (ROM depth %d)" % (a.output, len(words), a.depth))
    if a.listing:
        with open(a.listing, "w") as f:
            for (addr, mn, ops, raw), w in zip(asm.text, words):
                f.write("%08x  %08x  %s\n" % (addr, w, raw))
    if a.data or asm.data:
        d = bytes(asm.data)
        d += b"\0" * ((-len(d)) % 4)
        dw = [int.from_bytes(d[i:i + 4], "little") for i in range(0, len(d), 4)]
        if len(dw) > a.data_depth:
            print("ERROR: the data has %d words but the RAM has only %d (use --data-depth)" % (len(dw), a.data_depth))
            sys.exit(1)
        name = a.data or "data.hex"
        with open(name, "w") as f:
            for w in dw + [0] * (a.data_depth - len(dw)):
                f.write("%08x\n" % w)
        print("%s: %d data words (RAM depth %d)" % (name, len(dw), a.data_depth))


if __name__ == "__main__":
    main()
