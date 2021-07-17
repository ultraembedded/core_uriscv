### uriscv - Another tiny RISC-V implementation

Github:   [https://github.com/ultraembedded/core_uriscv](https://github.com/ultraembedded/core_uriscv)

Simple, small, multi-cycle 32-bit RISC-V CPU implementation.  
Most instructions take 2 cycles, apart from load/stores which take 4+ cycles (depending on memory latency), and division which can take up-to 34 cycles.

## Features
* 32-bit RISC-V ISA CPU core.
* Support RISC-V’s integer (I), multiplication and division (M), and CSR instructions (Z) extensions (RV32IMZicsr).
* Implements base ISA spec v2.1 and parts of the privileged ISA spec v1.11.
* Supports machine mode privilege level only.
* Configurable support for exceptions, interrupts, timers, multiplication, division and error traps.
* Verified using random instruction sequences using cosimulation against [C++ ISA model](https://github.com/ultraembedded/exactstep).
* Synthesizable Verilog 2001, Verilator and FPGA friendly.
* Coremark:  **1.48 CoreMark/MHz** (with HW mul/div)
* Dhrystone: **0.58 DMIPS/MHz** ('legal compile options' / 337 instructions per iteration / with HW mul/div)

**For my higher performance pipelined cores, see here:**
* Coremark:  **2.94CM/MHZ** - [http://github.com/ultraembedded/riscv](http://github.com/ultraembedded/riscv)
* Coremark:  **4.1CM/MHz** - [http://github.com/ultraembedded/biriscv](http://github.com/ultraembedded/biriscv)

## Getting Started

#### Cloning

To clone this project and its dependencies;

```
git clone https://github.com/ultraembedded/core_uriscv.git

```

#### Running Helloworld

To run a simple test image on the core RTL using Icarus Verilog;

```
# Install Icarus Verilog (Debian / Ubuntu / Linux Mint)
sudo apt-get install iverilog

# [or] Install Icarus Verilog (Redhat / Centos)
#sudo yum install iverilog

# Run a simple test image (test.elf)
cd tb/tb_core_icarus
make
```

The expected output is;
```
Starting bench
VCD info: dumpfile waveform.vcd opened for output.

Test:
1. Initialised data
2. Multiply
3. Divide
4. Shift left
5. Shift right
6. Shift right arithmetic
7. Signed comparision
8. Word access
9. Byte access
10. Comparision
```
#### Configuration

| Param Name                | Valid Range  | Description                                                 |
| ------------------------- |:------------:| ------------------------------------------------------------|
| SUPPORT_MUL               | 1/0          | Enable multiplication instructions.                         |
| SUPPORT_DIV               | 1/0          | Enable division instructions.                               |
| SUPPORT_CSR               | 1/0          | Global enable for CSR/trap/interrupt handling.              |
| SUPPORT_TRAP_LSU_ALIGN    | 1/0          | Enable unaligned memory load / store exception.             |
| SUPPORT_MTVEC             | 1/0          | Configurable exception entry address.                       |
| SUPPORT_MTVAL             | 1/0          | Support MTVAL CSR (holds bad addr / opcode).                |
| SUPPORT_MIP_MIE           | 1/0          | Support MIE and MIP CSR registers.                          |
| SUPPORT_MSCRATCH          | 1/0          | Support MSCRATCH CSR registers (SW read/write).             |
| SUPPORT_MCYCLE            | 1/0          | Support cycle counter / rdtime.                             |
| SUPPORT_MTIMECMP          | 1/0          | Non-std - support timer compare interrupt.                  |
| SUPPORT_TRAP_INVALID_OPC  | 1/0          | Fault on invalid opcodes (enable SW emulation).             |
| SUPPORT_BRAM_REGFILE      | 1/0          | FPGA BlockRAM friendly reg file (inst take 1 cycle longer). |
| ISR_VECTOR                | 'h0-FFFFFFFF | ISR addr = reset_vector + ISR_VECTOR (SUPPORT_MTVEC = 0).   |

