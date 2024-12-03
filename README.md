# General-Purpose In-cache Computing (*MVE*) Simulator

*MVE* is **an in-cache computing simulator** designed on top of Ramulator \[1\] to simulate the data-parallel applications in-cache.

Similar to the base simulator (Ramulator), *MVE* is a trace-driven simulator that is designed to reply
the dynamic instruction traces. The current version only accepts a design-specific vector ISA extension.

## Trace Format

Traces provided to this simulator contain 4 types of instructions:

1. **CPU Load and Store Instructions:** *MVE* simulates the execution of scalar CPU loads and stores.
Each line of CPU load and store trace must be in this format:

  - `<load/store> <address> <num-cpuinst>`

      - where `<num-cpuinst>` shows the number of compute instructions before this trace line.


2. **In-Cache Config Operations:** The length of vector instructions is #SA x 256.
The customized ISA changes the vector dimension and length using the following instructions:

  - `<instruction> -1 -1 -1 <config-dim> <config-val> <num-cpuinst>`


3. **In-Cache Vector Memory Operations:** Vector memory operations include multiple strides.
Vector memory instructions in this version of *MVE* comes with this configuration:

  - `<instruction> <dst> <address> -1 <stride3> <stride2> <stride1> <stride0> <num-cpuinst>`

      - If the memory operation is random, base random addresses follow the trace line.

      - `[mem-trace] <base-addr-0> <base-addr-1> ...`


4. **In-Cache Vector Compute Operations:** Vector compute instructions in this version of *MVE* comes with this configuration:

  - `<instruction> <dst> <src1> <src2> -1 -1 -1 -1 <num-cpuinst>`


## Usage

First compile the simulator using these commands:

    $ cd ramulator
    $ make -j

Then, run your simulator using this command:

    $ ./ramulator <config-file> --mode=MVE --core=1 <core-type> --stats <stat-file> <trace-file>

where:
  - `<config-file>` contains DRAM config, proportional CPU and DRAM frequency, and in-cache computing level.
  You can find an example in `./configs/LPDDR4-config-MVE2.cfg` for in-L2 computing.
  
  - `<core-type>` Currently, we support 3 core configurations: `prime`, `gold`, `silver` which are Cortex-A76 cores of Qualcomm Snapdragon 855 SoC.

  - `<stat-file>` is where your simulation results will be stored.

  - `<trace-file>` contains the trace of dynamic instructions.


## Debug

To activate debugging logs, please compile ramulator with `DEBUG` flag:
  
    $ make -j CFLAGS='-DDEBUG'

## How it works?

Refer to [a relative link](MVE_README.md)

## References

[\[1\] Kim et al. *Ramulator: A Fast and Extensible DRAM Simulator.* IEEE CAL
2015.](https://people.inf.ethz.ch/omutlu/pub/ramulator_dram_simulator-ieee-cal15.pdf)  
