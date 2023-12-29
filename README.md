# SDRAM Controller and SDRAM in Caravel User Project
## Run User Project Memory Enabled counter_la Testbench
Run iverilog simulation
```sh
cd ~/SOC-Design-Lab-D/testbench/counter_la_mm
source run_sim
```

Validate the `Call function matmul() in User Project BRAM (mprjram, 0x38000000) return value passed, 0x003e` is printed
```
Reading counter_la_mm.hex
counter_la_mm.hex loaded into memory
Memory 5 bytes = 0x6f 0x00 0x00 0x0b 0x13
VCD info: dumpfile counter_la_mm.vcd opened for output.
LA Test 1 started
Call function matmul() in User Project BRAM (mprjram, 0x38000000) return value passed, 0x003e
Call function matmul() in User Project BRAM (mprjram, 0x38000000) return value passed, 0x0044
Call function matmul() in User Project BRAM (mprjram, 0x38000000) return value passed, 0x004a
Call function matmul() in User Project BRAM (mprjram, 0x38000000) return value passed, 0x0050
LA Test 2 passed
```
