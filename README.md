# Reproducing GPSR in ns-3

Course project for Network Performance Evaluation (2627I_INT3308_1).
Student: Trần Anh Khoa – 24022806.

This repository implements GPSR (B. Karp, H. T. Kung, *GPSR: Greedy Perimeter Stateless Routing for Wireless Networks*, ACM MobiCom 2000) in ns-3.43 and reruns the experiments from Section 3 of the paper.

## Repository layout

| Path | Contents |
| --- | --- |
| `src/gpsr/` | GPSR module: beacons, neighbor table, greedy forwarding, perimeter forwarding (RNG/GG), MAC failure feedback |
| `scratch/gpsr-paper-scenario.cc` | Simulation scenario following Section 3 of the paper, with all measurements |
| `scratch/gpsr-void-test.cc` | Test of perimeter mode on a topology with a void |
| `run_sim_and_plot.py` | Runs the simulations in parallel, averages the results and plots the figures |
| `results/` | Summary data (`simulation_summary.csv`) and figures from the 102 runs |
| `report/` | Report (PDF, in Vietnamese) and its LaTeX source (build with XeLaTeX) |

## Installation

Copy `src/gpsr` and the two files in `scratch/` into the ns-3.43 directory, then build:

```bash
cp -r src/gpsr         ~/ns-allinone-3.43/ns-3.43/src/
cp scratch/*.cc        ~/ns-allinone-3.43/ns-3.43/scratch/
cp run_sim_and_plot.py ~/ns-allinone-3.43/ns-3.43/
cd ~/ns-allinone-3.43/ns-3.43
./ns3 configure -d release --enable-examples
./ns3 build
```

## Running

```bash
# a single run: 50 nodes, B = 1.5 s, pause time 0
./ns3 run "scratch/gpsr-paper-scenario --nNodes=50 --helloInterval=1.5 --pauseTime=0 --runIndex=1"

# perimeter-mode test (RNG and GG)
./ns3 run "scratch/gpsr-void-test --planarMode=0"
./ns3 run "scratch/gpsr-void-test --planarMode=1"

# all GPSR experiments (102 runs of 900 s each) and plots
python3 run_sim_and_plot.py --jobs 6 --scaling --no-dsr
```

Results are written to `results/`. If `results/simulation_raw_results.csv` already exists, the script skips the configurations it already contains.

## Notes

- The scenario uses a static ARP table by default (`--staticArp=1`); see Section 2.3 of the report. Use `--staticArp=0` for ns-3's dynamic ARP.
- The reported results cover GPSR only (`--no-dsr`); Section 2.4 of the report explains why ns-3's DSR is not used as a baseline.
- Each run also prints a `DIAG` line counting dropped packets by cause (ARP, IP, MAC).
