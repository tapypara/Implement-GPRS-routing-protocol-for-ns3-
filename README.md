# Greedy Perimeter Stateless Routing (GPSR) for ns-3.40

An accurate implementation of the **Greedy Perimeter Stateless Routing (GPSR)** geographic routing protocol based on the landmark paper by **Brad Karp and H. T. Kung (MobiCom 2000)** for the **ns-3.40** discrete-event network simulator.

---

## 1\. Protocol Overview

**Greedy Perimeter Stateless Routing (GPSR)** is a geographic routing protocol designed for wireless ad-hoc and mobile sensor networks (MANETs/WSNs). Unlike traditional ad-hoc routing protocols that maintain global network topology maps (such as DSR or AODV), GPSR makes routing decisions statelessly using only the physical geographic coordinates of network nodes and immediate 1-hop neighbor locations.

GPSR combines two main packet forwarding algorithms:

1. **Greedy Forwarding**: The current node forwards incoming packets to the 1-hop neighbor physically closest to destination node D.
2. **Perimeter Forwarding**: When a packet encounters a local maximum (a node with no 1-hop neighbor closer to destination D than itself), GPSR seamlessly switches to Perimeter Mode to navigate around the perimeter of the void/obstacle using planar graph traversal (Right-Hand Rule).

---

## 2\. Implementation Architecture &amp; Key Features (ns-3.40)

This implementation strictly adheres to the protocol parameters and algorithms specified in the MobiCom 2000 paper:

### 2.1\. Greedy Mode &amp; Beaconing Mechanism

* **Beacon Interval (B)**: Default configured at B = 1.5s (`HelloInterval`).
* **Jitter Avoidance**: Employs `UniformRandomVariable(0.5 * b, 1.5 * b)` to randomize beacon transmission intervals around B, preventing synchronized beacon collisions among adjacent nodes.
* **Neighbor Table Expiration (T)**: Configured at T = 4.5 \* B = 6.75s (`m_entryLifeTime`).
* **Beacon Header (`HelloHeader`)**: Encapsulates two 32-bit floating-point coordinates (`float x, y`), yielding an exact 8-byte header overhead.

### 2.2\. Perimeter Mode &amp; Graph Planarization

* **Planarization Algorithms**: Supports two graph planarization algorithms to eliminate intersecting edges:  
  * **RNG (Relative Neighborhood Graph)**: Default mode (`PlanarMode = 0`).
  * **GG (Gabriel Graph)**: Optional mode (`PlanarMode = 1`).
* **State Tagging (`PerimeterTag`)**: Fully encapsulates packet state fields according to Table 1 of the original paper:  
  * M: Routing mode (Greedy / Perimeter).
  * Lp: Location of local maximum where Perimeter mode was entered.
  * Lf: Location of point on face edge closest to destination D.
  * e0: First edge traversed on current planar face (for loop detection).
  * D: Destination node coordinates.
* **Right-Hand Rule Traversal**: Uses counter-clockwise angle calculation (`GetAngle`, `BestAngle`) to select the next hop along the planar face.
* **Face Changing**: Evaluates segment intersections between candidate next-hop edges and the Lp -&gt; D line segment (`SegmentIntersection`).
* **Perimeter Exit**: Returns immediately to Greedy mode when reaching a node closer to D than Lp.
* **Loop Detection**: Checks for duplicate e0 traversal (`tag.GetE0Sender() == myIp &amp;&amp; tag.GetE0Receiver() == nextHop`) to drop unroutable packets.

### 2.3\. MAC Feedback &amp; Position Piggybacking

* **MAC Failure Feedback**: Listens for `DroppedMpdu` events with `WIFI_MAC_DROP_REACHED_RETRY_LIMIT` from the Wifi MAC layer to purge unreachable neighbors immediately without waiting for the 4.5B timer.
* **Position Piggybacking**: Extracts the previous hop's location from data packets in `RouteInput` to dynamically update neighbor tables.

---

## 3\. Simulation Scenario Design (Scenario 2 - MobiCom 2000 Setup)

The simulation script (`gpsr-paper-scenario.cc`) replicates the experimental setup from the original paper's ns-2 CMU Wireless Extension:

### 3.1\. Physical &amp; MAC Layer Parameters

* **Wifi Standard**: 802.11b at 2 Mbps (`DsssRate2Mbps`).
* **Tx Power**: 17.5 dBm (\~250m transmission range).
* **RTS/CTS**: Enabled (`rtsCts = true`), matching the ns-2 default (`RtsThreshold = 0`) to mitigate hidden-terminal issues in high-density topologies.

### 3.2\. Traffic &amp; Mobility Models

* **Traffic**: Constant Bit Rate (CBR) UDP traffic (64-byte payload at 4 pkts/sec = 2.048 Kbps per flow).
* **Sources**: 30 simultaneous CBR flows (`nSources = 30`).
* **Mobility**: Random Waypoint Mobility Model in a 1500m x 300m area with maximum speed V\_max = 20 m/s (V\_min = 1 m/s to prevent speed decay).

---

## 4\. Directory Structure

```
ns-3.40/
├── src/
│   └── gpsr/
│       ├── model/
│       │   ├── gpsr-routing-protocol.h
│       │   ├── gpsr-routing-protocol.cc
│       │   ├── gpsr-ptable.h
│       │   ├── gpsr-ptable.cc
│       │   └── gpsr-packet.h
│       ├── helper/
│       │   ├── gpsr-helper.h
│       │   └── gpsr-helper.cc
│       └── CMakeLists.txt
└── scratch/
    └── gpsr-paper-scenario.cc

```

---

## 5\. Build &amp; Execution Guide

### 5.1\. Prerequisites

* **ns-3 version**: `ns-3.40`
* C++17 compliant compiler (`g++` or `clang++`), CMake, and Python 3.

### 5.2\. Compilation

From the `ns-3.40` root directory:

```
./ns3 configure --enable-examples --enable-tests
./ns3 build

```

### 5.3\. Running Simulation Scenarios

#### Basic Run:

```
./ns3 run scratch/gpsr-paper-scenario

```

#### Parameter Sweep Examples:

* **Vary Pause Time (0s, 30s, 60s, 120s)**:  
```  
./ns3 run "scratch/gpsr-paper-scenario --pauseTime=30.0"  
```
* **Vary Node Count (50, 112, 200 nodes)**:  
```  
./ns3 run "scratch/gpsr-paper-scenario --nNodes=112"  
```
* **Change Random Seed Index (1 to 6)**:  
```  
./ns3 run "scratch/gpsr-paper-scenario --runIndex=2"  
```
* **Switch Planarization Mode (RNG=0, GG=1)**:  
```  
./ns3 run "scratch/gpsr-paper-scenario --planarMode=1"  
```

---

## 6\. Output &amp; Metrics

Each run prints a standardized CSV metric line to stdout:

```
RESULT,
