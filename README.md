# Meshinator

> *Like mycelium, the network doesn't break — it grows around the wound.*

**Autonomous mesh intelligence for tactical drone swarms**  
*Evil Incorporated — Junction x Aalto Defence Hackathon 2026*

---

## The Problem

Modern drone systems rely on a fragile star topology: each drone maintains a peer-to-peer connection to a single controller or command-and-control (C2) node. Lose that link — through jamming, signal attenuation, or physical loss of a node — and the entire swarm goes dark.

Off-the-shelf mesh solutions are either too expensive to fit the economics of modern autonomous systems, or too naive to survive contested electromagnetic environments. We needed something else.

---

## The Solution: Meshinator

Meshinator replaces the fragile hub-and-spoke topology with a **two-tier nested cluster architecture** inspired by how biological systems — mycelium networks, immune systems, ant colonies — self-heal without any central coordinator.

Drones organise into **peer clusters of 3–5 units**, each member mutually responsible for its neighbours. These clusters in turn connect into **clusters of clusters**, forming a scalable, resilient mesh that degrades gracefully under pressure.

---

## How It Works

### Layer 1 — Intra-Cluster: The Witness Protocol

Every drone in a cluster continuously exchanges **alive-ping** messages with its peers. If a node stops receiving pings from a neighbour, it doesn't immediately declare it lost. Instead, it broadcasts a query to the rest of the cluster:

> *"Can you still see node X?"*

Neighbours respond with when they last observed node X. If the consensus is that node X has been silent too long, the cluster:

1. **Declares node X missing**
2. **Estimates its predicted last position** based on trajectory and last known coordinates
3. **Dispatches the closest available drone** from the cluster to move into the gap between the missing node and the rest — bridging the broken link and restoring mesh connectivity

This is the **biomimetic witness protocol**: distributed fault detection through peer consensus, without any central authority.

### Layer 2 — Inter-Cluster: Cluster-of-Clusters

The same witness logic is applied at a higher level. Clusters themselves send health signals to one another. If a cluster loses too many members to remain viable, neighbouring clusters detect the degradation and trigger **disassociation and reassociation**:

- Surviving drones regroup across former cluster boundaries
- New clusters of the correct size (3–5 members) reform automatically
- The swarm restructures itself into a healthy topology without operator intervention

### The Healing Behaviour (Simulation Demo)

The Python simulation demonstrates this end-to-end. When a node is reported missing:

- Other nodes read the missing-drone event from the shared message bus
- The closest available drone calculates an intercept position — the midpoint between itself and the last known location of the missing node
- It repositions to that bridging point
- The missing drone, now within radio range again, can resume pinging and re-enters the mesh

---

## Architecture

```
Controller
    │
    └── Cluster of Clusters
            ├── Cluster A  [drone 1 ── drone 2 ── drone 3]
            ├── Cluster B  [drone 4 ── drone 5 ── drone 6]
            └── Cluster C  [drone 7 ── drone 8 ── drone 9]
```

Each cluster maintains full mesh connectivity internally. Inter-cluster links connect cluster representatives. Loss of any single node or link triggers autonomous healing at the appropriate layer.

---

## Key Features

| Feature | Description |
|---|---|
| **Biomimetic witness protocol** | Distributed consensus-based node fault detection — no single point of failure in the decision loop |
| **Autonomous bridging** | Missing nodes trigger automatic repositioning of a healthy drone to restore the link |
| **Two-tier nested clustering** | Scalable from a handful of drones to large swarms without architectural changes |
| **Cluster reassociation** | When clusters lose members, survivors regroup into valid-sized clusters automatically |
| **Real-time mesh overlay dashboard** | Live visualisation of node positions, health status, packet statistics, and event feed |
| **Jamming resilience** | No reliance on a single uplink; swarm can operate and self-coordinate with a degraded or absent C2 link |

---

## Implementation

### Transmission Layer
Raw IEEE 802.11 frames via packet-injection-capable USB Wi-Fi adapters, using the `kova-wfb-rs` library (Rust core, Python bindings) (IMPORTANT: WE DID NOT WRITE ANY RUST!). Custom radiotap headers for signal metadata.

### Mesh Layer
Python simulation with a `Cluster` and `Node` abstraction. Each node runs an independent message loop, reads events from peers, maintains a neighbour registry with last-seen timestamps, and participates in witness queries.

### Application Layer
- **MeshOps Tactical Dashboard** — real-time web UI showing mesh topology, node registry, packet statistics, active/missing/down state, and a live event feed
- **Autonomous healing logic** — `mesh_healing.py` handles bridging drone dispatch; `mesh_restructure.py` handles reassociation events

### Repository Structure

```
meshinator/
├── simulation/
│   └── src/
│       ├── drone.py            # Node class, message loop, witness protocol
│       ├── mesh_healing.py     # Bridging drone dispatch logic
│       ├── mesh_restructure.py # Cluster reassociation logic
│       └── examples/           # Topology input files (0.toml, 1.toml, 2.toml)
├── src/                        # C/Rust radio transmission layer
│   ├── main.cpp
│   └── Node.hpp
└── thirdparty/                 # kova-wfb-rs bindings
```

---

## Running the Simulation

```bash
cd simulation
python -m venv myenv && source myenv/bin/activate
pip install -r requirements.txt
```

Each `.toml` file defines a node's ID, initial position, and cluster membership. Nodes communicate via local sockets in simulation mode, and via raw Wi-Fi frames in hardware mode.

---

## The Dashboard

The **MeshOps Tactical Dashboard** provides a real-time operator view of the mesh:

- **Mesh topology map** — live node positions with active/missing/down colour coding
- **Node details panel** — per-node position (X, Y, Z), packet sent/received/total counters, last-seen timestamp
- **Node registry table** — fleet-wide status at a glance
- **Event feed** — timestamped log of all ping events, missing declarations, and healing actions
- **Fleet metrics** — active nodes, missing nodes, down nodes, total packets, packets/min

Supports both **Mock** (simulated data stream) and **Live** (real hardware) modes.

---

## Future Directions

- **Dynamic cluster reassociation** — clusters are configured at launch but can autonomously rebalance mid-mission as the swarm spreads or contracts
- **Improved inter-cluster communication** — dedicated cluster-head election and optimised cross-cluster routing
- **Positional overlays** — integrating GPS/IMU data from the swarm into the dashboard for mission planning and visualisation
- **Encryption layer** — end-to-end authenticated frames to resist spoofing in adversarial RF environments

---

## Built With

- **Python 3** — simulation, mesh logic, healing and restructure algorithms
- **Rust / C** — low-level radio transmission using library provided by `kova-wfb-rs`
- **IEEE 802.11 raw frames** — packet injection for infrastructure-free communication
- **React / Next.js** — MeshOps tactical dashboard

---

## Hardware

Each node runs on a device equipped with a packet-injection-capable USB Wi-Fi adapter (provided by Kova Labs). The same codebase runs in simulation (sockets) and on hardware (raw 802.11 frames) with a single config change.

---

## About Evil Incorporated

*We have a button for that.*

Built at **Junction x Aalto Defence Hackathon 2026** in response to the Kova Labs challenge: build a tactical mesh communication system from the ground up, using low-power radio hardware, for autonomous drone coordination in contested environments.

---

*Meshinator — because the network doesn't break. It grows around the wound.*
