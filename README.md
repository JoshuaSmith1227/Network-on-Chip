# Network-on-Chip (NoC) Router Implementation

A high-performance, synthesizable SystemVerilog implementation of a Network-on-Chip routing architecture with dual-router topology and packet-based communication.

## Overview

This project implements a complete NoC system designed for efficient inter-component communication in multi-core systems. The implementation features a dual-router architecture with six endpoint nodes, supporting concurrent packet routing with round-robin arbitration and multi-level queuing.

### Key Features

- **Dual-router mesh topology** with 4 ports per router
- **Packet-based communication** with 32-bit packet structure
- **Byte-serial routing** (8-bit transfers) with parallel endpoint interfaces (32-bit)
- **Multi-level queuing** with per-source-destination queuing to prevent head-of-line blocking
- **Round-robin arbitration** for fair bandwidth allocation
- **Backpressure flow control** to prevent packet loss
- **Configurable FIFO depths** with capacity-aware admission control

## Architecture

### Network Topology

```
     N0 ─┐         ┌─ N3
     N1 ─┤ Router0 ├─ Router1 ├─ N4
     N2 ─┘    ↕    └─────↕     └─ N5
```

The network consists of two routers (Router0 and Router1) interconnected via dedicated ports. Each router supports 4 bidirectional ports that can connect to either nodes or other routers. Routing decisions are made based on static destination node IDs.

### Packet Format

```systemverilog
typedef struct packed {
  logic [3:0] src;   // Source node ID (0-5)
  logic [3:0] dest;  // Destination node ID (0-5)
  logic [23:0] data; // Payload data
} pkt_t;
```

**Total packet size:** 32 bits

## Protocol Specifications

### Node-to-Router Communication (Byte-Serial)

The byte-serial protocol streams packets over 4 clock cycles:

1. **Cycle 0:** `{src[3:0], dest[3:0]}` - Header byte with source and destination IDs
2. **Cycle 1:** `data[23:16]` - Most significant data byte
3. **Cycle 2:** `data[15:8]` - Middle data byte
4. **Cycle 3:** `data[7:0]` - Least significant data byte

**Handshake signals:**
- `free_outbound` - Receiver ready to accept data
- `put_outbound` - Sender has valid data (asserted during all 4 bytes)
- `payload_outbound[7:0]` - Data bus

### Testbench-to-Node Communication (Parallel)

Full 32-bit packets are transferred in a single cycle:

**Handshake signals:**
- `pkt_in_avail` - Testbench has valid packet
- `cQ_full` - Node's input FIFO is full
- `pkt_in[31:0]` - Full packet data

## Implementation Details

### Node Module

The `Node` module serves as the interface between testbench components and the routing network.

**Key components:**
- **Input FIFO**: Depth-5 circular buffer for incoming packets from testbench
- **Byte serializer**: Converts 32-bit packets to 4-byte streams
- **Byte deserializer**: Reassembles 4-byte streams back into 32-bit packets
- **Dual FSM architecture**: Separate state machines for TX and RX paths

**Node-to-Router FSM states:**
- `IDLE_N2R` - Waiting for packet in FIFO
- `DATA1_N2R` - Transmitting byte 1
- `DATA2_N2R` - Transmitting byte 2  
- `DATA3_N2R` - Transmitting byte 3, FIFO dequeue

**Router-to-Node FSM states:**
- `IDS_R2N` - Receiving header byte
- `DATA1_R2N` - Receiving data byte 1
- `DATA2_R2N` - Receiving data byte 2
- `DATA3_R2N` - Receiving data byte 3, packet complete

### Router Module

The `Router` module implements the core packet switching logic with advanced queuing.

**Architecture:**

1. **Packet Builders (4 instances):**
   - One per input port
   - Deserializes incoming byte streams into complete packets
   - Implements backpressure when router capacity is exceeded

2. **Multi-Level Queue Structure:**
   - 4 output ports × 3 sub-queues per port = 12 total queues
   - Each sub-queue represents a different source port
   - Depth: 32 packets per queue
   - Prevents head-of-line blocking

3. **Round-Robin Arbitration:**
   - Fair scheduling across all active flows
   - Per-port arbiter with rotating priority pointer
   - Ensures no source starvation

4. **Packet Senders (4 instances):**
   - One per output port
   - Serializes packets to byte streams
   - Handles outbound flow control

**Routing Logic:**

The router uses static destination-based routing:

```systemverilog
function automatic logic [1:0] nodeToPort(input logic [3:0] dest_node);
  if(ROUTERID == 0) begin
    if     (dest_node == 4'd0) return 2'd0;
    else if(dest_node == 4'd1) return 2'd2;
    else if(dest_node == 4'd2) return 2'd3;
    else                       return 2'd1; // Route to Router1
  end else begin
    if     (dest_node == 4'd3) return 2'd0;
    else if(dest_node == 4'd4) return 2'd1;
    else if(dest_node == 4'd5) return 2'd2;
    else                       return 2'd3; // Route to Router0
  end
endfunction
```

**Capacity Management:**

The router implements admission control to prevent internal queue overflow:
- Tracks total packet count across all queues
- Maximum capacity: 24 packets (headroom for in-flight packets)
- Backpressure applied when capacity limit reached

### FIFO Implementation

**Node FIFO:**
- **Depth:** 5 entries
- **Width:** 32 bits
- Circular buffer with separate read/write pointers
- Combinational read, registered write
- Full/empty status flags

**Router FIFO:**
- **Depth:** Parameterizable (default 32)
- **Width:** Parameterizable (default 32)
- Optimized for high throughput
- Binary counter for occupancy tracking

## Design Decisions

### Why Multi-Level Queuing?

Traditional single-queue-per-port designs suffer from **head-of-line blocking**: a packet destined for a busy output blocks other packets to idle outputs. By maintaining separate queues per source-destination pair, the router achieves:
- **Increased throughput** under mixed traffic patterns
- **Reduced latency** for non-conflicting flows
- **Better fairness** across different source-destination pairs

### Why Round-Robin Arbitration?

Round-robin provides predictable, fair bandwidth allocation:
- **Prevents starvation** of any particular source
- **Low complexity** compared to priority-based schemes
- **Deterministic behavior** for testing and verification

### Capacity-Based Admission Control

Rather than relying solely on per-queue full flags, the router tracks total capacity:
- **Prevents deadlock** scenarios
- **Provides early warning** of congestion
- **Simplifies backpressure** logic

## Testing

The implementation has been verified against comprehensive testbenches:

### Test Scenarios

- **BASIC:** Single packet transfers between all node pairs within same router
- **ACROSS:** Single packet transfers across the inter-router bridge
- **BROADCAST:** Triangular concurrent transmissions (e.g., N0→N1, N1→N2, N2→N3 simultaneously)
- **STRESS_SRC:** Sustained bombardment of single source node
- **STRESS_DEST:** Sustained bombardment of single destination node
- **FAIRNESS:** Verifies round-robin fairness across bridge (±10% tolerance)
- **PERFORMANCE:** Measures cycle count under realistic traffic patterns

### Running Tests

```bash
# Compile
make full

# Run all tests
./simv +BASIC +ACROSS +BROADCAST +STRESS_SRC +STRESS_DEST +FAIRNESS +PERFORMANCE

# Run with waveform viewer
./simv -gui +PERFORMANCE +VERBOSE=2

# Extended runtime
./simv +PERFORMANCE +vcs+finish+100000
```

### Verbosity Levels

- `+VERBOSE=1` - Basic test results
- `+VERBOSE=2` - Packet-level tracing
- `+VERBOSE=3` - Detailed cycle-by-cycle debug

## Performance Characteristics

- **Latency:** 4 cycles (serialization) + routing delays + 4 cycles (deserialization)
- **Throughput:** 1 byte/cycle per port under full load
- **Max concurrent flows:** Up to 12 non-conflicting flows
- **Fairness:** Guaranteed round-robin scheduling within ±10%

## Synthesis Notes

The design follows FSM-D (Finite State Machine with Datapath) methodology and is fully synthesizable:

- No latches or combinational loops
- Synchronous resets on all registers
- Parameterized for easy FPGA/ASIC adaptation
- No vendor-specific primitives

**Resource estimates (approximate):**
- ~2000 LUTs per router
- ~1000 FFs per router
- Block RAM for FIFOs (depends on depth)

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| Clock frequency | Up to 100MHz (typical) |
| Data width | 8 bits (serial), 32 bits (parallel) |
| Packet size | 32 bits (4 bytes) |
| Node FIFO depth | 5 packets |
| Router queue depth | 32 packets/queue |
| Total queues | 12 per router |
| Number of routers | 2 |
| Number of nodes | 6 |
| Arbitration | Round-robin |
| Flow control | Credit-based backpressure |

## License

This implementation is provided for educational and research purposes.

## Author

Josh - Carnegie Mellon University

---

*Note: This implementation demonstrates advanced digital design concepts including pipelined FSMs, multi-level queuing, and distributed flow control mechanisms commonly found in commercial NoC architectures.*
