`default_nettype none
`include "Router.svh"
`include "RouterPkg.pkg"

//////
////// Network on Chip (NoC) 18-341
////// Node module
//////

// ================================================================
// Node
// - Buffers TB->Node packets in a FIFO
// - Streams Node->Router as 4 bytes: {src,dest,data[23:16],[15:8],[7:0]}
// - Reassembles Router->Node stream (4 bytes) back into a pkt_t
// - Reflects assembled inbound pkt_t back to TB
// Handshakes:
//   TB -> Node:    pkt_in/pkt_in_avail, cQ_full
//   Node -> Router: free_outbound, put_outbound, payload_outbound
//   Router -> Node: free_inbound, put_inbound, payload_inbound
// ================================================================


module Node #(parameter NODEID = 0) (
  input logic clock, reset_n,

  //Interface to testbench: the blue arrows
  input  pkt_t pkt_in,        // Data packet from the TB
  input  logic pkt_in_avail,  // The packet from TB is available
  output logic cQ_full,       // The queue is full

  output pkt_t pkt_out,       // Outbound packet from node to TB
  output logic pkt_out_avail, // The outbound packet is available

  //Interface with the router: black arrows
  input  logic       free_outbound,    // Router is free
  output logic       put_outbound,     // Node is transferring to router
  output logic [7:0] payload_outbound, // Data sent from node to router

  output logic       free_inbound,     // Node is free
  input  logic       put_inbound,      // Router is transferring to node
  input  logic [7:0] payload_inbound); // Data sent from router to node

    // FIFO logic variables
    logic empty, full;
    logic re, we;
    logic [31:0] data_out, data_in;

    FIFO #(32) queue(
        .clock(clock),
        .reset_n(reset_n),
        .data_in(data_in),
        .we(we),
        .re(re),
        .data_out(data_out),
        .full(full),
        .empty(empty)
    );

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    //                      NODE -> ROUTER
    // ----------------------------------------------------------
    // ----------------------------------------------------------

    // Streams one packet as 4 bytes over 4 cycles:
    //   cycle0: {src,dest}
    //   cycle1: data[23:16]
    //   cycle2: data[15:8]
    //   cycle3: data[7:0]  (also pops FIFO here)

    enum logic [2:0] {
        IDLE_N2R  = 3'b000,
        DATA1_N2R = 3'b010,
        DATA2_N2R = 3'b011,
        DATA3_N2R = 3'b100
    } state_N2R, nextstate_N2R;

    logic data_outbound;        // strobe to router
    logic data_outbound_next;   // byte currently sent

    logic [7:0] chunk_out;      // current outbound payload
    logic [7:0] next_chunk_out; // next outbound payload

    pkt_t package_to_send;      // stored package from Node
    pkt_t next_package_to_send; // stored package from Node (next cycle)

    assign put_outbound = data_outbound;

    always_comb begin
        next_package_to_send    = package_to_send;
        next_chunk_out          = chunk_out;
        nextstate_N2R           = state_N2R;
        data_outbound_next      = 1'b0;
        re                      = 1'b0;

        case(state_N2R)
            IDLE_N2R: begin
                if(!empty && free_outbound) begin
                    next_chunk_out = {data_out[31:28], data_out[27:24]};
                    data_outbound_next = 1'b1;

                    // if doesn't work for router get rid of next_ and add ADDR_N2R state
                    next_package_to_send.src  = data_out[31:28];
                    next_package_to_send.dest = data_out[27:24];
                    next_package_to_send.data = data_out[23:0];

                    nextstate_N2R = DATA1_N2R;
                end else begin
                    next_chunk_out = '0;
                    nextstate_N2R = IDLE_N2R;
                end
            end
            DATA1_N2R: begin
                next_chunk_out = package_to_send.data[23:16];
                nextstate_N2R = DATA2_N2R;
            end
            DATA2_N2R: begin
                next_chunk_out = package_to_send.data[15:8];
                nextstate_N2R = DATA3_N2R;
            end
            DATA3_N2R: begin
                re = 1'b1;
                next_chunk_out = package_to_send.data[7:0];
                nextstate_N2R = IDLE_N2R;
            end
        endcase
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if(!reset_n) begin
            state_N2R       <= IDLE_N2R;
            chunk_out       <= '0;
            data_outbound   <= 1'b0;
            package_to_send <= '0;
        end else begin
            state_N2R       <= nextstate_N2R;
            chunk_out       <= next_chunk_out;
            data_outbound   <= data_outbound_next;
            package_to_send <= next_package_to_send;
        end
    end

    assign payload_outbound = chunk_out;

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    //                      ROUTER -> NODE
    // ----------------------------------------------------------
    // ----------------------------------------------------------

    // Assembles 4 incoming bytes into one pkt_t:
    //   cycle0: {src,dest}  (when free_inbound && put_inbound)
    //   cycle1: data[23:16]
    //   cycle2: data[15:8]
    //   cycle3: data[7:0] -> pkt_out/pkt_out_avail strobed next clock


    enum logic [2:0] {
        IDS_R2N   = 3'b000,
        DATA1_R2N = 3'b001,
        DATA2_R2N = 3'b010,
        DATA3_R2N = 3'b011
    } state_R2N, nextstate_R2N;

    logic [31:0] held_package;       // package that we are building
    logic [31:0] held_package_next;  // package that we are building (next cycle)

    logic receiving;                 // one cycle pulse indicating that data is sending
    logic receiving_next;           // one cycle pulse indicating that data is sending (next cycle)
    
    pkt_t reg_val;                  // final constructed package
    pkt_t reg_val_next;             // final constructed package (next cycle)
    
    logic reg_loaded;               // final package has been constructed
    
    assign free_inbound = !receiving;

    always_comb begin
        held_package_next   = held_package;
        receiving_next      = receiving;
        nextstate_R2N       = state_R2N;
        reg_val_next        = reg_val;
        case(state_R2N)
            IDS_R2N: begin
                if(free_inbound && put_inbound) begin
                    held_package_next = {payload_inbound, 24'b0};
                    receiving_next = 1'b1;
                    nextstate_R2N = DATA1_R2N;
                end else begin
                    held_package_next = '0;
                    receiving_next = 1'b0;
                    nextstate_R2N = IDS_R2N;
                end
            end
            DATA1_R2N: begin
                held_package_next[23:16] = payload_inbound;
                nextstate_R2N = DATA2_R2N;
            end
            DATA2_R2N: begin
                held_package_next[15:8] = payload_inbound;
                nextstate_R2N = DATA3_R2N;
            end
            DATA3_R2N: begin
                held_package_next[7:0] = payload_inbound;
                reg_val_next = pkt_t'(held_package_next);
                held_package_next = '0;
                receiving_next = 1'b0;
                nextstate_R2N = IDS_R2N;
            end
        endcase
    end
    logic done_receiving;
    assign done_receiving = (state_R2N == DATA3_R2N);

    always_ff @(posedge clock, negedge reset_n) begin
        if(!reset_n) begin
            state_R2N       <= IDS_R2N;
            held_package    <= '0;
            receiving       <= 1'b0;
            reg_val         <= '0;
            reg_loaded      <= 1'b0;
        end else begin
            state_R2N       <= nextstate_R2N;
            held_package    <= held_package_next;
            receiving       <= receiving_next;
            reg_val         <= reg_val_next;
            reg_loaded      <= 1'b0;
            
            reg_loaded      <= done_receiving;
        end
    end

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    //                      TB -> NODE
    // ----------------------------------------------------------
    // ----------------------------------------------------------

    assign cQ_full = full;
    assign data_in = {pkt_in.src, pkt_in.dest, pkt_in.data};
    assign we = !full && pkt_in_avail;

    // ----------------------------------------------------------
    // ----------------------------------------------------------
    //                      NODE -> TB
    // ----------------------------------------------------------
    // ----------------------------------------------------------
 
    always_ff @(posedge clock, negedge reset_n) begin
        if(!reset_n) begin
            pkt_out             <= '0;
            pkt_out_avail       <= 1'b0;
        end else begin
            pkt_out_avail       <= 1'b0;
            if(reg_loaded) begin
                pkt_out         <= reg_val;
                pkt_out_avail   <= 1'b1;
            end
        end
    end


endmodule : Node

/*
 *  Create a FIFO (First In First Out) buffer with depth 4 using the given
 *  interface and constraints
 *    - The buffer is initally empty
 *    - Reads are combinational, so data_out is valid unless empty is asserted
 *    - Removal from the queue is processed on the clock edge.
 *    - Writes are processed on the clock edge
 *    - If a write is pending while the buffer is full, do nothing
 *    - If a read is pending while the buffer is empty, do nothing
 */
module FIFO #(parameter WIDTH=32) (
    input logic              clock, reset_n,
    input logic [WIDTH-1:0]  data_in,
    input logic              we, re,
    output logic [WIDTH-1:0] data_out,
    output logic             full, empty);

    logic [WIDTH-1:0] Q[5];
    logic [2:0] put_ptr, get_ptr; 
    logic [2:0] count, count_next;

    assign empty = (count == 0);
    assign full = (count == 3'd5);
    
    always_comb begin
        if(!empty)  data_out = Q[get_ptr];
        else        data_out = '0;
    end

    logic do_write, do_read;
    assign do_write = we && !full;
    assign do_read = re && !empty;

    function automatic logic [2:0] inc(input logic [2:0] p);
        return (p == 3'd4) ? 3'd0 : (p + 3'd1);
    endfunction

    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) begin
            count <= 0;
            get_ptr <= 0;
            put_ptr <= 0;
            for(int i = 0; i < 5; i++) begin
                Q[i] = '0;
            end
        end else begin
            
            if (do_write) begin
                Q[put_ptr] <= data_in;
                put_ptr    <= inc(put_ptr);
            end

            if (do_read) begin
                get_ptr <= inc(get_ptr);
            end

            case ({do_write, do_read})
                2'b10: count <= count + 3'd1;
                2'b01: count <= count - 3'd1;
                default: count <= count ;
            endcase

        end
    end
endmodule : FIFO