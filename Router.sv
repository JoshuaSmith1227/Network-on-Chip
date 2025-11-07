`default_nettype none
`include "Router.svh"
`include "RouterPkg.pkg"

//////
////// Network on Chip (NoC) 18-341
////// Router module - FIXED: Dequeue only when sender starts transmission
//////
module Router #(parameter ROUTERID = 0) (
    input logic             clock, reset_n,

    input logic [3:0]       free_outbound,
    input logic [3:0]       put_inbound,
    input logic [3:0][7:0]  payload_inbound,

    output logic [3:0]      free_inbound,
    output logic [3:0]      put_outbound,
    output logic [3:0][7:0] payload_outbound);

    // --------------------------------------------------
    //           CAPACITY TRACKING (32 packet limit)
    // --------------------------------------------------
    logic [5:0] curr_capacity;
    logic router_full;
    
    // 24 to give us wiggle room for 4 available packets + 4 packets being built
    assign router_full = (curr_capacity >= 6'd24);

    // --------------------------------------------------
    //              PACKET BUILDER 
    // --------------------------------------------------

    pkt_t [3:0] curr_packets;
    logic [3:0] packet_ready;
    logic [3:0] builder_free;
    
    pkt_t [3:0] held_packets;
    logic [3:0] held_valid;

    assign free_inbound = builder_free & {4{!router_full}}; 
    // port[i] can accept data if builder[i] is free and the router isn't full

    genvar i;
    generate
        for(i = 0; i < 4; i++) begin : gen_builder
            packetBuilder p0(
                .clock(clock),
                .reset_n(reset_n),
                .put_inbound(put_inbound[i]),
                .payload_inbound(payload_inbound[i]),
                .router_full(router_full),
                .backpressure(held_valid[i]),
                .free_inbound(builder_free[i]),
                .reg_val(curr_packets[i]),
                .reg_loaded(packet_ready[i])
            );
        end
    endgenerate

    // ----------------------------------------------------------------
    //              HELPER FUNCTIONS (gets proper queue indicies)
    // ----------------------------------------------------------------

    function automatic logic [1:0] nodeToPort(input logic [3:0] dest_node);
        if(ROUTERID == 0) begin
            if     (dest_node == 4'd0) nodeToPort = 2'd0;
            else if(dest_node == 4'd1) nodeToPort = 2'd2;
            else if(dest_node == 4'd2) nodeToPort = 2'd3;
            else                       nodeToPort = 2'd1;
        end else begin
            if     (dest_node == 4'd3) nodeToPort = 2'd0;
            else if(dest_node == 4'd4) nodeToPort = 2'd1;
            else if(dest_node == 4'd5) nodeToPort = 2'd2;
            else                       nodeToPort = 2'd3;
        end
    endfunction

    function automatic logic [1:0] indexMap(input logic [1:0] src_port, input logic [3:0] dest_node);
        logic [1:0] dest_port;
        dest_port = nodeToPort(dest_node);
        if (src_port < dest_port)
            indexMap = src_port;
        else
            indexMap = src_port - 2'd1;
    endfunction

    // -----------------------------------------------------------------------------------------------------------
    //                                      MAKE OUTBOUND QUEUES
    //                      --------------------------------------------------
    // queue_data[dest_node][src_port] = queue_data[node_to_port(dest_node, router_id)][indexMap(src_port, dest_node)]
    // --------------------------------------------------------------------------------------------------------------

    // make queue variables
    logic               q_empty [4][3];
    logic               q_full  [4][3];
    logic               q_we    [4][3];
    logic               q_re    [4][3];
    logic [31:0]        q_din   [4][3];
    logic [31:0]        q_dout  [4][3];

    genvar d, s;
    generate
        for(d = 0; d < 4; d++) begin : gen_dest_port
            for(s = 0; s < 3; s++) begin : gen_subq
                FIFO_router #(.WIDTH(32), .DEPTH(32)) q(
                    .clock(clock),
                    .reset_n(reset_n),
                    .data_in(q_din[d][s]),
                    .we(q_we[d][s]),
                    .re(q_re[d][s]),
                    .data_out(q_dout[d][s]),
                    .full(q_full[d][s]),
                    .empty(q_empty[d][s])
                );
            end
        end
    endgenerate

    // --------------------------------------------------
    //     ENQUEUE LOGIC WITH DELAYED DEQUEUE COUNT
    // --------------------------------------------------

    logic [3:0] packets_dequeued_comb;
    logic [3:0] packets_dequeued_prev;

    logic [1:0] dest_port [4];
    logic [1:0] subq [4];
    logic [3:0] dest_node [4];
    pkt_t pkt_to_enq [4];
    logic have_packet [4];

    always_comb begin
        packets_dequeued_comb = 4'd0;
        for(int p = 0; p < 4; p++) begin
            for(int s = 0; s < 3; s++) begin
                packets_dequeued_comb = packets_dequeued_comb + 4'(q_re[p][s]);
            end
        end
    end

    logic [3:0] enq_count;

    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) begin
            curr_capacity <= 6'd0;
            packets_dequeued_prev <= 4'd0;
            
            // clear all old writes
            for (int i=0; i<4; i++) begin
                for (int j=0; j<3; j++)
                    q_we[i][j] <= 1'b0;
                held_valid[i] <= 1'b0;
                held_packets[i] <= pkt_t'(32'd0);
            end
        end else begin
            packets_dequeued_prev <= packets_dequeued_comb;
            
            // clear all old writes
            for (int i=0; i<4; i++)
                for (int j=0; j<3; j++)
                    q_we[i][j] <= 1'b0;

            enq_count = 4'd0;
            
            for(int k = 0; k < 4; k++) begin
                if (held_valid[k]) begin
                    pkt_to_enq[k] = held_packets[k];
                    have_packet[k] = 1'b1;
                end else if (packet_ready[k]) begin
                    pkt_to_enq[k] = curr_packets[k];
                    have_packet[k] = 1'b1;
                end else begin
                    have_packet[k] = 1'b0;
                end
                
                if (have_packet[k]) begin
                    dest_node[k] = pkt_to_enq[k].dest;
                    dest_port[k] = nodeToPort(dest_node[k]);
                    subq[k] = indexMap(2'(k), dest_node[k]);
                end
            end
            
            for(int k = 0; k < 4; k++) begin
                if (have_packet[k]) begin
                    if (!q_full[dest_port[k]][subq[k]]) begin
                        q_din[dest_port[k]][subq[k]] <= {pkt_to_enq[k].src, 
                                                        pkt_to_enq[k].dest, 
                                                        pkt_to_enq[k].data};
                        q_we[dest_port[k]][subq[k]] <= 1'b1;
                        enq_count = enq_count + 4'd1;
                        
                        if (held_valid[k] && packet_ready[k]) begin
                            held_packets[k] <= curr_packets[k];
                            held_valid[k] <= 1'b1;
                        end else begin
                            held_valid[k] <= 1'b0;
                        end
                    end else begin
                        held_packets[k] <= pkt_to_enq[k];
                        held_valid[k] <= 1'b1;
                    end
                end
            end
            

            // inc curr_capacity and handle over/underflow
            if (enq_count >= packets_dequeued_prev)
                curr_capacity <= curr_capacity + (enq_count - packets_dequeued_prev);
            else if (curr_capacity >= (packets_dequeued_prev - enq_count))
                curr_capacity <= curr_capacity - (packets_dequeued_prev - enq_count);
            else
                curr_capacity <= 6'd0;
        end
    end

    // --------------------------------------------------
    //         ROUND-ROBIN SELECTOR
    // --------------------------------------------------
    logic [31:0] selected_data [4];
    logic [3:0] port_ready_to_send;

    genvar port_index;
    generate
        for(port_index = 0; port_index < 4; port_index++) begin : gen_selector
            logic [1:0] rr_ptr;
            logic [1:0] chosen_subq;
            logic any_packet_available;
            logic should_dequeue;
            
            // Check if any sub-queue has data
            assign any_packet_available = !q_empty[port_index][0] || 
                                         !q_empty[port_index][1] || 
                                         !q_empty[port_index][2];
            
            assign should_dequeue = port_ready_to_send[port_index] && 
                                   free_outbound[port_index] && 
                                   any_packet_available;
            
            always_ff @(posedge clock, negedge reset_n) begin
                if(!reset_n) begin
                    rr_ptr <= 2'd0;
                    selected_data[port_index] <= 32'd0;
                    q_re[port_index][0] <= 1'b0;
                    q_re[port_index][1] <= 1'b0;
                    q_re[port_index][2] <= 1'b0;
                end else begin
                    // Clear read enables
                    q_re[port_index][0] <= 1'b0;
                    q_re[port_index][1] <= 1'b0;
                    q_re[port_index][2] <= 1'b0;
                    
                    if(should_dequeue) begin
                        // Round-robin: try current ptr, then next two
                        if (!q_empty[port_index][rr_ptr]) begin
                            chosen_subq = rr_ptr;
                        end else if (!q_empty[port_index][(rr_ptr + 2'd1) % 3]) begin
                            chosen_subq = (rr_ptr + 2'd1) % 3;
                        end else begin
                            chosen_subq = (rr_ptr + 2'd2) % 3;
                        end
                        
                        // Latch data and issue read
                        if (!q_empty[port_index][chosen_subq]) begin
                            selected_data[port_index] <= q_dout[port_index][chosen_subq];
                            q_re[port_index][chosen_subq] <= 1'b1;
                            rr_ptr <= (chosen_subq + 2'd1) % 3;
                        end
                    end
                end
            end
        end
    endgenerate

    // --------------------------------------------------
    //           PACKET SENDER 
    // --------------------------------------------------
    genvar j;
    generate
        for(j = 0; j < 4; j++) begin : gen_sender
            logic any_packet_available;
            assign any_packet_available = !q_empty[j][0] || !q_empty[j][1] || !q_empty[j][2];
            
            packetSender sender(
                .clock(clock),
                .reset_n(reset_n),
                .free_outbound(free_outbound[j]),
                .empty(!any_packet_available),
                .data_in(selected_data[j]),
                .payload_outbound(payload_outbound[j]),
                .done_sending(port_ready_to_send[j]),
                .put_outbound(put_outbound[j])
            );
        end
    endgenerate

endmodule : Router

// --------------------------------------------------
//              PACKET BUILDER MODULE
// --------------------------------------------------
module packetBuilder(
    input logic clock, reset_n,
    input logic put_inbound,
    input logic [7:0] payload_inbound,
    input logic router_full,
    input logic backpressure,
    output logic free_inbound,
    output pkt_t reg_val,
    output logic reg_loaded
);

    enum logic [1:0] {
        IDLE  = 2'b00,
        DATA1 = 2'b01,
        DATA2 = 2'b10,
        DATA3 = 2'b11
    } state, nextstate;

    logic [31:0] held_package;
    pkt_t completed_packet;
    logic packet_valid;

    // Block at IDLE if router full OR if external backpressure
    assign free_inbound = (state == IDLE) && !router_full && !backpressure;
    assign reg_val = completed_packet;
    assign reg_loaded = packet_valid;


    always_comb begin
        nextstate = state;
        case(state)
            IDLE: begin
                if(put_inbound && free_inbound)
                    nextstate = DATA1;
            end
            DATA1: nextstate = DATA2;
            DATA2: nextstate = DATA3;
            DATA3: nextstate = IDLE;
        endcase
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if(!reset_n) begin
            state <= IDLE;
            held_package <= 32'd0;
            completed_packet <= pkt_t'(32'd0);
            packet_valid <= 1'b0;
        end else begin
            state <= nextstate;
            packet_valid <= 1'b0;
            
            case(state)
                IDLE: begin
                    if(put_inbound && free_inbound)
                        held_package[31:24] <= payload_inbound;
                end
                DATA1: held_package[23:16] <= payload_inbound;
                DATA2: held_package[15:8] <= payload_inbound;
                DATA3: begin
                    held_package[7:0] <= payload_inbound;
                    completed_packet <= pkt_t'({held_package[31:8], payload_inbound});
                    packet_valid <= 1'b1;
                end
            endcase
        end
    end
endmodule : packetBuilder

// --------------------------------------------------
//              PACKET SENDER MODULE
// --------------------------------------------------
module packetSender(
    input logic clock, reset_n,
    input logic free_outbound,
    input logic empty,
    input logic [31:0] data_in,
    output logic [7:0] payload_outbound,
    output logic put_outbound,
    output logic done_sending
);

    enum logic [2:0] {
        IDLE  = 3'b000,
        LOAD  = 3'b001,
        DATA1 = 3'b010,
        DATA2 = 3'b011,
        DATA3 = 3'b100
    } state, nextstate;

    pkt_t held_packet;
    logic start_send;

    assign done_sending = (state == IDLE);
    assign start_send = !empty && free_outbound && (state == IDLE);

    always_comb begin
        nextstate = state;
        case(state)
            IDLE: begin
                if(start_send)
                    nextstate = LOAD;
            end
            LOAD: nextstate = DATA1;
            DATA1: nextstate = DATA2;
            DATA2: nextstate = DATA3;
            DATA3: nextstate = IDLE;
        endcase
    end

    always_ff @(posedge clock, negedge reset_n) begin
        if(!reset_n) begin
            state <= IDLE;
            held_packet <= pkt_t'(32'd0);
            payload_outbound <= 8'd0;
            put_outbound <= 1'b0;
        end else begin
            state <= nextstate;
            put_outbound <= 1'b0;
            
            case(state)
                IDLE: begin
                end
                LOAD: begin
                    held_packet <= pkt_t'(data_in); 
                    payload_outbound <= {data_in[31:28], data_in[27:24]};
                    put_outbound <= 1'b1;
                end
                DATA1: payload_outbound <= held_packet.data[23:16];
                DATA2: payload_outbound <= held_packet.data[15:8];
                DATA3: payload_outbound <= held_packet.data[7:0];
            endcase
        end
    end

endmodule : packetSender

// --------------------------------------------------
//              ROUTER FIFO MODULE
// --------------------------------------------------
module FIFO_router #(parameter WIDTH=32, parameter DEPTH=4) (
    input logic              clock, reset_n,
    input logic [WIDTH-1:0]  data_in,
    input logic              we, re,
    output logic [WIDTH-1:0] data_out,
    output logic             full, empty
);

    localparam PTR_WIDTH = $clog2(DEPTH);
    
    logic [WIDTH-1:0] Q[DEPTH];
    logic [PTR_WIDTH-1:0] put_ptr, get_ptr;
    logic [PTR_WIDTH:0] count;

    assign empty = (count == {(PTR_WIDTH+1){1'b0}});
    assign full = (count == (PTR_WIDTH+1)'(DEPTH));
    assign data_out = empty ? {WIDTH{1'b0}} : Q[get_ptr];

    logic do_write, do_read;
    assign do_write = we && !full;
    assign do_read = re && !empty;

    always_ff @(posedge clock, negedge reset_n) begin
        if (!reset_n) begin
            count <= {(PTR_WIDTH+1){1'b0}};
            get_ptr <= {PTR_WIDTH{1'b0}};
            put_ptr <= {PTR_WIDTH{1'b0}};
        end else begin
            if (do_write) begin
                Q[put_ptr] <= data_in;
                put_ptr <= put_ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};
            end

            if (do_read)
                get_ptr <= get_ptr + {{(PTR_WIDTH-1){1'b0}}, 1'b1};

            case ({do_write, do_read})
                2'b10: count <= count + {{PTR_WIDTH{1'b0}}, 1'b1};
                2'b01: count <= count - {{PTR_WIDTH{1'b0}}, 1'b1};
                default: count <= count;
            endcase
        end
    end
endmodule : FIFO_router