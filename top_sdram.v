// =============================================================================
// top_sdram.v  –  Example top-level integration of sdram_random_controller
//
// Target board : Intel/Altera Cyclone (e.g. DE10-Lite, DE0-Nano …)
//
// This module demonstrates how to wire the sdram_random_controller to actual
// FPGA I/O pins.  A trivial test pattern is included:
//   • After reset, the FSM first writes the pattern 0xABCD to address 0,
//     then reads it back and compares.
//   • LED[0] is lit when the read-back matches (test PASS).
//   • LED[1] is lit when the read-back does not match (test FAIL).
//   • LED[2] blinks while the controller is busy.
// =============================================================================

module top_sdram (
    // ── Board signals ───────────────────────────────────────────────────────
    input  wire        CLOCK_100,   // 100 MHz oscillator
    input  wire        RESET_BTN,   // Active-low push-button reset

    // ── User I/O ────────────────────────────────────────────────────────────
    output wire [ 2:0] LED,         // Status LEDs

    // ── SDRAM pins (connect to your board's DRAM_ prefix signals) ───────────
    output wire [12:0] DRAM_ADDR,
    output wire [ 1:0] DRAM_BA,
    output wire        DRAM_CS_N,
    output wire        DRAM_RAS_N,
    output wire        DRAM_CAS_N,
    output wire        DRAM_WE_N,
    inout  wire [15:0] DRAM_DQ,
    output wire [ 1:0] DRAM_DQM,
    output wire        DRAM_CKE,
    output wire        DRAM_CLK    // Route CLOCK_100 directly to DRAM clock pin
);

// ---------------------------------------------------------------------------
// Drive SDRAM clock from the same 100 MHz source
// ---------------------------------------------------------------------------
assign DRAM_CLK = CLOCK_100;

// ---------------------------------------------------------------------------
// User-interface registers
// ---------------------------------------------------------------------------
reg  [24:0] addr;       // Word address for the next access
reg  [15:0] din;        // Write data
reg         we;         // 1 = write, 0 = read
reg         req;        // Request pulse

wire [15:0] dout;       // Read data returned by the controller
wire        ack;        // Operation-complete pulse
wire        busy;       // Controller busy flag

// ---------------------------------------------------------------------------
// Instantiate the random-access SDRAM controller
// ---------------------------------------------------------------------------
sdram_random_controller u_ctrl (
    .clk     (CLOCK_100),
    .reset_n (RESET_BTN),

    // User interface
    .addr    (addr),
    .din     (din),
    .we      (we),
    .req     (req),
    .dout    (dout),
    .ack     (ack),
    .busy    (busy),

    // SDRAM pins
    .SA      (DRAM_ADDR),
    .BA      (DRAM_BA),
    .CS_N    (DRAM_CS_N),
    .RAS_N   (DRAM_RAS_N),
    .CAS_N   (DRAM_CAS_N),
    .WE_N    (DRAM_WE_N),
    .DQ      (DRAM_DQ),
    .DQM     (DRAM_DQM),
    .CKE     (DRAM_CKE)
);

// ---------------------------------------------------------------------------
// Simple test FSM
//   State 0 : wait for controller to finish initialisation (busy goes low)
//   State 1 : issue a WRITE of 0xABCD to address 0
//   State 2 : wait for write ack
//   State 3 : issue a READ from address 0
//   State 4 : wait for read ack, capture data
//   State 5 : hold result forever
// ---------------------------------------------------------------------------
localparam [2:0]
    TS_WAIT_INIT  = 3'd0,
    TS_WRITE      = 3'd1,
    TS_WAIT_WACK  = 3'd2,
    TS_READ       = 3'd3,
    TS_WAIT_RACK  = 3'd4,
    TS_DONE       = 3'd5;

reg [2:0] test_state;
reg       pass;
reg       fail;

always @(posedge CLOCK_100 or negedge RESET_BTN) begin
    if (!RESET_BTN) begin
        test_state <= TS_WAIT_INIT;
        addr       <= 25'd0;
        din        <= 16'hABCD;
        we         <= 1'b0;
        req        <= 1'b0;
        pass       <= 1'b0;
        fail       <= 1'b0;
    end else begin
        req <= 1'b0;   // Default: no request

        case (test_state)

            // Wait until initialisation finishes (busy de-asserts for the first time)
            TS_WAIT_INIT: begin
                if (!busy)
                    test_state <= TS_WRITE;
            end

            // Issue a write to address 0
            TS_WRITE: begin
                addr       <= 25'd0;
                din        <= 16'hABCD;
                we         <= 1'b1;
                req        <= 1'b1;
                test_state <= TS_WAIT_WACK;
            end

            // Wait for write acknowledgement
            TS_WAIT_WACK: begin
                if (ack)
                    test_state <= TS_READ;
            end

            // Issue a read from address 0
            TS_READ: begin
                if (!busy) begin
                    addr       <= 25'd0;
                    we         <= 1'b0;
                    req        <= 1'b1;
                    test_state <= TS_WAIT_RACK;
                end
            end

            // Wait for read acknowledgement, compare data
            TS_WAIT_RACK: begin
                if (ack) begin
                    pass       <= (dout == 16'hABCD);
                    fail       <= (dout != 16'hABCD);
                    test_state <= TS_DONE;
                end
            end

            // Hold result – nothing more to do
            TS_DONE: begin
                // Idle forever; controller will self-refresh
            end

            default:
                test_state <= TS_WAIT_INIT;

        endcase
    end
end

// ---------------------------------------------------------------------------
// LED output
//   LED[0] – PASS (data matched)
//   LED[1] – FAIL (data mismatch)
//   LED[2] – controller busy
// ---------------------------------------------------------------------------
assign LED[0] = pass;
assign LED[1] = fail;
assign LED[2] = busy;

endmodule
