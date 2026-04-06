// =============================================================================
// sdram_random_controller.v  –  Simple SDRAM random-access controller
//
// Supports:
//   • 100 MHz clock
//   • 16-bit data bus
//   • 25-bit word address (32 MB)
//   • Single-word reads and writes (no streaming, no FIFO)
//   • Auto-precharge after every access (no open-row tracking needed)
//   • Periodic auto-refresh managed internally
//
// User interface:
//   req  – assert for 1 clock to start an operation
//   we   – 1 = write, 0 = read  (sampled with req)
//   addr – 25-bit word address  (sampled with req)
//   din  – write data           (sampled with req)
//   ack  – pulses for 1 clock when the operation is complete
//          (for reads, dout is also valid on the ack cycle)
//   busy – high while the controller is not in IDLE
//          (do not assert req while busy)
// =============================================================================

`include "sdram_params.v"

module sdram_random_controller (
    input  wire        clk,       // 100 MHz system clock
    input  wire        reset_n,   // Active-low synchronous reset

    // ── User interface ─────────────────────────────────────────────────────
    input  wire [24:0] addr,      // Word address for the access
    input  wire [15:0] din,       // Data to write
    input  wire        we,        // 1 = write, 0 = read
    input  wire        req,       // Request pulse (1 clock wide, only when !busy)
    output reg  [15:0] dout,      // Read-data output (valid on ack)
    output reg         ack,       // Operation-complete pulse (1 clock)
    output wire        busy,      // High while controller cannot accept a new req

    // ── SDRAM pins ─────────────────────────────────────────────────────────
    output reg  [12:0] SA,        // Multiplexed row / column address
    output reg  [ 1:0] BA,        // Bank address
    output reg         CS_N,      // Chip select       (active-low)
    output reg         RAS_N,     // Row address strobe (active-low)
    output reg         CAS_N,     // Column address strobe (active-low)
    output reg         WE_N,      // Write enable      (active-low)
    inout  wire [15:0] DQ,        // Bidirectional data bus
    output reg  [ 1:0] DQM,       // Data-byte mask    (0 = byte active)
    output wire        CKE        // Clock enable (tied high)
);

// ---------------------------------------------------------------------------
// State encoding
// ---------------------------------------------------------------------------
localparam [4:0]
    ST_INIT_WAIT  = 5'd0,   // Wait ≥ 200 µs after power-up
    ST_INIT_PRE   = 5'd1,   // Issue PRECHARGE ALL
    ST_INIT_PRE_W = 5'd2,   // Wait tRP after PRECHARGE
    ST_INIT_REF   = 5'd3,   // Issue AUTO REFRESH  (repeated NUM_INIT_REF times)
    ST_INIT_REF_W = 5'd4,   // Wait tRFC after AUTO REFRESH
    ST_INIT_MRS   = 5'd5,   // Issue LOAD MODE REGISTER
    ST_INIT_MRS_W = 5'd6,   // Wait tMRD after mode-register load
    ST_IDLE       = 5'd7,   // Idle – wait for req or refresh
    ST_ACTIVATE   = 5'd8,   // Issue ACTIVATE  (open the target row)
    ST_RCD_WAIT   = 5'd9,   // Wait remainder of tRCD
    ST_WRITE      = 5'd10,  // Issue WRITE + auto-precharge, drive DQ
    ST_WRITE_WAIT = 5'd11,  // Wait tWR + tRP for auto-precharge to finish
    ST_READ       = 5'd12,  // Issue READ + auto-precharge
    ST_CL_WAIT    = 5'd13,  // Wait remainder of CAS latency
    ST_READ_DATA  = 5'd14,  // Capture DQ, assert ack
    ST_REFRESH    = 5'd15,  // Issue AUTO REFRESH (periodic)
    ST_REF_WAIT   = 5'd16;  // Wait tRFC after periodic AUTO REFRESH

// ---------------------------------------------------------------------------
// Registers
// ---------------------------------------------------------------------------
reg  [ 4:0] state;          // Current FSM state
reg  [15:0] timer;          // General-purpose countdown timer
reg  [15:0] ref_timer;      // Time until next required refresh
reg         ref_req;        // Refresh needed flag
reg  [ 3:0] init_ref_cnt;   // Init-phase refresh counter

// Registered request inputs (captured on req)
reg  [24:0] reg_addr;
reg  [15:0] reg_din;
reg         reg_we;

// DQ output control
reg         dq_oe;          // 1 = drive DQ, 0 = tristate
reg  [15:0] dq_out;         // Value to place on DQ when driving

// ---------------------------------------------------------------------------
// Static assignments
// ---------------------------------------------------------------------------
assign CKE  = 1'b1;                         // Clock always enabled
assign DQ   = dq_oe ? dq_out : 16'hzzzz;   // Tristate when not writing
assign busy = (state != ST_IDLE);           // Busy whenever not in IDLE

// ---------------------------------------------------------------------------
// Convenience macros for address field extraction
// ---------------------------------------------------------------------------
wire [ 1:0] w_bank = reg_addr[`SDRAM_BANK_LSB + `SDRAM_BANK_WIDTH - 1 : `SDRAM_BANK_LSB];
wire [12:0] w_row  = reg_addr[`SDRAM_ROW_LSB  + `SDRAM_ROW_WIDTH  - 1 : `SDRAM_ROW_LSB ];
wire [ 9:0] w_col  = reg_addr[`SDRAM_COL_LSB  + `SDRAM_COL_WIDTH  - 1 : `SDRAM_COL_LSB ];

// Column address with SA[10]=1 for auto-precharge
wire [12:0] w_col_ap = {2'b00, 1'b1, w_col};   // SA[10]=1 selects auto-precharge

// ---------------------------------------------------------------------------
// Main FSM
// All SDRAM outputs and internal state are updated here.
// ---------------------------------------------------------------------------
always @(posedge clk or negedge reset_n) begin
    if (!reset_n) begin
        // ── Reset state ────────────────────────────────────────────────────
        state        <= ST_INIT_WAIT;
        timer        <= `SDRAM_INIT_CYCLES - 1;
        ref_timer    <= `SDRAM_REF_PERIOD  - 1;
        ref_req      <= 1'b0;
        init_ref_cnt <= 4'd0;

        // Outputs to SDRAM: deselect (CS_N=1, NOP)
        SA    <= 13'd0;
        BA    <=  2'd0;
        CS_N  <= 1'b1;
        RAS_N <= 1'b1;
        CAS_N <= 1'b1;
        WE_N  <= 1'b1;
        DQM   <= 2'b11;
        dq_oe <= 1'b0;
        dq_out <= 16'h0000;

        // User interface outputs
        ack   <= 1'b0;
        dout  <= 16'h0000;

        // Captured request
        reg_addr <= 25'd0;
        reg_din  <= 16'h0000;
        reg_we   <=  1'b0;

    end else begin
        // ── Defaults (reasserted every cycle unless overridden below) ──────
        //  NOP: CS=0, RAS=1, CAS=1, WE=1
        CS_N  <= 1'b0;
        RAS_N <= 1'b1;
        CAS_N <= 1'b1;
        WE_N  <= 1'b1;
        DQM   <= 2'b11;
        dq_oe <= 1'b0;
        ack   <= 1'b0;

        // ── Refresh timer (counts independently of state) ──────────────────
        if (state == ST_REFRESH) begin
            // Just entered / in refresh – reset the refresh timer and flag
            ref_timer <= `SDRAM_REF_PERIOD - 1;
            ref_req   <= 1'b0;
        end else if (ref_timer == 0) begin
            ref_timer <= `SDRAM_REF_PERIOD - 1;
            ref_req   <= 1'b1;
        end else begin
            ref_timer <= ref_timer - 1'b1;
        end

        // ── State machine ──────────────────────────────────────────────────
        case (state)

            // ------------------------------------------------------------------
            // INITIALISATION SEQUENCE
            // ------------------------------------------------------------------

            // 1. Hold SDRAM deselected for ≥ 200 µs
            ST_INIT_WAIT: begin
                CS_N <= 1'b1;   // Deselect during power-up wait
                if (timer == 0)
                    state <= ST_INIT_PRE;
                else
                    timer <= timer - 1'b1;
            end

            // 2. PRECHARGE ALL BANKS
            ST_INIT_PRE: begin
                // PRECHARGE ALL: CS=0, RAS=0, CAS=1, WE=0, SA[10]=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b0;
                CAS_N <= 1'b1;
                WE_N  <= 1'b0;
                SA    <= 13'h400;   // SA[10]=1 → precharge ALL banks
                BA    <=  2'b00;
                timer <= `SDRAM_T_RP - 2;
                state <= ST_INIT_PRE_W;
            end

            // 3. Wait tRP after PRECHARGE
            ST_INIT_PRE_W: begin
                if (timer == 0) begin
                    init_ref_cnt <= 4'd0;
                    state        <= ST_INIT_REF;
                end else
                    timer <= timer - 1'b1;
            end

            // 4. Issue AUTO REFRESH (repeated SDRAM_NUM_INIT_REF times)
            ST_INIT_REF: begin
                // AUTO REFRESH: CS=0, RAS=0, CAS=0, WE=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b0;
                CAS_N <= 1'b0;
                WE_N  <= 1'b1;
                timer <= `SDRAM_T_RFC - 2;
                state <= ST_INIT_REF_W;
            end

            // 5. Wait tRFC, loop until all init refreshes are done
            ST_INIT_REF_W: begin
                if (timer == 0) begin
                    if (init_ref_cnt == `SDRAM_NUM_INIT_REF - 1)
                        state <= ST_INIT_MRS;
                    else begin
                        init_ref_cnt <= init_ref_cnt + 1'b1;
                        state        <= ST_INIT_REF;
                    end
                end else
                    timer <= timer - 1'b1;
            end

            // 6. LOAD MODE REGISTER
            ST_INIT_MRS: begin
                // MODE REGISTER SET: CS=0, RAS=0, CAS=0, WE=0, BA=0
                CS_N  <= 1'b0;
                RAS_N <= 1'b0;
                CAS_N <= 1'b0;
                WE_N  <= 1'b0;
                BA    <=  2'b00;
                SA    <= `SDRAM_MODE_REG;
                timer <= `SDRAM_T_MRD - 2;
                state <= ST_INIT_MRS_W;
            end

            // 7. Wait tMRD, then initialisation complete
            ST_INIT_MRS_W: begin
                if (timer == 0) begin
                    // Reset refresh timer so first refresh is correctly spaced
                    ref_timer <= `SDRAM_REF_PERIOD - 1;
                    ref_req   <= 1'b0;
                    state     <= ST_IDLE;
                end else
                    timer <= timer - 1'b1;
            end

            // ------------------------------------------------------------------
            // NORMAL OPERATION
            // ------------------------------------------------------------------

            // IDLE: accept a new request or service a pending refresh
            ST_IDLE: begin
                if (ref_req) begin
                    state <= ST_REFRESH;
                end else if (req) begin
                    // Capture request inputs
                    reg_addr <= addr;
                    reg_din  <= din;
                    reg_we   <= we;
                    state    <= ST_ACTIVATE;
                end
            end

            // ACTIVATE the target row
            ST_ACTIVATE: begin
                // ACTIVATE: CS=0, RAS=0, CAS=1, WE=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b0;
                CAS_N <= 1'b1;
                WE_N  <= 1'b1;
                BA    <= w_bank;
                SA    <= w_row;
                // Wait tRCD-1 more cycles (this cycle already counts as 1)
                timer <= `SDRAM_T_RCD - 2;
                state <= ST_RCD_WAIT;
            end

            // Wait until tRCD has elapsed since ACTIVATE
            ST_RCD_WAIT: begin
                if (timer == 0)
                    state <= reg_we ? ST_WRITE : ST_READ;
                else
                    timer <= timer - 1'b1;
            end

            // -- WRITE path --------------------------------------------------

            // Issue WRITE with auto-precharge; drive write data on DQ
            ST_WRITE: begin
                // WRITE + auto-precharge: CS=0, RAS=1, CAS=0, WE=0, SA[10]=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b1;
                CAS_N <= 1'b0;
                WE_N  <= 1'b0;
                BA    <= w_bank;
                SA    <= w_col_ap;   // column address, SA[10]=1 for auto-precharge
                DQM   <= 2'b00;      // all bytes active
                dq_oe <= 1'b1;
                dq_out <= reg_din;
                ack   <= 1'b1;       // Acknowledge write immediately
                // Wait tWR + tRP for auto-precharge to complete
                timer <= (`SDRAM_T_WR + `SDRAM_T_RP) - 2;
                state <= ST_WRITE_WAIT;
            end

            // Hold DQ off while auto-precharge completes
            ST_WRITE_WAIT: begin
                if (timer == 0)
                    state <= ST_IDLE;
                else
                    timer <= timer - 1'b1;
            end

            // -- READ path ---------------------------------------------------

            // Issue READ with auto-precharge
            ST_READ: begin
                // READ + auto-precharge: CS=0, RAS=1, CAS=0, WE=1, SA[10]=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b1;
                CAS_N <= 1'b0;
                WE_N  <= 1'b1;
                BA    <= w_bank;
                SA    <= w_col_ap;   // column address, SA[10]=1 for auto-precharge
                DQM   <= 2'b00;      // all bytes active (enable output)
                // Wait tCL-1 more cycles (this cycle already counts as 1)
                timer <= `SDRAM_T_CL - 2;
                state <= ST_CL_WAIT;
            end

            // Wait for CAS latency to expire
            ST_CL_WAIT: begin
                DQM <= 2'b00;   // Keep DQM=0 so the SDRAM outputs data
                if (timer == 0)
                    state <= ST_READ_DATA;
                else
                    timer <= timer - 1'b1;
            end

            // Capture valid read data and signal the user
            ST_READ_DATA: begin
                dout  <= DQ;
                ack   <= 1'b1;
                state <= ST_IDLE;
            end

            // -- REFRESH path ------------------------------------------------

            // Issue AUTO REFRESH
            ST_REFRESH: begin
                // AUTO REFRESH: CS=0, RAS=0, CAS=0, WE=1
                CS_N  <= 1'b0;
                RAS_N <= 1'b0;
                CAS_N <= 1'b0;
                WE_N  <= 1'b1;
                timer <= `SDRAM_T_RFC - 2;
                state <= ST_REF_WAIT;
            end

            // Wait tRFC after AUTO REFRESH
            ST_REF_WAIT: begin
                if (timer == 0)
                    state <= ST_IDLE;
                else
                    timer <= timer - 1'b1;
            end

            default:
                state <= ST_IDLE;

        endcase
    end
end

endmodule
