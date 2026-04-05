// =============================================================================
// sdram_params.v  –  Parameters and constants for the SDRAM random-access
//                    controller.
//
// Target device : generic 32 MB SDRAM (e.g. ISSI IS42S16320F, W9825G6KH …)
//                 16-bit wide data bus, 4 banks, 8192 rows, 1024 columns
// Clock         : 100 MHz
// =============================================================================

// ---------------------------------------------------------------------------
// Bus widths
// ---------------------------------------------------------------------------
`define SDRAM_ASIZE       25   // Total word-address width  (2^25 = 32 M words)
`define SDRAM_DSIZE       16   // Data bus width
`define SDRAM_SASIZE      13   // SDRAM row / column address pin count

// ---------------------------------------------------------------------------
// Address-field positions within the 25-bit word address
//
//  [24:23]  bank    (2 bits  → 4 banks)
//  [22:10]  row     (13 bits → 8192 rows)
//  [ 9: 0]  column  (10 bits → 1024 columns)
// ---------------------------------------------------------------------------
`define SDRAM_BANK_LSB    23
`define SDRAM_BANK_WIDTH   2

`define SDRAM_ROW_LSB     10
`define SDRAM_ROW_WIDTH   13

`define SDRAM_COL_LSB      0
`define SDRAM_COL_WIDTH   10

// ---------------------------------------------------------------------------
// Timing parameters  (clock cycles at 100 MHz, 10 ns/cycle)
// ---------------------------------------------------------------------------
`define SDRAM_INIT_CYCLES 24000  // Power-up stable delay ≥ 200 µs  (24 000 × 10 ns = 240 µs)
`define SDRAM_REF_PERIOD    780  // Refresh interval = 7.8 µs  (JEDEC: 64 ms / 8192 rows = 7.8125 µs)
`define SDRAM_NUM_INIT_REF    8  // Auto-refresh cycles issued during initialisation

`define SDRAM_T_RCD           3  // ACTIVATE → READ / WRITE  (tRCD, min 20 ns → 3 cyc)
`define SDRAM_T_CL            3  // CAS latency                (CL  = 3 clocks)
`define SDRAM_T_RP            3  // Precharge period           (tRP, min 20 ns → 3 cyc)
`define SDRAM_T_RFC           9  // Auto-refresh cycle time    (tRFC, min 63 ns → 7 cyc; use 9)
`define SDRAM_T_WR            2  // Write-recovery before auto-precharge (tWR, min 14 ns → 2 cyc)
`define SDRAM_T_MRD           2  // Mode-register-set delay    (tMRD, min 2 cycles)

// ---------------------------------------------------------------------------
// Mode-register value loaded during initialisation
//
//  A[2:0] = 000  burst length  = 1 (single-word)
//  A[3]   =   0  burst type    = sequential
//  A[6:4] = 011  CAS latency   = 3
//  A[8:7] =  00  operating mode = standard
//  A[9]   =   0  write-burst mode = same as read burst (BL = 1)
//  A[12:10] = 00  reserved
// ---------------------------------------------------------------------------
`define SDRAM_MODE_REG   13'b000_0_00_011_0_000   // = 13'h030
