`timescale 1ns/1ps

module top_tangnano9k(
  input  wire        clk_in,   // 27MHz
  input  wire        btn1_n,    // reset (active-low)
  input  wire        btn2_n,    // short=step, long=hb toggle (active-low)
  output wire [5:0]  led_n
);

  // ---------------- sync reset/button ----------------
  reg [2:0] rst_sync;
  reg [2:0] b2_sync;

  always @(posedge clk_in) begin
    rst_sync <= {rst_sync[1:0], btn1_n};
    b2_sync  <= {b2_sync[1:0],  btn2_n};
  end

  wire rst_n = rst_sync[2];

  // ============================================================
  // BTN2 DEBOUNCE
  // ============================================================
  localparam integer CLK_HZ        = 27_000_000;
  localparam integer DEBOUNCE_MS   = 20;
  localparam integer DB_LIMIT      = (CLK_HZ/1000)*DEBOUNCE_MS;

  reg        b2_db;
  reg [31:0] db_cnt;

  always @(posedge clk_in) begin
    if (!rst_n) begin
      b2_db  <= 1'b1;
      db_cnt <= 32'd0;
    end else begin
      if (b2_sync[2] == b2_db) begin
        db_cnt <= 32'd0;
      end else begin
        if (db_cnt >= DB_LIMIT) begin
          b2_db  <= b2_sync[2];
          db_cnt <= 32'd0;
        end else begin
          db_cnt <= db_cnt + 32'd1;
        end
      end
    end
  end

  // ============================================================
  // LONG PRESS LOGIC
  // ============================================================
  localparam integer LONG_MS     = 500;
  localparam integer LONG_LIMIT  = (CLK_HZ/1000)*LONG_MS;

  reg [31:0] hold_cnt;

  localparam S_IDLE     = 2'd0;
  localparam S_PRESSING = 2'd1;
  localparam S_LONG     = 2'd2;

  reg [1:0] st;

  reg step_pulse_r;
  reg hb_toggle_pulse_r;

  always @(posedge clk_in) begin
    if (!rst_n) begin
      st                <= S_IDLE;
      hold_cnt          <= 32'd0;
      step_pulse_r      <= 1'b0;
      hb_toggle_pulse_r <= 1'b0;
    end else begin
      step_pulse_r      <= 1'b0;
      hb_toggle_pulse_r <= 1'b0;

      case (st)
        S_IDLE: begin
          hold_cnt <= 32'd0;
          if (b2_db == 1'b0) begin
            st       <= S_PRESSING;
            hold_cnt <= 32'd0;
          end
        end

        S_PRESSING: begin
          if (b2_db == 1'b0) begin
            if (hold_cnt >= LONG_LIMIT) begin
              hb_toggle_pulse_r <= 1'b1;
              st                <= S_LONG;
            end else begin
              hold_cnt <= hold_cnt + 32'd1;
            end
          end else begin
            step_pulse_r <= 1'b1;
            st           <= S_IDLE;
          end
        end

        S_LONG: begin
          if (b2_db == 1'b1) begin
            st <= S_IDLE;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

  // ---------------- HB toggle ----------------
  reg hb;
  always @(posedge clk_in) begin
    if (!rst_n) begin
      hb <= 1'b0;
    end else if (hb_toggle_pulse_r) begin
      hb <= ~hb;
    end
  end

  // ---------------- blink generators ----------------
  reg [23:0] blink_div;
  always @(posedge clk_in) begin
    if (!rst_n) blink_div <= 24'd0;
    else        blink_div <= blink_div + 24'd1;
  end

  wire blink_slow = blink_div[23];
  wire blink_mid  = blink_div[22];
  wire blink_fast = blink_div[21];

  // ---------------- CPU ----------------
  wire        dbg_we;
  wire [3:0]  dbg_waddr;
  wire [15:0] dbg_wdata;

  wire        halted;
  wire        dbg_stall;
  wire        dbg_flush;
  wire        dbg_bubble;
  wire [3:0]  dbg_op;

  wire [15:0] dbg_ifid_instr;
  wire [15:0] dbg_pc;

  // Step, HALT sonrası core zaten donuyor ama ekstra güvenlik:
  wire cpu_en = step_pulse_r & ~halted;

  cpu_core_pipelined U_CPU(
    .clk(clk_in),
    .rst_n(rst_n),
    .en(cpu_en),

    .dbg_we(dbg_we),
    .dbg_waddr(dbg_waddr),
    .dbg_wdata(dbg_wdata),

    .halted(halted),
    .dbg_bubble(dbg_bubble),
    .dbg_stall(dbg_stall),
    .dbg_flush(dbg_flush),
    .dbg_opcode_ifid(dbg_op),

    .dbg_ifid_instr(dbg_ifid_instr),
    .dbg_pc(dbg_pc)
  );

  // ---------------- latch last WB data ----------------
  reg [15:0] last_wdata;
  always @(posedge clk_in) begin
    if (!rst_n) begin
      last_wdata <= 16'h0000;
    end else if (cpu_en && dbg_we) begin
      last_wdata <= dbg_wdata;
    end
  end

  // ---------------- latch hazards (visible blink) ----------------
  reg last_stall, last_flush, last_bubble;
  always @(posedge clk_in) begin
    if (!rst_n) begin
      last_stall  <= 1'b0;
      last_flush  <= 1'b0;
      last_bubble <= 1'b0;
    end else if (cpu_en) begin
      last_stall  <= dbg_stall;
      last_flush  <= dbg_flush;
      last_bubble <= dbg_bubble;
    end
  end

  // ---------------- STATUS LED (priority) ----------------
  wire status_led =
      halted      ? 1'b1 :
      last_flush  ? blink_fast :
      last_stall  ? blink_slow :
      last_bubble ? blink_mid  :
                    1'b0;

  // ---------------- LED mapping (2 pages via HB) ----------------
  reg [5:0] led_on;

  always @(*) begin
    if (hb == 1'b0) begin
      // Page0: WB data
      led_on[0] = 1'b0;
      led_on[1] = last_wdata[0];
      led_on[2] = last_wdata[1];
      led_on[3] = last_wdata[2];
      led_on[4] = last_wdata[3];
      led_on[5] = last_wdata[4];
    end else begin
      // Page1: hazards + halt + status
      led_on[0] = 1'b1;
      led_on[1] = last_stall  ? blink_slow : 1'b0;
      led_on[2] = last_flush  ? blink_fast : 1'b0;
      led_on[3] = last_bubble ? blink_mid  : 1'b0;
      led_on[4] = halted;       // <<< FIX: direct sticky halt
      led_on[5] = status_led;
    end
  end

  // active-low LEDs
  assign led_n = ~led_on;

endmodule
