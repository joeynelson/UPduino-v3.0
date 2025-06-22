// ===========================================================================
// 8‑bit  HSV  ➜ 8‑bit RGB converter (3‑stage pipeline, 1‑pixel/clk)
// ----------------------------------------------------------------------------
// Author:  Joey Nelson, 2025
// License: MIT / CC‑BY‑SA
// ============================================================================

module hsv2rgb_8u
(
    input  wire         clk,          // system clock

    // Input interface --------------------------------------------------------
    input  wire          in_valid,     // 1‑clk strobe that hsv_in is valid
    input  wire [23:0]  hsv_in,       // {H[23:16], S[15:8], V[7:0]}
                                        // H, S, V are unsigned 0‑255

    // Output interface -------------------------------------------------------
    output reg [7:0]   red_out,
    output reg [7:0]   green_out,
    output reg [7:0]   blue_out
);

    // ------------------------------------------------------------------------
    // Stage 0 : unpack & scale hue to sextant number and intra‑sector fraction
    // ------------------------------------------------------------------------
    // Hue sector width = 256 / 6 = 42 + 2/3  (fixed‑point 8.8 format)
    //  -> Multiply H by 6 to obtain 0‑1535; top three MSBs give sector 0‑5,
    //     lower 8 bits give fractional part (0‑255) for interpolation.
    // ------------------------------------------------------------------------
    reg [2:0]  st0_sector;   // 0‑5
    reg [7:0]  st0_frac;     // 0‑255
    reg [7:0]  st0_S, st0_V;
    reg [10:0] h6;

    always @(posedge clk) begin

      st0_S      <= hsv_in[15:8];
      st0_V      <= hsv_in[7:0];

      // multiply by 6: {8'h0, H} * 6 =  +<<1 and +<<2
      // Keep top 11 bits → [sector(2:0), frac(7:0)]
      h6          = {3'b000, hsv_in[23:16]} * 6;   // 8*6 = 11 bits
      st0_sector  <= h6[10:8];
      st0_frac    <= h6[7:0];
   end

    // ------------------------------------------------------------------------
    // Stage 1 : compute the three auxiliary values p,q,t (8‑bit each)
    //           p = V * (1‑S)
    //           q = V * (1 ‑ S * f)
    //           t = V * (1 ‑ S * (1‑f))
    //
    // Fixed‑point arithmetic:
    //   S, V, f are 0‑255 → treat as 8‑bit unsigned (range 0‑1 in Q0.8)
    //   All products use 16‑bit intermediate with rounding to 8 bits.
    // ------------------------------------------------------------------------
    reg [2:0]  st1_sector;
    reg [7:0]  st1_p, st1_q, st1_t, st1_V;
    reg [7:0]  invS, Sf, Sf1;
  
    function automatic [7:0] mul8x8_q0p8 (input [7:0] a, input [7:0] b);
        // Returns rounded (a*b)/255
        reg [15:0] prod;
        begin
            prod = a * b + 8'd128;      // add 0.5*255 for rounding
            mul8x8_q0p8 = prod[15:8];   // divide by 255 (>>8)
        end
    endfunction

    always @(posedge clk) begin
      st1_sector <= st0_sector;
      st1_V      <= st0_V;

      // 1‑S
      invS = 8'd255 - st0_S;
      st1_p = mul8x8_q0p8(st0_V, invS);

      // S * f   (f in Q0.8)
      Sf   = mul8x8_q0p8(st0_S, st0_frac);
      // S*(1‑f)
      Sf1  = mul8x8_q0p8(st0_S, 8'd255 - st0_frac);

      st1_q = mul8x8_q0p8(st0_V, 8'd255 - Sf);
      st1_t = mul8x8_q0p8(st0_V, 8'd255 - Sf1);
    end

    // ------------------------------------------------------------------------
    // Stage 2 : multiplex p,q,t,V according to sector to form final RGB
    // ------------------------------------------------------------------------
    reg        st2_valid;
    reg [7:0]  st2_R, st2_G, st2_B;

    always @(posedge clk) begin
                case (st1_sector)
                    3'd0 : begin
                            st2_R <= st1_V;  
                            st2_G <= st1_t;  
                            st2_B <= st1_p;
                          end
                    3'd1 : begin
                            st2_R <= st1_q;  
                            st2_G <= st1_V;  
                            st2_B <= st1_p;
                          end
                    3'd2 : begin
                            st2_R <= st1_p;  
                            st2_G <= st1_V;  
                            st2_B <= st1_t;
                          end
                    3'd3 : begin
                            st2_R <= st1_p;  
                            st2_G <= st1_q;  
                            st2_B <= st1_V;
                          end
                    3'd4 : begin
                            st2_R <= st1_t;  
                            st2_G <= st1_p;  
                            st2_B <= st1_V;
                          end
                    default : begin // sector 5
                            st2_R <= st1_V;  
                            st2_G <= st1_p;  
                            st2_B <= st1_q;
                          end
                endcase
    end

    // ------------------------------------------------------------------------
    // Output register (optional 4th cycle).  If you do not need an extra
    // register stage for timing closure, tie out_valid/rgb_out directly to
    // st2_* signals.
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
            red_out <= st2_R;
            green_out <= st2_G;
            blue_out <= st2_B;
    end

 endmodule

//----------------------------------------------------------------------------
//                                                                          --
//                         Module Declaration                               --
//                                                                          --
//----------------------------------------------------------------------------
module rgb_blink (
  // outputs
  output wire led_red  , // Red
  output wire led_blue , // Blue
  output wire led_green  // Green
);

  wire        int_osc            ;
  reg  [14:0] frequency_counter_i;
  reg  [7:0] pwm_count;
  wire  [7:0] r,g,b;
  reg        ron,gon,bon;
  reg [7:0] hue,val;
  reg [3:0] hcount, vcount;
  reg v_dir;


//----------------------------------------------------------------------------
//                                                                          --
//                       Internal Oscillator                                --
//                                                                          --
//----------------------------------------------------------------------------
  SB_HFOSC u_SB_HFOSC (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));
  defparam u_SB_HFOSC.CLKHF_DIV = "0b10";

//----------------------------------------------------------------------------
//                                                                          --
//                       Counter                                            --
//                                                                          --
//----------------------------------------------------------------------------
  always @(posedge int_osc) begin
    frequency_counter_i <= frequency_counter_i + 1;
    pwm_count <= frequency_counter_i[14:7];
    if(frequency_counter_i == 0) begin
      hcount <= hcount == 14 ? 0 : hcount + 1;
      vcount <= vcount == 3 ? 0 : vcount + 1;
      if(hcount == 0) begin
        hue <= hue + 1;
      end
      if(vcount == 0) begin
        if(v_dir) begin
          if(val == 255) begin
            v_dir = ~v_dir;
          end
          else begin
            val <= val + 1;
          end
        end
        else begin
          if(val == 0) begin
            v_dir = ~v_dir;
          end else begin
            val <= val - 1;
          end
        end
      end
    end

    ron <= r >= pwm_count;
    gon <= g >= pwm_count;
    bon <= b >= pwm_count;
  end

  hsv2rgb_8u  hsv2rgb (
    .clk(int_osc),
    .hsv_in({hue, 8'hff, val}),
    .red_out(r),
    .green_out(g),
    .blue_out(b)
    );
//----------------------------------------------------------------------------
//                                                                          --
//                       Instantiate RGB primitive                          --
//                                                                          --
//----------------------------------------------------------------------------
  SB_RGBA_DRV RGB_DRIVER (
    .RGBLEDEN(1'b1                                            ),
    .RGB0PWM (ron),
    .RGB1PWM (gon),
    .RGB2PWM (bon),
    .CURREN  (1'b1                                            ),
    .RGB0    (led_green                                       ), //Actual Hardware connection
    .RGB1    (led_blue                                        ),
    .RGB2    (led_red                                         )
  );
  defparam RGB_DRIVER.RGB0_CURRENT = "0b000001";
  defparam RGB_DRIVER.RGB1_CURRENT = "0b000001";
  defparam RGB_DRIVER.RGB2_CURRENT = "0b000001";

endmodule
