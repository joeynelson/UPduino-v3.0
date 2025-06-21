// ===========================================================================
// 8‑bit  HSV  ➜ 8‑bit RGB converter (3‑stage pipeline, 1‑pixel/clk)
// ----------------------------------------------------------------------------
// Author:  <your name>, 2025
// License: MIT / CC‑BY‑SA
// ============================================================================

module hsv2rgb_8u
(
    input  wire         clk,          // system clock
    input  wire         rst,          // synchronous reset (active‑high)

    // Input interface --------------------------------------------------------
    input  wire          in_valid,     // 1‑clk strobe that hsv_in is valid
    input  wire [23:0]  hsv_in,       // {H[23:16], S[15:8], V[7:0]}
                                        // H, S, V are unsigned 0‑255

    // Output interface -------------------------------------------------------
    output reg         out_valid,    // 1‑clk strobe that rgb_out is valid
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
    reg        st0_valid;
    reg [2:0]  st0_sector;   // 0‑5
    reg [7:0]  st0_frac;     // 0‑255
    reg [7:0]  st0_S, st0_V;
    reg [10:0] h6;

    always @(posedge clk) begin
        if (rst) begin
            st0_valid <= 1'b0;
        end
        else begin
            st0_valid <= in_valid;

            if (in_valid) begin
                st0_S      <= hsv_in[15:8];
                st0_V      <= hsv_in[7:0];

                // multiply by 6: {8'h0, H} * 6 =  +<<1 and +<<2
                // Keep top 11 bits → [sector(2:0), frac(7:0)]
                h6          = {3'b000, hsv_in[23:16]} * 6;   // 8*6 = 11 bits
                st0_sector  <= h6[10:8];
                st0_frac    <= h6[7:0];
            end
        end
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
    reg        st1_valid;
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
        if (rst) begin
            st1_valid <= 1'b0;
        end
        else begin
            st1_valid  <= st0_valid;
            st1_sector <= st0_sector;
            st1_V      <= st0_V;

            if (st0_valid) begin
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
        end
    end

    // ------------------------------------------------------------------------
    // Stage 2 : multiplex p,q,t,V according to sector to form final RGB
    // ------------------------------------------------------------------------
    reg        st2_valid;
    reg [7:0]  st2_R, st2_G, st2_B;

    always @(posedge clk) begin
        if (rst) begin
            st2_valid <= 1'b0;
        end
        else begin
            st2_valid <= st1_valid;

            if (st1_valid) begin
                case (st1_sector)
                    3'd0 : begin
                            st2_R <= st1_V;  st2_G <= st1_t;  st2_B <= st1_p;
                          end
                    3'd1 : begin
                            st2_R <= st1_q;  st2_G <= st1_V;  st2_B <= st1_p;
                          end
                    3'd2 : begin
                            st2_R <= st1_p;  st2_G <= st1_V;  st2_B <= st1_t;
                          end
                    3'd3 : begin
                            st2_R <= st1_p;  st2_G <= st1_q;  st2_B <= st1_V;
                          end
                    3'd4 : begin
                            st2_R <= st1_t;  st2_G <= st1_p;  st2_B <= st1_V;
                          end
                    default : begin // sector 5
                            st2_R <= st1_V;  st2_G <= st1_p;  st2_B <= st1_q;
                          end
                endcase
            end
        end
    end

    // ------------------------------------------------------------------------
    // Output register (optional 4th cycle).  If you do not need an extra
    // register stage for timing closure, tie out_valid/rgb_out directly to
    // st2_* signals.
    // ------------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            out_valid <= 1'b0;
        end
        else begin
            out_valid <= st2_valid;
            red_out <= st2_R;
            green_out <= st2_G;
            blue_out <= st2_B;
        end
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
  reg  [27:0] frequency_counter_i;
  reg  [7:0] pwm_count;
  wire  [7:0] r,g,b;
  reg        ron,gon,bon;

//----------------------------------------------------------------------------
//                                                                          --
//                       Internal Oscillator                                --
//                                                                          --
//----------------------------------------------------------------------------
  SB_HFOSC u_SB_HFOSC (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));


//----------------------------------------------------------------------------
//                                                                          --
//                       Counter                                            --
//                                                                          --
//----------------------------------------------------------------------------
  always @(posedge int_osc) begin
    frequency_counter_i <= frequency_counter_i + 1;
    pwm_count <= frequency_counter_i[18:11];
    ron <= r >= pwm_count;
    gon <= g >= pwm_count;
    bon <= b >= pwm_count;
  end

  hsv2rgb_8u  hsv2rgb (
    .clk(int_osc),
    .rst(0),
    .in_valid(1),
    .hsv_in(freqency_counter_i[27:20]),
    .out_valid(),
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
