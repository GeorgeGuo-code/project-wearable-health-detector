//=============================================================================
// Module     : hr_maxim
// Description: Maxim-style time-domain HR detector.
//              Streaming Maxim filter chain + peak detection on raw IR.
//              No BRAM needed — fully streaming at 100 Hz sample rate.
//
// Filter chain (per Maxim MAX30102 reference firmware):
//   ac[n] = raw[n] - dc[n]          (DC from ppg_filter)
//   ma4[n] = avg(ac[n..n+3])        (4-point MA)
//   diff[n] = ma4[n+1] - ma4[n]    (first difference)
//   ma2[n] = avg(diff[n..n+1])     (2-point MA)
//   hamm[n] = sum(w[k]*ma2[n+k])   (5-point Hamming, w={41,276,512,276,41})
//   output[n] = hamm[n] / 128   (boosted gain for weak PPG, was /1024)
//
// Peak detection on filtered output:
//   - Threshold = mean(|output|)
//   - Find peaks above threshold, min distance 40 samples (0.4s)
//   - HR = 6000 / median_peak_interval
//=============================================================================

`timescale 1ns / 1ps

module hr_maxim (
    input  wire              clk,
    input  wire              rst_n,
    input  wire              in_valid,       // 100 Hz, from ppg_filter filt_valid
    input  wire [17:0]       ir_ac,         // raw IR from MAX30102 driver
    output reg  [7:0]        hr_bpm,
    output reg               hr_valid,
    output reg               hr_locked,
    output reg  signed [17:0] hamm_out,       // Maxim filter output (debug)
    output wire signed [17:0] threshold_out,   // peak threshold (debug)
    output wire               dec_valid        // 25Hz pulse for UART trigger
);

// hamm_out and threshold_out are registered inside the main always block
// to avoid cross-block non-blocking-assignment ordering issues.
assign threshold_out = threshold;
// dec_valid is registered so it pulses one cycle AFTER hamm_out updates,
// ensuring the frame builder captures the current value, not the stale one.
reg dec_valid_r;
assign dec_valid = dec_valid_r;

    // ----------------------------------------------------------------
    // Maxim filter chain pipeline registers
    // ----------------------------------------------------------------
    // Stage 1: 4-point MA
    reg signed [17:0] ma4_sr0, ma4_sr1, ma4_sr2, ma4_sr3;
    reg signed [20:0] ma4_sum;   // 4×262k ≈ 1M → needs 21 bits
    reg signed [17:0] ma4_out;

    // Stage 2: first difference
    reg signed [17:0] ma4_d1;
    reg signed [18:0] diff_tmp;

    // Stage 3: 2-point MA on diff
    reg signed [17:0] ma2_d1;
    reg signed [18:0] ma2_tmp;
    reg signed [17:0] ma2_out;

    // Stage 4: 5-point Hamming
    reg signed [17:0] hamm_sr0, hamm_sr1, hamm_sr2, hamm_sr3, hamm_sr4;
    reg signed [31:0] hamm_acc;
    wire signed [17:0] hamm_val     = hamm_acc >>> 7;   // /128, boosted from /1024 for weak PPG
    wire        [17:0] abs_hamm_val = hamm_val[17] ? $unsigned(-hamm_val) : $unsigned(hamm_val);

    // ----------------------------------------------------------------
    // Decimation: 100Hz → 25Hz (4:1).  Maxim filter chain diff stage
    // heavily attenuates slow signals at 100 Hz; at 25 Hz the same
    // PPG produces ~4× larger hamm amplitude.
    // ----------------------------------------------------------------
    reg [1:0] dec_cnt;              // 0→process, 1-3→skip
    // dec_valid is now an output port (assigned via assign)

    // Online peak detector state
    reg [15:0] sample_cnt;          // total samples processed (at 25Hz)
    reg [15:0] period_sum;          // sum of recent peak intervals
    reg [7:0]  peak_count;          // number of peaks found
    reg [15:0] last_peak_pos;       // position of last peak
    reg [15:0] dist_since_peak;     // samples since last peak

    reg signed [17:0] pk_max_val;   // max value in current peak
    reg [15:0] pk_max_pos;          // position of max in current peak
    reg               in_peak;       // currently rising/in peak
    reg signed [17:0] pk_prev;       // previous filtered value

    // Threshold tracking (running average of |hamm_out|)
    reg [31:0] thresh_acc;
    reg signed [17:0] threshold;

    // Period buffer for median
    reg [15:0] periods_0, periods_1, periods_2;
    reg [1:0]  period_wp;
    reg [15:0] med_a, med_b, med_c, med_tmp;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ma4_sr0 <= 0; ma4_sr1 <= 0; ma4_sr2 <= 0; ma4_sr3 <= 0;
            ma4_out <= 0; ma4_d1 <= 0;
            ma2_d1  <= 0; ma2_out <= 0;
            hamm_sr0 <= 0; hamm_sr1 <= 0; hamm_sr2 <= 0;
            hamm_sr3 <= 0; hamm_sr4 <= 0;
            hamm_out <= 0;
            sample_cnt <= 0; period_sum <= 0; peak_count <= 0;
            last_peak_pos <= 0; dist_since_peak <= 0;
            pk_max_val <= 0; pk_max_pos <= 0; in_peak <= 0; pk_prev <= 0;
            dec_cnt    <= 0;
            thresh_acc <= 0; threshold <= 0;
            periods_0 <= 0; periods_1 <= 0; periods_2 <= 0;
            period_wp <= 0;
            hr_bpm <= 0; hr_valid <= 0; hr_locked <= 0;
            dec_valid_r <= 0;
        end else begin
            // dec_valid_r updates every cycle (not gated by in_valid)
            dec_valid_r <= in_valid && (dec_cnt == 2'd0);
            if (in_valid) begin
            dec_cnt <= dec_cnt + 2'd1;
            if (dec_cnt != 2'd0) begin
                // Skip: wait for next decimated cycle
            end else begin
            hr_valid <= 0;
            sample_cnt <= sample_cnt + 1;

            // ============================================================
            // Maxim filter chain (combinational pipeline)
            // ============================================================

            // Stage 1: 4-point MA
            ma4_sr3 <= ma4_sr2;
            ma4_sr2 <= ma4_sr1;
            ma4_sr1 <= ma4_sr0;
            ma4_sr0 <= ir_ac;
            ma4_sum = ma4_sr0 + ma4_sr1 + ma4_sr2 + ma4_sr3;
            ma4_out <= ma4_sum >>> 2;

            // Stage 2: first difference
            diff_tmp = ma4_out - ma4_d1;
            ma4_d1 <= ma4_out;

            // Stage 3: 2-point MA
            ma2_tmp = diff_tmp + ma2_d1;
            ma2_out <= ma2_tmp >>> 1;
            ma2_d1 <= diff_tmp[17:0];

            // Stage 4: 5-point Hamming (negated)
            hamm_sr4 <= hamm_sr3;
            hamm_sr3 <= hamm_sr2;
            hamm_sr2 <= hamm_sr1;
            hamm_sr1 <= hamm_sr0;
            hamm_sr0 <= ma2_tmp >>> 1;

            hamm_acc = 0;
            hamm_acc = hamm_acc + hamm_sr0 * 41;
            hamm_acc = hamm_acc + hamm_sr1 * 276;
            hamm_acc = hamm_acc + hamm_sr2 * 512;
            hamm_acc = hamm_acc + hamm_sr3 * 276;
            hamm_acc = hamm_acc + hamm_sr4 * 41;
            hamm_out <= hamm_val;   // registered inside main always block

            // ============================================================
            // Running threshold: EMA of |hamm_val| with alpha = 1/64.
            // thresh_acc tracks EMA * 64; threshold = thresh_acc / 64.
            // Skip first 20 decimated samples to avoid filter start-up
            // transient from polluting the threshold estimate.
            // ============================================================
            if (sample_cnt > 20) begin
                // Leaky integrator: thresh_acc <= thresh_acc*(1 - 1/64) + |hamm_val|
                thresh_acc <= thresh_acc - (thresh_acc >>> 6) + {14'd0, abs_hamm_val};
            end

            // threshold = thresh_acc / 64  (~EMA of |hamm_val|)
            threshold <= thresh_acc[23:6];

            // ============================================================
            // Online peak detection
            // ============================================================
            pk_prev <= hamm_val;
            dist_since_peak <= dist_since_peak + 1;

            if (sample_cnt <= 20) begin
                // Warmup: skip peak detection while filter chain fills
                in_peak <= 0;
            end else if (!in_peak) begin
                // Looking for rising edge above threshold
                if (hamm_val > threshold && hamm_val > pk_prev) begin
                    in_peak <= 1;
                    pk_max_val <= hamm_val;
                    pk_max_pos <= sample_cnt;
                end
            end else begin
                // Inside a peak: track maximum
                if (hamm_val > pk_max_val) begin
                    pk_max_val <= hamm_val;
                    pk_max_pos <= sample_cnt;
                end
                // Peak ends when value drops below threshold
                if (hamm_val < threshold) begin
                    in_peak <= 0;

                    // Record peak if min distance satisfied
                    if (dist_since_peak > 10 && peak_count < 15) begin
                        if (peak_count > 0) begin
                            // Store interval from previous peak
                            case (period_wp)
                                2'd0: periods_0 <= pk_max_pos - last_peak_pos;
                                2'd1: periods_1 <= pk_max_pos - last_peak_pos;
                                2'd2: periods_2 <= pk_max_pos - last_peak_pos;
                            endcase
                            period_wp <= period_wp + 1;
                        end
                        last_peak_pos <= pk_max_pos;
                        peak_count <= peak_count + 1;
                        dist_since_peak <= 0;
                    end
                end
            end

            // ============================================================
            // HR calculation from median of last 3 intervals
            // ============================================================
            if (peak_count >= 4 && period_wp == 2'd3) begin
                // Median of 3 periods
                med_a = periods_0; med_b = periods_1; med_c = periods_2;
                if (med_a > med_b) begin med_tmp = med_a; med_a = med_b; med_b = med_tmp; end
                if (med_b > med_c) begin med_tmp = med_b; med_b = med_c; med_c = med_tmp; end
                if (med_a > med_b) begin med_tmp = med_a; med_a = med_b; med_b = med_tmp; end

                if (med_b > 0 && med_b < 250) begin
                    hr_bpm   <= 1500 / med_b;   // 60s * 25Hz = 1500
                    hr_locked <= 1;
                    hr_valid  <= 1;
                end
                period_wp <= 0;  // restart collection
            end

            // Timeout: if no peak for > 75 samples (3s at 25Hz), reset.
            // Also reset threshold accumulator so it can re-converge from
            // the current signal level rather than staying stuck high.
            if (dist_since_peak > 75) begin
                peak_count  <= 0;
                period_wp   <= 0;
                hr_locked   <= 0;
                thresh_acc  <= 0;
                threshold   <= 0;
            end
            end // dec_valid processing
        end
        end // else begin (outer: dec_valid_r update)
    end

endmodule
