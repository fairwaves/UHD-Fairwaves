// Copyright 2012-2013 Fairwaves LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "lms6002d.hpp"

static int verbosity = 0;

/************************************************************************/
/* LMS6002D Control Class implementation                                */
/************************************************************************/

void lms6002d_dev::dump()
{
    for (int i = 0; i < 128; i++) {
        switch (i) {
            case 0x0C:
            case 0x0D:
            case 0x37:
            case 0x38:
            case 0x39:
            case 0x3A:
            case 0x3B:
            case 0x3C:
            case 0x3D:
            case 0x69:
            case 0x6A:
            case 0x6B:
            case 0x6C:
            case 0x6D:
                continue;
        }
        printf("i=%x LMS=%x\n", i, read_reg(i));
    }
}

double lms6002d_dev::txrx_pll_tune(uint8_t reg, double ref_clock, double out_freq)
{
    // Supported frequency ranges and corresponding FREQSEL values.
    static const struct vco_sel { int64_t fmin; int64_t fmax; int8_t value; } freqsel[] = {
        { 0.2325e9,   0.285625e9, 0x27 },
        { 0.285625e9, 0.336875e9, 0x2f },
        { 0.336875e9, 0.405e9,    0x37 },
        { 0.405e9,    0.465e9,    0x3f },
        { 0.465e9,    0.57125e9,  0x26 },
        { 0.57125e9,  0.67375e9,  0x2e },
        { 0.67375e9,  0.81e9,     0x36 },
        { 0.81e9,     0.93e9,     0x3e },
        { 0.93e9,     1.1425e9,   0x25 },
        { 1.1425e9,   1.3475e9,   0x2d },
        { 1.3475e9,   1.62e9,     0x35 },
        { 1.62e9,     1.86e9,     0x3d },
        { 1.86e9,     2.285e9,    0x24 },
        { 2.285e9,    2.695e9,    0x2c },
        { 2.695e9,    3.24e9,     0x34 },
        { 3.24e9,     3.72e9,     0x3c },
    };

    if (verbosity>0) printf("lms6002d_dev::txrx_pll_tune(ref_clock=%f, out_freq=%f)\n", ref_clock, out_freq);

    // Find frequency range and FREQSEL for the given frequency
    int8_t found_freqsel = -1;
    for (unsigned i = 0; i < (int)sizeof(freqsel) / sizeof(freqsel[0]); i++) {
        if (out_freq > freqsel[i].fmin && out_freq <= freqsel[i].fmax) {
            found_freqsel = freqsel[i].value; break;
        }
    }
    if (found_freqsel == -1)
    {
        // Unsupported frequency range
        return -1;
    }

    // Calculate NINT, NFRAC
    int64_t vco_x = 1 << ((found_freqsel & 0x7) - 3);
    int64_t nint = vco_x * out_freq / ref_clock;
    int64_t nfrac = (1 << 23) * (vco_x * out_freq - nint * ref_clock) / ref_clock;
    double actual_freq = (nint + nfrac/double(1<<23)) * (ref_clock/vco_x);

    // DEBUG
    if (verbosity>0) printf("FREQSEL=%d VCO_X=%d NINT=%d  NFRAC=%d ACTUAL_FREQ=%f\n\n", (int)found_freqsel, (int)vco_x, (int)nint, (int)nfrac, actual_freq);

    // Write NINT, NFRAC
    write_reg(reg + 0x0, (nint >> 1) & 0xff);    // NINT[8:1]
    write_reg(reg + 0x1, ((nfrac >> 16) & 0x7f) | ((nint & 0x1) << 7)); //NINT[0] nfrac[22:16]
    write_reg(reg + 0x2, (nfrac >> 8) & 0xff);  // NFRAC[15:8]
    write_reg(reg + 0x3, (nfrac) & 0xff);     // NFRAC[7:0]
    // Write FREQSEL
    lms_write_bits(reg + 0x5, (0x3f << 2), (found_freqsel << 2)); // FREQSEL[5:0]
    // Reset VOVCOREG, OFFDOWN to default
    // -- I think this is not needed here, as it changes settings which
    //    we may want to set beforehand.
//    write_reg(reg + 0x8, 0x40); // VOVCOREG[3:1] OFFDOWN[4:0]
//    write_reg(reg + 0x9, 0x94); // VOVCOREG[0] VCOCAP[5:0]

    // DEBUG
    //reg_dump();

    // Poll VOVCO
    int start_i = -1;
    int stop_i = -1;
    enum State { VCO_HIGH, VCO_NORM, VCO_LOW } state = VCO_HIGH;
    for (int i = 0; i < 64; i++) {
        // Update VCOCAP
        lms_write_bits(reg + 0x9, 0x3f, i);
        //usleep(50);

        int comp = read_reg(reg + 0x0a);
        switch (comp >> 6) {
        case 0x02: //HIGH
            break;
        case 0x01: //LOW
            if (state == VCO_NORM) {
                stop_i = i - 1;
                state = VCO_LOW;
                if (verbosity>1) printf("Low\n");
            }
            break;
        case 0x00: //NORMAL
            if (state == VCO_HIGH) {
                start_i = i;
                state = VCO_NORM;
                if (verbosity>1) printf("Norm\n");
            }
            break;
        default: //ERROR
            printf("ERROR: Incorrect VCOCAP reading while tuning\n");
            return -1;
        }
        if (verbosity>1) printf("VOVCO[%d]=%x\n", i, (comp>>6));
    }
    if (VCO_NORM == state)
        stop_i = 63;

    if (start_i == -1 || stop_i == -1) {
        printf("ERROR: Can't find VCOCAP value while tuning\n");
        return -1;
    }

    // Tune to the middle of the found VCOCAP range
    int avg_i = (start_i + stop_i) / 2;
    if (verbosity>0) printf("START=%d STOP=%d SET=%d\n", start_i, stop_i, avg_i);
    lms_write_bits(reg + 0x09, 0x3f, avg_i);

    // Return actual frequency we've tuned to
    return actual_freq;
}

void lms6002d_dev::init()
{
    if (verbosity>0) printf("lms6002d_dev::init()\n");
    write_reg(0x09, 0x00); // RXOUTSW (disabled), CLK_EN (all disabled)
    write_reg(0x17, 0xE0);
    write_reg(0x27, 0xE3);
    write_reg(0x64, 0x32);
    write_reg(0x70, 0x01);
    write_reg(0x79, 0x37);
    write_reg(0x59, 0x09);
    write_reg(0x47, 0x40);
    // RF Settings
//    write_reg(0x41, 0x15); // VGA1GAIN
//    write_reg(0x45, 0x00); // VGA2GAIN, ENVD

    //reg_dump();
}

int lms6002d_dev::general_dc_calibration_loop(uint8_t dc_addr, uint8_t calibration_reg_base)
{
    uint8_t reg_val;
    int try_cnt_limit = 10;
    uint8_t DC_REGVAL = 0;

    if (verbosity > 0) printf("DC Offset Calibration for addr %d:\n", dc_addr);
    reg_val = read_reg(calibration_reg_base+0x03);
    // DC_ADDR := ADDR
    reg_val = (reg_val & 0xf8) | dc_addr;
    write_reg(calibration_reg_base+0x03, reg_val);
    // DC_START_CLBR := 1
    reg_val = reg_val | (1 << 5);
    write_reg(calibration_reg_base+0x03, reg_val);
    // DC_START_CLBR := 0
    reg_val = reg_val ^ (1 << 5);
    write_reg(calibration_reg_base+0x03, reg_val);

    while (try_cnt_limit--)
    {
        if (verbosity > 1) printf("cnt=%d\n", try_cnt_limit);

        // Wait for 6.4(1.6) us
        //usleep(6.4);

        // Read DC_CLBR_DONE
        reg_val  = read_reg(calibration_reg_base+0x01);
        int DC_CLBR_DONE = (reg_val >> 1) & 0x1;
        if (verbosity > 1) printf(" DC_CLBR_DONE=%d\n", DC_CLBR_DONE);

        // DC_CLBR_DONE == 1?
        if (DC_CLBR_DONE == 1)
            continue;

        // Read DC_LOCK
        reg_val  = read_reg(calibration_reg_base+0x01);
        int DC_LOCK = (reg_val >> 2) & 0x7;
        if (verbosity > 1) printf(" DC_LOCK=%d\n", DC_LOCK);

        // Read DC_REGVAL
        DC_REGVAL = read_reg(calibration_reg_base+0x00);
        if (verbosity > 1) printf("DC_REGVAL = %d\n", DC_REGVAL);

        // DC_LOCK != 0 or 7? We're done.
        if (DC_LOCK != 0 && DC_LOCK != 7) break;
    }

    // Return the value
    return DC_REGVAL;
}

int lms6002d_dev::general_dc_calibration(uint8_t dc_addr, uint8_t calibration_reg_base)
{
    // Set DC_REGVAL to 31
    write_reg(calibration_reg_base+0x00, 31);
    // Run the calibration first time
    int DC_REGVAL = general_dc_calibration_loop(dc_addr, calibration_reg_base);
    // Unchanged DC_REGVAL may mean either calibration failure or that '31' is
    // the best calibration vaue. We're going to re-check that.
    if (31 == DC_REGVAL)
    {
        // Set DC_REGVAL to a value other then 31, e.g. 0
        write_reg(calibration_reg_base+0x00, 0);
        // Retry the calibration
        DC_REGVAL = general_dc_calibration_loop(dc_addr, calibration_reg_base);
        // If DC_REGVAL has been changed, then calibration succeeded.
        if (0 == DC_REGVAL)
        {
            // PANIC: Algorithm does Not Converge!
            // From LimeMicro FAQ: "[This] condition should not happen as this is being
            // checked in our production test."
            printf("Error: DC Offset Calibration does not converge!\n");
            return -1;
        }
    }

    if (verbosity > 0) printf("Successful DC Offset Calibration for register bank 0x%X, DC addr %d. Result: 0x%X\n",
                              calibration_reg_base, dc_addr, DC_REGVAL);
    return DC_REGVAL;
}

bool lms6002d_dev::lpf_tuning_dc_calibration()
{
    bool result = false;
    // Save TopSPI::CLK_EN[5] Register
    // TopSPI::CLK_EN[5] := 1
    uint8_t clk_en_save = read_reg(0x09);
    lms_set_bits(0x09, (1 << 5));

    // Perform DC Calibration Procedure in TopSPI with ADDR := 0 and get Result
    // DCCAL := TopSPI::DC_REGVAL
    int DCCAL = general_dc_calibration(0, 0x0);
    if (DCCAL >= 0)
    {
        // RxLPFSPI::DCO_DACCAL := DCCAL
        lms_write_bits(0x35, 0x3f, DCCAL);
        // TxLPFSPI::DCO_DACCAL := DCCAL
        lms_write_bits(0x55, 0x3f, DCCAL);
        // Success
        result = true;
    }

    // Restore TopSPI::CLK_EN[5] Register
    write_reg(0x09, clk_en_save);

    return result;
}

bool lms6002d_dev::txrx_lpf_dc_calibration(bool is_tx)
{
    bool result;

    // Determine base address for control registers
    int control_reg_base = (is_tx)?0x30:0x50;

    // Save TopSPI::CLK_EN Register
    // TopSPI::CLK_EN := 1
    uint8_t clk_en_save = read_reg(0x09);
    lms_set_bits(0x09, (is_tx)?(1 << 1):(1 << 3));

    // Perform DC Calibration Procedure in LPFSPI with ADDR := 0 (For channel I)
    result = general_dc_calibration(0, control_reg_base) >= 0;
    // Perform DC Calibration Procedure in LPFSPI with ADDR := 1 (For channel Q)
    result = general_dc_calibration(1, control_reg_base) >= 0 && result;

    // Restore TopSPI::CLK_EN Register
    write_reg(0x09, clk_en_save);

    return result;
}

int lms6002d_dev::rxvga2_dc_calibration()
{
    bool result;

    // Set base address for control registers
    uint8_t control_reg_base = 0x60;

    // Save TopSPI::CLK_EN Register
    // TopSPI::CLK_EN := 1
    uint8_t clk_en_save = read_reg(0x09);
    lms_set_bits(0x09, (1 << 4));

    // Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 0 (For DC Reference channel)
    result = general_dc_calibration(0, control_reg_base) >= 0;
    // Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 1 (For VGA2A_I channel)
    result = general_dc_calibration(1, control_reg_base) >= 0 && result;
    // Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 2 (For VGA2A_Q channel)
    result = general_dc_calibration(2, control_reg_base) >= 0 && result;
    // Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 3 (For VGA2B_I channel)
    result = general_dc_calibration(3, control_reg_base) >= 0 && result;
    // Perform DC Calibration Procedure in RxVGA2SPI with ADDR := 4 (For VGA2B_Q channel)
    result = general_dc_calibration(4, control_reg_base) >= 0 && result;

    // Restore TopSPI::CLK_EN Register
    write_reg(0x09, clk_en_save);

    return result;
}

void lms6002d_dev::lpf_bandwidth_tuning(int ref_clock, uint8_t lpf_bandwidth_code)
{
    // Save registers 0x05 and 0x09, because we will modify them during tx_enable()
    uint8_t reg_save_05 = read_reg(0x05);
    uint8_t reg_save_09 = read_reg(0x09);

    // Enable TxPLL and tune it to 320MHz
    tx_enable();
    tx_pll_tune(ref_clock, int(320e6));

    // Use 40MHz generatedFrom TxPLL: TopSPI::CLKSEL_LPFCAL := 0
    // Power Up LPF tuning clock generation block: TopSPI::PD_CLKLPFCAL := 0
    uint8_t reg_save_06 = read_reg(0x06);
    lms_clear_bits(0x06, (1 << 3) | (1 << 2));

    // Set TopSPI::BWC_LPFCAL
    uint8_t t = lms_write_bits(0x07, 0x0f, lpf_bandwidth_code);
    if (verbosity >= 3) printf("code = %x %x %x\n", lpf_bandwidth_code, t, read_reg(0x07));
    // TopSPI::RST_CAL_LPFCAL := 1 (Rst Active)
    uint8_t rst_lpfcal_save = read_reg(0x06);
    lms_set_bits(0x06, 0x01);
    // ...Delay 100ns...
    // TopSPI::RST_CAL_LPFCAL := 0 (Rst Inactive)
    write_reg(0x06, rst_lpfcal_save & ~0x01);
    // RCCAL := TopSPI::RCCAL_LPFCAL
    uint8_t RCCAL = read_reg(0x01) >> 5;
    if (verbosity >= 3) printf("RCCAL = %d\n", RCCAL);
    // RxLPFSPI::RCCAL_LPF := RCCAL
    lms_write_bits(0x56, (7 << 4), (RCCAL << 4));
    // TxLPFSPI::RCCAL_LPF := RCCAL
    lms_write_bits(0x36, (7 << 4), (RCCAL << 4));

    // Restore registers 0x05, 0x06 and 0x09
    write_reg(0x06, reg_save_06);
    write_reg(0x05, reg_save_05);
    write_reg(0x09, reg_save_09);
}

void lms6002d_dev::auto_calibration(int ref_clock, int lpf_bandwidth_code)
{
    if (verbosity > 0) printf("LPF Tuning...\n");
    lpf_tuning_dc_calibration();
    if (verbosity > 0) printf("LPF Bandwidth Tuning...\n");
    lpf_bandwidth_tuning(ref_clock, lpf_bandwidth_code);

    if (verbosity > 0) printf("Tx LPF DC calibration...\n");
    txrx_lpf_dc_calibration(true);

    // Disable Rx
    // We use this way of disabling Rx, because we have to leave
    // RXMIX working. If we disable MIX, calibration will not fail,
    // but DC cancellation will be a bit worse. And setting LNASEL_RXFE
    // to 0 disables RXMIX. So instead of that we select LNA1 and then
    // connect it to the internal termination resistor with
    // IN1SEL_MIX_RXFE and RINEN_MIX_RXFE configuration bits.
    uint8_t lna = get_rx_lna();
    //   1. Select LNA1
    set_rx_lna(1);
    //   2. Connect LNA to external inputs.
    //      IN1SEL_MIX_RXFE: Selects the input to the mixer
    uint8_t reg_save_71 = read_reg(0x71);
    lms_clear_bits(0x71, (1 << 7));
    //   3. Enable internal termination resistor.
    //      RINEN_MIX_RXFE: Termination resistor on external mixer input enable
    uint8_t reg_save_7C = read_reg(0x7C);
    lms_set_bits(0x7C, (1 << 2));
    // Set RxVGA2 gain to max
    uint8_t rx_vga2gain = set_rx_vga2gain(30);
    // Calibrate!
    if (verbosity > 0) printf("Rx LPF DC calibration...\n");
    txrx_lpf_dc_calibration(false);
    if (verbosity > 0) printf("RxVGA2 DC calibration...\n");
    rxvga2_dc_calibration();

    // Restore saved values
    set_rx_vga2gain(rx_vga2gain);
    write_reg(0x71, reg_save_71);
    write_reg(0x7C, reg_save_7C);
    set_rx_lna(lna);
}
