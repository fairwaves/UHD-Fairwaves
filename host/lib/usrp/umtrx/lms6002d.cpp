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
    write_reg(0x41, 0x15); // VGA1GAIN
    write_reg(0x45, 0x00); // VGA2GAIN, ENVD

    //reg_dump();
}
