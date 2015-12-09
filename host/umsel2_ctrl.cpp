// Copyright 2015-2015 Fairwaves LLC
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

#include "umsel2_ctrl.hpp"
#include "umtrx_regs.hpp"
#include <map>

static const int REG0_NVALUE_SHIFT = 4;
static const int REG0_NVALUE_MASK = 0xffff;
static const int REG0_PRESCALER_SHIFT = 20;
static const int REG0_AUTOCAL_SHIFT = 21;
static const int REG1_MFRAC_SHIFT = 4;
static const int REG1_MFRAC_MASK = 0xffffff;
static const int REG2_AUX_FRAC_SHIFT = 18;
static const int REG2_AUX_FRAC_MASK = ((1<<14)-1);
static const int REG2_AUX_MOD_SHIFT = 4;
static const int REG2_AUX_MOD_MASK = ((1<<14)-1);
static const int REG3_SD_LOAD_SHIFT = 30;
static const int REG3_PHASE_RSYNC_SHIFT = 29;
static const int REG3_PHASE_ADJ_SHIFT = 28;
static const int REG3_PHASE_SHIFT = 4;
static const int REG3_PHASE_MASK = ((1<<24)-1);
static const int REG4_MUXOUT_SHIFT = 27;
static const int REG4_MUXOUT_MASK = ((1<<3)-1);
static const int REG4_REF_DBL_SHIFT = 26;
static const int REG4_REF_DIV_SHIFT = 25;
static const int REG4_R_SHIFT = 15;
static const int REG4_R_MASK = ((1<<10)-1);
static const int REG4_DBL_BUFF_SHIFT = 14;
static const int REG4_CURRENT_SHIFT = 10;
static const int REG4_CURRENT_MASK = ((1<<4)-1);
static const int REG4_REF_MODE_SHIFT = 9;
static const int REG4_MUX_LOGIC_SHIFT = 8;
static const int REG4_PD_POL_SHIFT = 7;
static const int REG4_PWR_DOWN_SHIFT = 6;
static const int REG4_CP_3STATE_SHIFT = 5;
static const int REG4_CNTR_RESET = 4;
static const int REG5_RESERVED = 0x00800025;
static const int REG6_GATED_BLEED_SHIFT = 30;
static const int REG6_NEG_BLEED_SHIFT = 29;
static const int REG6_RESERVED_SHIFT = 25;
static const int REG6_RESERVED_VALUE = 0xA;
static const int REG6_FB_SEL_SHIFT = 24;
static const int REG6_RF_DIV_SHIFT = 21;
static const int REG6_RF_DIV_MASK = ((1<<3)-1);
static const int REG6_CP_BLEED_CURR_SHIFT = 13;
static const int REG6_CP_BLEED_CURR_MASK = ((1<<8)-1);
static const int REG6_MLTD_SHIFT = 11;
static const int REG6_AUX_PWR_EN_SHIFT = 9;
static const int REG6_AUX_PWR_SHIFT = 7;
static const int REG6_AUX_PWR_MASK = ((1<<2)-1);
static const int REG6_PWR_EN_SHIFT = 6;
static const int REG6_PWR_SHIFT = 4;
static const int REG6_PWR_MASK = ((1<<2)-1);
static const int REG7_RESERVED_SHIFT = 26;
static const int REG7_RESERVED_VALUE = 0x4;
static const int REG7_LE_SYNC_SHIFT = 25;
static const int REG7_LD_CYCLE_CNT_SHIFT = 8;
static const int REG7_LD_CYCLE_CNT_MASK = ((1<<2)-1);
static const int REG7_LOL_MODE_SHIFT = 7;
static const int REG7_FRAC_N_PREC_SHIFT = 5;
static const int REG7_FRAC_N_PREC_MASK = ((1<<2)-1);
static const int REG7_LD_MODE_SHIFT = 4;
static const int REG8_RESERVED = 0x102D0428;
static const int REG9_VCO_BAND_SHIFT = 24;
static const int REG9_VCO_BAND_MASK = ((1<<8)-1);
static const int REG9_TIMEOUT_SHIFT = 14;
static const int REG9_TIMEOUT_MASK = ((1<<10)-1);
static const int REG9_AUTO_LVL_TO_SHIFT = 9;
static const int REG9_AUTO_LVL_TO_MASK = ((1<<5)-1);
static const int REG9_SYNT_LOCK_TO_SHIFT = 4;
static const int REG9_SYNT_LOCK_TO_MASK = ((1<<5)-1);
static const int REG10_RESERVED_SHIFT = 14;
static const int REG10_RESERVED_VALUE = 0x300;
static const int REG10_ADC_CLK_DIV_SHIFT = 6;
static const int REG10_ADC_CLK_DIV_MASK = ((1<<8)-1);
static const int REG10_ADC_CONV_SHIFT = 5;
static const int REG10_ADC_EN_SHIFT = 4;
static const int REG11_RESERVED = 0x0061300B;
static const int REG12_RESYNC_CLOCK_SHIFT = 16;
static const int REG12_RESYNC_CLOCK_MASK = ((1<<16)-1);
static const int REG12_RESERVED_SHIFT = 4;
static const int REG12_RESERVED_VALUE = 0x41;

#define MODIFY_FIELD(reg, val, mask, shift) \
    reg = ((reg & ~(mask << shift)) | ((val & mask) << shift))

class umsel2_ctrl_impl : public umsel2_ctrl
{
public:

    umsel2_ctrl_impl(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface, const double ref_clock):
        _ctrl(ctrl), _spiface(spiface), _ref_clock(ref_clock)
    {
        this->init_synth(SPI_SS_AUX1);
        this->init_synth(SPI_SS_AUX2);
    }

    ~umsel2_ctrl_impl(void)
    {
        try
        {
            this->pd_synth(SPI_SS_AUX1);
            this->pd_synth(SPI_SS_AUX2);
        }
        catch(...){}
    }

    uhd::freq_range_t get_rx_freq_range(const int)
    {
        return uhd::freq_range_t(54e6, 4400e6);
    }

    double set_rx_freq(const int which, const double freq)
    {
        const int slaveno = (which == 1)?SPI_SS_AUX1 : SPI_SS_AUX2;
        return this->tune_synth(slaveno, freq);
    }

    uhd::sensor_value_t get_locked(const int which)
    {
        boost::uint32_t irq = _ctrl->peek32(U2_REG_IRQ_RB);
        bool locked = false;
        if (which == 1) locked = (irq & AUX_LD1_IRQ_BIT) != 0;
        if (which == 2) locked = (irq & AUX_LD2_IRQ_BIT) != 0;
        return uhd::sensor_value_t("LO", locked, "locked", "unlocked");
    }

private:

    void init_synth(const int slaveno)
    {
        //reset the registers
        _regs[slaveno][0] = 0;
        _regs[slaveno][1] = 0;
        _regs[slaveno][2] = 0;
        _regs[slaveno][3] = 0;
        _regs[slaveno][4] = 0;
        _regs[slaveno][5] = REG5_RESERVED;
        _regs[slaveno][6] = REG6_RESERVED_VALUE << REG6_RESERVED_SHIFT;
        _regs[slaveno][7] = REG7_RESERVED_VALUE << REG7_RESERVED_SHIFT;
        _regs[slaveno][8] = REG8_RESERVED;
        _regs[slaveno][9] = 0;
        _regs[slaveno][10] = REG10_RESERVED_VALUE << REG10_RESERVED_SHIFT;
        _regs[slaveno][11] = REG11_RESERVED;
        _regs[slaveno][12] = REG12_RESERVED_VALUE << REG12_RESERVED_SHIFT;

        //muxout to lock detect
        MODIFY_FIELD(_regs[slaveno][4], 6, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);

        //write all registers (init sequence counts down)
        for (int addr = 12; addr >= 0; addr--)
            this->write_reg(slaveno, addr);
    }

    void pd_synth(const int slaveno)
    {
        //muxout to lock 3state
        MODIFY_FIELD(_regs[slaveno][4], 0, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);

        //charge pump 3state
        MODIFY_FIELD(_regs[slaveno][4], 1, 0x1, REG4_CP_3STATE_SHIFT);

        //power down
        MODIFY_FIELD(_regs[slaveno][4], 1, 0x1, REG4_PWR_DOWN_SHIFT);

        //outputs off
        MODIFY_FIELD(_regs[slaveno][6], 0, 0x1, REG6_AUX_PWR_EN_SHIFT);
        MODIFY_FIELD(_regs[slaveno][6], 0, 0x1, REG6_PWR_EN_SHIFT);

        //write all registers
        for (int addr = 12; addr >= 0; addr--)
            this->write_reg(slaveno, addr);
    }

    double tune_synth(const int slaveno, const double freq)
    {
        
    }

    void write_reg(const int slaveno, const int addr)
    {
        int value = (_regs[slaveno][addr] & ~0xf) | addr;
        _spiface->write_spi(slaveno, uhd::spi_config_t::EDGE_RISE, value, 32);
    }

    uhd::wb_iface::sptr _ctrl;
    uhd::spi_iface::sptr _spiface;
    const double _ref_clock;
    std::map<int, std::map<int, int> > _regs;
};

umsel2_ctrl::sptr umsel2_ctrl::make(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface, const double ref_clock)
{
    return umsel2_ctrl::sptr(new umsel2_ctrl_impl(ctrl, spiface, ref_clock));
}
