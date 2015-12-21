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
#include <uhd/exception.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <cmath>
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

    umsel2_ctrl_impl(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface, const double ref_clock, const bool verbose):
        _ctrl(ctrl), _spiface(spiface), _ref_clock(ref_clock), verbose(verbose)
    {
        this->init_synth(SPI_SS_AUX1);
        this->init_synth(SPI_SS_AUX2);

        //--------- basic self tests, use the muxout to verify communication ----------//

        //set mux out to ground in both cases
        MODIFY_FIELD(_regs[SPI_SS_AUX1][4], 2, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        MODIFY_FIELD(_regs[SPI_SS_AUX2][4], 2, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        this->write_reg(SPI_SS_AUX1, 4);
        this->write_reg(SPI_SS_AUX2, 4);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD1_IRQ_BIT) == 0);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD2_IRQ_BIT) == 0);

        //set slave1 to muxout vdd
        MODIFY_FIELD(_regs[SPI_SS_AUX1][4], 1, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        MODIFY_FIELD(_regs[SPI_SS_AUX2][4], 2, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        this->write_reg(SPI_SS_AUX1, 4);
        this->write_reg(SPI_SS_AUX2, 4);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD1_IRQ_BIT) != 0);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD2_IRQ_BIT) == 0);

        //set slave2 to muxout vdd
        MODIFY_FIELD(_regs[SPI_SS_AUX1][4], 2, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        MODIFY_FIELD(_regs[SPI_SS_AUX2][4], 1, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        this->write_reg(SPI_SS_AUX1, 4);
        this->write_reg(SPI_SS_AUX2, 4);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD1_IRQ_BIT) == 0);
        UHD_ASSERT_THROW((_ctrl->peek32(U2_REG_IRQ_RB) & AUX_LD2_IRQ_BIT) != 0);

        //restore lock detect out
        MODIFY_FIELD(_regs[SPI_SS_AUX1][4], 6, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        MODIFY_FIELD(_regs[SPI_SS_AUX2][4], 6, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);
        this->write_reg(SPI_SS_AUX1, 4);
        this->write_reg(SPI_SS_AUX2, 4);
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

        //------------------------------------------------------------//
        //----------------------- register 0 -------------------------//
        //------------------------------------------------------------//

        //autocal enabled
        MODIFY_FIELD(_regs[slaveno][0], 1, 0x1, REG0_AUTOCAL_SHIFT);

        //prescaler 4/5
        MODIFY_FIELD(_regs[slaveno][0], 0, 0x1, REG0_PRESCALER_SHIFT);

        //------------------------------------------------------------//
        //----------------------- register 3 -------------------------//
        //------------------------------------------------------------//

        //sd load reset, phase resync, phase adjust = disabled
        MODIFY_FIELD(_regs[slaveno][3], 0, 0x1, REG3_SD_LOAD_SHIFT);
        MODIFY_FIELD(_regs[slaveno][3], 0, 0x1, REG3_PHASE_RSYNC_SHIFT);
        MODIFY_FIELD(_regs[slaveno][3], 0, 0x1, REG3_PHASE_ADJ_SHIFT);
        MODIFY_FIELD(_regs[slaveno][3], 0, REG3_PHASE_MASK, REG3_PHASE_SHIFT);

        //------------------------------------------------------------//
        //----------------------- register 4 -------------------------//
        //------------------------------------------------------------//

        //muxout to lock detect
        MODIFY_FIELD(_regs[slaveno][4], 6, REG4_MUXOUT_MASK, REG4_MUXOUT_SHIFT);

        //double buff disabled
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_DBL_BUFF_SHIFT);

        //charge pump current 0.31mA@5.1k
        MODIFY_FIELD(_regs[slaveno][4], 0, REG4_CURRENT_MASK, REG4_CURRENT_SHIFT);

        //refin single ended
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_REF_MODE_SHIFT);

        //mux level 3V
        MODIFY_FIELD(_regs[slaveno][4], 1, 0x1, REG4_MUX_LOGIC_SHIFT);

        //PD polarity positive
        MODIFY_FIELD(_regs[slaveno][4], 1, 0x1, REG4_PD_POL_SHIFT);

        //power down disabled
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_PWR_DOWN_SHIFT);

        //charge-pump 3-state disabled
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_CP_3STATE_SHIFT);

        //counter reset disabled
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_CNTR_RESET);

        //------------------------------------------------------------//
        //----------------------- register 6 -------------------------//
        //------------------------------------------------------------//

        //feedback fundamental
        MODIFY_FIELD(_regs[slaveno][6], 1, 0x1, REG6_FB_SEL_SHIFT);

        //bleed current 7.5uA
        MODIFY_FIELD(_regs[slaveno][6], 2, REG6_CP_BLEED_CURR_MASK, REG6_CP_BLEED_CURR_SHIFT);

        //mute until lock detect disabled
        MODIFY_FIELD(_regs[slaveno][6], 0, 0x1, REG6_MLTD_SHIFT);

        //aux output disabled (-1dBm)
        MODIFY_FIELD(_regs[slaveno][6], 0, 0x1, REG6_AUX_PWR_EN_SHIFT);
        MODIFY_FIELD(_regs[slaveno][6], 1, REG6_AUX_PWR_MASK, REG6_AUX_PWR_SHIFT);

        //RF output power (5dBm)
        MODIFY_FIELD(_regs[slaveno][6], 1, 0x1, REG6_PWR_EN_SHIFT);
        MODIFY_FIELD(_regs[slaveno][6], 3, REG6_PWR_MASK, REG6_PWR_SHIFT);

        //negative bleed enabled
        MODIFY_FIELD(_regs[slaveno][6], 1, 0x1, REG6_NEG_BLEED_SHIFT);

        //gated bleed disabled
        MODIFY_FIELD(_regs[slaveno][6], 0, 0x1, REG6_GATED_BLEED_SHIFT);

        //------------------------------------------------------------//
        //----------------------- register 7 -------------------------//
        //------------------------------------------------------------//

        //LE Sync REFin
        MODIFY_FIELD(_regs[slaveno][7], 1, 0x1, REG7_LE_SYNC_SHIFT);

        //LD Cycles
        MODIFY_FIELD(_regs[slaveno][7], 1024, REG7_LD_CYCLE_CNT_MASK, REG7_LD_CYCLE_CNT_SHIFT);

        //LOL Mode disabled
        MODIFY_FIELD(_regs[slaveno][7], 0, 0x1, REG7_LOL_MODE_SHIFT);

        //Frac-N LD Prec 5.0ns
        MODIFY_FIELD(_regs[slaveno][7], 0, REG7_FRAC_N_PREC_MASK, REG7_FRAC_N_PREC_SHIFT);

        //LD Mode Frac-N
        MODIFY_FIELD(_regs[slaveno][7], 0, 0x1, REG7_LD_MODE_SHIFT);

        //------------------------------------------------------------//
        //----------------------- register 10 ------------------------//
        //------------------------------------------------------------//

        //adc enable
        MODIFY_FIELD(_regs[slaveno][10], 1, 0x1, REG10_ADC_EN_SHIFT);

        //adc conversion enable
        MODIFY_FIELD(_regs[slaveno][10], 1, 0x1, REG10_ADC_CONV_SHIFT);

        //------------------------------------------------------------//
        //----------------------- register 12 ------------------------//
        //------------------------------------------------------------//

        //phase resync 0
        MODIFY_FIELD(_regs[slaveno][12], 0, REG12_RESYNC_CLOCK_MASK, REG12_RESYNC_CLOCK_SHIFT);

        //write all registers
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

    double tune_synth(const int slaveno, const double RFout)
    {
        if (verbose) std::cout << " tune_synth(slaveno=" << slaveno << ")" << std::endl;
        if (verbose) std::cout << " RFout " << (RFout/1e6) << " MHz" << std::endl;

        //determine the reference out divider and VCOout
        double VCOout = 0;
        int RFOUTDIVSEL = 0;
        while (true)
        {
            int RFOUTDIV = 1 << RFOUTDIVSEL;
            VCOout = RFout*RFOUTDIV;
            if (VCOout < 3.4e9) RFOUTDIVSEL++;
            else break;
        }
        if (verbose) std::cout << " RFOUTDIV " << (1 << RFOUTDIVSEL) << "" << std::endl;
        if (verbose) std::cout << " VCOout " << (VCOout/1e6) << " MHz" << std::endl;

        //use doubler to increase the pfd frequency (good for noise performance)
        int REFDBL = 0;
        int REFDIV = 0;

        //prescaler settings
        int PRESCALER = 0; //4/5
        const int Nmin = (PRESCALER==0)?23:75;

        //calculate the R divider, N divider, and PDF frequency
        double NDIV = 0;
        int RDIV = 1;
        double fPFD = 0;
        while (true)
        {
            fPFD = _ref_clock*(double(1+REFDBL)/double(RDIV*(1+REFDIV)));
            NDIV = VCOout/fPFD;
            if (NDIV < Nmin) RDIV++;
            else break;
        }
        if (verbose) std::cout << " RDIV " << RDIV << "" << std::endl;
        if (verbose) std::cout << " NDIV " << NDIV << "" << std::endl;
        if (verbose) std::cout << " fPFD " << (fPFD/1e6) << " MHz" << std::endl;

        //calculate the integer parts of the N divider
        int NINT = int(NDIV);
        double NFRAC = std::ldexp(NDIV-NINT, 24);
        int FRAC1 = int(NFRAC);
        int MOD2 = fPFD/1e6; //TODO MOD2 = fPFD/GCD(fPFD, fCHSP)
        int FRAC2 = int((NFRAC-FRAC1)*MOD2);
        if (verbose) std::cout << " NINT " << NINT << "" << std::endl;
        if (verbose) std::cout << " FRAC1 " << FRAC1 << "" << std::endl;
        if (verbose) std::cout << " MOD2 " << MOD2 << "" << std::endl;
        if (verbose) std::cout << " FRAC2 " << FRAC2 << "" << std::endl;

        //VCO Band Division
        //PFD/(band division × 16) < 150 kHz
        int VCObanddiv = 1;
        while (not(fPFD/(VCObanddiv*16) < 150e3)) VCObanddiv++;
        if (verbose) std::cout << " VCObanddiv " << VCObanddiv << "" << std::endl;

        //Maximize ALC Wait (to reduce Timeout to minimize time) so
        //that ALC Wait = 30 and Synthesizer Lock Timeout = 12.
        int ALC = 30;
        int SLT = 12;
        int TIMEOUT = std::ceil((fPFD*50e-6)/ALC);
        if (verbose) std::cout << " TIMEOUT " << TIMEOUT << "" << std::endl;

        //ADC Clock Divider (ADC_CLK_DIV)
        //PFD/((ADC_CLK_DIV × 4) × 2) < 100 kHz
        /*
        int ADC_CLK_DIV = 1;
        while (not(fPFD/((ADC_CLK_DIV*4)*2) < 100e3)) ADC_CLK_DIV++;
        if (verbose) std::cout << " ADC_CLK_DIV " << ADC_CLK_DIV << "" << std::endl;
        const long sleepUs = long(1e6*16*ADC_CLK_DIV/fPFD);
        */

        //Copied from the ADI GUI
        //after trying to juxtapose the documentation with the GUI
        int ADC_CLK_DIV = 65;
        const long sleepUs = 160;

        //load registers
        MODIFY_FIELD(_regs[slaveno][0], NINT, REG0_NVALUE_MASK, REG0_NVALUE_SHIFT);
        MODIFY_FIELD(_regs[slaveno][0], PRESCALER, 0x1, REG0_PRESCALER_SHIFT);
        MODIFY_FIELD(_regs[slaveno][0], 1/*enb*/, 0x1, REG0_AUTOCAL_SHIFT);
        MODIFY_FIELD(_regs[slaveno][1], FRAC1, REG1_MFRAC_MASK, REG1_MFRAC_SHIFT);
        MODIFY_FIELD(_regs[slaveno][2], MOD2, REG2_AUX_MOD_MASK, REG2_AUX_MOD_SHIFT);
        MODIFY_FIELD(_regs[slaveno][2], FRAC2, REG2_AUX_FRAC_MASK, REG2_AUX_FRAC_SHIFT);
        MODIFY_FIELD(_regs[slaveno][4], RDIV, REG4_R_MASK, REG4_R_SHIFT);
        MODIFY_FIELD(_regs[slaveno][4], REFDIV, 0x1, REG4_REF_DIV_SHIFT);
        MODIFY_FIELD(_regs[slaveno][4], REFDBL, 0x1, REG4_REF_DBL_SHIFT);
        MODIFY_FIELD(_regs[slaveno][6], RFOUTDIVSEL, REG6_RF_DIV_MASK, REG6_RF_DIV_SHIFT);
        MODIFY_FIELD(_regs[slaveno][9], SLT, REG9_SYNT_LOCK_TO_MASK, REG9_SYNT_LOCK_TO_SHIFT);
        MODIFY_FIELD(_regs[slaveno][9], ALC, REG9_AUTO_LVL_TO_MASK, REG9_AUTO_LVL_TO_SHIFT);
        MODIFY_FIELD(_regs[slaveno][9], TIMEOUT, REG9_TIMEOUT_MASK, REG9_TIMEOUT_SHIFT);
        MODIFY_FIELD(_regs[slaveno][9], VCObanddiv, REG9_VCO_BAND_MASK, REG9_VCO_BAND_SHIFT);
        MODIFY_FIELD(_regs[slaveno][10], ADC_CLK_DIV, REG10_ADC_CLK_DIV_MASK, REG10_ADC_CLK_DIV_SHIFT);

        //write other registers
        this->write_reg(slaveno, 6);
        this->write_reg(slaveno, 9);
        this->write_reg(slaveno, 10);

        //FREQUENCY UPDATE SEQUENCE
        MODIFY_FIELD(_regs[slaveno][4], 1, 0x1, REG4_CNTR_RESET);
        this->write_reg(slaveno, 4);
        this->write_reg(slaveno, 2);
        this->write_reg(slaveno, 1);
        MODIFY_FIELD(_regs[slaveno][0], 0, 0x1, REG0_AUTOCAL_SHIFT);
        this->write_reg(slaveno, 0);
        MODIFY_FIELD(_regs[slaveno][4], 0, 0x1, REG4_CNTR_RESET);
        this->write_reg(slaveno, 4);
        boost::this_thread::sleep(boost::posix_time::microseconds(sleepUs));
        if (verbose) std::cout << " sleep time " << (sleepUs) << " us" << std::endl;
        MODIFY_FIELD(_regs[slaveno][0], 1, 0x1, REG0_AUTOCAL_SHIFT);
        this->write_reg(slaveno, 0);

        //calculate actual tune value
        double Nactual = NINT + std::ldexp(double(FRAC1 + FRAC2/double(MOD2)), -24);
        double RFoutactual = (fPFD*Nactual)/(1 << RFOUTDIVSEL);
        if (verbose) std::cout << " Nactual " << Nactual << "" << std::endl;
        if (verbose) std::cout << " RFoutactual " << (RFoutactual/1e6) << " MHz" << std::endl;
        if (verbose) boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        if (verbose) std::cout << " Locked " << this->get_locked((slaveno==SPI_SS_AUX1)?1:2).value << "" << std::endl;
        return RFoutactual;
    }

    void write_reg(const int slaveno, const int addr)
    {
        int value = (_regs[slaveno][addr] & ~0xf) | addr;
        if (verbose) std::cout << "write_reg[" << addr << "] = 0x" << std::hex << value << std::dec << std::endl;
        _spiface->write_spi(slaveno, uhd::spi_config_t::EDGE_RISE, value, 32);
    }

    uhd::wb_iface::sptr _ctrl;
    uhd::spi_iface::sptr _spiface;
    const double _ref_clock;
    std::map<int, std::map<int, int> > _regs;
    const bool verbose;
};

umsel2_ctrl::sptr umsel2_ctrl::make(uhd::wb_iface::sptr ctrl, uhd::spi_iface::sptr spiface, const double ref_clock, const bool verbose)
{
    return umsel2_ctrl::sptr(new umsel2_ctrl_impl(ctrl, spiface, ref_clock, verbose));
}
