/***********************************************************************
 * The interface to lms6002d that get registered into the property tree
 **********************************************************************/

#include "lms6002d_ctrl.hpp"
#include "lms6002d.hpp"
#include "cores/adf4350_regs.hpp"

#include <uhd/utils/log.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/dict.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <boost/array.hpp>
#include <boost/math/special_functions/round.hpp>
#include <utility>
#include <cmath>
#include <cfloat>
#include <limits>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

/***********************************************************************
 * The LMS6002D dboard constants
 **********************************************************************/
static const freq_range_t lms_freq_range(0.2325e9, 3.72e9);

//Multiplied by 2 for conversion to complex bandpass from lowpass
static const freq_range_t lms_bandwidth_range = list_of
    (range_t(2 * 0.5   * 1e6)) // A hack. See implementation for details.
    (range_t(2 * 0.75  * 1e6))
    (range_t(2 * 0.875 * 1e6))
    (range_t(2 * 1.25  * 1e6))
    (range_t(2 * 1.375 * 1e6))
    (range_t(2 * 1.5   * 1e6))
    (range_t(2 * 1.92  * 1e6))
    (range_t(2 * 2.5   * 1e6))
    (range_t(2 * 2.75  * 1e6))
    (range_t(2 * 3     * 1e6))
    (range_t(2 * 3.5   * 1e6))
    (range_t(2 * 4.375 * 1e6))
    (range_t(2 * 5     * 1e6))
    (range_t(2 * 6     * 1e6))
    (range_t(2 * 7     * 1e6))
    (range_t(2 * 10    * 1e6))
    (range_t(2 * 14    * 1e6))
;

static const std::vector<std::string> lms_tx_antennas = list_of("TX0")("TX1")("TX2")("CAL");

static const std::vector<std::string> lms_rx_antennas = list_of("RX0")("RX1")("RX2")("RX3")("CAL");

static const uhd::dict<std::string, gain_range_t> lms_tx_gain_ranges = map_list_of
    // Use a single control to manually control how VGA1/VGA2 are set
    ("VGA", gain_range_t(-35+0, -4+25, double(1.0)))
;

static const uhd::dict<std::string, gain_range_t> lms_tx_internal_gain_ranges = map_list_of
    ("VGA1", gain_range_t(-35, -4, double(1.0)))
    ("VGA2", gain_range_t(0, 25, double(1.0)))
;

static const uhd::dict<std::string, gain_range_t> lms_rx_gain_ranges = map_list_of
    ("VGA1", gain_range_t(0, 126, double(1.0)))
	// We limit Rx VGA2 to 30dB, as docs say higher values are dangerous
    ("VGA2", gain_range_t(0, 30, double(3.0)))
// ToDo: Add LNA control here
;


static int verbosity = 0;

// LMS6002D interface class for UmTRX

class umtrx_lms6002d_dev: public lms6002d_dev {
    uhd::spi_iface::sptr _spiface;
    const int _slaveno;
public:
    umtrx_lms6002d_dev(uhd::spi_iface::sptr spiface, const int slaveno) : _spiface(spiface), _slaveno(slaveno) {};

    virtual void write_reg(uint8_t addr, uint8_t data) {
        if (verbosity>2) printf("lms6002d_ctrl_impl::write_reg(addr=0x%x, data=0x%x)\n", addr, data);
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected
        uint8_t data = _spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, addr << 8, 16);
        if (verbosity>2) printf("lms6002d_ctrl_impl::read_reg(addr=0x%x) data=0x%x\n", addr, data);
        return data;
    }
};

// LMS6002D virtual daughter board for UmTRX

class lms6002d_ctrl_impl : public lms6002d_ctrl {
public:
    lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const int adf4350_spi_number, const double clock_rate);

    double set_rx_freq(const double freq)
    {
        return this->set_freq(dboard_iface::UNIT_RX, freq);
    }

    double set_tx_freq(const double freq)
    {
        return this->set_freq(dboard_iface::UNIT_TX, freq);
    }

    bool set_rx_enabled(const bool enb)
    {
        return this->set_enabled(dboard_iface::UNIT_RX, enb);
    }

    bool set_tx_enabled(const bool enb)
    {
        return this->set_enabled(dboard_iface::UNIT_TX, enb);
    }

    uhd::freq_range_t get_rx_bw_range(void)
    {
        return lms_bandwidth_range;
    }

    uhd::freq_range_t get_tx_bw_range(void)
    {
        return lms_bandwidth_range;
    }

    uhd::freq_range_t get_rx_freq_range(void)
    {
        return lms_freq_range;
    }

    uhd::freq_range_t get_tx_freq_range(void)
    {
        return lms_freq_range;
    }

    std::vector<std::string> get_tx_antennas(void)
    {
        return lms_tx_antennas;
    }

    std::vector<std::string> get_rx_antennas(void)
    {
        return lms_rx_antennas;
    }

    std::vector<std::string> get_tx_gains(void)
    {
        return lms_tx_gain_ranges.keys();
    }

    std::vector<std::string> get_rx_gains(void)
    {
        return lms_rx_gain_ranges.keys();
    }

    uhd::gain_range_t get_rx_gain_range(const std::string &name)
    {
        return lms_rx_gain_ranges[name];
    }

    uhd::gain_range_t get_tx_gain_range(const std::string &name)
    {
        return lms_tx_gain_ranges[name];
    }

    uint8_t get_tx_vga1dc_i_int(void)
    {
        return lms.get_tx_vga1dc_i_int();
    }

    uint8_t get_tx_vga1dc_q_int(void)
    {
        return lms.get_tx_vga1dc_i_int();
    }

    double set_freq(dboard_iface::unit_t unit, double f) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq(%f)\n", f);
        unsigned ref_freq = _clock_rate;
        double actual_freq = 0;
        if (unit==dboard_iface::UNIT_TX) {
            actual_freq = lms.tx_pll_tune(ref_freq, f);
        } else if (unit==dboard_iface::UNIT_RX) {
#if 1
            actual_freq = lms.rx_pll_tune(ref_freq, f);
#else
            // New beta version of the code for UmSEL support.
            const double umsel_if = 359.5e6;
            double actual_lms_freq = lms.rx_pll_tune(ref_freq, umsel_if);
            if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq() actual_lms_freq=%f\n", actual_lms_freq);
            double adf4350_freq = f - actual_lms_freq;
            actual_freq = tune_adf4350(adf4350_freq);
            if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq() adf4350 freq=%f\n", actual_freq);
            actual_freq += actual_lms_freq;
            if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq() actual_freq=%f\n", actual_freq);
#endif
        } else {
            assert(!"Wrong units_t value passed to lms6002d_ctrl_impl::set_freq()");
        }
        if (actual_freq<0)
            actual_freq = 0;
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq() actual_freq=%f\n", actual_freq);
        return actual_freq;
    }

    bool set_enabled(dboard_iface::unit_t unit, bool en) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_enabled(%d)\n", en);
        if (unit==dboard_iface::UNIT_RX) {
            if (en)
                lms.rx_enable();
            else
                lms.rx_disable();
        } else { // unit==dboard_iface::UNIT_TX
            if (en)
                lms.tx_enable();
            else
                lms.tx_disable();
        }
        //dump();
        return en;
    }

    double set_rx_gain(double gain, const std::string &name) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_gain(%f, %s)\n", gain, name.c_str());
        assert_has(lms_rx_gain_ranges.keys(), name, "LMS6002D rx gain name");
        if(name == "VGA1"){
            lms.set_rx_vga1gain(gain);
        } else if(name == "VGA2"){
            lms.set_rx_vga2gain(gain);
        } else UHD_THROW_INVALID_CODE_PATH();
        return gain;
    }

    void set_rx_ant(const std::string &ant) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_ant(%s)\n", ant.c_str());
        //validate input
        assert_has(lms_rx_antennas, ant, "LMS6002D rx antenna name");

        if (ant == "CAL") {
            // Enable RF loopback if disabled
            if (!rf_loopback_enabled) {
                if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_ant(%s) enabling RF loopback for LNA%d\n", ant.c_str(), lms.get_rx_lna());
                lms.rf_loopback_enable(lms.get_rx_lna());
                rf_loopback_enabled = true;
            }
        } else {
            // Disable RF loopback if enabled
            if (rf_loopback_enabled) {
                lms.rf_loopback_disable();
                rf_loopback_enabled = false;
            }

            if (ant == "RX0") {
                lms.set_rx_lna(0);
            } else if (ant == "RX1") {
                lms.set_rx_lna(1);
            } else if (ant == "RX2") {
                lms.set_rx_lna(2);
            } else if (ant == "RX3") {
                lms.set_rx_lna(3);
            } else {
                UHD_THROW_INVALID_CODE_PATH();
            }
        }
    }

    double set_rx_bandwidth(double bandwidth) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        lms.set_rx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

    double set_tx_gain(double gain, const std::string &name) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_gain(%f, %s)\n", gain, name.c_str());
        //validate input
        assert_has(lms_tx_gain_ranges.keys(), name, "LMS6002D tx gain name");

        if (name == "VGA") {
            // Calculate the best combination of VGA1 and VGA2 gains.
            // For simplicity we just try to use VGA2 as much as possible
            // and only then engage VGA1.
            int desired_vga2 = int(gain) - tx_vga1gain;
            if (desired_vga2 < 0)
                tx_vga2gain = 0;
            else if (desired_vga2 > 25)
                tx_vga2gain = 25;
            else
                tx_vga2gain = desired_vga2;
            tx_vga1gain = int(gain) - tx_vga2gain;

            // Set the gains in hardware
            if (verbosity>1) printf("lms6002d_ctrl_impl::set_tx_gain() VGA1=%d VGA2=%d\n", tx_vga1gain, tx_vga2gain);
            lms.set_tx_vga1gain(tx_vga1gain);
            lms.set_tx_vga2gain(tx_vga2gain);
        } else if (name == "VGA1") {
            if (verbosity>1) printf("db_lms6002d::set_tx_gain() VGA1=%d\n", int(gain));
            lms.set_tx_vga1gain(int(gain));
        } else if (name == "VGA2") {
            if (verbosity>1) printf("db_lms6002d::set_tx_gain() VGA2=%d\n", int(gain));
            lms.set_tx_vga2gain(int(gain));
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        // Return combined gain
        return tx_vga1gain+tx_vga2gain;
    }

    void set_tx_ant(const std::string &ant) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_ant(%s)\n", ant.c_str());
        //validate input
        assert_has(lms_tx_antennas, ant, "LMS6002D tx antenna ant");

        if (ant == "TX0") {
            lms.set_tx_pa(0);
        } else if (ant == "TX1") {
            lms.set_tx_pa(1);
        } else if (ant == "TX2") {
            lms.set_tx_pa(2);
        } else if (ant == "CAL") {
            // TODO: Turn on loopback
            lms.set_tx_pa(0);
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    }

    double set_tx_bandwidth(double bandwidth) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        lms.set_tx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

    uint8_t _set_tx_vga1dc_i_int(uint8_t offset) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_i_int(%d)\n", offset);
        lms.set_tx_vga1dc_i_int(offset);
        return offset;
    }

    uint8_t _set_tx_vga1dc_q_int(uint8_t offset) {
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_q_int(%d)\n", offset);
        lms.set_tx_vga1dc_q_int(offset);
        return offset;
    }

private:
    umtrx_lms6002d_dev lms;        // Interface to the LMS chip.
    int tx_vga1gain, tx_vga2gain;  // Stored values of Tx VGA1 and VGA2 gains.
    bool rf_loopback_enabled;      // Whether RF loopback is enabled.

    // Tune ADF4350 on an UmSEL
    double tune_adf4350(double target_freq);

    uhd::spi_iface::sptr _spiface;
    const int _lms_spi_number;
    const int _adf4350_spi_number;
    const double _clock_rate;
};

lms6002d_ctrl::sptr lms6002d_ctrl::make(uhd::spi_iface::sptr spiface, const int lms_spi_number, const int adf4350_spi_number, const double clock_rate)
{
    return sptr(new lms6002d_ctrl_impl(spiface, lms_spi_number, adf4350_spi_number, clock_rate));
}

/***********************************************************************
 * Tuning
 **********************************************************************/
double lms6002d_ctrl_impl::tune_adf4350(double target_freq) {
    UHD_LOGV(often) << boost::format(
        "UmSEL tune: target frequency %f Mhz"
    ) % (target_freq/1e6) << std::endl;

    //clip the input
    // TODO::::::::::::::::::::::::::::::::
//    target_freq = sbx_freq_range.clip(target_freq);

    //map prescaler setting to mininmum integer divider (N) values (pg.18 prescaler)
    static const uhd::dict<int, int> prescaler_to_min_int_div = map_list_of
        (0,23) //adf4350_regs_t::PRESCALER_4_5
        (1,75) //adf4350_regs_t::PRESCALER_8_9
    ;

    //map rf divider select output dividers to enums
    static const uhd::dict<int, adf4350_regs_t::rf_divider_select_t> rfdivsel_to_enum = map_list_of
        (1,  adf4350_regs_t::RF_DIVIDER_SELECT_DIV1)
        (2,  adf4350_regs_t::RF_DIVIDER_SELECT_DIV2)
        (4,  adf4350_regs_t::RF_DIVIDER_SELECT_DIV4)
        (8,  adf4350_regs_t::RF_DIVIDER_SELECT_DIV8)
        (16, adf4350_regs_t::RF_DIVIDER_SELECT_DIV16)
    ;

    double actual_freq, pfd_freq;
    // TODO:: May be *2? Check
    double ref_freq = _clock_rate * 2;
    int R=0, BS=0, N=0, FRAC=0, MOD=0;
    int RFdiv = 1;
    adf4350_regs_t::reference_divide_by_2_t T     = adf4350_regs_t::REFERENCE_DIVIDE_BY_2_DISABLED;
    adf4350_regs_t::reference_doubler_t     D     = adf4350_regs_t::REFERENCE_DOUBLER_DISABLED;

    //Reference doubler for 50% duty cycle
    // if ref_freq < 12.5MHz enable regs.reference_divide_by_2
    if(ref_freq <= 12.5e6) D = adf4350_regs_t::REFERENCE_DOUBLER_ENABLED;

    //increase RF divider until acceptable VCO frequency
    //start with target_freq*2 because mixer has divide by 2
    double vco_freq = target_freq;
    while (vco_freq < 2.2e9) {
        vco_freq *= 2;
        RFdiv *= 2;
    }

    //use 8/9 prescaler for vco_freq > 3 GHz (pg.18 prescaler)
    adf4350_regs_t::prescaler_t prescaler = vco_freq > 3e9 ? adf4350_regs_t::PRESCALER_8_9 : adf4350_regs_t::PRESCALER_4_5;

    /*
     * The goal here is to loop though possible R dividers,
     * band select clock dividers, N (int) dividers, and FRAC
     * (frac) dividers.
     *
     * Calculate the N and F dividers for each set of values.
     * The loop exists when it meets all of the constraints.
     * The resulting loop values are loaded into the registers.
     *
     * from pg.21
     *
     * f_pfd = f_ref*(1+D)/(R*(1+T))
     * f_vco = (N + (FRAC/MOD))*f_pfd
     *    N = f_vco/f_pfd - FRAC/MOD = f_vco*((R*(T+1))/(f_ref*(1+D))) - FRAC/MOD
     * f_rf = f_vco/RFdiv)
     * f_actual = f_rf/2
     */
    for(R = 1; R <= 1023; R+=1){
        //PFD input frequency = f_ref/R ... ignoring Reference doubler/divide-by-2 (D & T)
        pfd_freq = ref_freq*(1+D)/(R*(1+T));

        //keep the PFD frequency at or below 25MHz (Loop Filter Bandwidth)
        if (pfd_freq > 25e6) continue;

        //ignore fractional part of tuning
        N = int(std::floor(vco_freq/pfd_freq));

        //keep N > minimum int divider requirement
        if (N < prescaler_to_min_int_div[prescaler]) continue;

        for(BS=1; BS <= 255; BS+=1){
            //keep the band select frequency at or below 100KHz
            //constraint on band select clock
            if (pfd_freq/BS > 100e3) continue;
            goto done_loop;
        }
    } done_loop:

    //Fractional-N calculation
    MOD = 4095; //max fractional accuracy
    FRAC = int((vco_freq/pfd_freq - N)*MOD);

    //Reference divide-by-2 for 50% duty cycle
    // if R even, move one divide by 2 to to regs.reference_divide_by_2
    if(R % 2 == 0){
        T = adf4350_regs_t::REFERENCE_DIVIDE_BY_2_ENABLED;
        R /= 2;
    }

    //actual frequency calculation
    actual_freq = double((N + (double(FRAC)/double(MOD)))*ref_freq*(1+int(D))/(R*(1+int(T)))/RFdiv);

    // TODO:: get_locked!
//    UHD_LOGV(often)
    std::cout
        << boost::format("UmSEL Intermediates: ref=%0.2f, outdiv=%f, fbdiv=%f") % (ref_freq*(1+int(D))/(R*(1+int(T)))) % double(RFdiv*2) % double(N + double(FRAC)/double(MOD)) << std::endl
        << boost::format("UmSEL tune: R=%d, BS=%d, N=%d, FRAC=%d, MOD=%d, T=%d, D=%d, RFdiv=%d, LD=%s"
            ) % R % BS % N % FRAC % MOD % T % D % RFdiv % true /* this->get_locked(unit).to_pp_string() */ << std::endl
        << boost::format("UmSEL Frequencies (MHz): REQ=%0.2f, ACT=%0.2f, VCO=%0.2f, PFD=%0.2f, BAND=%0.2f"
            ) % (target_freq/1e6) % (actual_freq/1e6) % (vco_freq/1e6) % (pfd_freq/1e6) % (pfd_freq/BS/1e6) << std::endl;

    //load the register values
    adf4350_regs_t regs;

    // TODO:: What?
//    if ((unit == dboard_iface::UNIT_TX) and (actual_freq == sbx_tx_lo_2dbm.clip(actual_freq)))
//        regs.output_power = adf4350_regs_t::OUTPUT_POWER_2DBM;
//    else
        regs.output_power = adf4350_regs_t::OUTPUT_POWER_5DBM;

    regs.frac_12_bit = FRAC;
    regs.int_16_bit = N;
    regs.mod_12_bit = MOD;
    regs.prescaler = prescaler;
    regs.r_counter_10_bit = R;
    regs.reference_divide_by_2 = T;
    regs.reference_doubler = D;
    regs.band_select_clock_div = BS;
    UHD_ASSERT_THROW(rfdivsel_to_enum.has_key(RFdiv));
    regs.rf_divider_select = rfdivsel_to_enum[RFdiv];

    regs.mute_till_lock_detect = adf4350_regs_t::MUTE_TILL_LOCK_DETECT_MUTE_ENABLED;
    regs.charge_pump_current = adf4350_regs_t::CHARGE_PUMP_CURRENT_2_50MA;
    regs.double_buffer = adf4350_regs_t::DOUBLE_BUFFER_ENABLED;
    regs.muxout = adf4350_regs_t::MUXOUT_3STATE;
    regs.low_noise_and_spur = adf4350_regs_t::LOW_NOISE_AND_SPUR_LOW_NOISE;



    //write the registers
    //correct power-up sequence to write registers (5, 4, 3, 2, 1, 0)

#if 0
    for(addr=5; addr>=0; addr--){
//        UHD_LOGV(often)
        std::cout << boost::format(
            "UmSEL SPI Reg (0x%02x): 0x%08x"
        ) % addr % regs.get_reg(addr) << std::endl;
        this->get_iface()->write_spi(
            uhd::usrp::dboard_iface::UNIT_SYNT, spi_config_t::EDGE_RISE,
            regs.get_reg(addr), 32
        );
    }
#else
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, 0x0580000|5, 32);
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, 0x0BFF4F8|4, 32);
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, 0x0040000|3, 32);
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, 0x1006E40|2, 32);
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, 0x8008208|1, 32);
    _spiface->write_spi(_adf4350_spi_number, spi_config_t::EDGE_RISE, (325<<15)|(1<<3)|0, 32);
#endif

    //return the actual frequency
//    UHD_LOGV(often) << boost::format(
    std::cout << boost::format(
        "UmSEL tune: actual frequency %f Mhz"
    ) % (actual_freq/1e6) << std::endl;
    return actual_freq;
}

// LMS RX dboard configuration

lms6002d_ctrl_impl::lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const int adf4350_spi_number, const double clock_rate) :
                                             lms(umtrx_lms6002d_dev(spiface, lms_spi_number)),
                                             tx_vga1gain(lms.get_tx_vga1gain()),
                                             tx_vga2gain(lms.get_tx_vga2gain()),
                                             rf_loopback_enabled(false),
                                             _spiface(spiface),
                                             _lms_spi_number(lms_spi_number),
                                             _adf4350_spi_number(adf4350_spi_number),
                                             _clock_rate(clock_rate)
{
    ////////////////////////////////////////////////////////////////////
    // LMS6002D initialization
    ////////////////////////////////////////////////////////////////////
    lms.init();
    // Set proper values for Tx and Rx Fsync and IQ interleaving.
    lms.set_txrx_polarity_and_interleaving(0, lms6002d_dev::INTERLEAVE_IQ, 1, lms6002d_dev::INTERLEAVE_QI);
    // Rx and Tx will be enabled/disabled during the property tree initialization
    // at later steps of initialization, so it doesn't hurt that we enable them here.
    lms.rx_enable();
    lms.tx_enable();
    // -10dB is a good value for calibration if don't know a target gain yet
    lms.set_tx_vga1gain(-10);

    // Perform autocalibration
    lms.auto_calibration(_clock_rate, 0xf);
}

