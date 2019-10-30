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
#include "umtrx_log_adapter.hpp"
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
#include <boost/thread/recursive_mutex.hpp>
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
    //listing VGA2 first means that overall gain is first distributed to VGA2
    ("VGA2", gain_range_t(0, 25, double(1.0)))
    ("VGA1", gain_range_t(-35, -4, double(1.0)))
    // Use a single control to manually control how VGA1/VGA2 are set
//    ("VGA", gain_range_t(-35+0, -4+25, double(1.0)))
;

static const uhd::dict<std::string, gain_range_t> lms_rx_gain_ranges = map_list_of
    // VGA1 follows the approximation of [dB] = 5 + 20*log10(127/(127-Code)) where 0 <= Code <= 120
    ("VGA1", gain_range_t( list_of
         (range_t(5.00))(range_t(5.07))(range_t(5.14))(range_t(5.21))(range_t(5.28))
         (range_t(5.35))(range_t(5.42))(range_t(5.49))(range_t(5.57))(range_t(5.64))
         (range_t(5.71))(range_t(5.79))(range_t(5.86))(range_t(5.94))(range_t(6.01))
         (range_t(6.09))(range_t(6.17))(range_t(6.25))(range_t(6.33))(range_t(6.41))
         (range_t(6.49))(range_t(6.57))(range_t(6.65))(range_t(6.74))(range_t(6.82))
         (range_t(6.90))(range_t(6.99))(range_t(7.08))(range_t(7.16))(range_t(7.25))
         (range_t(7.34))(range_t(7.43))(range_t(7.52))(range_t(7.61))(range_t(7.71))
         (range_t(7.80))(range_t(7.90))(range_t(7.99))(range_t(8.09))(range_t(8.19))
         (range_t(8.29))(range_t(8.39))(range_t(8.49))(range_t(8.59))(range_t(8.69))
         (range_t(8.80))(range_t(8.91))(range_t(9.01))(range_t(9.12))(range_t(9.23))
         (range_t(9.35))(range_t(9.46))(range_t(9.57))(range_t(9.69))(range_t(9.81))
         (range_t(9.93))(range_t(10.05))(range_t(10.17))(range_t(10.30))(range_t(10.43))
         (range_t(10.55))(range_t(10.69))(range_t(10.82))(range_t(10.95))(range_t(11.09))
         (range_t(11.23))(range_t(11.37))(range_t(11.51))(range_t(11.66))(range_t(11.81))
         (range_t(11.96))(range_t(12.11))(range_t(12.27))(range_t(12.43))(range_t(12.59))
         (range_t(12.76))(range_t(12.92))(range_t(13.10))(range_t(13.27))(range_t(13.45))
         (range_t(13.63))(range_t(13.82))(range_t(14.01))(range_t(14.21))(range_t(14.41))
         (range_t(14.61))(range_t(14.82))(range_t(15.03))(range_t(15.25))(range_t(15.48))
         (range_t(15.71))(range_t(15.95))(range_t(16.19))(range_t(16.45))(range_t(16.71))
         (range_t(16.97))(range_t(17.25))(range_t(17.53))(range_t(17.83))(range_t(18.13))
         (range_t(18.45))(range_t(18.78))(range_t(19.12))(range_t(19.47))(range_t(19.84))
         (range_t(20.23))(range_t(20.63))(range_t(21.06))(range_t(21.50))(range_t(21.97))
         (range_t(22.47))(range_t(22.99))(range_t(23.55))(range_t(24.15))(range_t(24.80))
         (range_t(25.49))(range_t(26.25))(range_t(27.08))(range_t(27.99))(range_t(29.01))
         (range_t(30.17))
         ))
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
        if (verbosity>2) printf("umtrx_lms6002d_dev::write_reg(addr=0x%x, data=0x%x)\n", addr, data);
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        _spiface->write_spi(_slaveno, spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected
        uint8_t data = _spiface->read_spi(_slaveno, spi_config_t::EDGE_RISE, addr << 8, 16);
        if (verbosity>2) printf("umtrx_lms6002d_dev::read_reg(addr=0x%x) data=0x%x\n", addr, data);
        return data;
    }
};

// LMS6002D virtual daughter board for UmTRX

class lms6002d_ctrl_impl : public lms6002d_ctrl {
public:
    lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate);

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

    uhd::sensor_value_t get_rx_pll_locked()
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return uhd::sensor_value_t("LO", lms.get_rx_pll_locked(), "locked", "unlocked");
    }

    uhd::sensor_value_t get_tx_pll_locked()
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return uhd::sensor_value_t("LO", lms.get_tx_pll_locked(), "locked", "unlocked");
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
        boost::recursive_mutex::scoped_lock l(_mutex);
        return lms.get_tx_vga1dc_i_int();
    }

    uint8_t get_tx_vga1dc_q_int(void)
    {
        boost::recursive_mutex::scoped_lock l(_mutex);
        return lms.get_tx_vga1dc_q_int();
    }

protected:

    double set_freq(dboard_iface::unit_t unit, double f) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq(%f)\n", f);
        unsigned ref_freq = _clock_rate;
        double actual_freq = 0;
        if (unit==dboard_iface::UNIT_TX) {
            actual_freq = lms.tx_pll_tune(ref_freq, f);
        } else if (unit==dboard_iface::UNIT_RX) {
            actual_freq = lms.rx_pll_tune(ref_freq, f);
        } else {
            assert(!"Wrong units_t value passed to lms6002d_ctrl_impl::set_freq()");
        }
        if (actual_freq<0)
            actual_freq = 0;
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_freq() actual_freq=%f\n", actual_freq);
        return actual_freq;
    }

    bool set_enabled(dboard_iface::unit_t unit, bool en) {
        boost::recursive_mutex::scoped_lock l(_mutex);
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
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_gain(%f, %s)\n", gain, name.c_str());
        assert_has(lms_rx_gain_ranges.keys(), name, "LMS6002D rx gain name");
        if(name == "VGA1"){
            lms.set_rx_vga1gain(gain);
            return lms.get_rx_vga1gain();
        } else if(name == "VGA2"){
            lms.set_rx_vga2gain(gain);
            return lms.get_rx_vga2gain();
        } else UHD_THROW_INVALID_CODE_PATH();
        return gain;
    }

    void set_rx_ant(const std::string &ant) {
        boost::recursive_mutex::scoped_lock l(_mutex);
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
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        lms.set_rx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

    double set_tx_gain(double gain, const std::string &name) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_gain(%f, %s)\n", gain, name.c_str());
        //validate input
        assert_has(lms_tx_gain_ranges.keys(), name, "LMS6002D tx gain name");

        if (name == "VGA1") {
            if (verbosity>1) printf("db_lms6002d::set_tx_gain() VGA1=%d\n", int(gain));
            lms.set_tx_vga1gain(int(gain));
            return lms.get_tx_vga1gain();
        } else if (name == "VGA2") {
            if (verbosity>1) printf("db_lms6002d::set_tx_gain() VGA2=%d\n", int(gain));
            lms.set_tx_vga2gain(int(gain));
            return lms.get_tx_vga2gain();
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        // Return combined gain
        return tx_vga1gain+tx_vga2gain;
    }

    void set_tx_ant(const std::string &ant) {
        boost::recursive_mutex::scoped_lock l(_mutex);
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
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        lms.set_tx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

    uint8_t _set_tx_vga1dc_i_int(uint8_t offset) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_i_int(%d)\n", offset);
        lms.set_tx_vga1dc_i_int(offset);
        return offset;
    }

    uint8_t _set_tx_vga1dc_q_int(uint8_t offset) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_tx_vga1dc_q_int(%d)\n", offset);
        lms.set_tx_vga1dc_q_int(offset);
        return offset;
    }

    void set_rxfe_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxfe_dc_i(%d)\n", value);
        lms.set_rxfe_dc_i(value);
    }

    uint8_t get_rxfe_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxfe_dc_i()\n");
        return lms.get_rxfe_dc_i();
    }

    void set_rxfe_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxfe_dc_q(%d)\n", value);
        lms.set_rxfe_dc_q(value);
    }

    uint8_t get_rxfe_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxfe_dc_q()\n");
        return lms.get_rxfe_dc_q();
    }

    void set_rxlpf_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxlpf_dc_i(%d)\n", value);
        lms.set_rxlpf_dc_i(value);
    }

    uint8_t get_rxlpf_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxlpf_dc_i()\n");
        return lms.get_rxlpf_dc_i();
    }

    void set_rxlpf_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxlpf_dc_q(%d)\n", value);
        lms.set_rxlpf_dc_q(value);
    }

    uint8_t get_rxlpf_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxlpf_dc_q()\n");
        return lms.get_rxlpf_dc_q();
    }

    void set_rxvga2_dc_reference(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxvga2_dc_reference(%d)\n", value);
        lms.set_rxvga2_dc_reference(value);
    }

    uint8_t get_rxvga2_dc_reference() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxvga2_dc_reference()\n");
        return lms.get_rxvga2_dc_reference();
    }

    void set_rxvga2a_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxvga2a_dc_i(%d)\n", value);
        lms.set_rxvga2a_dc_i(value);
    }

    uint8_t get_rxvga2a_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxvga2a_dc_i()\n");
        return lms.get_rxvga2a_dc_i();
    }

    void set_rxvga2a_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxvga2a_dc_q(%d)\n", value);
        lms.set_rxvga2a_dc_q(value);
    }

    uint8_t get_rxvga2a_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxvga2a_dc_q()\n");
        return lms.get_rxvga2a_dc_q();
    }

    void set_rxvga2b_dc_i(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxvga2b_dc_i(%d)\n", value);
        lms.set_rxvga2b_dc_i(value);
    }

    uint8_t get_rxvga2b_dc_i() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxvga2b_dc_i()\n");
        return lms.get_rxvga2b_dc_i();
    }

    void set_rxvga2b_dc_q(uint8_t value) {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::set_rxvga2b_dc_q(%d)\n", value);
        lms.set_rxvga2b_dc_q(value);
    }

    uint8_t get_rxvga2b_dc_q() {
        boost::recursive_mutex::scoped_lock l(_mutex);
        if (verbosity>0) printf("lms6002d_ctrl_impl::get_rxvga2b_dc_q()\n");
        return lms.get_rxvga2b_dc_q();
    }

private:
    umtrx_lms6002d_dev lms;        // Interface to the LMS chip.
    int tx_vga1gain, tx_vga2gain;  // Stored values of Tx VGA1 and VGA2 gains.
    bool rf_loopback_enabled;      // Whether RF loopback is enabled.

    uhd::spi_iface::sptr _spiface;
    const int _lms_spi_number;
    const double _clock_rate;

    boost::recursive_mutex _mutex;
};

lms6002d_ctrl::sptr lms6002d_ctrl::make(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate)
{
    return sptr(new lms6002d_ctrl_impl(spiface, lms_spi_number, clock_rate));
}

// LMS RX dboard configuration

lms6002d_ctrl_impl::lms6002d_ctrl_impl(uhd::spi_iface::sptr spiface, const int lms_spi_number, const double clock_rate) :
                                             lms(umtrx_lms6002d_dev(spiface, lms_spi_number)),
                                             tx_vga1gain(lms.get_tx_vga1gain()),
                                             tx_vga2gain(lms.get_tx_vga2gain()),
                                             rf_loopback_enabled(false),
                                             _spiface(spiface),
                                             _lms_spi_number(lms_spi_number),
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

