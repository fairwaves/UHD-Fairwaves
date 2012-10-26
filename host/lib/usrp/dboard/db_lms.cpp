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

#include "../umtrx/umtrx_impl.hpp"

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

/***********************************************************************
 * The LMS6002D dboard constants
 **********************************************************************/
static const freq_range_t lms_freq_range(0.2325e9, 3.72e9);

//Multiplied by 2 for conversion to complex bandpass from lowpass
static const freq_range_t lms_bandwidth_range = list_of
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
//    ("VGA1", gain_range_t(-35, -4, double(1.0)))
//    ("VGA2", gain_range_t(0, 25, double(1.0)))
    // Use a single control to manually control how VGA1/VGA2 are set
    ("VGA", gain_range_t(-35+0, -4+25, double(1.0)))
;

static const uhd::dict<std::string, gain_range_t> lms_rx_gain_ranges = map_list_of
    // We limit Rx VGA2 to 30dB, as docs say higher values are dangerous
    ("VGA2", gain_range_t(0, 30, double(3.0)))
// ToDo: Add LNA control here
;


static int verbosity = 0;

// LMS boards for UMTRX

class lms_rx : public rx_dboard_base, public lms6002d_dev {
public:
    lms_rx(ctor_args_t args);

    virtual void write_reg(uint8_t addr, uint8_t data) {
        if (verbosity>2) printf("lms_rx::write_reg(addr=0x%x, data=0x%x)\n", addr, data);
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        this->get_iface()->write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected
        uint8_t data = this->get_iface()->read_write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, addr << 8, 16);
        if (verbosity>2) printf("lms_tx::read_reg(addr=0x%x) data=0x%x\n", addr, data);
        return data;
    }


    double set_freq(double f) {
        if (verbosity>0) printf("lms_rx::set_freq(%f)\n", f);
        if (this->rx_pll_tune(26e6, f))
            return f;
        //dump();
        return 0;
    }

    bool set_enabled(bool en) {
        if (verbosity>0) printf("lms_rx::set_enabled(%d)\n", en);
        if (en)
            rx_enable();
        else
            rx_disable();
        //dump();
        return en;
    }

    double set_rx_gain(double gain, const std::string &name) {
        if (verbosity>0) printf("lms_rx::set_rx_gain(%f, %s)\n", gain, name.c_str());
        assert_has(lms_rx_gain_ranges.keys(), name, "LMS6002D rx gain name");
        if(name == "VGA2"){
            set_rx_vga2gain(gain);
        } else UHD_THROW_INVALID_CODE_PATH();
        return gain;
    }

    void set_rx_ant(const std::string &ant) {
        if (verbosity>0) printf("lms_rx::set_rx_ant(%s)\n", ant.c_str());
        //validate input
        assert_has(lms_rx_antennas, ant, "LMS6002D rx antenna name");

        if (ant == "RX0") {
            set_rx_lna(0);
        } else if (ant == "RX1") {
            set_rx_lna(1);
        } else if (ant == "RX2") {
            set_rx_lna(2);
        } else if (ant == "RX3") {
            set_rx_lna(3);
        } else if (ant == "CAL") {
            // TODO: Turn on loopback
            set_rx_lna(0);
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    }

    double set_rx_bandwidth(double bandwidth) {
        if (verbosity>0) printf("lms_tx::set_rx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        set_rx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

};

class lms_tx : public tx_dboard_base, public lms6002d_dev {
public:
    lms_tx(ctor_args_t args);

    virtual void write_reg(uint8_t addr, uint8_t data) {
        if (verbosity>2) printf("lms_tx::write_reg(addr=0x%x, data=0x%x)\n", addr, data);
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        this->get_iface()->write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected
        uint8_t data = this->get_iface()->read_write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, addr << 8, 16);
        if (verbosity>2) printf("lms_tx::read_reg(addr=0x%x) data=0x%x\n", addr, data);
        return data;

    }

    double set_freq(double f) {
        if (verbosity>2) printf("lms_tx::set_freq(%f)\n", f);
        if (this->tx_pll_tune(26e6, f))
            return f;
        //dump();
        return 0;
    }

    bool set_enabled(bool en) {
        if (verbosity>0) printf("lms_tx::set_enabled(%d)\n", en);
        if (en)
            tx_enable();
        else
            tx_disable();
        //dump();
        return en;
    }

    double set_tx_gain(double gain, const std::string &name) {
        if (verbosity>0) printf("lms_tx::set_tx_gain(%f, %s)\n", gain, name.c_str());
        //validate input
        assert_has(lms_tx_gain_ranges.keys(), name, "LMS6002D tx gain name");

        if (name == "VGA") {
            // Calculate the best combination of VGA1 and VGA2 gains.
            // For simplicity we just try to use VGA2 as much as possible
            // and only then engage VGA1.
            int desired_vga2 = int(gain) - vga1gain;
            if (desired_vga2 < 0)
                vga2gain = 0;
            else if (desired_vga2 > 25)
                vga2gain = 25;
            else
                vga2gain = desired_vga2;
            vga1gain = int(gain) - vga2gain;

            // Set the gains in hardware
            if (verbosity>1) printf("lms_tx::set_tx_gain() VGA1=%d VGA2=%d\n", vga1gain, vga2gain);
            set_tx_vga1gain(vga1gain);
            set_tx_vga2gain(vga2gain);
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }

        // Return combined gain
        return vga1gain+vga2gain;
    }

    void set_tx_ant(const std::string &ant) {
        if (verbosity>0) printf("lms_tx::set_tx_ant(%s)\n", ant.c_str());
        //validate input
        assert_has(lms_tx_antennas, ant, "LMS6002D tx antenna ant");

        if (ant == "TX0") {
            set_tx_pa(0);
        } else if (ant == "TX1") {
            set_tx_pa(1);
        } else if (ant == "TX2") {
            set_tx_pa(2);
        } else if (ant == "CAL") {
            // TODO: Turn on loopback
            set_tx_pa(0);
        } else {
            UHD_THROW_INVALID_CODE_PATH();
        }
    }

    double set_tx_bandwidth(double bandwidth) {
        if (verbosity>0) printf("lms_tx::set_tx_bandwidth(%f)\n", bandwidth);
        // Get the closest available bandwidth
        bandwidth = lms_bandwidth_range.clip(bandwidth);

        // convert complex bandpass to lowpass bandwidth and to kHz
        set_tx_lpf(int(bandwidth/2/1e3));

        return bandwidth;
    }

    uint8_t _set_tx_vga1dc_i_int(uint8_t offset) {
        if (verbosity>0) printf("lms_tx::set_tx_vga1dc_i_int(%d)\n", offset);
        set_tx_vga1dc_i_int(offset);
        return offset;
    }

    uint8_t _set_tx_vga1dc_q_int(uint8_t offset) {
        if (verbosity>0) printf("lms_tx::set_tx_vga1dc_q_int(%d)\n", offset);
        set_tx_vga1dc_q_int(offset);
        return offset;
    }

private:
    int vga1gain, vga2gain;  // Stored values of VGA1 and VGA2 gains.
};

// Register the LMS dboards

static dboard_base::sptr make_lms_rx(dboard_base::ctor_args_t args) {
    return dboard_base::sptr(new lms_rx(args));
}

static dboard_base::sptr make_lms_tx(dboard_base::ctor_args_t args) {
    return dboard_base::sptr(new lms_tx(args));
}

UHD_STATIC_BLOCK(reg_lms_dboards){
    dboard_manager::register_dboard(0xfa07, &make_lms_tx, "LMS TX");
    dboard_manager::register_dboard(0xfa09, &make_lms_rx, "LMS RX");
}

// LMS RX dboard configuration

lms_rx::lms_rx(ctor_args_t args) : rx_dboard_base(args){ // Register properties
    this->get_rx_subtree()->create<std::string>("name")
        .set(std::string(str(boost::format("%s - %s") % get_rx_id().to_pp_string() % get_subdev_name())));

    BOOST_FOREACH(const std::string &name, lms_rx_gain_ranges.keys()){
        this->get_rx_subtree()->create<double>("gains/"+name+"/value")
            .coerce(boost::bind(&lms_rx::set_rx_gain, this, _1, name))
            .set((lms_rx_gain_ranges[name].start()+lms_rx_gain_ranges[name].start())/2.0);
        this->get_rx_subtree()->create<meta_range_t>("gains/"+name+"/range")
            .set(lms_rx_gain_ranges[name]);
    }

    this->get_rx_subtree()->create<double>("freq/value")
        .coerce(boost::bind(&lms_rx::set_freq, this, _1));
    this->get_rx_subtree()->create<meta_range_t>("freq/range")
        .set(lms_freq_range);

    this->get_rx_subtree()->create<std::string>("antenna/value")
        .subscribe(boost::bind(&lms_rx::set_rx_ant, this, _1))
        .set("RX3");
    this->get_rx_subtree()->create<std::vector<std::string> >("antenna/options")
        .set(lms_rx_antennas);
    this->get_rx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_rx_subtree()->create<std::string>("connection").set("IQ");
    this->get_rx_subtree()->create<bool>("enabled")
        .coerce(boost::bind(&lms_rx::set_enabled, this, _1));

    this->get_rx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_rx_subtree()->create<double>("bandwidth/value")
        .coerce(boost::bind(&lms_rx::set_rx_bandwidth, this, _1))
        .set(double(2*0.75e6));
    this->get_rx_subtree()->create<meta_range_t>("bandwidth/range")
        .set(lms_bandwidth_range);
}

// LMS TX dboard configuration

lms_tx::lms_tx(ctor_args_t args) : tx_dboard_base(args),
                                   vga1gain(get_tx_vga1gain()),
                                   vga2gain(get_tx_vga2gain())
{ // Register properties
    this->get_tx_subtree()->create<std::string>("name")
        .set(std::string(str(boost::format("%s - %s") % get_tx_id().to_pp_string() % get_subdev_name())));

    BOOST_FOREACH(const std::string &name, lms_tx_gain_ranges.keys()){
        this->get_tx_subtree()->create<double>("gains/"+name+"/value")
            .coerce(boost::bind(&lms_tx::set_tx_gain, this, _1, name))
            .set((lms_tx_gain_ranges[name].start()+lms_tx_gain_ranges[name].start())/2.0);
        this->get_tx_subtree()->create<meta_range_t>("gains/"+name+"/range")
            .set(lms_tx_gain_ranges[name]);
    }

    this->get_tx_subtree()->create<double>("freq/value")
        .coerce(boost::bind(&lms_tx::set_freq, this, _1));
    this->get_tx_subtree()->create<meta_range_t>("freq/range")
        .set(lms_freq_range);

    this->get_tx_subtree()->create<std::string>("antenna/value")
        .subscribe(boost::bind(&lms_tx::set_tx_ant, this, _1))
        .set("TX2");
    this->get_tx_subtree()->create<std::vector<std::string> >("antenna/options")
        .set(lms_tx_antennas);
    this->get_tx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_tx_subtree()->create<std::string>("connection").set("IQ");
    this->get_tx_subtree()->create<bool>("enabled")
        .coerce(boost::bind(&lms_tx::set_enabled, this, _1));

    this->get_tx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_tx_subtree()->create<double>("bandwidth/value")
        .coerce(boost::bind(&lms_tx::set_tx_bandwidth, this, _1))
        .set(double(2*0.75e6));
    this->get_tx_subtree()->create<meta_range_t>("bandwidth/range")
        .set(lms_bandwidth_range);

    // UmTRX specific calibration
    this->get_tx_subtree()->create<uint8_t>("cal/dc_i/value")
        .subscribe(boost::bind(&lms_tx::_set_tx_vga1dc_i_int, this, _1))
        .publish(boost::bind(&lms_tx::get_tx_vga1dc_i_int, this));
    this->get_tx_subtree()->create<uint8_t>("cal/dc_q/value")
        .subscribe(boost::bind(&lms_tx::_set_tx_vga1dc_q_int, this, _1))
        .publish(boost::bind(&lms_tx::get_tx_vga1dc_q_int, this));
}

