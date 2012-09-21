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

// LMS boards for UMTRX

class lms_rx : public rx_dboard_base, public lms6002d_dev {
public:
    lms_rx(ctor_args_t args);

    virtual void write_reg(uint8_t addr, uint8_t data) {
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        this->get_iface()->write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected

        return this->get_iface()->read_write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, addr << 8, 16);
    }


    double set_freq(double f) {
        printf("lms_rx = %f\n", f);
        if (this->rx_pll_tune(26e6, f))
            return f;
        //dump();
        return 0;
    }

    bool set_enabled(bool en) {
        printf("lms_rx_en = %d\n", en);
        if (en)
            rx_enable();
        else
            rx_disable();
        //dump();
        return en;
    }
};

class lms_tx : public tx_dboard_base, public lms6002d_dev {
public:
    lms_tx(ctor_args_t args);

    virtual void write_reg(uint8_t addr, uint8_t data) {
        uint16_t command = (((uint16_t)0x80 | (uint16_t)addr) << 8) | (uint16_t)data;
        this->get_iface()->write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, command, 16);
    }
    virtual uint8_t read_reg(uint8_t addr) {
        if(addr > 127) return 0; // incorrect address, 7 bit long expected

        return this->get_iface()->read_write_spi((uhd::usrp::dboard_iface::unit_t)1,
            spi_config_t::EDGE_RISE, addr << 8, 16);
    }

    double set_freq(double f) {
        printf("lms_tx = %f\n", f);
        if (this->tx_pll_tune(26e6, f))
            return f;
        //dump();
        return 0;
    }

    bool set_enabled(bool en) {
        printf("lms_tx_en = %d\n", en);
        if (en)
            tx_enable();
        else
            tx_disable();
        //dump();
        return en;
    }
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
    this->get_rx_subtree()->create<std::string>("name").set(std::string(str(boost::format("%s - %s") % get_rx_id().to_pp_string() % get_subdev_name())));
    this->get_rx_subtree()->create<int>("gains"); //phony property so this dir exists
    this->get_rx_subtree()->create<double>("freq/value")
        .coerce(boost::bind(&lms_rx::set_freq, this, _1));

    this->get_rx_subtree()->create<meta_range_t>("freq/range").set(freq_range_t(double(0.0), double(0.0)));
    this->get_rx_subtree()->create<std::string>("antenna/value").set("");
    this->get_rx_subtree()->create<std::vector<std::string> >("antenna/options").set(list_of(""));
    this->get_rx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_rx_subtree()->create<std::string>("connection").set("IQ");
    this->get_rx_subtree()->create<bool>("enabled")
        .coerce(boost::bind(&lms_rx::set_enabled, this, _1))
        .set(true);

    this->get_rx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_rx_subtree()->create<double>("bandwidth/value").set(double(0.0));
    this->get_rx_subtree()->create<meta_range_t>("bandwidth/range").set(freq_range_t(0.3, 3.8));
}

// LMS TX dboard configuration

lms_tx::lms_tx(ctor_args_t args) : tx_dboard_base(args) { // Register properties
    this->get_tx_subtree()->create<std::string>("name").set(std::string(str(boost::format("%s - %s") % get_tx_id().to_pp_string() % get_subdev_name())));
    this->get_tx_subtree()->create<int>("gains"); //phony property so this dir exists
    this->get_tx_subtree()->create<double>("freq/value")
        .coerce(boost::bind(&lms_tx::set_freq, this, _1));

    this->get_tx_subtree()->create<meta_range_t>("freq/range").set(freq_range_t(double(0.0), double(0.0)));
    this->get_tx_subtree()->create<std::string>("antenna/value").set("");
    this->get_tx_subtree()->create<std::vector<std::string> >("antenna/options").set(list_of(""));
    this->get_tx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_tx_subtree()->create<std::string>("connection").set("IQ");
    this->get_tx_subtree()->create<bool>("enabled")
        .coerce(boost::bind(&lms_tx::set_enabled, this, _1))
        .set(true);

    this->get_tx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_tx_subtree()->create<double>("bandwidth/value").set(double(0.0));
    this->get_tx_subtree()->create<meta_range_t>("bandwidth/range").set(freq_range_t(0.3, 3.8));
}
