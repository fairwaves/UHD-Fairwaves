#include <uhd/types/ranges.hpp>
#include <uhd/utils/assert_has.hpp>
#include <uhd/utils/static.hpp>
#include <uhd/utils/msg.hpp>
#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/foreach.hpp>
#include <boost/tuple/tuple.hpp>
#include <vector>

using namespace uhd;
using namespace uhd::usrp;
using namespace boost::assign;

// LMS boards for UMTRX

class lms_rx : public rx_dboard_base {
public:
    lms_rx(ctor_args_t args);
};

class lms_tx : public tx_dboard_base {
public:
    lms_tx(ctor_args_t args);
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
    this->get_rx_subtree()->create<double>("freq/value").set(double(0.0));
    this->get_rx_subtree()->create<meta_range_t>("freq/range").set(freq_range_t(double(0.0), double(0.0)));
    this->get_rx_subtree()->create<std::string>("antenna/value").set("");
    this->get_rx_subtree()->create<std::vector<std::string> >("antenna/options").set(list_of(""));
    this->get_rx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_rx_subtree()->create<std::string>("connection").set("IQ");
    this->get_rx_subtree()->create<bool>("enabled").set(true); //always enabled
    this->get_rx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_rx_subtree()->create<double>("bandwidth/value").set(double(0.0));
    this->get_rx_subtree()->create<meta_range_t>("bandwidth/range").set(freq_range_t(0.3, 3.8));
}

// LMS TX dboard configuration

lms_tx::lms_tx(ctor_args_t args) : tx_dboard_base(args) { // Register properties
    this->get_tx_subtree()->create<std::string>("name").set(std::string(str(boost::format("%s - %s") % get_tx_id().to_pp_string() % get_subdev_name())));
    this->get_tx_subtree()->create<int>("gains"); //phony property so this dir exists
    this->get_tx_subtree()->create<double>("freq/value").set(double(0.0));
    this->get_tx_subtree()->create<meta_range_t>("freq/range").set(freq_range_t(double(0.0), double(0.0)));
    this->get_tx_subtree()->create<std::string>("antenna/value").set("");
    this->get_tx_subtree()->create<std::vector<std::string> >("antenna/options").set(list_of(""));
    this->get_tx_subtree()->create<int>("sensors"); //phony property so this dir exists
    this->get_tx_subtree()->create<std::string>("connection").set("IQ");
    this->get_tx_subtree()->create<bool>("enabled").set(true); //always enabled
    this->get_tx_subtree()->create<bool>("use_lo_offset").set(false);
    this->get_tx_subtree()->create<double>("bandwidth/value").set(double(0.0));
    this->get_tx_subtree()->create<meta_range_t>("bandwidth/range").set(freq_range_t(0.3, 3.8));
}
