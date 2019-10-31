//
// Copyright 2014 Fairwaves LLC
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

#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/paths.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/sensors.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <boost/random.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include <ctime>

namespace po = boost::program_options;


/***********************************************************************
 * Main
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    std::string args;
    unsigned pa_dcdc_r = 0;
    int divsw1 = -1;
    int divsw2 = -1;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "help message")
        ("verbose", "enable some verbose")
        ("debug_raw_data", "save raw captured signals to files")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args [default = \"\"]")
        ("dcdc_cal,C", "Calibrate DC/DC")
        ("paen1",      "Enable PA1")
        ("paen2",      "Enable PA2")
        ("padis1",     "Disable PA1")
        ("padis2",     "Disable PA2")
        ("divsw1",  po::value<int>(&divsw1), "1 - Enable / 0 - Disable DivSW1")
        ("divsw2",  po::value<int>(&divsw2), "1 - Enable / 0 - Disable DivSW2")
        ("palow",      "Turn off internal DC/DC")
        ("pa_dcdc_r",  po::value<unsigned>(&pa_dcdc_r),"Turn on internal DC/DC")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << desc << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    const uhd::fs_path mb_path = "/mboards/0";

    if (vm.count("paen1")) {
        tree->access<bool>(mb_path / "pa_en1").set(true);
    }
    if (vm.count("paen2")) {
        tree->access<bool>(mb_path / "pa_en2").set(true);
    }
    if (vm.count("palow")) {
        tree->access<bool>(mb_path / "pa_nlow").set(false);
    }
    if (vm.count("padis1")) {
        tree->access<bool>(mb_path / "pa_en1").set(false);
    }
    if (vm.count("padis2")) {
        tree->access<bool>(mb_path / "pa_en2").set(false);
    }
    if (vm.count("pa_dcdc_r")) {
        tree->access<bool>(mb_path / "pa_nlow").set(true);
        tree->access<uint8_t>(mb_path / "pa_dcdc_r").set(pa_dcdc_r);
    }

    if (vm.count("dcdc_cal")) {
        tree->access<bool>(mb_path / "pa_nlow").set(true);
        unsigned i;
        for (i = 0; i < 256; i++) {
            tree->access<uint8_t>(mb_path / "pa_dcdc_r").set(i);
            if (i == 0) {
                boost::this_thread::sleep(boost::posix_time::seconds(1));
            }

            boost::this_thread::sleep(boost::posix_time::milliseconds(100)); // Wait for value to settle

            std::cout << "[" << std::setw(3) << i << "]="
                      << tree->access<uhd::sensor_value_t>(mb_path / "sensors" / "voltageDCOUT").get().to_pp_string().c_str()
                      << std::endl;
        }
    }
    if (vm.count("divsw1")) {
        tree->access<bool>(mb_path / "dboards" / "A" / "rx_frontends" / "0" / "diversiy").set(divsw1 ? 1 : 0);
    }
    if (vm.count("divsw2")) {
        tree->access<bool>(mb_path / "dboards" / "B" / "rx_frontends" / "0" / "diversiy").set(divsw1 ? 1 : 0);
    }

    return 0;
}
