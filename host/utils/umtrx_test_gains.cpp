//
// Copyright 2015 Fairwaves LLC
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

static size_t failCount = 0;

#define CHECK(expr, expected) { \
    const double actual = expr; \
    const bool ok = (actual == expected); \
    if (not ok) failCount++; \
    std::cout << "Check: " << #expr << " == " << #expected << "\t\t\t" << ((ok)?"OK":"FAIL") << std::endl; \
    if (not ok) std::cout << "\t FAIL: actual = " << actual << std::endl; }

/***********************************************************************
 * Main
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[])
{
    std::string args;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args [default = \"\"]")
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

    //get the spi-interface to check gain values
    uhd::spi_iface::sptr spiface = tree->access<uhd::spi_iface::sptr>(mb_path / "spi_iface").get();

    for (size_t ch = 0; ch <= 1; ch++)
    {
        std::cout << std::endl << "==== Testing channel: " << ch << std::endl;

        std::cout << std::endl << "Check RX gain ranges:" << std::endl;
        CHECK(usrp->get_rx_gain_range("VGA1", ch).start(), 0);
        CHECK(usrp->get_rx_gain_range("VGA1", ch).stop(), 126);
        CHECK(usrp->get_rx_gain_range("VGA2", ch).start(), 0);
        CHECK(usrp->get_rx_gain_range("VGA2", ch).stop(), 30);
        CHECK(usrp->get_rx_gain_range(ch).start(), 0);
        CHECK(usrp->get_rx_gain_range(ch).stop(), 156);

        std::cout << std::endl << "Check TX gain ranges:" << std::endl;
        CHECK(usrp->get_tx_gain_range("VGA1", ch).start(), -35);
        CHECK(usrp->get_tx_gain_range("VGA1", ch).stop(), -4);
        CHECK(usrp->get_tx_gain_range("VGA2", ch).start(), 0);
        CHECK(usrp->get_tx_gain_range("VGA2", ch).stop(), 25);
        CHECK(usrp->get_tx_gain_range(ch).start(), -35);
        CHECK(usrp->get_tx_gain_range(ch).stop(), 21);

        std::cout << std::endl << "Test RX gain distribution:" << std::endl;
        usrp->set_rx_gain(0, ch);
        CHECK(usrp->get_rx_gain(ch), 0);
        CHECK(usrp->get_rx_gain("VGA1", ch), 0);
        CHECK(usrp->get_rx_gain("VGA2", ch), 0);
        usrp->set_rx_gain(15, ch);
        CHECK(usrp->get_rx_gain(ch), 15);
        CHECK(usrp->get_rx_gain("VGA1", ch), 15);
        CHECK(usrp->get_rx_gain("VGA2", ch), 0);
        usrp->set_rx_gain(129, ch);
        CHECK(usrp->get_rx_gain(ch), 129);
        CHECK(usrp->get_rx_gain("VGA1", ch), 126);
        CHECK(usrp->get_rx_gain("VGA2", ch), 3);

        std::cout << std::endl << "Test TX gain distribution:" << std::endl;
        usrp->set_tx_gain(-35, ch);
        CHECK(usrp->get_tx_gain(ch), -35);
        CHECK(usrp->get_tx_gain("VGA2", ch), 0);
        CHECK(usrp->get_tx_gain("VGA1", ch), -35);
        usrp->set_tx_gain(-10, ch);
        CHECK(usrp->get_tx_gain(ch), -10);
        CHECK(usrp->get_tx_gain("VGA2", ch), 0);
        CHECK(usrp->get_tx_gain("VGA1", ch), -10);
        usrp->set_tx_gain(10, ch);
        CHECK(usrp->get_tx_gain(ch), 10);
        CHECK(usrp->get_tx_gain("VGA2", ch), 14);
        CHECK(usrp->get_tx_gain("VGA1", ch), -4);
    }

    //print status
    std::cout << std::endl;
    const bool fail = failCount > 0;
    if (fail) std::cerr << std::endl << failCount << " TESTS FAILED!!!" << std::endl;
    else std::cout << std::endl << "ALL TESTS PASSED" << std::endl;
    std::cout << "Done!" << std::endl;
    return fail?EXIT_FAILURE:EXIT_SUCCESS;
}
