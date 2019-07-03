//
// Copyright 2010 Ettus Research LLC
// Copyright 2012-1015 Fairwaves, Inc
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

#include "usrp_cal_utils.hpp"
#include <uhd/utils/safe_main.hpp>
#include <boost/ref.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/math/special_functions/round.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include <ctime>

namespace po = boost::program_options;

/***********************************************************************
 * Calibration utility class
 **********************************************************************/
class dc_cal_t {
public:
    dc_cal_t(uhd::property<uint8_t> &dc_i_prop, uhd::property<uint8_t> &dc_q_prop,
             uhd::rx_streamer::sptr rx_stream,
             const size_t nsamps,
             double bb_dc_freq,
             double rx_rate,
             int verbose,
             bool debug_raw_data,
             int init_dc_i=128, int init_dc_q=128);

    double init();
    void run_q(int dc_q);
    void run_i(int dc_i);
    void run_iq(int dc_i, int dc_q);

    void set_dc_i(double i) {prop_set_check(_dc_i_prop, i);}
    void set_dc_q(double q) {prop_set_check(_dc_q_prop, q);}
    void set_dc_i_best() {set_dc_i(_best_dc_i);}
    void set_dc_q_best() {set_dc_q(_best_dc_q);}

    double get_lowest_offset() const {return _lowest_offset;}
    int get_best_dc_i() const {return _best_dc_i;}
    int get_best_dc_q() const {return _best_dc_q;}

protected:
    double _lowest_offset;
    int _best_dc_i;
    int _best_dc_q;
    uhd::property<uint8_t> &_dc_i_prop;
    uhd::property<uint8_t> &_dc_q_prop;

    uhd::rx_streamer::sptr _rx_stream;
    std::vector<samp_type> _buff;
    const size_t _nsamps;
    double _bb_dc_freq;
    double _rx_rate;
    int _verbose;
    bool _debug_raw_data;

    void prop_set_check(uhd::property<uint8_t> &prop, uint8_t val);

    double get_dbrms();
    bool run_x();
};

dc_cal_t::dc_cal_t(uhd::property<uint8_t> &dc_i_prop, uhd::property<uint8_t> &dc_q_prop,
                   uhd::rx_streamer::sptr rx_stream,
                   const size_t nsamps,
                   double bb_dc_freq,
                   double rx_rate,
                   int verbose,
                   bool debug_raw_data,
                   int init_dc_i,
                   int init_dc_q)
    : _best_dc_i(init_dc_i), _best_dc_q(init_dc_q)
    , _dc_i_prop(dc_i_prop), _dc_q_prop(dc_q_prop)
    , _rx_stream(rx_stream)
    , _nsamps(nsamps)
    , _bb_dc_freq(bb_dc_freq)
    , _rx_rate(rx_rate)
    , _verbose(verbose)
    , _debug_raw_data(debug_raw_data)
{
}

double dc_cal_t::init()
{
    set_dc_i_best();
    set_dc_q_best();

    //get the DC offset tone size
    _lowest_offset = get_dbrms();

    if (_verbose) printf("initial_dc_dbrms = %2.0f dB\n", _lowest_offset);
    if (_debug_raw_data) write_samples_to_file(_buff, "initial_samples.dat");

    return _lowest_offset;
}

void dc_cal_t::run_q(int dc_q)
{
    if (_verbose) printf("      dc_q = %d", dc_q);
    set_dc_q(dc_q);
    if (run_x())
        _best_dc_q = dc_q;
}

void dc_cal_t::run_i(int dc_i)
{
    if (_verbose) printf("      dc_i = %d", dc_i);
    set_dc_i(dc_i);
    if (run_x())
        _best_dc_i = dc_i;
}

void dc_cal_t::run_iq(int dc_i, int dc_q)
{
    if (_verbose) printf("      dc_i = %d dc_q = %d", dc_i, dc_q);
    set_dc_i(dc_i);
    set_dc_q(dc_q);
    if (run_x()) {
        _best_dc_i = dc_i;
        _best_dc_q = dc_q;
    }
}

void dc_cal_t::prop_set_check(uhd::property<uint8_t> &prop, uint8_t val)
{
    prop.set(val);
    uint8_t val_read = prop.get();
    if (val_read != val)
        throw std::runtime_error(
            str(boost::format("Calibration property sets incorrectly. Requested %d, read back %d")
                          % int(val) % int(val_read)));
}

double dc_cal_t::get_dbrms()
{
    //receive some samples
    capture_samples(_rx_stream, _buff, _nsamps);
    //calculate dB rms
    return compute_tone_dbrms(_buff, _bb_dc_freq/_rx_rate);
}

bool dc_cal_t::run_x()
{
    bool better = false;

    //get the DC offset tone size
    const double dc_dbrms = get_dbrms();
    if (_verbose) printf("    dc_dbrms = %2.0f dB", dc_dbrms);

    if (dc_dbrms < _lowest_offset){
        _lowest_offset = dc_dbrms;
        better = true;
        if (_verbose) printf("    *");
        if (_debug_raw_data) write_samples_to_file(_buff, "best_samples.dat");
    }
    if (_verbose) printf("\n");

    return better;
}

/***********************************************************************
 * Calibration method: Downhill
 **********************************************************************/
static result_t calibrate_downhill(dc_cal_t &dc_cal,
                                   double tx_lo,
                                   int verbose)
{
    //bounds and results from searching
    int dc_i_start, dc_i_stop, dc_i_step;
    int dc_q_start, dc_q_stop, dc_q_step;

    //capture initial uncorrected value
    const double initial_dc_dbrms = dc_cal.init();

    for (size_t i = 0; i < 6; i++)
    {
        if (verbose) printf("  iteration %ld  best_i = %d  best_q = %d\n", i, dc_cal.get_best_dc_i(), dc_cal.get_best_dc_q());

        switch (i) {
        case 0:
            dc_i_start = 0;
            dc_i_stop  = 256;
            dc_q_start = 0;
            dc_q_stop  = 256;
            dc_i_step = 10;
            dc_q_step = 10;
            break;
        case 1:
            dc_i_start = dc_cal.get_best_dc_i() - 15;
            dc_i_stop  = dc_cal.get_best_dc_i() + 15;
            dc_q_start = dc_cal.get_best_dc_q() - 15;
            dc_q_stop  = dc_cal.get_best_dc_q() + 15;
            dc_i_step = 1;
            dc_q_step = 1;
            break;
        case 2:
        case 3:
            dc_i_start = dc_cal.get_best_dc_i() - 3;
            dc_i_stop  = dc_cal.get_best_dc_i() + 3;
            dc_q_start = dc_cal.get_best_dc_q() - 3;
            dc_q_stop  = dc_cal.get_best_dc_q() + 3;
            dc_i_step = 1;
            dc_q_step = 1;
            break;
        default:
            dc_i_start = dc_cal.get_best_dc_i() - 1;
            dc_i_stop  = dc_cal.get_best_dc_i() + 1;
            dc_q_start = dc_cal.get_best_dc_q() - 1;
            dc_q_stop  = dc_cal.get_best_dc_q() + 1;
            dc_i_step = 1;
            dc_q_step = 1;
            break;
        };

        if (i <= 2) {
            // Itereate through I and Q sequentially

            if (verbose) printf("    I in [%d; %d] step %d Q = %d\n",
                                            dc_i_start, dc_i_stop, dc_i_step, dc_cal.get_best_dc_q());
            dc_cal.set_dc_q_best();
            for (int dc_i = dc_i_start; dc_i <= dc_i_stop; dc_i += dc_i_step){
                dc_cal.run_i(dc_i);
            }

            if (verbose) printf("    I = %d Q in [%d; %d] step %d\n",
                                            dc_cal.get_best_dc_i(), dc_q_start, dc_q_stop, dc_q_step);
            dc_cal.set_dc_i_best();
            for (int dc_q = dc_q_start; dc_q <= dc_q_stop; dc_q += dc_q_step){
                dc_cal.run_q(dc_q);
            }
        } else {
            // Itereate through all combinations of I and Q

            if (verbose) printf("    I in [%d; %d] step %d Q in [%d; %d] step %d\n",
                                dc_i_start, dc_i_stop, dc_i_step,
                                dc_q_start, dc_q_stop, dc_q_step);
            for (int dc_i = dc_i_start; dc_i <= dc_i_stop; dc_i += dc_i_step) {
                for (int dc_q = dc_q_start; dc_q <= dc_q_stop; dc_q += dc_q_step) {
                    dc_cal.run_iq(dc_i, dc_q);
                }
            }
        }

    }

    // Calibration result
    result_t result;
    result.freq = tx_lo;
    result.real_corr = dc_cal.get_best_dc_i();
    result.imag_corr = dc_cal.get_best_dc_q();
    result.best = dc_cal.get_lowest_offset();
    result.delta = initial_dc_dbrms - result.best;

    // Output to console
    std::cout
        << result.freq/1e6 << " MHz "
        << "I/Q = " << result.real_corr << "/" << result.imag_corr << " "
        << "(" << dc_offset_int2double(result.real_corr) << "/"
        <<  dc_offset_int2double(result.imag_corr) << ") "
        << "leakage = " << result.best << " dB, "
        << "improvement = " << result.delta << " dB\n"
        << std::flush
    ;

    return result;
}

/***********************************************************************
 * Main
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    std::string args, which, serial;
    int verbose;
    int vga1_gain, vga2_gain, rx_gain;
    double tx_wave_freq, tx_wave_ampl, rx_offset;
    double freq_start, freq_stop, freq_step;
    size_t nsamps;
    size_t ntrials;
    int single_test_i, single_test_q;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("verbose", "enable some verbose")
        ("debug_raw_data", "save raw captured signals to files")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args [default = \"\"]")
        ("which", po::value<std::string>(&which)->default_value("A"), "Which chain A or B?")
        ("vga1", po::value<int>(&vga1_gain)->default_value(-20), "LMS6002D Tx VGA1 gain [-35 to -4]")
        ("vga2", po::value<int>(&vga2_gain)->default_value(22), "LMS6002D Tx VGA2 gain [0 to 25]")
        ("rx_gain", po::value<int>(&rx_gain)->default_value(50), "LMS6002D Rx combined gain [0 to 156]")
        ("tx_wave_freq", po::value<double>(&tx_wave_freq)->default_value(50e3), "Transmit wave frequency in Hz")
        ("tx_wave_ampl", po::value<double>(&tx_wave_ampl)->default_value(0.7), "Transmit wave amplitude in counts")
        ("rx_offset", po::value<double>(&rx_offset)->default_value(300e3), "RX LO offset from the TX LO in Hz")
        ("freq_start", po::value<double>(&freq_start), "Frequency start in Hz (do not specify for default)")
        ("freq_stop", po::value<double>(&freq_stop), "Frequency stop in Hz (do not specify for default)")
        ("freq_step", po::value<double>(&freq_step)->default_value(default_freq_step), "Step size for LO sweep in Hz")
        ("nsamps", po::value<size_t>(&nsamps)->default_value(default_num_samps), "Samples per data capture")
        ("ntrials", po::value<size_t>(&ntrials)->default_value(1), "Num trials per TX LO")
        ("single_test", "Perform a single measurement and exit (freq = freq_start, I = single_test_i, Q = single_test_q]")
        ("single_test_i", po::value<int>(&single_test_i)->default_value(128), "Only in the single test mode! I channel calibration value [0 to 255]")
        ("single_test_q", po::value<int>(&single_test_q)->default_value(128), "Only in the single test mode! Q channel calibration value [0 to 255]")
        ("append", "Append measurements to the calibratoin file instead of rewriting [default=overwrite]")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UmTRX Generate TX DC Offset Calibration Table %s") % desc << std::endl;
        std::cout <<
            "This application measures leakage between RX and TX using LMS6002D internal RF loopback to self-calibrate.\n"
            << std::endl;
        return EXIT_FAILURE;
    }

    verbose = vm.count("verbose");

    // Create a USRP device
    uhd::usrp::multi_usrp::sptr usrp = setup_usrp_for_cal(args, which, serial, vga1_gain, vga2_gain, rx_gain, verbose);

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //create a transmitter thread
    std::atomic<bool> interrupted(false);
    boost::thread_group threads;
    threads.create_thread(boost::bind(&tx_thread, usrp, tx_wave_freq, tx_wave_ampl, boost::ref(interrupted)));

    //store the results here
    std::vector<result_t> results;

    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    const uhd::fs_path tx_fe_path = "/mboards/0/dboards/"+which+"/tx_frontends/0";
    uhd::property<uint8_t> &dc_i_prop = tree->access<uint8_t>(tx_fe_path / "lms6002d/tx_dc_i/value");
    uhd::property<uint8_t> &dc_q_prop = tree->access<uint8_t>(tx_fe_path / "lms6002d/tx_dc_q/value");

    if (not vm.count("freq_start")) freq_start = usrp->get_tx_freq_range().start() + 50e6;
    if (not vm.count("freq_stop")) freq_stop = usrp->get_tx_freq_range().stop() - 50e6;
    UHD_MSG(status) << boost::format("Calibration frequency type: DC offset") << std::endl;
    UHD_MSG(status) << boost::format("Calibration frequency range: %d MHz -> %d MHz") % (freq_start/1e6) % (freq_stop/1e6) << std::endl;

    for (double tx_lo_i = freq_start; tx_lo_i <= freq_stop; tx_lo_i += freq_step){
        const double tx_lo = tune_rx_and_tx(usrp, tx_lo_i, rx_offset);

        //frequency constants for this tune event
        const double actual_rx_rate = usrp->get_rx_rate();
        const double actual_tx_freq = usrp->get_tx_freq();
        const double actual_rx_freq = usrp->get_rx_freq();
        const double bb_dc_freq = actual_tx_freq - actual_rx_freq;
        if (verbose) printf("actual_rx_rate = %0.2f MHz\n", actual_rx_rate/1e6);
        if (verbose) printf("actual_tx_freq = %0.2f MHz\n", actual_tx_freq/1e6);
        if (verbose) printf("actual_rx_freq = %0.2f MHz\n", actual_rx_freq/1e6);
        if (verbose) printf("bb_dc_freq = %0.2f MHz\n", bb_dc_freq/1e6);

        for (size_t trial_no = 0; trial_no < ntrials; trial_no++)
        {
            if (vm.count("single_test"))
            {
                dc_cal_t dc_cal(dc_i_prop, dc_q_prop,
                                rx_stream,
                                nsamps,
                                bb_dc_freq,
                                actual_rx_rate,
                                verbose,
                                vm.count("debug_raw_data"),
                                single_test_i, single_test_q);

                const double dc_dbrms = dc_cal.init();;
                printf("I = %d Q = %d ", single_test_i, single_test_q);
                printf("dc_dbrms = %2.1f dB\n", dc_dbrms);
            } else {
                dc_cal_t dc_cal(dc_i_prop, dc_q_prop,
                                rx_stream,
                                nsamps,
                                bb_dc_freq,
                                actual_rx_rate,
                                verbose,
                                vm.count("debug_raw_data"));
                // Perform normal calibration
                results.push_back(calibrate_downhill(dc_cal, tx_lo, verbose));
            }
        }
    }
    std::cout << std::endl;

    //stop the transmitter
    interrupted = true;
    threads.join_all();

    if (not vm.count("single_test"))
        store_results(usrp, results, "tx", "dc", vm.count("append"));

    return EXIT_SUCCESS;
}
