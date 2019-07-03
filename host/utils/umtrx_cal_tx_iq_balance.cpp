//
// Copyright 2010,2012 Ettus Research LLC
// Copyright 2015 Fairwaves, Inc
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
#include <boost/math/special_functions/round.hpp>
#include <iostream>
#include <complex>
#include <ctime>
#include <cstdlib>

namespace po = boost::program_options;

static const size_t num_search_steps = 5;
static const size_t num_search_iters = 7;

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

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("verbose", "enable some verbose")
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
        ("append", "Append measurements to the calibratoin file instead of rewriting [default=overwrite]")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UmTRX Generate TX IQ Balance Calibration Table %s") % desc << std::endl;
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

    //re-usable buffer for samples
    std::vector<samp_type> buff;

    //store the results here
    std::vector<result_t> results;

    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    const uhd::fs_path tx_fe_path = "/mboards/0/tx_frontends/"+which;
    uhd::property<std::complex<double> > &iq_prop = tree->access<std::complex<double> >(tx_fe_path / "iq_balance" / "value");

    if (not vm.count("freq_start")) freq_start = usrp->get_tx_freq_range().start() + 50e6;
    if (not vm.count("freq_stop")) freq_stop = usrp->get_tx_freq_range().stop() - 50e6;
    UHD_MSG(status) << boost::format("Calibration frequency type: IQ balance") << std::endl;
    UHD_MSG(status) << boost::format("Calibration frequency range: %d MHz -> %d MHz") % (freq_start/1e6) % (freq_stop/1e6) << std::endl;

    for (double tx_lo_i = freq_start; tx_lo_i <= freq_stop; tx_lo_i += freq_step){
        const double tx_lo = tune_rx_and_tx(usrp, tx_lo_i, rx_offset);

        //frequency constants for this tune event
        const double actual_rx_rate = usrp->get_rx_rate();
        const double actual_tx_freq = usrp->get_tx_freq();
        const double actual_rx_freq = usrp->get_rx_freq();
        const double bb_tone_freq = actual_tx_freq + tx_wave_freq - actual_rx_freq;
        const double bb_imag_freq = actual_tx_freq - tx_wave_freq - actual_rx_freq;

        //capture initial uncorrected value
        iq_prop.set(0.0);
        capture_samples(rx_stream, buff, nsamps);
        const double initial_suppression = compute_tone_dbrms(buff, bb_tone_freq/actual_rx_rate) - compute_tone_dbrms(buff, bb_imag_freq/actual_rx_rate);

        //bounds and results from searching
        std::complex<double> best_correction;
        double phase_corr_start = -.3, phase_corr_stop = .3, phase_corr_step;
        double ampl_corr_start = -.3, ampl_corr_stop = .3, ampl_corr_step;
        double best_suppression = 0, best_phase_corr = 0, best_ampl_corr = 0;

        for (size_t i = 0; i < num_search_iters; i++){

            phase_corr_step = (phase_corr_stop - phase_corr_start)/(num_search_steps-1);
            ampl_corr_step = (ampl_corr_stop - ampl_corr_start)/(num_search_steps-1);

            for (double phase_corr = phase_corr_start; phase_corr <= phase_corr_stop + phase_corr_step/2; phase_corr += phase_corr_step){
            for (double ampl_corr = ampl_corr_start; ampl_corr <= ampl_corr_stop + ampl_corr_step/2; ampl_corr += ampl_corr_step){

                const std::complex<double> correction(ampl_corr, phase_corr);
                iq_prop.set(correction);

                //receive some samples
                capture_samples(rx_stream, buff, nsamps);

                const double tone_dbrms = compute_tone_dbrms(buff, bb_tone_freq/actual_rx_rate);
                const double imag_dbrms = compute_tone_dbrms(buff, bb_imag_freq/actual_rx_rate);
                const double suppression = tone_dbrms - imag_dbrms;

                if (suppression > best_suppression){
                    best_correction = correction;
                    best_suppression = suppression;
                    best_phase_corr = phase_corr;
                    best_ampl_corr = ampl_corr;
                }

            }}

            if (verbose) std::cout << "best_phase_corr " << best_phase_corr << std::endl;
            if (verbose) std::cout << "best_ampl_corr " << best_ampl_corr << std::endl;
            if (verbose) std::cout << "best_suppression " << best_suppression << std::endl;

            phase_corr_start = best_phase_corr - phase_corr_step;
            phase_corr_stop = best_phase_corr + phase_corr_step;
            ampl_corr_start = best_ampl_corr - ampl_corr_step;
            ampl_corr_stop = best_ampl_corr + ampl_corr_step;
        }

        if (best_suppression > 30){ //most likely valid, keep result
            result_t result;
            result.freq = tx_lo;
            result.real_corr = best_correction.real();
            result.imag_corr = best_correction.imag();
            result.best = best_suppression;
            result.delta = best_suppression - initial_suppression;
            results.push_back(result);
            if (verbose){
                std::cout << boost::format("TX IQ: %f MHz: best suppression %f dB, corrected %f dB") % (tx_lo/1e6) % result.best % result.delta << std::endl;
            }
            else std::cout << "." << std::flush;
        }

    }
    std::cout << std::endl;

    //stop the transmitter
    interrupted = true;
    threads.join_all();

    store_results(usrp, results, "tx", "iq", vm.count("append"));

    return EXIT_SUCCESS;
}

