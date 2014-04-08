//
// Copyright 2010 Ettus Research LLC
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
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/paths.hpp>
#include <uhd/utils/algorithm.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <iostream>
#include <complex>
#include <cmath>
#include <ctime>

namespace po = boost::program_options;

/***********************************************************************
 * Transmit thread
 **********************************************************************/
static void tx_thread(uhd::usrp::multi_usrp::sptr usrp, const double tx_wave_freq, const double tx_wave_ampl){
    uhd::set_thread_priority_safe();

    //create a transmit streamer
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //setup variables and allocate buffer
    uhd::tx_metadata_t md;
    md.has_time_spec = false;
    std::vector<samp_type> buff(tx_stream->get_max_num_samps()*10);

    //values for the wave table lookup
    size_t index = 0;
    const double tx_rate = usrp->get_tx_rate();
    const size_t step = boost::math::iround(wave_table_len * tx_wave_freq/tx_rate);
    wave_table table(tx_wave_ampl);


    //fill buff and send until interrupted
    while (not boost::this_thread::interruption_requested()){
        for (size_t i = 0; i < buff.size(); i++){
            buff[i] = table(index += step);
        }
        tx_stream->send(&buff.front(), buff.size(), md);
    }

    //send a mini EOB packet
    md.end_of_burst = true;
    tx_stream->send("", 0, md);
}

/***********************************************************************
 * Tune RX and TX routine
 **********************************************************************/
static double tune_rx_and_tx(uhd::usrp::multi_usrp::sptr usrp, const double tx_lo_freq, const double rx_offset){
    //tune the transmitter with no cordic
    uhd::tune_request_t tx_tune_req(tx_lo_freq);
    tx_tune_req.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;
    tx_tune_req.dsp_freq = 0;
    usrp->set_tx_freq(tx_tune_req);

    //tune the receiver
    usrp->set_rx_freq(uhd::tune_request_t(usrp->get_tx_freq(), rx_offset));

    //wait for the LOs to become locked
    boost::this_thread::sleep(boost::posix_time::milliseconds(50));
    boost::system_time start = boost::get_system_time();
    while (not usrp->get_tx_sensor("lo_locked").to_bool() or not usrp->get_rx_sensor("lo_locked").to_bool()){
        if (boost::get_system_time() > start + boost::posix_time::milliseconds(100)){
            throw std::runtime_error("timed out waiting for TX and/or RX LO to lock");
        }
    }

    return usrp->get_tx_freq();
}

/***********************************************************************
 * Main
 **********************************************************************/
int UHD_SAFE_MAIN(int argc, char *argv[]){
    std::string args;
    double tx_wave_freq, tx_wave_ampl, rx_offset;
    double freq_start, freq_stop, freq_step, compl_i, compl_q, polar_mag, polar_phase;
    size_t nsamps;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("verbose", "enable some verbose")
        ("debug_raw_data", "save raw captured signals to files")
        ("args", po::value<std::string>(&args)->default_value(""), "device address args [default = \"\"]")
        ("tx_wave_freq", po::value<double>(&tx_wave_freq)->default_value(50e3), "Transmit wave frequency in Hz")
        ("tx_wave_ampl", po::value<double>(&tx_wave_ampl)->default_value(0.7), "Transmit wave amplitude in counts")
        ("rx_offset", po::value<double>(&rx_offset)->default_value(1e6), "RX LO offset from the TX LO in Hz")
	("compl_i", po::value<double>(&compl_i), "Enforced correction for I (complex)")
        ("compl_q", po::value<double>(&compl_q), "Enforced correction for Q (complex)")
	("polar_mag", po::value<double>(&polar_mag), "Enforced correction, magnitude (polar)")
        ("polar_phase", po::value<double>(&polar_phase), "Enforced correction, phase (polar)")
        ("freq_start", po::value<double>(&freq_start), "Frequency start in Hz (do not specify for default)")
        ("freq_stop", po::value<double>(&freq_stop), "Frequency stop in Hz (do not specify for default)")
        ("freq_step", po::value<double>(&freq_step)->default_value(default_freq_step), "Step size for LO sweep in Hz")
        ("nsamps", po::value<size_t>(&nsamps)->default_value(default_num_samps), "Samples per data capture")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("USRP Generate TX IQ Balance Calibration Table %s") % desc << std::endl;
        std::cout <<
            "This application measures leakage between RX and TX on an XCVR daughterboard to self-calibrate.\n"
            << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    //apply manual corrections
    if (vm.count("compl_i") and vm.count("compl_q")) {
	std::cout << "Applying complex I/Q corrections <" << compl_i << ", " << compl_q << ">...";
	usrp->set_tx_iq_balance(std::complex<double>(compl_i, compl_q));
	std::cout << "done. Exit.\n";
	return 0;
    }
    if (vm.count("polar_mag") and vm.count("polar_phase")) {
	std::cout << "Applying polar I/Q corrections <" << polar_mag << ", " << polar_phase << ">...";
	usrp->set_tx_iq_balance(std::polar<double>(polar_mag, polar_phase));
	std::cout << "done. Exit.\n";
	return 0;
    }

    //set the antennas to cal
    if (not uhd::has(usrp->get_rx_antennas(), "CAL") or not uhd::has(usrp->get_tx_antennas(), "CAL")){
        throw std::runtime_error("This board does not have the CAL antenna option, cannot self-calibrate.");
    }
    usrp->set_rx_antenna("CAL");
    usrp->set_tx_antenna("CAL");

    //set optimum defaults
    set_optimum_defaults(usrp);

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //create a transmitter thread
    boost::thread_group threads;
    threads.create_thread(boost::bind(&tx_thread, usrp, tx_wave_freq, tx_wave_ampl));

    //re-usable buffer for samples
    std::vector<samp_type> buff;

    //store the results here
    std::vector<result_t> results;

    if (not vm.count("freq_start")) freq_start = usrp->get_tx_freq_range().start() + 50e6;
    if (not vm.count("freq_stop")) freq_stop = usrp->get_tx_freq_range().stop() - 50e6;

    if (vm.count("verbose")) printf("freq_start = %0.2f MHz\n", freq_start/1e6);
    if (vm.count("verbose")) printf("freq_stop = %0.2f MHz\n", freq_stop/1e6);
    if (vm.count("verbose")) printf("freq_step = %0.2f MHz\n", freq_step/1e6);
    if (vm.count("verbose")) printf("rx_offset = %0.2f MHz\n", rx_offset/1e6);

    for (double tx_lo_i = freq_start; tx_lo_i <= freq_stop; tx_lo_i += freq_step){
        const double tx_lo = tune_rx_and_tx(usrp, tx_lo_i, rx_offset);
        if (vm.count("verbose")) printf("-------------------------\n");
        if (vm.count("verbose")) printf("Calibrating for %0.1f MHz\n", tx_lo/1e6);

        //frequency constants for this tune event
        const double actual_rx_rate = usrp->get_rx_rate();
        const double actual_tx_freq = usrp->get_tx_freq();
        const double actual_rx_freq = usrp->get_rx_freq();
        const double bb_tone_freq = actual_tx_freq + tx_wave_freq - actual_rx_freq;
        const double bb_imag_freq = actual_tx_freq - tx_wave_freq - actual_rx_freq;
        if (vm.count("verbose")) printf("actual_rx_rate = %0.2f MHz\n", actual_rx_rate/1e6);
        if (vm.count("verbose")) printf("actual_tx_freq = %0.2f MHz\n", actual_tx_freq/1e6);
        if (vm.count("verbose")) printf("actual_rx_freq = %0.2f MHz\n", actual_rx_freq/1e6);
        if (vm.count("verbose")) printf("bb_tone_freq = %0.2f MHz\n", bb_tone_freq/1e6);
        if (vm.count("verbose")) printf("bb_imag_freq = %0.2f MHz\n", bb_imag_freq/1e6);

        //capture initial uncorrected value
        usrp->set_tx_iq_balance(std::polar<double>(1.0, 0.0));
        capture_samples(usrp, rx_stream, buff, nsamps);
        const double initial_tone_dbrms = compute_tone_dbrms(buff, bb_tone_freq/actual_rx_rate);
        const double initial_image_dbrms = compute_tone_dbrms(buff, bb_imag_freq/actual_rx_rate);
        const double initial_suppression = initial_tone_dbrms - initial_image_dbrms;
        if (vm.count("verbose")) printf("initial_tone_dbrms = %2.0f dB\n", initial_tone_dbrms);
        if (vm.count("verbose")) printf("initial_image_dbrms = %2.0f dB\n", initial_image_dbrms);
        if (vm.count("verbose")) printf("initial_suppression = %2.0f dB\n", initial_suppression);

        if (vm.count("debug_raw_data")) write_samples_to_file(buff, "initial_samples.dat");

        //bounds and results from searching
        std::complex<double> best_correction;
        double phase_corr_start = -.3, phase_corr_stop = .3, phase_corr_step;
        double ampl_corr_start = -.3, ampl_corr_stop = .3, ampl_corr_step;
        double best_suppression = initial_suppression, best_phase_corr = 0, best_ampl_corr = 0;

        for (size_t i = 0; i < num_search_iters; i++){
            if (vm.count("verbose")) printf("  iteration %lu\n", i);

            phase_corr_step = (phase_corr_stop - phase_corr_start)/(num_search_steps-1);
            ampl_corr_step = (ampl_corr_stop - ampl_corr_start)/(num_search_steps-1);

            for (double phase_corr = phase_corr_start; phase_corr <= phase_corr_stop + phase_corr_step/2; phase_corr += phase_corr_step){
            for (double ampl_corr = ampl_corr_start; ampl_corr <= ampl_corr_stop + ampl_corr_step/2; ampl_corr += ampl_corr_step){
                if (vm.count("verbose")) printf("    phase_corr = %0.2f ampl_corr = %0.2f", phase_corr, ampl_corr);

                const std::complex<double> correction = std::polar(ampl_corr+1, phase_corr*tau);
                usrp->set_tx_iq_balance(correction);

                //receive some samples
                capture_samples(usrp, rx_stream, buff, nsamps);

                const double tone_dbrms = compute_tone_dbrms(buff, bb_tone_freq/actual_rx_rate);
                const double imag_dbrms = compute_tone_dbrms(buff, bb_imag_freq/actual_rx_rate);
                const double suppression = tone_dbrms - imag_dbrms;
                if (vm.count("verbose")) printf("    tone_dbrms = %2.0f dB", tone_dbrms);
                if (vm.count("verbose")) printf("    imag_dbrms = %2.0f dB", imag_dbrms);
                if (vm.count("verbose")) printf("    suppression = %2.0f dB", suppression);

                if (suppression > best_suppression){
                    best_correction = correction;
                    best_suppression = suppression;
                    best_phase_corr = phase_corr;
                    best_ampl_corr = ampl_corr;
                    if (vm.count("verbose")) printf("    *");
                    if (vm.count("debug_raw_data")) write_samples_to_file(buff, "best_samples.dat");
                }
                if (vm.count("verbose")) printf("\n");

            }}

            //std::cout << "best_phase_corr " << best_phase_corr << std::endl;
            //std::cout << "best_ampl_corr " << best_ampl_corr << std::endl;
            //std::cout << "best_suppression " << best_suppression << std::endl;

            phase_corr_start = best_phase_corr - phase_corr_step;
            phase_corr_stop = best_phase_corr + phase_corr_step;
            ampl_corr_start = best_ampl_corr - ampl_corr_step;
            ampl_corr_stop = best_ampl_corr + ampl_corr_step;
        }

        if (vm.count("verbose")) printf("  best_corr phase = %0.5f ampl = %0.5f real = %0.5f imag = %0.5f",
                                        best_phase_corr, best_ampl_corr, best_correction.real(), best_correction.imag());
        if (vm.count("verbose")) printf("  suppression = %2.0f dB\n", best_suppression);

        if (best_suppression > 10){ //most likely valid, keep result
            result_t result;
            result.freq = tx_lo;
            result.real_corr = best_correction.real();
            result.imag_corr = best_correction.imag();
            result.best = best_suppression;
            result.delta = best_suppression - initial_suppression;
            results.push_back(result);
            if (vm.count("verbose")){
                std::cout << boost::format("TX IQ: %f MHz: best suppression %f dB, corrected %f dB") % (tx_lo/1e6) % result.best % result.delta << std::endl;
            }
            else std::cout << "." << std::flush;
        }

    }
    std::cout << std::endl;

    //stop the transmitter
    threads.interrupt_all();
    threads.join_all();

    store_results(usrp, results, "TX", "tx", "iq");

    return 0;
}
