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

#include <uhd/utils/paths.hpp>
#ifdef THREAD_PRIORITY_HPP_DEPRECATED
#  include <uhd/utils/thread.hpp>
#else // THREAD_PRIORITY_HPP_DEPRECATED
#  include <uhd/utils/thread_priority.hpp>
#endif // THREAD_PRIORITY_HPP_DEPRECATED
#include <uhd/utils/algorithm.hpp>
#include "umtrx_log_adapter.hpp"
#include <uhd/property_tree.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/version.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/thread/thread.hpp>
#include <boost/math/special_functions/round.hpp>
#include <iostream>
#include <vector>
#include <complex>
#include <cmath>
#include <fstream>
#include <atomic>

namespace fs = boost::filesystem;

struct result_t{double freq, real_corr, imag_corr, best, delta;};

typedef std::complex<float> samp_type;

/***********************************************************************
 * Constants
 **********************************************************************/
static const double tau = 6.28318531;
static const size_t wave_table_len = 8192;
static const double default_freq_step = 1e6;
static const size_t default_num_samps = 10000;

/***********************************************************************
 * Sinusoid wave table
 **********************************************************************/
class wave_table{
public:
    wave_table(const double ampl){
        _table.resize(wave_table_len);
        for (size_t i = 0; i < wave_table_len; i++){
            _table[i] = samp_type(std::polar(ampl, (tau*i)/wave_table_len));
        }
    }

    inline samp_type operator()(const size_t index) const{
        return _table[index % wave_table_len];
    }

private:
    std::vector<samp_type > _table;
};

/***********************************************************************
 * Compute power of a tone
 **********************************************************************/
static inline double compute_tone_dbrms(
    const std::vector<samp_type > &samples,
    const double freq //freq is fractional
){
    //shift the samples so the tone at freq is down at DC
    //and average the samples to measure the DC component
    samp_type average = 0;
    for (size_t i = 0; i < samples.size(); i++){
        average += samp_type(std::polar(1.0, -freq*tau*i)) * samples[i];
    }

    return 20*std::log10(std::abs(average/float(samples.size())));
}

/***********************************************************************
 * Write a dat file
 **********************************************************************/
static inline void write_samples_to_file(
    const std::vector<samp_type > &samples, const std::string &file
){
    std::ofstream outfile(file.c_str(), std::ofstream::binary);
    outfile.write((const char*)&samples.front(), samples.size()*sizeof(samp_type));
    outfile.close();
}

/***********************************************************************
 * Retrieve d'board serial
 **********************************************************************/
static std::string get_serial(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::string &tx_rx
){
    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    // Will work on 1st subdev, top-level must make sure it's the right one
    uhd::usrp::subdev_spec_t subdev_spec = usrp->get_rx_subdev_spec();
    const uhd::fs_path db_path = "/mboards/0/dboards/" + subdev_spec[0].db_name + "/" + tx_rx + "_eeprom";
    const uhd::usrp::dboard_eeprom_t db_eeprom = tree->access<uhd::usrp::dboard_eeprom_t>(db_path).get();
    return db_eeprom.serial;
}

/***********************************************************************
 * Convert integer calibration values to floats
 **********************************************************************/
static double dc_offset_int2double(uint8_t corr)
{
    return (corr-128)/128.0;
}

/***********************************************************************
 * Store data to file
 **********************************************************************/
static void store_results(
    uhd::usrp::multi_usrp::sptr usrp,
    const std::vector<result_t> &results,
    const std::string &rx_tx, // "tx" or "rx"
    const std::string &what,  // Type of test, e.g. "iq"
    bool append
){
    std::ofstream cal_data;
    bool write_header=true;
    std::string rx_tx_upper = boost::to_upper_copy(rx_tx);
    std::string serial = get_serial(usrp, rx_tx);

    //make the calibration file path
    //UHD4 deprecated get_app_path and uses designated calibration path (introduced earlier)
    //Don't break existing UHD3 installs, so use cal path only on UHD4
#if UHD_VERSION >= 4000000
    fs::path cal_data_path = fs::path(uhd::get_cal_data_path());
#else
    fs::path cal_data_path = fs::path(uhd::get_app_path()) / ".uhd";
    fs::create_directory(cal_data_path);
    cal_data_path = cal_data_path / "cal";
#endif
    fs::create_directory(cal_data_path);
    cal_data_path = cal_data_path / str(boost::format("%s_%s_cal_v0.2_%s.csv") % rx_tx % what % serial);
    if (fs::exists(cal_data_path)){
        if (append)
            write_header = false;
        else
            fs::rename(cal_data_path, cal_data_path.string() + str(boost::format(".%d") % time(NULL)));
    }

    cal_data.open(cal_data_path.string().c_str(), std::ofstream::out | std::ofstream::app);

    if (write_header)
    {
        //fill the calibration file
        cal_data << boost::format("name, %s Frontend Calibration\n") % rx_tx_upper;
        cal_data << boost::format("serial, %s\n") % serial;
        cal_data << boost::format("timestamp, %d\n") % time(NULL);
        cal_data << boost::format("version, 0, 1\n");
        cal_data << boost::format("DATA STARTS HERE\n");
        // For DC calibration we also store LMS6002D integer values
        if (what == "dc")
            cal_data << "lo_frequency, correction_real, correction_imag, measured, delta, int_i, int_q\n";
        else
            cal_data << "lo_frequency, correction_real, correction_imag, measured, delta\n";
    }

    for (size_t i = 0; i < results.size(); i++){
        // Write to file
        cal_data << results[i].freq;
        if (what == "dc") {
            cal_data << ", " << dc_offset_int2double(results[i].real_corr);
            cal_data << ", " << dc_offset_int2double(results[i].imag_corr);
        } else {
            cal_data << ", " << results[i].real_corr;
            cal_data << ", " << results[i].imag_corr;
        }
        cal_data << ", " << results[i].best;
        cal_data << ", " << results[i].delta;
        if (what == "dc") {
            cal_data << ", " << results[i].real_corr;
            cal_data << ", " << results[i].imag_corr;
        }
        cal_data << "\n";
    }

    std::cout << "wrote cal data to " << cal_data_path << std::endl;
}

/***********************************************************************
 * Data capture routine
 **********************************************************************/
static void capture_samples(
    uhd::rx_streamer::sptr rx_stream,
    std::vector<samp_type > &buff,
    const size_t nsamps_requested
){
    size_t num_rx_samps;
    buff.resize(nsamps_requested);
    uhd::rx_metadata_t md;

    for (int i=0; i<10; i++) {

    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = buff.size();
    stream_cmd.stream_now = true;
    rx_stream->issue_stream_cmd(stream_cmd);
    num_rx_samps = rx_stream->recv(&buff.front(), buff.size(), md);

    //validate the received data
    if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE
     && md.error_code != uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
        throw std::runtime_error(str(boost::format(
            "Unexpected error code 0x%x"
        ) % md.error_code));
    }
    //we can live if all the data didnt come in
    if (num_rx_samps > buff.size()/2){
        buff.resize(num_rx_samps);
        return;
    }
    if (num_rx_samps == buff.size()) break;
    }

    if (num_rx_samps != buff.size()){
        throw std::runtime_error("did not get all the samples requested");
    }
}

/***********************************************************************
 * Transmit thread
 **********************************************************************/
static void tx_thread(uhd::usrp::multi_usrp::sptr usrp, const double tx_wave_freq, const double tx_wave_ampl, std::atomic<bool> &interrupted){
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
    while (not interrupted){
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

    boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    return usrp->get_tx_freq();
}

/***********************************************************************
 * Setup function
 **********************************************************************/
static uhd::usrp::multi_usrp::sptr setup_usrp_for_cal(const std::string &args, const std::string &which, std::string &serial,
                                                      int vga1_gain, int vga2_gain, int rx_gain, int verbose)
{
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    // Do we have an UmTRX here?
    uhd::property_tree::sptr tree = usrp->get_device()->get_tree();
    const uhd::fs_path mb_path = "/mboards/0";
    const std::string mb_name = tree->access<std::string>(mb_path / "name").get();
    if (mb_name.find("UMTRX") == std::string::npos){
        throw std::runtime_error("This utility supports only UmTRX hardware.");
    }

    //set subdev spec
    usrp->set_rx_subdev_spec(which+":0");
    usrp->set_tx_subdev_spec(which+":0");

    UHD_MSG(status) << "Running calibration for " << usrp->get_tx_subdev_name(0) << std::endl;
    serial = get_serial(usrp, "tx");
    UHD_MSG(status) << "Daughterboard serial: " << serial << std::endl;

    //set the antennas to cal
    if (not uhd::has(usrp->get_rx_antennas(), "CAL") or not uhd::has(usrp->get_tx_antennas(), "CAL")){
        throw std::runtime_error("This board does not have the CAL antenna option, cannot self-calibrate.");
    }
    usrp->set_rx_antenna("CAL");
    usrp->set_tx_antenna("CAL");

    //set optimum defaults
    //  GSM symbol rate * 4
    usrp->set_tx_rate(13e6/12);
    usrp->set_rx_rate(13e6/12);
    //  500kHz LPF
    usrp->set_tx_bandwidth(1e6);
    usrp->set_rx_bandwidth(1e6);
    // Our recommended VGA1/VGA2
    usrp->set_tx_gain(vga1_gain, "VGA1");
    usrp->set_tx_gain(vga2_gain, "VGA2");
    usrp->set_rx_gain(rx_gain);
    if (verbose) printf("actual Tx VGA1 gain = %.0f dB\n", usrp->get_tx_gain("VGA1"));
    if (verbose) printf("actual Tx VGA2 gain = %.0f dB\n", usrp->get_tx_gain("VGA2"));
    if (verbose) printf("actual Rx gain = %.0f dB\n", usrp->get_rx_gain());

    return usrp;
}
