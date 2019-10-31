//
// Copyright 2010-2011,2014 Ettus Research LLC
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

#ifdef THREAD_PRIORITY_HPP_DEPRECATED
#  include <uhd/utils/thread.hpp>
#else // THREAD_PRIORITY_HPP_DEPRECATED
#  include <uhd/utils/thread_priority.hpp>
#endif // THREAD_PRIORITY_HPP_DEPRECATED
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/thread/thread.hpp>
#include <boost/format.hpp>
#include <iostream>
#include <complex>

namespace po = boost::program_options;

/***********************************************************************
 * TX chain test
 **********************************************************************/
static void test_tx_chain(uhd::usrp::multi_usrp::sptr usrp, const uhd::stream_args_t &stream_args, const float ampl, const double begin_delta, const size_t total_num_samps)
{
    //create a transmit streamer
    uhd::tx_streamer::sptr tx_stream = usrp->get_tx_stream(stream_args);

    //allocate buffer with data to send
    std::vector<std::complex<float> > buff(tx_stream->get_max_num_samps(), std::complex<float>(ampl, ampl));
    std::vector<std::complex<float> *> buffs(tx_stream->get_num_channels(), &buff.front()); //same buff all channels

    //setup metadata for the first packet
    uhd::tx_metadata_t md;
    md.start_of_burst = false;
    md.end_of_burst = false;
    md.has_time_spec = true;
    md.time_spec = uhd::time_spec_t(begin_delta) + usrp->get_time_now();

    //the first call to send() will block this many seconds before sending:
    const double timeout = begin_delta + 0.1; //timeout (delay before transmit + padding)

    size_t num_acc_samps = 0; //number of accumulated samples
    while(num_acc_samps < total_num_samps){
        size_t samps_to_send = std::min(total_num_samps - num_acc_samps, buff.size());

        //send a single packet
        size_t num_tx_samps = tx_stream->send(
            buffs, samps_to_send, md, timeout
        );

        //do not use time spec for subsequent packets
        md.has_time_spec = false;

        if (num_tx_samps < samps_to_send) std::cerr << "Send timeout..." << std::endl;
        //std::cout << boost::format("Sent packet: %u samples") % num_tx_samps << std::endl;

        num_acc_samps += num_tx_samps;
    }

    //send a mini EOB packet
    md.end_of_burst   = true;
    tx_stream->send("", 0, md);

    //std::cout << std::endl << "Waiting for async burst ACK... " << std::flush;
    uhd::async_metadata_t async_md;
    bool got_async_burst_ack = false;
    //loop through all messages for the ACK packet (may have underflow messages in queue)
    while (not got_async_burst_ack and tx_stream->recv_async_msg(async_md, timeout)){
        got_async_burst_ack = (async_md.event_code == uhd::async_metadata_t::EVENT_CODE_BURST_ACK);
    }
    std::cout << (got_async_burst_ack? "success" : "fail") << std::endl;
}

/***********************************************************************
 * RX chain test
 **********************************************************************/
static void test_rx_chain(uhd::usrp::multi_usrp::sptr usrp, const uhd::stream_args_t &stream_args, const double begin_delta, const size_t total_num_samps)
{
    //create a receive streamer
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    //setup streaming
    uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = false;
    stream_cmd.time_spec = uhd::time_spec_t(begin_delta) + usrp->get_time_now();
    rx_stream->issue_stream_cmd(stream_cmd);

    //meta-data will be filled in by recv()
    uhd::rx_metadata_t md;

    //allocate buffer to receive with samples
    std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
    std::vector<std::complex<float> *> buffs(rx_stream->get_num_channels(), &buff.front()); //same buff all channels

    //the first call to recv() will block this many seconds before receiving
    double timeout = begin_delta + 0.1; //timeout (delay before receive + padding)

    size_t num_acc_samps = 0; //number of accumulated samples
    while(num_acc_samps < total_num_samps){
        //receive a single packet
        size_t num_rx_samps = rx_stream->recv(
            buffs, buff.size(), md, timeout, true
        );

        //use a small timeout for subsequent packets
        timeout = 0.1;

        //handle the error code
        if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
        if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
            throw std::runtime_error(str(boost::format(
                "Unexpected error code 0x%x"
            ) % md.error_code));
        }

        //std::cout << boost::format(
        //    "Received packet: %u samples, %u full secs, %f frac secs"
        //) % num_rx_samps % md.time_spec.get_full_secs() % md.time_spec.get_frac_secs() << std::endl;

        num_acc_samps += num_rx_samps;
    }

    if (num_acc_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;
    std::cout << ((num_acc_samps == total_num_samps)? "success" : "fail") << std::endl;
}

int UHD_SAFE_MAIN(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args;
    double begin_delta;
    size_t num_samps;
    double rx_rate;
    double tx_rate;
    float ampl;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("secs", po::value<double>(&begin_delta)->default_value(0.5), "number of seconds time test setup")
        ("nsamps", po::value<size_t>(&num_samps)->default_value(100000), "total number of samples test with")
        ("rx_rate", po::value<double>(&rx_rate)->default_value(6.5e6), "rate of outgoing samples")
        ("tx_rate", po::value<double>(&tx_rate)->default_value(6.5e6), "rate of outgoing samples")
        ("ampl", po::value<float>(&ampl)->default_value(float(0.3)), "amplitude of each sample")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD TX Timed Samples %s") % desc << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

    //access the i2c interface
    uhd::i2c_iface::sptr i2c = usrp->get_device()->get_tree()->access<uhd::i2c_iface::sptr>("/mboards/0/i2c_iface").get();

    //set the tx sample rate
    if (usrp->get_tx_num_channels() > 0)
    {
        std::cout << boost::format("Setting TX Rate: %f Msps...") % (tx_rate/1e6) << std::endl;
        usrp->set_tx_rate(tx_rate);
        std::cout << boost::format("Actual TX Rate: %f Msps...") % (usrp->get_tx_rate()/1e6) << std::endl << std::endl;
    }

    //set the rx sample rate
    if (usrp->get_rx_num_channels() > 0)
    {
        std::cout << boost::format("Setting RX Rate: %f Msps...") % (rx_rate/1e6) << std::endl;
        usrp->set_rx_rate(rx_rate);
        std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;
    }

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));

    //test tx chains
    //*
    for (size_t i = 0; i < usrp->get_tx_num_channels(); i++)
    {
        std::cout << "===> TX test with DSP " << i << std::endl;
        uhd::stream_args_t stream_args("fc32");
        stream_args.channels.push_back(i);
        test_tx_chain(usrp, stream_args, ampl, begin_delta, num_samps);
    }
    if (usrp->get_tx_num_channels() >= 2)
    {
        std::cout << "===> TX test all DSP " << std::endl;
        uhd::stream_args_t stream_args("fc32");
        for (size_t i = 0; i < usrp->get_tx_num_channels(); i++) stream_args.channels.push_back(i);
        test_tx_chain(usrp, stream_args, ampl, begin_delta, num_samps);
    }
    //*/

    //test rx chains
    for (size_t i = 0; i < usrp->get_rx_num_channels(); i++)
    {
        std::cout << "===> RX test with DSP " << i << std::endl;
        uhd::stream_args_t stream_args("fc32");
        stream_args.channels.push_back(i);
        test_rx_chain(usrp, stream_args, begin_delta, num_samps);
    }
    if (usrp->get_rx_num_channels() >= 2)
    {
        std::cout << "===> RX test all DSP " << std::endl;
        uhd::stream_args_t stream_args("fc32");
        for (size_t i = 0; i < usrp->get_rx_num_channels(); i++) stream_args.channels.push_back(i);
        test_rx_chain(usrp, stream_args, begin_delta, num_samps);
    }

    //finished
    std::cout << std::endl << "Done!" << std::endl << std::endl;

    return EXIT_SUCCESS;
}
