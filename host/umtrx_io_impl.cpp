//
// Copyright 2012-2014 Fairwaves
// Copyright 2010-2011 Ettus Research LLC
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

#include "umtrx_impl.hpp"
#include "umtrx_regs.hpp"
#include "usrp2/fw_common.h"
#include "cores/validate_subdev_spec.hpp"
#include "cores/async_packet_handler.hpp"
#include "cores/super_recv_packet_handler.hpp"
#include "cores/super_send_packet_handler.hpp"

//A reasonable number of frames for send/recv and async/sync
static const size_t DEFAULT_NUM_FRAMES = 32;

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

/***********************************************************************
 * Subdevice spec
 **********************************************************************/
void umtrx_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &spec)
{
    //sanity checking
    validate_subdev_spec(_tree, spec, "rx");
}

void umtrx_impl::update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &spec)
{
    //sanity checking
    validate_subdev_spec(_tree, spec, "tx");
}

/***********************************************************************
 * Update rates
 **********************************************************************/
void umtrx_impl::update_rates(void)
{
    fs_path root = "/mboards/0";
    _tree->access<double>(root / "tick_rate").update();

    //and now that the tick rate is set, init the host rates to something
    BOOST_FOREACH(const std::string &name, _tree->list(root / "rx_dsps"))
    {
        _tree->access<double>(root / "rx_dsps" / name / "rate" / "value").update();
    }
    BOOST_FOREACH(const std::string &name, _tree->list(root / "tx_dsps"))
    {
        _tree->access<double>(root / "tx_dsps" / name / "rate" / "value").update();
    }
}

void umtrx_impl::update_rx_samp_rate(const size_t dsp, const double rate)
{
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::recv_packet_streamer>(_rx_streamers[dsp].lock());
    if (not my_streamer) return;

    my_streamer->set_samp_rate(rate);
    const double adj = _rx_dsps[dsp]->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void umtrx_impl::update_tx_samp_rate(const size_t dsp, const double rate)
{
    boost::shared_ptr<sph::send_packet_streamer> my_streamer =
        boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[dsp].lock());
    if (not my_streamer) return;

    my_streamer->set_samp_rate(rate);
    const double adj = _tx_dsps[dsp]->get_scaling_adjustment();
    my_streamer->set_scale_factor(adj);
}

void umtrx_impl::update_tick_rate(const double rate)
{
    //update the tick rate on all existing streamers -> thread safe
    for (size_t i = 0; i < _rx_streamers.size(); i++)
    {
        boost::shared_ptr<sph::recv_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::recv_packet_streamer>(_rx_streamers[i].lock());
        if (not my_streamer) continue;
        my_streamer->set_tick_rate(rate);
    }
    for (size_t i = 0; i < _tx_streamers.size(); i++)
    {
        boost::shared_ptr<sph::send_packet_streamer> my_streamer =
            boost::dynamic_pointer_cast<sph::send_packet_streamer>(_tx_streamers[i].lock());
        if (not my_streamer) continue;
        my_streamer->set_tick_rate(rate);
    }
}

/***********************************************************************
 * Transport creation and framer programming
 **********************************************************************/
static void program_stream_dest(
    zero_copy_if::sptr &xport, const size_t which, const device_addr_t &args
){
    //perform an initial flush of transport
    while (xport->get_recv_buff(0.0)){}

    //program the stream command
    usrp2_stream_ctrl_t stream_ctrl = usrp2_stream_ctrl_t();
    stream_ctrl.sequence = uhd::htonx(boost::uint32_t(0 /* don't care seq num */));
    stream_ctrl.vrt_hdr = uhd::htonx(boost::uint32_t(USRP2_INVALID_VRT_HEADER));
    stream_ctrl.which = uhd::htonx(boost::uint32_t(which));

    //send the partial stream control without destination
    managed_send_buffer::sptr send_buff = xport->get_send_buff();
    std::memcpy(send_buff->cast<void *>(), &stream_ctrl, sizeof(stream_ctrl));
    send_buff->commit(sizeof(stream_ctrl));
}

uhd::transport::zero_copy_if::sptr umtrx_impl::make_xport(const size_t which, const uhd::device_addr_t &args)
{
    zero_copy_xport_params default_params;
    default_params.send_frame_size = transport::udp_simple::mtu;
    default_params.recv_frame_size = transport::udp_simple::mtu;
    default_params.num_send_frames = DEFAULT_NUM_FRAMES;
    default_params.num_recv_frames = DEFAULT_NUM_FRAMES;
    udp_zero_copy::buff_params ignored_params;
    zero_copy_if::sptr xport = udp_zero_copy::make(_device_ip_addr, BOOST_STRINGIZE(USRP2_UDP_SERVER_PORT), default_params, ignored_params, args);
    program_stream_dest(xport, which, args);
    _iface->peek32(0); //peek to ensure the zpu processed the program_stream_dest()
    return xport;
}

/***********************************************************************
 * Receive streamer
 **********************************************************************/
uhd::rx_streamer::sptr umtrx_impl::get_rx_stream(const uhd::stream_args_t &args_)
{
    stream_args_t args = args_;

    //setup defaults for unspecified values
    args.otw_format = args.otw_format.empty()? "sc16" : args.otw_format;
    args.channels = args.channels.empty()? std::vector<size_t>(1, 0) : args.channels;

    //setup the transport hints (default to a large recv buff)
    if (not args.args.has_key("recv_buff_size"))
    {
        #if defined(UHD_PLATFORM_MACOS) || defined(UHD_PLATFORM_BSD)
            //limit buffer resize on macos or it will error
            args.args["recv_buff_size"] = "1e6";
        #elif defined(UHD_PLATFORM_LINUX) || defined(UHD_PLATFORM_WIN32)
            //set to half-a-second of buffering at max rate
            args.args["recv_buff_size"] = "50e6";
        #endif
    }
    zero_copy_xport_params default_params;
    default_params.send_frame_size = transport::udp_simple::mtu;
    default_params.recv_frame_size = transport::udp_simple::mtu;
    default_params.num_send_frames = DEFAULT_NUM_FRAMES;
    default_params.num_recv_frames = DEFAULT_NUM_FRAMES;

    //create the transport
    std::vector<zero_copy_if::sptr> xports(_rx_dsps.size());
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        size_t which = ~0;
        if (dsp == 0) which = UMTRX_DSP_RX0_FRAMER;
        if (dsp == 1) which = UMTRX_DSP_RX1_FRAMER;
        UHD_ASSERT_THROW(which != ~0);
        xports[dsp] = make_xport(which, args.args);
    }

    //calculate packet size
    static const size_t hdr_size = 0
        + vrt::max_if_hdr_words32*sizeof(boost::uint32_t)
        + sizeof(vrt::if_packet_info_t().tlr) //forced to have trailer
        - sizeof(vrt::if_packet_info_t().cid) //no class id ever used
        - sizeof(vrt::if_packet_info_t().tsi) //no int time ever used
    ;
    const size_t bpp = xports[0]->get_recv_frame_size() - hdr_size;
    const size_t bpi = convert::get_bytes_per_item(args.otw_format);
    const size_t spp = unsigned(args.args.cast<double>("spp", bpp/bpi));

    //make the new streamer given the samples per packet
    boost::shared_ptr<sph::recv_packet_streamer> my_streamer = boost::make_shared<sph::recv_packet_streamer>(spp);

    //init some streamer stuff
    my_streamer->resize(args.channels.size());
    my_streamer->set_vrt_unpacker(&vrt::if_hdr_unpack_be);

    //set the converter
    uhd::convert::id_type id;
    id.input_format = args.otw_format + "_item32_be";
    id.num_inputs = 1;
    id.output_format = args.cpu_format;
    id.num_outputs = 1;
    my_streamer->set_converter(id);

    //bind callbacks for the handler
    for (size_t chan_i = 0; chan_i < args.channels.size(); chan_i++)
    {
        const size_t dsp = args.channels[chan_i];
        _rx_dsps[dsp]->set_nsamps_per_packet(spp); //seems to be a good place to set this
        _rx_dsps[dsp]->setup(args);
        my_streamer->set_xport_chan_get_buff(chan_i, boost::bind(
            &zero_copy_if::get_recv_buff, xports[dsp], _1
        ), true /*flush*/);
        my_streamer->set_issue_stream_cmd(chan_i, boost::bind(
            &rx_dsp_core_200::issue_stream_command, _rx_dsps[dsp], _1));
        _rx_streamers[dsp] = my_streamer; //store weak pointer
    }

    //set the packet threshold to be an entire socket buffer's worth
    const size_t packets_per_sock_buff = size_t(50e6/xports[0]->get_recv_frame_size());
    my_streamer->set_alignment_failure_threshold(packets_per_sock_buff);

    //sets all tick and samp rates on this streamer
    this->update_rates();

    return my_streamer;
}


/***********************************************************************
 * Transmit streamer
 **********************************************************************/
uhd::tx_streamer::sptr umtrx_impl::get_tx_stream(const uhd::stream_args_t &args)
{
    
}

bool umtrx_impl::recv_async_msg(uhd::async_metadata_t &, double)
{
    
}
