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

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;
namespace pt = boost::posix_time;

uhd::rx_streamer::sptr umtrx_impl::get_rx_stream(const uhd::stream_args_t &args){}
uhd::tx_streamer::sptr umtrx_impl::get_tx_stream(const uhd::stream_args_t &args){}
bool umtrx_impl::recv_async_msg(uhd::async_metadata_t &, double){}

void umtrx_impl::update_tick_rate(const double rate){}


    void umtrx_impl::update_rx_subdev_spec(const uhd::usrp::subdev_spec_t &){}
    void umtrx_impl::update_tx_subdev_spec(const uhd::usrp::subdev_spec_t &){}
    void umtrx_impl::update_clock_source(const std::string &){}

void umtrx_impl::update_rx_samp_rate(const size_t, const double rate){}
    void umtrx_impl::update_tx_samp_rate(const size_t, const double rate){}
