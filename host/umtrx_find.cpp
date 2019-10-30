// Copyright 2012-2014 Fairwaves LLC
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

#include "usrp2/fw_common.h"
#include "umtrx_iface.hpp"
#include "umtrx_log_adapter.hpp"
#include <uhd/utils/log.hpp>
#include <uhd/utils/byteswap.hpp>
#include <uhd/types/device_addr.hpp>
#include <uhd/transport/if_addrs.hpp>
#include <uhd/transport/udp_simple.hpp>
#include <boost/asio.hpp>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;
namespace asio = boost::asio;

device_addrs_t umtrx_find(const device_addr_t &hint) {

    device_addrs_t umtrx_addrs;

    //Return an empty list of addresses when a resource is specified,
    //since a resource is intended for a different, non-USB, device.
    if (hint.has_key("resource")) return umtrx_addrs;

    //return an empty list of addresses when type is set to non-umtrx
    if (hint.has_key("type") and hint["type"] != "umtrx") return umtrx_addrs;

    //if no address was specified, send a broadcast on each interface
    if (not hint.has_key("addr")){
        BOOST_FOREACH(const if_addrs_t &if_addrs, get_if_addrs()){
            //avoid the loopback device
            if (if_addrs.inet == asio::ip::address_v4::loopback().to_string()) continue;

            //create a new hint with this broadcast address
            device_addr_t new_hint = hint;
            new_hint["addr"] = if_addrs.bcast;

            //call discover with the new hint and append results
            device_addrs_t new_umtrx_addrs = umtrx_find(new_hint);
            umtrx_addrs.insert(umtrx_addrs.begin(),
                new_umtrx_addrs.begin(), new_umtrx_addrs.end()
            );
        }
        return umtrx_addrs;
    }

    //Create a UDP transport to communicate:
    //Some devices will cause a throw when opened for a broadcast address.
    //We print and recover so the caller can loop through all bcast addrs.
    udp_simple::sptr udp_transport;
    try{
        udp_transport = udp_simple::make_broadcast(hint["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT));
    }
    catch(const std::exception &e){
        UHD_MSG(error) << boost::format("Cannot open UDP transport on %s\n%s") % hint["addr"] % e.what() << std::endl;
        return umtrx_addrs; //dont throw, but return empty address so caller can insert
    }

    //send a hello control packet
    usrp2_ctrl_data_t ctrl_data_out = usrp2_ctrl_data_t();
    ctrl_data_out.proto_ver = uhd::htonx<boost::uint32_t>(USRP2_FW_COMPAT_NUM);
    ctrl_data_out.id = uhd::htonx<boost::uint32_t>(UMTRX_CTRL_ID_REQUEST);
    try
    {
        udp_transport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
    }
    catch(const std::exception &ex)
    {
        UHD_MSG(error) << "UMTRX Network discovery error " << ex.what() << std::endl;
    }
    catch(...)
    {
        UHD_MSG(error) << "UMTRX Network discovery unknown error " << std::endl;
    }

    //loop and recieve until the timeout
    boost::uint8_t usrp2_ctrl_data_in_mem[udp_simple::mtu]; //allocate max bytes for recv
    const usrp2_ctrl_data_t *ctrl_data_in = reinterpret_cast<const usrp2_ctrl_data_t *>(usrp2_ctrl_data_in_mem);
    while(true){
        size_t len = udp_transport->recv(asio::buffer(usrp2_ctrl_data_in_mem));
        if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == UMTRX_CTRL_ID_RESPONSE){

            //make a boost asio ipv4 with the raw addr in host byte order
            device_addr_t new_addr;
            new_addr["type"] = "umtrx";
            //We used to get the address from the control packet.
            //Now now uses the socket itself to yield the address.
            //boost::asio::ip::address_v4 ip_addr(ntohl(ctrl_data_in->data.ip_addr));
            //new_addr["addr"] = ip_addr.to_string();
            new_addr["addr"] = udp_transport->get_recv_addr();

            //Attempt a simple 2-way communication with a connected socket.
            //Reason: Although the USRP will respond the broadcast above,
            //we may not be able to communicate directly (non-broadcast).
            udp_simple::sptr ctrl_xport = udp_simple::make_connected(
                new_addr["addr"], BOOST_STRINGIZE(USRP2_UDP_CTRL_PORT)
            );
            ctrl_xport->send(boost::asio::buffer(&ctrl_data_out, sizeof(ctrl_data_out)));
            size_t len = ctrl_xport->recv(asio::buffer(usrp2_ctrl_data_in_mem));
            if (len > offsetof(usrp2_ctrl_data_t, data) and ntohl(ctrl_data_in->id) == UMTRX_CTRL_ID_RESPONSE){
                //found the device, open up for communication!
            }
            else{
                //otherwise we don't find it...
                continue;
            }

            //Attempt to read the name from the EEPROM and perform filtering.
            //This operation can throw due to compatibility mismatch.
            try{
                umtrx_iface::sptr iface = umtrx_iface::make(ctrl_xport);
                if (iface->is_device_locked()) continue; //ignore locked devices
                mboard_eeprom_t mb_eeprom = iface->mb_eeprom;
                new_addr["name"] = mb_eeprom["name"];
                new_addr["serial"] = mb_eeprom["serial"];
            }
            catch(const std::exception &){
                //set these values as empty string so the device may still be found
                //and the filter's below can still operate on the discovered device
                new_addr["name"] = "";
                new_addr["serial"] = "";
            }

            //filter the discovered device below by matching optional keys
            if (
                (not hint.has_key("name")   or hint["name"]   == new_addr["name"]) and
                (not hint.has_key("serial") or hint["serial"] == new_addr["serial"])
            ){
                umtrx_addrs.push_back(new_addr);
            }

            //dont break here, it will exit the while loop
            //just continue on to the next loop iteration
        }
        if (len == 0) break; //timeout
    }

    return umtrx_addrs;
}
