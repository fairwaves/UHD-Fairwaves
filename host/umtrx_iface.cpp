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

#include "umtrx_regs.hpp"
#include "usrp2/fw_common.h"
#include "umtrx_impl.hpp"
#include "umtrx_iface.hpp"
#include <uhd/exception.hpp>
#include "umtrx_log_adapter.hpp"
#include "missing/platform.hpp"
#include <uhd/utils/tasks.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/types/dict.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/asio.hpp> //used for htonl and ntohl
#include <boost/assign/list_of.hpp>
#include <boost/format.hpp>
#include <boost/bind/bind.hpp>
#include <boost/tokenizer.hpp>
#include <boost/functional/hash.hpp>
#include <algorithm>
#include <iostream>
#undef NDEBUG //evil hack for debug
#include <cassert>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::transport;

static const double CTRL_RECV_TIMEOUT = 1.0;
static const size_t CTRL_RECV_RETRIES = 3;

//custom timeout error for retry logic to catch/retry
struct timeout_error : uhd::runtime_error
{
    timeout_error(const std::string &what):
        uhd::runtime_error(what)
    {
        //NOP
    }
};

static const boost::uint32_t MIN_PROTO_COMPAT_SPI = 7;
static const boost::uint32_t MIN_PROTO_COMPAT_I2C = 7;
// The register compat number must reflect the protocol compatibility
// and the compatibility of the register mapping (more likely to change).
static const boost::uint32_t MIN_PROTO_COMPAT_REG = 10;

class umtrx_iface_impl : public umtrx_iface{
public:
/***********************************************************************
 * Structors
 **********************************************************************/
    umtrx_iface_impl(udp_simple::sptr ctrl_transport):
        _ctrl_transport(ctrl_transport),
        _ctrl_seq_num(0),
        _protocol_compat(USRP2_FW_COMPAT_NUM)
    {
        //Obtain the firmware's compat number.
        usrp2_ctrl_data_t ctrl_data;
        ctrl_data.id = htonl(UMTRX_CTRL_ID_REQUEST);
        ctrl_data = ctrl_send_and_recv(ctrl_data, _protocol_compat, ~0);
        if (ntohl(ctrl_data.id) != UMTRX_CTRL_ID_RESPONSE)
            throw uhd::runtime_error(str(boost::format("unexpected firmware response: -->%c<--") % (char)ntohl(ctrl_data.id)));

        //Save the response compat number for future communication.
        _protocol_compat = ntohl(ctrl_data.proto_ver);

        // Read EEPROM with UMTRX extensions
        load_umtrx_eeprom(mb_eeprom, *this);
    }

    ~umtrx_iface_impl(void){UHD_SAFE_CALL(
        this->lock_device(false);
    )}

/***********************************************************************
 * Device locking
 **********************************************************************/

    void lock_device(bool lock){
        if (lock){
            this->pokefw(U2_FW_REG_LOCK_GPID, get_process_hash());
            _lock_task = task::make(boost::bind(&umtrx_iface_impl::lock_task, this));
        }
        else{
            _lock_task.reset(); //shutdown the task
            this->pokefw(U2_FW_REG_LOCK_TIME, 0); //unlock
        }
    }

    bool is_device_locked(void){
        //never assume lock with fpga image mismatch
        if ((this->peek32(U2_REG_COMPAT_NUM_RB) >> 16) != USRP2_FPGA_COMPAT_NUM) return false;

        boost::uint32_t lock_time = this->peekfw(U2_FW_REG_LOCK_TIME);
        boost::uint32_t lock_gpid = this->peekfw(U2_FW_REG_LOCK_GPID);

        //may not be the right tick rate, but this is ok for locking purposes
        const boost::uint32_t lock_timeout_time = boost::uint32_t(3*100e6);

        //if the difference is larger, assume not locked anymore
        if ((lock_time & 1) == 0) return false; //bit0 says unlocked
        const boost::uint32_t time_diff = this->get_curr_time() - lock_time;
        if (time_diff >= lock_timeout_time) return false;

        //otherwise only lock if the device hash is different that ours
        return lock_gpid != get_process_hash();
    }

    void lock_task(void){
        //re-lock in task
        this->pokefw(U2_FW_REG_LOCK_TIME, this->get_curr_time());
        //sleep for a bit
        boost::this_thread::sleep(boost::posix_time::milliseconds(1500));
    }

    boost::uint32_t get_curr_time(void){
        return this->peek32(U2_REG_TIME64_LO_RB_IMM) | 1; //bit 1 says locked
    }

/***********************************************************************
 * Peek and Poke
 **********************************************************************/
    void poke32(wb_addr_type addr, boost::uint32_t data){
        this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FPGA_POKE32>(addr, data);
    }

    boost::uint32_t peek32(wb_addr_type addr){
        return this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FPGA_PEEK32>(addr);
    }

    void poke16(wb_addr_type addr, boost::uint16_t data){
        this->get_reg<boost::uint16_t, USRP2_REG_ACTION_FPGA_POKE16>(addr, data);
    }

    boost::uint16_t peek16(wb_addr_type addr){
        return this->get_reg<boost::uint16_t, USRP2_REG_ACTION_FPGA_PEEK16>(addr);
    }

    void pokefw(wb_addr_type addr, boost::uint32_t data)
    {
        this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_POKE32>(addr, data);
    }

    boost::uint32_t peekfw(wb_addr_type addr)
    {
        return this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_PEEK32>(addr);
    }

    template <class T, usrp2_reg_action_t action>
    T get_reg(wb_addr_type addr, T data = 0){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_GET_THIS_REGISTER_FOR_ME_BRO);
        out_data.data.reg_args.addr = htonl(addr);
        out_data.data.reg_args.data = htonl(boost::uint32_t(data));
        out_data.data.reg_args.action = action;

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_REG);
        UHD_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_OMG_GOT_REGISTER_SO_BAD_DUDE);
        return T(ntohl(in_data.data.reg_args.data));
    }

/***********************************************************************
 * SPI
 **********************************************************************/
    boost::uint32_t transact_spi(
        int which_slave,
        const spi_config_t &config,
        boost::uint32_t data,
        size_t num_bits,
        bool readback
    ){
        static const uhd::dict<spi_config_t::edge_t, int> spi_edge_to_otw = boost::assign::map_list_of
            (spi_config_t::EDGE_RISE, USRP2_CLK_EDGE_RISE)
            (spi_config_t::EDGE_FALL, USRP2_CLK_EDGE_FALL)
        ;

        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_TRANSACT_ME_SOME_SPI_BRO);
        out_data.data.spi_args.dev = htonl(which_slave);
        out_data.data.spi_args.miso_edge = spi_edge_to_otw[config.miso_edge];
        out_data.data.spi_args.mosi_edge = spi_edge_to_otw[config.mosi_edge];
        out_data.data.spi_args.readback = (readback)? 1 : 0;
        out_data.data.spi_args.num_bits = num_bits;
        out_data.data.spi_args.data = htonl(data);

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        UHD_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_OMG_TRANSACTED_SPI_DUDE);

        return ntohl(in_data.data.spi_args.data);
    }

/***********************************************************************
 * I2C
 **********************************************************************/
    void write_i2c(boost::uint16_t addr, const byte_vector_t &buf){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_WRITE_THESE_I2C_VALUES_BRO);
        out_data.data.i2c_args.addr = addr;
        out_data.data.i2c_args.bytes = buf.size();

        //limitation of i2c transaction size
        UHD_ASSERT_THROW(buf.size() <= sizeof(out_data.data.i2c_args.data));

        //copy in the data
        std::copy(buf.begin(), buf.end(), out_data.data.i2c_args.data);

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_I2C);
        UHD_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_COOL_IM_DONE_I2C_WRITE_DUDE);
    }

    byte_vector_t read_i2c(boost::uint16_t addr, size_t num_bytes){
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(USRP2_CTRL_ID_DO_AN_I2C_READ_FOR_ME_BRO);
        out_data.data.i2c_args.addr = addr;
        out_data.data.i2c_args.bytes = num_bytes;

        //limitation of i2c transaction size
        UHD_ASSERT_THROW(num_bytes <= sizeof(out_data.data.i2c_args.data));

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_I2C);
        UHD_ASSERT_THROW(ntohl(in_data.id) == USRP2_CTRL_ID_HERES_THE_I2C_DATA_DUDE);
        UHD_ASSERT_THROW(in_data.data.i2c_args.addr = num_bytes);

        //copy out the data
        byte_vector_t result(num_bytes);
        std::copy(in_data.data.i2c_args.data, in_data.data.i2c_args.data + num_bytes, result.begin());
        return result;
    }

/***********************************************************************
 * Send/Recv a ZPU action
 **********************************************************************/

    uint32_t send_zpu_action(uint32_t action, uint32_t data)
    {
        //setup the out data
        usrp2_ctrl_data_t out_data = usrp2_ctrl_data_t();
        out_data.id = htonl(UMTRX_CTRL_ID_ZPU_REQUEST);
        out_data.data.zpu_action.action = htonl(action);
        out_data.data.zpu_action.data = htonl(data);

        //send and recv
        usrp2_ctrl_data_t in_data = this->ctrl_send_and_recv(out_data, MIN_PROTO_COMPAT_SPI);
        UHD_ASSERT_THROW(ntohl(in_data.id) == UMTRX_CTRL_ID_ZPU_RESPONSE);

        return ntohl(in_data.data.zpu_action.data);
    }

/***********************************************************************
 * Send/Recv over control
 **********************************************************************/
    usrp2_ctrl_data_t ctrl_send_and_recv(
        const usrp2_ctrl_data_t &out_data,
        boost::uint32_t lo = USRP2_FW_COMPAT_NUM,
        boost::uint32_t hi = USRP2_FW_COMPAT_NUM
    ){
        boost::mutex::scoped_lock lock(_ctrl_mutex);

        for (size_t i = 0; i < CTRL_RECV_RETRIES; i++){
            try{
                return ctrl_send_and_recv_internal(out_data, lo, hi, CTRL_RECV_TIMEOUT/CTRL_RECV_RETRIES);
            }
            catch(const timeout_error &e){
                UHD_MSG(error)
                    << "Control packet attempt " << i
                    << ", sequence number " << _ctrl_seq_num
                    << ":\n" << e.what() << std::endl;
            }
        }
        throw uhd::runtime_error("link dead: timeout waiting for control packet ACK");
    }

    usrp2_ctrl_data_t ctrl_send_and_recv_internal(
        const usrp2_ctrl_data_t &out_data,
        boost::uint32_t lo, boost::uint32_t hi,
        const double timeout
    ){
        //fill in the seq number and send
        usrp2_ctrl_data_t out_copy = out_data;
        out_copy.proto_ver = htonl(_protocol_compat);
        out_copy.seq = htonl(++_ctrl_seq_num);
        _ctrl_transport->send(boost::asio::buffer(&out_copy, sizeof(usrp2_ctrl_data_t)));

        //loop until we get the packet or timeout
        boost::uint8_t usrp2_ctrl_data_in_mem[udp_simple::mtu]; //allocate max bytes for recv
        const usrp2_ctrl_data_t *ctrl_data_in = reinterpret_cast<const usrp2_ctrl_data_t *>(usrp2_ctrl_data_in_mem);
        while(true){
            size_t len = _ctrl_transport->recv(boost::asio::buffer(usrp2_ctrl_data_in_mem), timeout);
            boost::uint32_t compat = ntohl(ctrl_data_in->proto_ver);
            if(len >= sizeof(boost::uint32_t) and (hi < compat or lo > compat)){
                throw uhd::runtime_error(str(boost::format(
                    "\nPlease update the firmware and FPGA images for your device.\n"
                    "See the application notes for UmTRX for instructions.\n"
                    "Expected protocol compatibility number %s, but got %d:\n"
                    "The firmware build is not compatible with the host code build."
                ) % ((lo == hi)? (boost::format("%d") % hi) : (boost::format("[%d to %d]") % lo % hi)) % compat));
            }
            if (len >= sizeof(usrp2_ctrl_data_t) and ntohl(ctrl_data_in->seq) == _ctrl_seq_num){
                return *ctrl_data_in;
            }
            if (len == 0) break; //timeout
            //didnt get seq or bad packet, continue looking...
        }
        throw timeout_error("no control response, possible packet loss");
    }

    rev_type get_rev(void){
        std::string hw = mb_eeprom["hardware"];
        if (hw.empty()) return USRP_NXXX;
        switch (boost::lexical_cast<boost::uint16_t>(hw)){
        case 0xFA00: return UMTRX_REV0;
        }
        return USRP_NXXX; //unknown type
    }

    const std::string get_cname(void){
        switch(this->get_rev()){
        case UMTRX_REV0: return "UMTRX-REV0";
        case USRP_NXXX: return "USRP-N???";
        }
        UHD_THROW_INVALID_CODE_PATH();
    }

    const std::string get_fw_version_string(void){
        boost::uint32_t minor = this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_PEEK32>(U2_FW_REG_VER_MINOR);
        boost::uint32_t githash = this->get_reg<boost::uint32_t, USRP2_REG_ACTION_FW_PEEK32>(U2_FW_REG_GIT_HASH);
        return str(boost::format("%u.%u-g%x") % _protocol_compat % minor % githash);
    }

private:
    //this lovely lady makes it all possible
    udp_simple::sptr _ctrl_transport;

    //used in send/recv
    boost::mutex _ctrl_mutex;
    boost::uint32_t _ctrl_seq_num;
    boost::uint32_t _protocol_compat;

    //lock thread stuff
    task::sptr _lock_task;
};

/***********************************************************************
 * Public make function for usrp2 interface
 **********************************************************************/
umtrx_iface::sptr umtrx_iface::make(udp_simple::sptr ctrl_transport){
    return umtrx_iface::sptr(new umtrx_iface_impl(ctrl_transport));
}

