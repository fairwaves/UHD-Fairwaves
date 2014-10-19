#!/usr/bin/env python
#
# Copyright 2010-2011 Ettus Research LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#

# TODO: make it autodetect UHD devices

import optparse
import math
import os
import re
import struct
import socket
import sys
import time
import platform
import subprocess

########################################################################
# constants
########################################################################
UDP_FW_UPDATE_PORT = 49154
UDP_MAX_XFER_BYTES = 1024
UDP_TIMEOUT = 3
UDP_POLL_INTERVAL = 0.10 #in seconds

USRP2_FW_PROTO_VERSION = 7 #should be unused after r6

#from bootloader_utils.h

class USRP_CONST:
    FPGA_IMAGE_SIZE_BYTES = 1572864
    FW_IMAGE_SIZE_BYTES = 31744
    SAFE_FPGA_IMAGE_LOCATION_ADDR = 0x00000000
    SAFE_FW_IMAGE_LOCATION_ADDR = 0x003F0000
    PROD_FPGA_IMAGE_LOCATION_ADDR = 0x00180000
    PROD_FW_IMAGE_LOCATION_ADDR = 0x00300000

class UMTRX_CONST:
    FPGA_IMAGE_SIZE_BYTES = 2464964
    FW_IMAGE_SIZE_BYTES = 0x3fff
    SAFE_FPGA_IMAGE_LOCATION_ADDR = 0x00000000
    SAFE_FW_IMAGE_LOCATION_ADDR = 0x006F0000
    PROD_FPGA_IMAGE_LOCATION_ADDR = 0x00300000
    PROD_FW_IMAGE_LOCATION_ADDR = 0x00600000

FLASH_DATA_PACKET_SIZE = 256

#see fw_common.h
FLASH_ARGS_FMT = '!LLLLL256s'
FLASH_INFO_FMT = '!LLLLL256x'
FLASH_IP_FMT =   '!LLLL260x'
FLASH_HW_REV_FMT = '!LLLL260x'

n2xx_revs = {
             0x0a00: (["n200_r3", "n200_r2"], USRP_CONST()),
             0x0a10: (["n200_r4"], USRP_CONST()),
             0x0a01: (["n210_r3", "n210_r2"], USRP_CONST()),
             0x0a11: (["n210_r4"], USRP_CONST()),
             0xfa00: (["umtrx"], UMTRX_CONST())
            }

class update_id_t:
  USRP2_FW_UPDATE_ID_WAT = ord(' ')
  USRP2_FW_UPDATE_ID_OHAI_LOL = ord('a')
  USRP2_FW_UPDATE_ID_OHAI_OMG = ord('A')
  USRP2_FW_UPDATE_ID_WATS_TEH_FLASH_INFO_LOL = ord('f')
  USRP2_FW_UPDATE_ID_HERES_TEH_FLASH_INFO_OMG = ord('F')
  USRP2_FW_UPDATE_ID_ERASE_TEH_FLASHES_LOL = ord('e')
  USRP2_FW_UPDATE_ID_ERASING_TEH_FLASHES_OMG = ord('E')
  USRP2_FW_UPDATE_ID_R_U_DONE_ERASING_LOL = ord('d')
  USRP2_FW_UPDATE_ID_IM_DONE_ERASING_OMG = ord('D')
  USRP2_FW_UPDATE_ID_NOPE_NOT_DONE_ERASING_OMG = ord('B')
  USRP2_FW_UPDATE_ID_WRITE_TEH_FLASHES_LOL = ord('w')
  USRP2_FW_UPDATE_ID_WROTE_TEH_FLASHES_OMG = ord('W')
  USRP2_FW_UPDATE_ID_READ_TEH_FLASHES_LOL = ord('r')
  USRP2_FW_UPDATE_ID_KK_READ_TEH_FLASHES_OMG = ord('R')
  USRP2_FW_UPDATE_ID_RESET_MAH_COMPUTORZ_LOL = ord('s')
  USRP2_FW_UPDATE_ID_RESETTIN_TEH_COMPUTORZ_OMG = ord('S')
  USRP2_FW_UPDATE_ID_I_CAN_HAS_HW_REV_LOL = ord('v')
  USRP2_FW_UPDATE_ID_HERES_TEH_HW_REV_OMG = ord('V')
  USRP2_FW_UPDATE_ID_KTHXBAI = ord('~')

_seq = -1
def seq():
    global _seq
    _seq = _seq+1
    return _seq

########################################################################
# helper functions
########################################################################
def unpack_flash_args_fmt(s):
    return struct.unpack(FLASH_ARGS_FMT, s) #(proto_ver, pktid, seq, flash_addr, length, data)

def unpack_flash_info_fmt(s):
    return struct.unpack(FLASH_INFO_FMT, s) #(proto_ver, pktid, seq, sector_size_bytes, memory_size_bytes)

def unpack_flash_ip_fmt(s):
    return struct.unpack(FLASH_IP_FMT, s) #(proto_ver, pktid, seq, ip_addr)

def unpack_flash_hw_rev_fmt(s):
    return struct.unpack(FLASH_HW_REV_FMT, s) #proto_ver, pktid, seq, hw_rev

def pack_flash_args_fmt(proto_ver, pktid, seq, flash_addr, length, data=bytes()):
    return struct.pack(FLASH_ARGS_FMT, proto_ver, pktid, seq, flash_addr, length, data)

def pack_flash_info_fmt(proto_ver, pktid, seq, sector_size_bytes, memory_size_bytes):
    return struct.pack(FLASH_INFO_FMT, proto_ver, pktid, seq, sector_size_bytes, memory_size_bytes)

def pack_flash_hw_rev_fmt(proto_ver, pktid, seq, hw_rev):
    return struct.pack(FLASH_HW_REV_FMT, proto_ver, pktid, seq, hw_rev)

def is_valid_fpga_image(fpga_image):
    for i in range(0,63):
        if fpga_image[i:i+1] == bytes(b'\xFF'): continue
        if fpga_image[i:i+2] == bytes(b'\xAA\x99'): return True
    return False

def is_valid_fw_image(fw_image):
    return fw_image[:4] == bytes(b'\x0B\x0B\x0B\x0B')


########################################################################
# interface discovery and device enumeration
########################################################################

def get_interfaces():
    if(platform.system() is "Windows"): return win_get_interfaces()
    else: return unix_get_interfaces()

def unix_get_interfaces():
    ipint = subprocess.check_output(["/sbin/ip", "-4", "a", "s", "up"])
    ip_addr_re = "brd\D*(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})"
    bcasts = re.findall(ip_addr_re, ipint)
    return bcasts

def win_get_interfaces():
    from ctypes import Structure, windll, sizeof
    from ctypes import POINTER, byref
    from ctypes import c_ulong, c_uint, c_ubyte, c_char
    MAX_ADAPTER_DESCRIPTION_LENGTH = 128
    MAX_ADAPTER_NAME_LENGTH = 256
    MAX_ADAPTER_ADDRESS_LENGTH = 8
    class IP_ADDR_STRING(Structure):
        pass
    LP_IP_ADDR_STRING = POINTER(IP_ADDR_STRING)
    IP_ADDR_STRING._fields_ = [
        ("next", LP_IP_ADDR_STRING),
        ("ipAddress", c_char * 16),
        ("ipMask", c_char * 16),
        ("context", c_ulong)]
    class IP_ADAPTER_INFO (Structure):
        pass
    LP_IP_ADAPTER_INFO = POINTER(IP_ADAPTER_INFO)
    IP_ADAPTER_INFO._fields_ = [
        ("next", LP_IP_ADAPTER_INFO),
        ("comboIndex", c_ulong),
        ("adapterName", c_char * (MAX_ADAPTER_NAME_LENGTH + 4)),
        ("description", c_char * (MAX_ADAPTER_DESCRIPTION_LENGTH + 4)),
        ("addressLength", c_uint),
        ("address", c_ubyte * MAX_ADAPTER_ADDRESS_LENGTH),
        ("index", c_ulong),
        ("type", c_uint),
        ("dhcpEnabled", c_uint),
        ("currentIpAddress", LP_IP_ADDR_STRING),
        ("ipAddressList", IP_ADDR_STRING),
        ("gatewayList", IP_ADDR_STRING),
        ("dhcpServer", IP_ADDR_STRING),
        ("haveWins", c_uint),
        ("primaryWinsServer", IP_ADDR_STRING),
        ("secondaryWinsServer", IP_ADDR_STRING),
        ("leaseObtained", c_ulong),
        ("leaseExpires", c_ulong)]
    GetAdaptersInfo = windll.iphlpapi.GetAdaptersInfo
    GetAdaptersInfo.restype = c_ulong
    GetAdaptersInfo.argtypes = [LP_IP_ADAPTER_INFO, POINTER(c_ulong)]
    adapterList = (IP_ADAPTER_INFO * 10)()
    buflen = c_ulong(sizeof(adapterList))
    rc = GetAdaptersInfo(byref(adapterList[0]), byref(buflen))
    if rc == 0:
        for a in adapterList:
            adNode = a.ipAddressList
            while True:
                #convert ipAddr and ipMask into hex addrs that can be turned into a bcast addr
                ipAddr = adNode.ipAddress.decode()
                ipMask = adNode.ipMask.decode()
                if ipAddr and ipMask:
                    hexAddr = struct.unpack("<L", socket.inet_aton(ipAddr))[0]
                    hexMask = struct.unpack("<L", socket.inet_aton(ipMask))[0]
                    if(hexAddr and hexMask): #don't broadcast on 255.255.255.255, that's just lame
                        yield socket.inet_ntoa(struct.pack("<L", (hexAddr & hexMask) | (~hexMask) & 0xFFFFFFFF))
                adNode = adNode.next
                if not adNode:
                    break

def enumerate_devices():
    for bcast_addr in get_interfaces():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(0.1)
        out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_OHAI_LOL, 0, 0, 0)
        sock.sendto(out_pkt, (bcast_addr, UDP_FW_UPDATE_PORT))
        still_goin = True
        while(still_goin):
            try:
                pkt = sock.recv(UDP_MAX_XFER_BYTES)
                (proto_ver, pktid, rxseq, ip_addr) = unpack_flash_ip_fmt(pkt)
                if(pktid == update_id_t.USRP2_FW_UPDATE_ID_OHAI_OMG):
                    yield socket.inet_ntoa(struct.pack("<L", socket.ntohl(ip_addr)))
            except socket.timeout:
                still_goin = False

########################################################################
# Burner class, holds a socket and send/recv routines
########################################################################
class burner_socket(object):
    def __init__(self, addr):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(UDP_TIMEOUT)
        self._sock.connect((addr, UDP_FW_UPDATE_PORT))
        self.set_callbacks(lambda *a: None, lambda *a: None)
        self.init_update() #check that the device is there
        self.get_hw_rev()

    def set_callbacks(self, progress_cb, status_cb):
        self._progress_cb = progress_cb
        self._status_cb = status_cb

    def send_and_recv(self, pkt):
        self._sock.send(pkt)
        return self._sock.recv(UDP_MAX_XFER_BYTES)

    #just here to validate comms
    def init_update(self):
        out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_OHAI_LOL, seq(), 0, 0)
        try: in_pkt = self.send_and_recv(out_pkt)
        except socket.timeout: raise Exception("No response from device")
        (proto_ver, pktid, rxseq, ip_addr) = unpack_flash_ip_fmt(in_pkt)
        if pktid == update_id_t.USRP2_FW_UPDATE_ID_OHAI_OMG:
            print("USRP-N2XX found.")
        else:
            raise Exception("Invalid reply received from device.")

    def get_hw_rev(self):
        out_pkt = pack_flash_hw_rev_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_I_CAN_HAS_HW_REV_LOL, seq(), 0)
        in_pkt = self.send_and_recv(out_pkt)
        (proto_ver, pktid, rxseq, hw_rev) = unpack_flash_hw_rev_fmt(in_pkt)
        if(pktid != update_id_t.USRP2_FW_UPDATE_ID_HERES_TEH_HW_REV_OMG): hw_rev = 0
        return socket.ntohs(hw_rev)

    memory_size_bytes = 0
    sector_size_bytes = 0
    def get_flash_info(self):
        if (self.memory_size_bytes != 0) and (self.sector_size_bytes != 0):
            return (self.memory_size_bytes, self.sector_size_bytes)

        out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_WATS_TEH_FLASH_INFO_LOL, seq(), 0, 0)
        in_pkt = self.send_and_recv(out_pkt)

        (proto_ver, pktid, rxseq, self.sector_size_bytes, self.memory_size_bytes) = unpack_flash_info_fmt(in_pkt)

        if pktid != update_id_t.USRP2_FW_UPDATE_ID_HERES_TEH_FLASH_INFO_OMG:
            raise Exception("Invalid reply %c from device." % (chr(pktid)))

        return (self.memory_size_bytes, self.sector_size_bytes)

    def burn_fw(self, fw, fpga, reset, safe, check_rev=True, erase=False):
        (flash_size, sector_size) = self.get_flash_info()
        hw_rev = self.get_hw_rev()

        if n2xx_revs.has_key(hw_rev): print("Hardware type: %s" % n2xx_revs[hw_rev][0][0])
        print("Flash size: %i\nSector size: %i\n" % (flash_size, sector_size))

        if erase:
            if fpga is not None:
                hw_const = n2xx_revs[hw_rev][1]

                if safe: image_location = hw_const.SAFE_FPGA_IMAGE_LOCATION_ADDR
                else:    image_location = hw_const.PROD_FPGA_IMAGE_LOCATION_ADDR

                print("Begin FPGA erase: this should take about 5 seconds...")
                start_time = time.time()
                self.erase_image(image_location, hw_const.FPGA_IMAGE_SIZE_BYTES)
                print("Time elapsed: %f seconds"%(time.time() - start_time))
                print("\n\n")

            if fw is not None:
                hw_const = n2xx_revs[hw_rev][1]

                if safe: image_location = hw_const.SAFE_FW_IMAGE_LOCATION_ADDR
                else:    image_location = hw_const.PROD_FW_IMAGE_LOCATION_ADDR

                print("Begin firmware erase: this should take about 1 second...")
                start_time = time.time()
                self.erase_image(image_location, hw_const.FW_IMAGE_SIZE_BYTES)
                print("Time elapsed: %f seconds"%(time.time() - start_time))
                print("\n\n")

            return

        if fpga:
            #validate fpga image name against hardware rev
            if(check_rev and hw_rev != 0 and not any(name in fpga for name in n2xx_revs[hw_rev][0])):
                raise Exception("Error: incorrect FPGA image version. Please use the correct image for device %s" % n2xx_revs[hw_rev][0][0])

            hw_const = n2xx_revs[hw_rev][1]

            if safe: image_location = hw_const.SAFE_FPGA_IMAGE_LOCATION_ADDR
            else:    image_location = hw_const.PROD_FPGA_IMAGE_LOCATION_ADDR

            fpga_file = open(fpga, 'rb')
            fpga_image = fpga_file.read()

            if len(fpga_image) > hw_const.FPGA_IMAGE_SIZE_BYTES:
                raise Exception("Error: FPGA image file too large.")

            if not is_valid_fpga_image(fpga_image):
                raise Exception("Error: Invalid FPGA image file.")

            if (len(fpga_image) + image_location) > flash_size:
                raise Exception("Error: Cannot write past end of device")

            print("Begin FPGA write: this should take about 1 minute...")
            start_time = time.time()
            self.erase_image(image_location, hw_const.FPGA_IMAGE_SIZE_BYTES)
            self.write_image(fpga_image, image_location)
            self.verify_image(fpga_image, image_location)
            print("Time elapsed: %f seconds"%(time.time() - start_time))
            print("\n\n")

        if fw:
            hw_const = n2xx_revs[hw_rev][1]

            if safe: image_location = hw_const.SAFE_FW_IMAGE_LOCATION_ADDR
            else:    image_location = hw_const.PROD_FW_IMAGE_LOCATION_ADDR

            fw_file = open(fw, 'rb')
            fw_image = fw_file.read()

            if len(fw_image) > hw_const.FW_IMAGE_SIZE_BYTES:
                raise Exception("Error: Firmware image file too large.")

            if not is_valid_fw_image(fw_image):
                raise Exception("Error: Invalid firmware image file.")

            if (len(fw_image) + image_location) > flash_size:
                raise Exception("Error: Cannot write past end of device")

            print("Begin firmware write: this should take about 1 second...")
            start_time = time.time()
            self.erase_image(image_location, hw_const.FW_IMAGE_SIZE_BYTES)
            self.write_image(fw_image, image_location)
            self.verify_image(fw_image, image_location)
            print("Time elapsed: %f seconds"%(time.time() - start_time))
            print("\n\n")

        if reset: self.reset_usrp()

    def write_image(self, image, addr):
        print("Writing image")
        self._status_cb("Writing")
        writedata = image
        #we split the image into smaller (256B) bits and send them down the wire
        (mem_size, sector_size) = self.get_flash_info()
        if (addr + len(writedata)) > mem_size:
            raise Exception("Error: Cannot write past end of device")

        while writedata:
            out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_WRITE_TEH_FLASHES_LOL, seq(), addr, FLASH_DATA_PACKET_SIZE, writedata[:FLASH_DATA_PACKET_SIZE])
            in_pkt = self.send_and_recv(out_pkt)

            (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)

            if pktid != update_id_t.USRP2_FW_UPDATE_ID_WROTE_TEH_FLASHES_OMG:
              raise Exception("Invalid reply %c from device." % (chr(pktid)))

            writedata = writedata[FLASH_DATA_PACKET_SIZE:]
            addr += FLASH_DATA_PACKET_SIZE
            self._progress_cb(float(len(image)-len(writedata))/len(image))

    def verify_image(self, image, addr):
        print("Verifying data")
        self._status_cb("Verifying")
        readsize = len(image)
        readdata = bytes()
        while readsize > 0:
            if readsize < FLASH_DATA_PACKET_SIZE: thisreadsize = readsize
            else: thisreadsize = FLASH_DATA_PACKET_SIZE
            out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_READ_TEH_FLASHES_LOL, seq(), addr, thisreadsize)
            in_pkt = self.send_and_recv(out_pkt)

            (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)

            if pktid != update_id_t.USRP2_FW_UPDATE_ID_KK_READ_TEH_FLASHES_OMG:
              raise Exception("Invalid reply %c from device." % (chr(pktid)))

            readdata += data[:thisreadsize]
            readsize -= FLASH_DATA_PACKET_SIZE
            addr += FLASH_DATA_PACKET_SIZE
            self._progress_cb(float(len(readdata))/len(image))

        print("Read back %i bytes" % len(readdata))
        #  print readdata

        #  for i in range(256, 512):
        #    print "out: %i in: %i" % (ord(image[i]), ord(readdata[i]))

        if readdata != image:
            raise Exception("Verify failed. Image did not write correctly.")
        else:
            print("Success.")

    def read_image(self, image, size, addr):
        print("Reading image")
        readsize = size
        readdata = str()
        while readsize > 0:
            if readsize < FLASH_DATA_PACKET_SIZE: thisreadsize = readsize
            else: thisreadsize = FLASH_DATA_PACKET_SIZE
            out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_READ_TEH_FLASHES_LOL, seq(), addr, thisreadsize)
            in_pkt = self.send_and_recv(out_pkt)

            (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)

            if pktid != update_id_t.USRP2_FW_UPDATE_ID_KK_READ_TEH_FLASHES_OMG:
              raise Exception("Invalid reply %c from device." % (chr(pktid)))

            readdata += data[:thisreadsize]
            readsize -= FLASH_DATA_PACKET_SIZE
            addr += FLASH_DATA_PACKET_SIZE

        print("Read back %i bytes" % len(readdata))

        #write to disk
        f = open(image, 'w')
        f.write(readdata)
        f.close()

    def reset_usrp(self):
        out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_RESET_MAH_COMPUTORZ_LOL, seq(), 0, 0)
        try: in_pkt = self.send_and_recv(out_pkt)
        except socket.timeout: return

        (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)
        if pktid == update_id_t.USRP2_FW_UPDATE_ID_RESETTIN_TEH_COMPUTORZ_OMG:
            raise Exception("Device failed to reset.")

    def erase_image(self, addr, length):
        self._status_cb("Erasing")
        #get flash info first
        (flash_size, sector_size) = self.get_flash_info()
        if (addr + length) > flash_size:
            raise Exception("Cannot erase past end of device")

        out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_ERASE_TEH_FLASHES_LOL, seq(), addr, length)
        in_pkt = self.send_and_recv(out_pkt)

        (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)

        if pktid != update_id_t.USRP2_FW_UPDATE_ID_ERASING_TEH_FLASHES_OMG:
            raise Exception("Invalid reply %c from device." % (chr(pktid)))

        print("Erasing %i bytes at %i" % (length, addr))
        start_time = time.time()

        #now wait for it to finish
        while(True):
            out_pkt = pack_flash_args_fmt(USRP2_FW_PROTO_VERSION, update_id_t.USRP2_FW_UPDATE_ID_R_U_DONE_ERASING_LOL, seq(), 0, 0)
            in_pkt = self.send_and_recv(out_pkt)

            (proto_ver, pktid, rxseq, flash_addr, rxlength, data) = unpack_flash_args_fmt(in_pkt)

            if pktid == update_id_t.USRP2_FW_UPDATE_ID_IM_DONE_ERASING_OMG: break
            elif pktid != update_id_t.USRP2_FW_UPDATE_ID_NOPE_NOT_DONE_ERASING_OMG:
                raise Exception("Invalid reply %c from device." % (chr(pktid)))
            time.sleep(0.01) #decrease network overhead by waiting a bit before polling
            self._progress_cb(min(1.0, (time.time() - start_time)/(length/80e3)))


########################################################################
# command line options
########################################################################
def get_options():
    parser = optparse.OptionParser()
    parser.add_option("--addr", type="string",                 help="USRP-N2XX device address",       default='')
    parser.add_option("--fw",   type="string",                 help="firmware image path (optional)", default='')
    parser.add_option("--fpga", type="string",                 help="fpga image path (optional)",     default='')
    parser.add_option("--reset", action="store_true",          help="reset the device after writing", default=False)
    parser.add_option("--read", action="store_true",           help="read to file instead of write from file", default=False)
    parser.add_option("--erase", action="store_true",          help="erase appropriate part of the flash, do not write anything", default=False)
    parser.add_option("--overwrite-safe", action="store_true", help="never ever use this option", default=False)
    parser.add_option("--dont-check-rev", action="store_true", help="disable revision checks", default=False)
    parser.add_option("--list", action="store_true",           help="list possible network devices", default=False)
    (options, args) = parser.parse_args()

    return options

########################################################################
# main
########################################################################
if __name__=='__main__':
    options = get_options()

    if options.list:
        print('Possible network devices:')
        print('  ' + '\n  '.join(enumerate_devices()))
        exit()

    if not options.addr: raise Exception('no address specified')

    if options.erase and (options.fpga is None and options.fw is None):
        raise Exception('Must specify either a firmware image or FPGA image to erase.')
    elif not options.fpga and not options.fw and not options.reset: raise Exception('Must specify either a firmware image or FPGA image, and/or reset.')

    if options.overwrite_safe and not options.read:
        print("Are you REALLY, REALLY sure you want to overwrite the safe image? This is ALMOST ALWAYS a terrible idea.")
        print("If your image is faulty, your USRP2+ will become a brick until reprogrammed via JTAG.")
        response = raw_input("""Type "yes" to continue, or anything else to quit: """)
        if response != "yes": sys.exit(0)

    burner = burner_socket(addr=options.addr)

    if options.read:
        hw_rev = burner.get_hw_rev()
        hw_const = n2xx_revs[hw_rev][1]

        if options.fw:
            file = options.fw
            if os.path.isfile(file):
                response = raw_input("File already exists -- overwrite? (y/n) ")
                if response != "y": sys.exit(0)
            size = hw_const.FW_IMAGE_SIZE_BYTES
            addr = hw_const.SAFE_FW_IMAGE_LOCATION_ADDR if options.overwrite_safe else hw_const.PROD_FW_IMAGE_LOCATION_ADDR
            burner.read_image(file, size, addr)

        if options.fpga:
            file = options.fpga
            if os.path.isfile(file):
                response = raw_input("File already exists -- overwrite? (y/n) ")
                if response != "y": sys.exit(0)
            size = hw_const.FPGA_IMAGE_SIZE_BYTES
            addr = hw_const.SAFE_FPGA_IMAGE_LOCATION_ADDR if options.overwrite_safe else hw_const.PROD_FPGA_IMAGE_LOCATION_ADDR
            burner.read_image(file, size, addr)

    else: burner.burn_fw(fw=options.fw, fpga=options.fpga, reset=options.reset, safe=options.overwrite_safe, check_rev=not options.dont_check_rev, erase=options.erase)
