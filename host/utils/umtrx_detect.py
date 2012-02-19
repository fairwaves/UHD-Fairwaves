#!/usr/bin/env python
#
# Copyright 2012 Fairwaves
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
import re, struct, socket, subprocess
########################################################################
# constants
########################################################################
UDP_CONTROL_PORT = 49152
UDP_MAX_XFER_BYTES = 1024
UDP_TIMEOUT = 3
UDP_POLL_INTERVAL = 0.10 #in seconds
USRP2_CONTROL_PROTO_VERSION = 11 # must match firmware proto

#see fw_common.h
CONTROL_FMT = '!LLL24x'
CONTROL_IP_FMT = '!LLLL20x'

n2xx_revs = {
  0x0a00: ["n200_r3", "n200_r2"],
  0x0a10: ["n200_r4"],
  0x0a01: ["n210_r3", "n210_r2"],
  0x0a11: ["n210_r4"],
  0xfa00: ["umtrx"],
  }

USRP2_CTRL_ID_HUH_WHAT = ord(' ')
USRP2_CTRL_ID_WAZZUP_BRO = ord('a')
USRP2_CTRL_ID_WAZZUP_DUDE = ord('A')
UMTRX_CTRL_ID_REQUEST = ord('u')
UMTRX_CTRL_ID_RESPONSE = ord('U')
USRP2_CTRL_ID_HOLLER_AT_ME_BRO = ord('l')

_seq = -1
def seq():
    global _seq
    _seq = _seq+1
    return _seq

########################################################################
# helper functions
########################################################################
def unpack_control_fmt(s):
    return struct.unpack(CONTROL_FMT, s) #proto_ver, pktid, seq, data

def unpack_control_ip_fmt(s):
    return struct.unpack(CONTROL_IP_FMT, s) #proto_ver, pktid, seq, ip_addr

def pack_control_fmt(proto_ver, pktid, seq):
    return struct.pack(CONTROL_FMT, proto_ver, pktid, seq)


########################################################################
# interface discovery and device enumeration
########################################################################
def command(*args):
    p = subprocess.Popen(args, stdout = subprocess.PIPE, stderr = subprocess.STDOUT)
    ret = p.wait()
    verbose = p.stdout.read().decode()
    if ret != 0: raise Exception(verbose)
    return verbose

def unix_get_interfaces():
    ifconfig = command("/sbin/ifconfig")
    ip_addr_re = "cast\D*(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})"
    bcasts = re.findall(ip_addr_re, ifconfig)
    return bcasts

def enumerate_devices():
  for bcast_addr in unix_get_interfaces():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    sock.settimeout(0.1)
    out_pkt = pack_control_fmt(USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST, 0)
    print "[%s] Sending %d bytes: %x, '%c'\n" % (bcast_addr, len(out_pkt), USRP2_CONTROL_PROTO_VERSION, UMTRX_CTRL_ID_REQUEST)
    sock.sendto(out_pkt, (bcast_addr, UDP_CONTROL_PORT))
    still_goin = True
    while(still_goin):
      try:
        pkt = sock.recv(UDP_MAX_XFER_BYTES)
        (proto_ver, pktid, rxseq, ip_addr) = unpack_control_ip_fmt(pkt)
        print "[%s] Received %d bytes: %x, '%c', %x, %s\n" % (bcast_addr, len(pkt), proto_ver, pktid, rxseq, socket.inet_ntoa(struct.pack("<L", socket.ntohl(ip_addr))))
      except socket.timeout:
        still_goin = False

########################################################################
# N. B. heavily stripped-down, unportable version
########################################################################
if __name__ == '__main__':

  print 'broadcasting over interfaces:'
  enumerate_devices()
  

