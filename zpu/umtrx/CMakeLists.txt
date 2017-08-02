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

########################################################################
INCLUDE(${CMAKE_SOURCE_DIR}/lib/CMakeLists.txt)
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR})

SET(MIN_STACK 1300)

ADD_DEFINITIONS(-DUSRP2P)
ADD_DEFINITIONS(-DUDP_UART_MASK=2 -DNUM_UARTS_STORAGE=1) #GPS=UART1 streaming enabled

SET(libumtrxfw_FILES
    ${COMMON_SRCS}
    ${CMAKE_SOURCE_DIR}/umtrx/spif.c
    ${CMAKE_SOURCE_DIR}/umtrx/spi_flash.c
    ${CMAKE_SOURCE_DIR}/umtrx/spi_flash_read.c
    ${CMAKE_SOURCE_DIR}/umtrx/bootloader_utils.c
    ${CMAKE_SOURCE_DIR}/umtrx/ethernet.c
    ${CMAKE_SOURCE_DIR}/umtrx/xilinx_s3_icap.c
    ${CMAKE_SOURCE_DIR}/umtrx/xilinx_s6_icap.c
    ${CMAKE_SOURCE_DIR}/umtrx/udp_fw_update.c
    ${CMAKE_SOURCE_DIR}/umtrx/u2p_init.c
)

# ADD_LIBRARY(libumtrxfw STATIC ${libumtrxfw_FILES})

ADD_SUBDIRECTORY(bootloader)

########################################################################
SET(GEN_OUTPUTS_BIN_SIZE 0x3fff)

ADD_EXECUTABLE(umtrx_txrx_uhd.elf ${CMAKE_SOURCE_DIR}/apps/txrx_uhd.c ${libumtrxfw_FILES})
# TARGET_LINK_LIBRARIES(umtrx_txrx_uhd.elf libumtrxfw)
GEN_OUTPUTS(umtrx_txrx_uhd.elf)
TEST_STACK_SIZE(umtrx_txrx_uhd.elf ${MIN_STACK})

