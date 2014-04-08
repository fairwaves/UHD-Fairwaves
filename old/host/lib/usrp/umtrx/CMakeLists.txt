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

########################################################################
# This file included, use CMake directory variables
########################################################################

########################################################################
# Conditionally configure the UmTRX support
########################################################################
LIBUHD_REGISTER_COMPONENT("UMTRX" ENABLE_UMTRX ON "ENABLE_LIBUHD" OFF)

IF(ENABLE_UMTRX)
    LIBUHD_APPEND_SOURCES(
        ${CMAKE_CURRENT_SOURCE_DIR}/dboard_iface.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/io_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/umtrx_impl.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/lms6002d.cpp
    )
ENDIF(ENABLE_UMTRX)
