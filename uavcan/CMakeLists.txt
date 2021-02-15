############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################
set(LIBUAVCAN_DIR ${PX4_SOURCE_DIR}/src/drivers/uavcan/libuavcan)
set(LIBUAVCAN_DIR_DRIVERS ${PX4_SOURCE_DIR}/src/drivers/uavcan/uavcan_drivers)
set(CMAKE_CXX_STANDARD 11)

px4_add_git_submodule(TARGET git_uavcan PATH ${LIBUAVCAN_DIR})
# generated DSDL
set(DSDLC_DIR "${CMAKE_CURRENT_SOURCE_DIR}/dsdl")
set(DSDLC_INPUTS
	"${DSDLC_DIR}/com"
	"${DSDLC_DIR}/ardupilot"
	"${LIBUAVCAN_DIR}/dsdl/uavcan"
)
set(DSDLC_OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/include/dsdlc_generated")
set(DSDLC_INPUT_FILES)
foreach(DSDLC_INPUT ${DSDLC_INPUTS})
	file(GLOB_RECURSE DSDLC_NEW_INPUT_FILES ${CMAKE_CURRENT_SOURCE_DIR} "${DSDLC_INPUT}/*.uavcan")
	list(APPEND DSDLC_INPUT_FILES ${DSDLC_NEW_INPUT_FILES})
endforeach(DSDLC_INPUT)
add_custom_command(OUTPUT px4_uavcan_dsdlc_run.stamp
	COMMAND ${PYTHON_EXECUTABLE} ${LIBUAVCAN_DIR}/libuavcan/dsdl_compiler/libuavcan_dsdlc ${DSDLC_INPUTS} -O${DSDLC_OUTPUT}
	COMMAND ${CMAKE_COMMAND} -E touch px4_uavcan_dsdlc_run.stamp
	DEPENDS ${DSDLC_INPUT_FILES}
	COMMENT "PX4 UAVCAN dsdl compiler"
)
add_custom_target(px4_uavcan_dsdlc DEPENDS px4_uavcan_dsdlc_run.stamp)


px4_add_module(
	MODULE examples__uavcan
	MAIN uavcan
	COMPILE_FLAGS
		-fexceptions
	INCLUDES
		${DSDLC_OUTPUT}
		${LIBUAVCAN_DIR}/libuavcan/include
		${LIBUAVCAN_DIR}/libuavcan/include/dsdlc_generated
		${LIBUAVCAN_DIR}/libuavcan_drivers/posix/include
		${LIBUAVCAN_DIR_DRIVERS}/${UAVCAN_DRIVER}/driver/include
	SRCS
		uavcan.cpp
	DEPENDS
	)
