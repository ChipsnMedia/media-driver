# Copyright (c) 2019-2021, Intel Corporation
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
# OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
# ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
# OTHER DEALINGS IN THE SOFTWARE.

set(TMP_SOURCES_
    ${CMAKE_CURRENT_LIST_DIR}/vp_cmd_packet.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_packet_pipe.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_ief.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_sfc_base.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_vebox_cmd_packet.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_kernel_obj.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_cmd_packet.cpp
    ${CMAKE_CURRENT_LIST_DIR}/vp_kernel_config.cpp
)

set(TMP_HEADERS_
    ${CMAKE_CURRENT_LIST_DIR}/vp_cmd_packet.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_packet_pipe.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_ief.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_sfc_base.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_vebox_cmd_packet.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_sfc_common.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_vebox_common.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_common.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_packet_shared_context.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_kernel_obj.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_render_cmd_packet.h
    ${CMAKE_CURRENT_LIST_DIR}/vp_kernel_config.h
)

set(SOURCES_NEW
    ${SOURCES_NEW}
    ${TMP_SOURCES_}
)

set(HEADERS_NEW
    ${HEADERS_NEW}
    ${TMP_HEADERS_}
)

set(COMMON_SOURCES_
    ${COMMON_SOURCES_}
    ${TMP_SOURCES_}
)

set(COMMON_HEADERS_
    ${COMMON_HEADERS_}
    ${TMP_HEADERS_}
)

source_group( VpHalNext\\Shared FILES ${TMP_SOURCES_} ${TMP_HEADERS_})

media_add_curr_to_include_path()