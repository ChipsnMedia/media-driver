/*
* Copyright (c) 2016-2017, Intel Corporation
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/
//!
//! \file     media_ddi_decode_const.h
//! \brief    Add some const string definition for media_libva_decoder
//!

// The defined const string is used as the Key for supported decoding codec list. And
// it is included by each decoding codec.
// And it is also included in media_libva_decoder.c. The corresponding string is used
// as the key to search and create one instance from the supported decoding list.

#ifndef _MEDIA_LIBVA_DECODE_CONST_H_
#define _MEDIA_LIBVA_DECODE_CONST_H_

#define DECODE_ID_NONE      "VIDEO_DEC_NONE"
#define DECODE_ID_AVC       "VIDEO_DEC_H264"
#define DECODE_ID_VP8       "VIDEO_DEC_VP8"
#define DECODE_ID_VP9       "VIDEO_DEC_VP9"
#define DECODE_ID_HEVC      "VIDEO_DEC_HEVC"
#define DECODE_ID_MPEG2     "VIDEO_DEC_MPEG2"
#define DECODE_ID_JPEG      "VIDEO_DEC_JPEG"
#define DECODE_ID_VC1       "VIDEO_DEC_VC1"
#define DECODE_ID_AVS       "VIDEO_DEC_AVS"

#endif /*  _MEDIA_LIBVA_DECODE_CONST_H_ */
