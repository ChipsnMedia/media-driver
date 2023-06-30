/*
* Copyright (c) 2021, Intel Corporation
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
//! \file     media_ddi_encode_av1.cpp
//! \brief    Defines class for DDI media av1 encode.
//!

#include "media_libva.h"
#include "media_libva_util.h"
#include "media_libva_encoder.h"
#include "media_ddi_factory.h"
#include "media_ddi_encode_base.h"
#include "media_ddi_encode_av1.h"
#include "media_ddi_encode_const.h"
#include "codec_def_encode_av1.h"
#include "mos_solo_generic.h"
#include "media_ddi_encode_const.h"
// 
enum
{
    ENCODE_MODE_NULL = 0,
    ENCODE_MODE_AVC,
    ENCODE_MODE_MPEG2,
    ENCODE_MODE_VP8,
    ENCODE_MODE_JPEG,
    ENCODE_MODE_HEVC,
    ENCODE_MODE_VP9,
    ENCODE_MODE_AV1,
    NUM_ENCODE_MODES
};

extern template class MediaDdiFactoryNoArg<DdiEncodeBase>;

static bool isEncodeAV1Registered =
MediaDdiFactoryNoArg<DdiEncodeBase>::RegisterCodec<DdiEncodeAV1>(ENCODE_ID_AV1);
DdiEncodeAV1::~DdiEncodeAV1()
{
}

VAStatus DdiEncodeAV1::ContextInitialize(
    CodechalSetting *codecHalSettings)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::RenderPicture(
    VADriverContextP ctx,
    VAContextID      context,
    VABufferID       *buffers,
    int32_t          numBuffers)
{

    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::EncodeInCodecHal(uint32_t numSlices)
{
    return VA_STATUS_SUCCESS;
}

// Reset the parameters before each frame
VAStatus DdiEncodeAV1::ResetAtFrameLevel()
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParseSeqParams(void *ptr)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParsePicParams(DDI_MEDIA_CONTEXT *mediaCtx, void *ptr)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParseTileGroupParams(void *ptr, uint32_t numTileGroupParams)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParsePackedHeaderParams(void *ptr)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParsePackedHeaderData(void *ptr)
{
    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParseMiscParams(void *ptr)
{

    return VA_STATUS_SUCCESS;
}

VAStatus DdiEncodeAV1::ParseSegMapParams(void *ptr)
{
    return VA_STATUS_SUCCESS;
}
uint32_t DdiEncodeAV1::getSequenceParameterBufferSize()
{
    return sizeof(VAEncSequenceParameterBufferAV1);
}

uint32_t DdiEncodeAV1::getPictureParameterBufferSize()
{
    return sizeof(VAEncPictureParameterBufferAV1);
}

uint32_t DdiEncodeAV1::getSliceParameterBufferSize()
{
    return sizeof(VAEncTileGroupBufferAV1);
}



