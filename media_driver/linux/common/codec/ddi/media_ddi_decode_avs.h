/*
* Copyright (c) 2022, Mthreads Corporation
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
//! \file     media_ddi_decode_avs.h
//! \brief    Defines DdiDecodeAVS class for AVS decode
//!

#ifndef __MEDIA_DDI_DECODER_AVS_H__
#define __MEDIA_DDI_DECODER_AVS_H__

#include <va/va.h>
#include "media_ddi_decode_base.h"

//!
//! \class  DdiDecodeAVS
//! \brief  Ddi decode AVS
//!
class DdiDecodeAVS : public DdiMediaDecode
{
public:
    //!
    //! \brief Constructor
    //!
    DdiDecodeAVS(DDI_DECODE_CONFIG_ATTR *ddiDecodeAttr) : DdiMediaDecode(ddiDecodeAttr){};

    //!
    //! \brief Destructor
    //!
    virtual ~DdiDecodeAVS(){};

    virtual VAStatus RenderPicture(
        VADriverContextP ctx,
        VAContextID      context,
        VABufferID       *buffers,
        int32_t          numBuffers) override {
        return VA_STATUS_SUCCESS;
    }

    virtual VAStatus CodecHalInit(
        DDI_MEDIA_CONTEXT *mediaCtx,
        void              *ptr) override {
        return VA_STATUS_SUCCESS;
    }

    virtual VAStatus AllocSliceControlBuffer(
        DDI_MEDIA_BUFFER  *buf) override {
        return VA_STATUS_SUCCESS;
    }

private:

};

#endif /* _MEDIA_DDI_DECODE_AVS_H */

