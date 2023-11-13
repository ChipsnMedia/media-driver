/*
 * Copyright (c) 2009-2021, Intel Corporation
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
//! \file     media_libva.cpp
//! \brief    libva(and its extension) interface implementaion.
//!

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <unistd.h>

#if !defined(ANDROID) && defined(X11_FOUND)
#include <X11/Xutil.h>
#endif

#include <linux/fb.h>

#include "media_libva.h"

#include "media_libva_util.h"
#include "media_libva_decoder.h"
#include "media_libva_encoder.h"
#if !defined(ANDROID) && defined(X11_FOUND)
#include "media_libva_putsurface_linux.h"
#endif
#include "media_libva_vp.h"
#include "media_ddi_prot.h"
#include "mos_os.h"

#include "hwinfo_linux.h"
#include "mediamemdecomp.h"
#include "mos_solo_generic.h"
#include "media_libva_caps.h"
#include "media_interfaces_mmd.h"
#include "media_interfaces_mcpy.h"
#include "media_user_settings_mgr.h"
#include "media_interfaces.h"
#include "mos_interface.h"
#include "drm_fourcc.h"
#include "media_libva_interface.h"
#include "media_libva_interface_next.h"

// #define FAKE_VPUAPI
#ifdef FAKE_VPUAPI
#define FAKE_PRODUCT_ID PRODUCT_ID_627
static uint32_t s_Inst;
static DecHandle s_decHandle = (DecHandle)&s_Inst;
static EncHandle s_EncHandle = (EncHandle)&s_Inst;
#endif

#ifdef __cplusplus
extern "C"
{
#endif
    extern VAStatus DdiDestroyContextCM(VADriverContextP vaDrvCtx, VAContextID vaCtxID);
#ifdef __cplusplus
}
#endif

extern template class MediaInterfacesFactory<MhwInterfaces>;

VAProcFilterType vp_supported_filters[DDI_VP_MAX_NUM_FILTERS] = {
    VAProcFilterNoiseReduction,
    VAProcFilterDeinterlacing,
    VAProcFilterSharpening,
    VAProcFilterColorBalance,
    VAProcFilterSkinToneEnhancement,
    VAProcFilterTotalColorCorrection,
    VAProcFilterHVSNoiseReduction,
    VAProcFilterHighDynamicRangeToneMapping};

VAProcColorStandardType vp_input_color_std[DDI_VP_NUM_INPUT_COLOR_STD] = {
    VAProcColorStandardBT601,
    VAProcColorStandardBT709,
    VAProcColorStandardSRGB,
    VAProcColorStandardSTRGB,
    VAProcColorStandardBT2020,
    VAProcColorStandardExplicit};

VAProcColorStandardType vp_output_color_std[DDI_VP_NUM_OUT_COLOR_STD] = {
    VAProcColorStandardBT601,
    VAProcColorStandardBT709,
    VAProcColorStandardSRGB,
    VAProcColorStandardSTRGB,
    VAProcColorStandardBT2020,
    VAProcColorStandardExplicit};

// Making this API public since media_libva_vp.c calls this
VAStatus DdiMedia_MapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id, /* in */
    void **pbuf        /* out */
);

VAStatus DdiMedia_UnmapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id /* in */
);

VAStatus DdiMedia_DestroyImage(
    VADriverContextP ctx,
    VAImageID image);

#ifdef CNM_VPUAPI_INTERFACE_CAP
const VAImageFormat s_supportedImageformatsVPU[] =
    {
        {VA_FOURCC_NV12, VA_LSB_FIRST, 12, 0, 0, 0, 0, 0},
        {VA_FOURCC_NV21, VA_LSB_FIRST, 12, 0, 0, 0, 0, 0},
        {VA_FOURCC_I420, VA_LSB_FIRST, 12, 0, 0, 0, 0, 0},
        {VA_FOURCC_YV12, VA_LSB_FIRST, 12, 0, 0, 0, 0, 0},
        {VA_FOURCC_P010, VA_LSB_FIRST, 24, 0, 0, 0, 0, 0},
};

#define VPUAPI_DECODER_CONFIG_ID_START 0
#define VPUAPI_DECODER_CONFIG_ID_LEN 1024
#define VPUAPI_ENCODER_CONFIG_ID_START (VPUAPI_DECODER_CONFIG_ID_START + VPUAPI_DECODER_CONFIG_ID_LEN)
#define VPUAPI_ENCODER_CONFIG_ID_LEN 1024
#define VPUAPI_MAX_CONFIG_ID (VPUAPI_ENCODER_CONFIG_ID_START + VPUAPI_ENCODER_CONFIG_ID_LEN)

typedef struct
{
    VAProfile actual_profile;
    VAEntrypoint actual_entrypoint;
    VAConfigAttribType attrType;
    int value;
    int configId;
} VpuApiAttrMap;

typedef struct
{
    VAProfile profile;
    VAEntrypoint entrypoint[VPUAPI_MAX_ENTRYPOINT];
    int sizeOfEntrypoints;
} VpuApiCapMap;

static VpuApiAttrMap s_vpuApiAttrs[VPUAPI_MAX_ATTRIBUTE];
static VpuApiCapMap s_vpuApiCaps[VPUAPI_MAX_PROFILE];
static int s_sizeOfVpuApiCapMap = 0;
static int s_sizeOfVpuApiAttrMap = 0;
static int vpuApiCapInitDecAttributes(VAProfile profile, VAEntrypoint entrypoint, int config_id, VpuApiAttrMap *attrMap)
{
    int numAttributes = 0;

    attrMap->attrType = VAConfigAttribDecSliceMode;
    attrMap->value = VA_DEC_SLICE_MODE_NORMAL;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribRTFormat;
    attrMap->value = VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribMaxPictureWidth;
    attrMap->value = VPUAPI_MAX_PIC_WIDTH;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribMaxPictureHeight;
    attrMap->value = VPUAPI_MAX_PIC_HEIGHT;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    // printf("-%s numAttributes=%d\n", __FUNCTION__, numAttributes);
    return numAttributes;
}

static int vpuApiCapInitEncAttributes(VAProfile profile, VAEntrypoint entrypoint, int config_id, VpuApiAttrMap *attrMap)
{
    int numAttributes = 0;

    attrMap->attrType = VAConfigAttribRTFormat;
    attrMap->value = VA_RT_FORMAT_YUV420 | VA_RT_FORMAT_YUV420_10;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribRateControl;
    attrMap->value = VA_RC_CQP | VA_RC_CBR | VA_RC_VBR;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncPackedHeaders;
    attrMap->value = VA_ENC_PACKED_HEADER_PICTURE | VA_ENC_PACKED_HEADER_SEQUENCE | VA_ENC_PACKED_HEADER_SLICE | VA_ENC_PACKED_HEADER_RAW_DATA | VA_ENC_PACKED_HEADER_MISC;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncQualityRange;
    attrMap->value = NUM_TARGET_USAGE_MODES - 1; // Indicates TUs from 1 upto the value reported are supported
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncInterlaced;
    attrMap->value = VA_ENC_INTERLACED_NONE;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncMaxRefFrames;
    if (entrypoint == VAEntrypointEncSliceLP)
    {
        attrMap->value = 1 | (0 << 16); // 1 for L0, 0 for L1
    }
    else
    {
        attrMap->value = 1 | (1 << 16); // 1 for L0, 1 for L1
    }
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncMaxSlices;
    attrMap->value = 128;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncSliceStructure;
    attrMap->value = VA_ENC_SLICE_STRUCTURE_ARBITRARY_ROWS | VA_ENC_SLICE_STRUCTURE_EQUAL_ROWS | VA_ENC_SLICE_STRUCTURE_EQUAL_MULTI_ROWS | VA_ENC_SLICE_STRUCTURE_POWER_OF_TWO_ROWS; // for AVC
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncQuantization;
    attrMap->value = VA_ENC_QUANTIZATION_NONE;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncIntraRefresh;
    // attrMap->value = VA_ENC_INTRA_REFRESH_ROLLING_COLUMN | VA_ENC_INTRA_REFRESH_ROLLING_ROW;
    attrMap->value = VA_ENC_INTRA_REFRESH_ROLLING_ROW;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncSkipFrame;
    attrMap->value = 0;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncryption;
    attrMap->value = VA_ATTRIB_NOT_SUPPORTED;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncROI;
    {
        VAConfigAttribValEncROI roi_attrib = {0};
        roi_attrib.bits.num_roi_regions = 16;
        roi_attrib.bits.roi_rc_priority_support = 1;
        roi_attrib.bits.roi_rc_qp_delta_support = 1;
        attrMap->value = roi_attrib.value;
    }
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribProcessingRate;
    attrMap->value = VA_PROCESSING_RATE_NONE;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncDirtyRect;
    attrMap->value = 0;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncParallelRateControl;
    attrMap->value = 1;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribCustomRoundingControl;
    attrMap->value = 0;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribQPBlockSize;
    attrMap->value = 0;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribMaxFrameSize;
    attrMap->value = 0;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribPredictionDirection;
    attrMap->value = VA_PREDICTION_DIRECTION_PREVIOUS | VA_PREDICTION_DIRECTION_FUTURE | VA_PREDICTION_DIRECTION_BI_NOT_EMPTY;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribMaxPictureWidth;
    attrMap->value = VPUAPI_MAX_PIC_WIDTH;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribMaxPictureHeight;
    attrMap->value = VPUAPI_MAX_PIC_HEIGHT;
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncAV1;
    {
        VAConfigAttribValEncAV1 av1Attrib = {0};
        av1Attrib.bits.support_128x128_superblock = 0;
        av1Attrib.bits.support_filter_intra = 0;
        av1Attrib.bits.support_intra_edge_filter = 0;
        av1Attrib.bits.support_interintra_compound = 0;
        av1Attrib.bits.support_masked_compound = 0;
        av1Attrib.bits.support_warped_motion = 0;
        av1Attrib.bits.support_palette_mode = 0;
        av1Attrib.bits.support_dual_filter = 0;
        av1Attrib.bits.support_jnt_comp = 0;
        av1Attrib.bits.support_ref_frame_mvs = 0;
        av1Attrib.bits.support_superres = 0;
        av1Attrib.bits.support_restoration = 0;
        av1Attrib.bits.support_allow_intrabc = 0;
        av1Attrib.bits.support_cdef_channel_strength = 0;
        attrMap->value = av1Attrib.value;
    }
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncAV1Ext1;
    {
        VAConfigAttribValEncAV1Ext1 av1Ext1Attrib = {0};
        av1Ext1Attrib.bits.interpolation_filter = 1;
        av1Ext1Attrib.bits.min_segid_block_size_accepted = 2; // 64x64
        av1Ext1Attrib.bits.segment_feature_support = 0;
        attrMap->value = av1Ext1Attrib.value;
    }
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    attrMap++;
    attrMap->attrType = VAConfigAttribEncAV1Ext2;
    {
        VAConfigAttribValEncAV1Ext2 av1Ext2Attrib = {0};
        av1Ext2Attrib.bits.tile_size_bytes_minus1 = 3;
        av1Ext2Attrib.bits.obu_size_bytes_minus1 = 3;
        av1Ext2Attrib.bits.tx_mode_support = 4;
        av1Ext2Attrib.bits.max_tile_num_minus1 = (2 * 16); // row : 16, col : 2
        attrMap->value = av1Ext2Attrib.value;
    }
    attrMap->configId = config_id;
    attrMap->actual_profile = profile;
    attrMap->actual_entrypoint = entrypoint;
    numAttributes++;

    // printf("-%s numAttributes=%d\n", __FUNCTION__, numAttributes);
    return numAttributes;
}

static void VpuApiCapInit()
{
    int i;
    int j;
    int k;
    int numAttr;
    int decConfigId = 0;
    int encConfigId = 0;
    VpuApiCapMap *capMap = &s_vpuApiCaps[0];
    VpuApiAttrMap *attrMap = &s_vpuApiAttrs[0];
    // printf("+%s \n", __FUNCTION__);

    memset(capMap, 0x00, sizeof(s_vpuApiCaps));
    for (i = 0; i < VPUAPI_MAX_PROFILE; i++)
    {
        capMap[i].profile = VAProfileNone;
    }
    memset(attrMap, 0x00, sizeof(s_vpuApiAttrs));
    for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
    {
        attrMap[i].actual_profile = VAProfileNone;
    }

    // wave decoder doesn't support VAConfigAttribEncryption and VAConfigAttribDecProcessing attribute. so configID will be one value(0)
    i = 0;
    j = 0;
    capMap[i].profile = VAProfileH264Main;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

    i++;
    capMap[i].profile = VAProfileH264High;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

#ifdef VA_PROFILE_H264_HIGH_10
    i++;
    capMap[i].profile = VAProfileH264High10;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }
#endif

    i++;
    capMap[i].profile = VAProfileH264ConstrainedBaseline;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

    i++;
    capMap[i].profile = VAProfileHEVCMain;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

    i++;
    capMap[i].profile = VAProfileHEVCMain10;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

    i++;
    capMap[i].profile = VAProfileVP9Profile0;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].sizeOfEntrypoints = 1;
    numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[0], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
    decConfigId++;
    j = j + numAttr;

    i++;
    capMap[i].profile = VAProfileVP9Profile2;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].sizeOfEntrypoints = 1;
    numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[0], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
    decConfigId++;
    j = j + numAttr;

    i++;
    capMap[i].profile = VAProfileAV1Profile0;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].entrypoint[1] = VAEntrypointEncSlice;
    capMap[i].entrypoint[2] = VAEntrypointEncSliceLP;
    capMap[i].sizeOfEntrypoints = 3;
    for (k = 0; k < capMap[i].sizeOfEntrypoints; k++)
    {
        if (capMap[i].entrypoint[k] == VAEntrypointVLD)
        {
            numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
            decConfigId++;
        }
        else
        {
            numAttr = vpuApiCapInitEncAttributes(capMap[i].profile, capMap[i].entrypoint[k], VPUAPI_ENCODER_CONFIG_ID_START + encConfigId, &attrMap[j]);
            encConfigId++;
        }
        j = j + numAttr;
    }

    i++;
    capMap[i].profile = VAProfileAV1Profile1;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].sizeOfEntrypoints = 1;
    numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[0], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
    decConfigId++;
    j = j + numAttr;

#ifdef VA_PROFILE_AVS2_MAIN_10
    i++;
    capMap[i].profile = VAProfileAVS2Main;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].sizeOfEntrypoints = 1;
    numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[0], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
    decConfigId++;
    j = j + numAttr;

    i++;
    capMap[i].profile = VAProfileAVS2Main10;
    capMap[i].entrypoint[0] = VAEntrypointVLD;
    capMap[i].sizeOfEntrypoints = 1;
    numAttr = vpuApiCapInitDecAttributes(capMap[i].profile, capMap[i].entrypoint[0], VPUAPI_DECODER_CONFIG_ID_START + decConfigId, &attrMap[j]);
    decConfigId++;
    j = j + numAttr;
#endif

    s_sizeOfVpuApiCapMap = i + 1;
    s_sizeOfVpuApiAttrMap = j;
    // printf("-%s s_sizeOfVpuApiCapMap=%d, s_sizeOfVpuApiAttrMap=%d\n", __FUNCTION__, s_sizeOfVpuApiCapMap, s_sizeOfVpuApiAttrMap);
}

static VAStatus VpuApiCapQuerySurfaceAttributes(
    VAConfigID configId,
    VASurfaceAttrib *attribList,
    uint32_t *numAttribs)
{
    DDI_CHK_NULL(numAttribs, "Null num_attribs", VA_STATUS_ERROR_INVALID_PARAMETER);

    if (attribList == nullptr)
    {
        *numAttribs = DDI_CODEC_GEN_MAX_SURFACE_ATTRIBUTES;
        return VA_STATUS_SUCCESS;
    }
    VAStatus status = VA_STATUS_ERROR_INVALID_CONFIG;
    int32_t profileTableIdx = -1;
    VAEntrypoint entrypoint;
    VAProfile profile;

    {
        int i;
        for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
        {
            if (s_vpuApiAttrs[i].configId == configId)
            {
                if (s_vpuApiAttrs[i].actual_profile != VAProfileNone)
                {
                    profile = s_vpuApiAttrs[i].actual_profile;
                    entrypoint = s_vpuApiAttrs[i].actual_entrypoint;
                    status = VA_STATUS_SUCCESS;
                    // printf("[CNM_DEBUG]     >>> i=%d, num=%d,   profile=0x%x, entrypoint=0x%x, attrType=0x%x, attrValue=0x%x\n", i, num, *profile, *entrypoint, s_vpuApiAttrs[i].attrType, s_vpuApiAttrs[i].value);
                    break;
                }
            }
        }
    }
    // printf("[CNM_DEBUG]-%s profile=0x%x, entrypoint=0x%x, num_attribs=%d\n", __FUNCTION__, *profile, *entrypoint, *num_attribs);
    DDI_CHK_RET(status, "Invalid config_id!");

    VASurfaceAttrib *attribs = (VASurfaceAttrib *)MOS_AllocAndZeroMemory(DDI_CODEC_GEN_MAX_SURFACE_ATTRIBUTES * sizeof(*attribs));
    if (attribs == nullptr)
    {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    uint32_t i = 0;

    // if (entrypoint == VAEntrypointVideoProc)   /* vpp */
    // {
    //     attribs[i].type = VASurfaceAttribPixelFormat;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VA_FOURCC('N', 'V', '1', '2');
    //     i++;

    //     attribs[i].type = VASurfaceAttribMaxWidth;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VP_MAX_PIC_WIDTH;
    //     i++;

    //     attribs[i].type = VASurfaceAttribMaxHeight;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VP_MAX_PIC_HEIGHT;
    //     i++;

    //     attribs[i].type = VASurfaceAttribMinWidth;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VP_MIN_PIC_WIDTH;
    //     i++;

    //     attribs[i].type = VASurfaceAttribMinHeight;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VP_MIN_PIC_HEIGHT;
    //     i++;

    //     for (uint32_t j = 0; j < m_numVpSurfaceAttr; j++)
    //     {
    //         attribs[i].type = VASurfaceAttribPixelFormat;
    //         attribs[i].value.type = VAGenericValueTypeInteger;
    //         attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //         attribs[i].value.value.i = m_vpSurfaceAttr[j];
    //         i++;
    //     }

    //     attribs[i].type = VASurfaceAttribMemoryType;
    //     attribs[i].value.type = VAGenericValueTypeInteger;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
    //         VA_SURFACE_ATTRIB_MEM_TYPE_USER_PTR |
    //         VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM |
    //         VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
    //     i++;

    //     attribs[i].type = VASurfaceAttribExternalBufferDescriptor;
    //     attribs[i].value.type = VAGenericValueTypePointer;
    //     attribs[i].flags = VA_SURFACE_ATTRIB_SETTABLE;
    //     attribs[i].value.value.p = nullptr; /* ignore */
    //     i++;
    // }
    // else if (entrypoint == VAEntrypointVLD)    /* vld */
    if (entrypoint == VAEntrypointVLD) /* vld */
    {
#ifdef VA_PROFILE_H264_HIGH_10
#ifdef VA_PROFILE_AVS2_MAIN_10
        if (profile == VAProfileHEVCMain10 || profile == VAProfileVP9Profile2 || profile == VAProfileH264High10 || profile == VAProfileAVS2Main10)
        {
#else
        if (profile == VAProfileHEVCMain10 || profile == VAProfileVP9Profile2 || profile == VAProfileH264High10)
        {
#endif
#else
        if (profile == VAProfileHEVCMain10 || profile == VAProfileVP9Profile2)
        {
#endif
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_P010;
            i++;
        }
        else if (profile == VAProfileAV1Profile0 || profile == VAProfileAV1Profile1)
        {
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV12;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_P010;
            i++;
        }
        else
        {
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV12;
            i++;
        }

        auto maxWidth = VPUAPI_MAX_PIC_WIDTH;
        auto maxHeight = VPUAPI_MAX_PIC_HEIGHT;
        // if(IsMpeg2Profile(profile))
        // {
        //     maxWidth = m_decMpeg2MaxWidth;
        //     maxHeight = m_decMpeg2MaxHeight;
        // }
        // else if(IsHevcProfile(profile))
        // {
        //     maxWidth = m_decHevcMaxWidth;
        //     maxHeight = m_decHevcMaxHeight;
        // }
        // else if(IsVc1Profile(profile))
        // {
        //     maxWidth = m_decVc1MaxWidth;
        //     maxHeight = m_decVc1MaxHeight;
        // }
        // else if(IsJpegProfile(profile))
        // {
        //     maxWidth = m_decJpegMaxWidth;
        //     maxHeight = m_decJpegMaxHeight;
        // }
        // else if(IsVp9Profile(profile))
        // {
        //     maxWidth = m_decVp9MaxWidth;
        //     maxHeight = m_decVp9MaxHeight;
        // }

        attribs[i].type = VASurfaceAttribMaxWidth;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = maxWidth;
        i++;

        attribs[i].type = VASurfaceAttribMaxHeight;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = maxHeight;
        i++;

        attribs[i].type = VASurfaceAttribMemoryType;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
        attribs[i].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_USER_PTR |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
        i++;
    }
    else if (entrypoint == VAEntrypointEncSlice || entrypoint == VAEntrypointEncSliceLP || entrypoint == VAEntrypointEncPicture || entrypoint == VAEntrypointFEI)
    {
        if (profile == VAProfileHEVCMain10)
        {
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_P010;
            i++;
        }
        else if (profile == VAProfileAV1Profile0 || profile == VAProfileAV1Profile1)
        {
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_I420;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_YV12;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV12;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV21;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_P010;
            i++;
        }
        else
        {
            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_I420;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_YV12;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV12;
            i++;

            attribs[i].type = VASurfaceAttribPixelFormat;
            attribs[i].value.type = VAGenericValueTypeInteger;
            attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
            attribs[i].value.value.i = VA_FOURCC_NV21;
            i++;
        }
        attribs[i].type = VASurfaceAttribMaxWidth;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = VPUAPI_MAX_PIC_WIDTH;

        //         if(profile == VAProfileJPEGBaseline)
        //         {
        //             attribs[i].value.value.i = ENCODE_JPEG_MAX_PIC_WIDTH;
        //         }
        // if(IsAvcProfile(profile)||IsHevcProfile(profile)||IsVp8Profile(profile))
        // {
        //     attribs[i].value.value.i = CODEC_4K_MAX_PIC_WIDTH;
        // }
        i++;

        attribs[i].type = VASurfaceAttribMaxHeight;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = VPUAPI_MAX_PIC_HEIGHT;
        //         if(profile == VAProfileJPEGBaseline)
        //         {
        //             attribs[i].value.value.i = ENCODE_JPEG_MAX_PIC_HEIGHT;
        //         }
        // if(IsAvcProfile(profile)||IsHevcProfile(profile)||IsVp8Profile(profile))
        // {
        //     attribs[i].value.value.i = CODEC_4K_MAX_PIC_HEIGHT;
        // }
        i++;

        attribs[i].type = VASurfaceAttribMinWidth;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = VPUAPI_MIN_ENC_PIC_WIDTH;
        //         if(profile == VAProfileJPEGBaseline)
        //         {
        //             attribs[i].value.value.i = m_encJpegMinWidth;
        //         }
        i++;

        attribs[i].type = VASurfaceAttribMinHeight;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE;
        attribs[i].value.value.i = VPUAPI_MIN_ENC_PIC_HEIGHT;
        //         if(profile == VAProfileJPEGBaseline)
        //         {
        //             attribs[i].value.value.i = m_encJpegMinHeight;
        //         }
        i++;

        attribs[i].type = VASurfaceAttribMemoryType;
        attribs[i].value.type = VAGenericValueTypeInteger;
        attribs[i].flags = VA_SURFACE_ATTRIB_GETTABLE | VA_SURFACE_ATTRIB_SETTABLE;
        attribs[i].value.value.i = VA_SURFACE_ATTRIB_MEM_TYPE_VA |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_USER_PTR |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM |
                                   VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
        i++;
    }
    else
    {
        MOS_FreeMemory(attribs);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }

    if (i > *numAttribs)
    {
        *numAttribs = i;
        MOS_FreeMemory(attribs);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }

    *numAttribs = i;
    MOS_SecureMemcpy(attribList, i * sizeof(*attribs), attribs, i * sizeof(*attribs));

    MOS_FreeMemory(attribs);
    return status;
}
#endif
#ifdef CNM_VPUAPI_INTERFACE
Int32 LoadFirmware(
    Int32 productId,
    Uint8 **retFirmware,
    Uint32 *retSizeInWord,
    const char *path)
{
    Int32 nread;
    Uint32 totalRead, allocSize, readSize = WAVE5_MAX_CODE_BUF_SIZE;
    Uint8 *firmware = NULL;
    osal_file_t fp;

    if ((fp = osal_fopen(path, "rb")) == NULL)
    {
        printf("[CNM_VPUAPI] Failed to open %s\n", path);
        return -1;
    }

    totalRead = 0;
    firmware = (Uint8 *)osal_malloc(readSize);
    allocSize = readSize;
    nread = 0;
    while (TRUE)
    {
        if (allocSize < (totalRead + readSize))
        {
            allocSize += 2 * nread;
            firmware = (Uint8 *)realloc(firmware, allocSize);
        }
        nread = osal_fread((void *)&firmware[totalRead], 1, readSize, fp);
        totalRead += nread;
        if (nread < (Int32)readSize)
            break;
    }
    *retSizeInWord = (totalRead + 1) / 2;

    osal_fclose(fp);

    *retFirmware = firmware;

    return 0;
}

static int32_t GetRenderTargetIndex(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VASurfaceID renderTarget)
{
    int32_t index = -1;

    for (int32_t i = 0; i < VPUAPI_MAX_FB_NUM; i++)
    {
        if (mediaCtx->renderTargets[i] == renderTarget)
        {
            index = i;
            break;
        }
    }

    return index;
}

static VASurfaceID GetUsedRenderTarget(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    VASurfaceID usedRenderTarget;
    int32_t i;

    DdiMediaUtil_LockMutex(&mediaCtx->vpuapiMutex);

    usedRenderTarget = mediaCtx->usedRenderTargets[0];

    for (i = 0; i < VPUAPI_MAX_FB_NUM - 1; i++)
        mediaCtx->usedRenderTargets[i] = mediaCtx->usedRenderTargets[i + 1];
    mediaCtx->usedRenderTargets[i] = VPUAPI_UNKNOWN_SURFACE_ID;

    DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);

    return usedRenderTarget;
}

static BOOL FindUsedRenderTarget(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VASurfaceID renderTarget)
{
    BOOL found = FALSE;

    DdiMediaUtil_LockMutex(&mediaCtx->vpuapiMutex);

    for (int32_t i = 0; i < VPUAPI_MAX_FB_NUM; i++)
    {
        if (mediaCtx->usedRenderTargets[i] == renderTarget)
        {
            found = TRUE;
            break;
        }
    }

    DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);

    return found;
}

static void SetUsedRenderTarget(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VASurfaceID renderTarget)
{
    DdiMediaUtil_LockMutex(&mediaCtx->vpuapiMutex);

    for (int32_t i = 0; i < VPUAPI_MAX_FB_NUM; i++)
    {
        if (mediaCtx->usedRenderTargets[i] == VPUAPI_UNKNOWN_SURFACE_ID)
        {
            mediaCtx->usedRenderTargets[i] = renderTarget;
            break;
        }
    }

    DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
}

static void *ConvertBufferData(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VABufferType type,
    uint32_t size,
    uint8_t *data)
{
    void *convData = NULL;

    convData = calloc(size, 1);
    memcpy(convData, data, size);

    switch (mediaCtx->decOP.bitstreamFormat)
    {
    case STD_VP9:
    {
        if (type == VAPictureParameterBufferType)
        {
            VADecPictureParameterBufferVP9 *vp9PicParam = (VADecPictureParameterBufferVP9 *)convData;
            for (int32_t i = 0; i < 8; i++)
                vp9PicParam->reference_frames[i] = GetRenderTargetIndex(mediaCtx, vp9PicParam->reference_frames[i]);
        }
        break;
    }
    case STD_AV1:
    {
        if (type == VAPictureParameterBufferType)
        {
            VADecPictureParameterBufferAV1 *av1PicParam = (VADecPictureParameterBufferAV1 *)convData;
            av1PicParam->current_frame = GetRenderTargetIndex(mediaCtx, av1PicParam->current_frame);
            av1PicParam->current_display_picture = GetRenderTargetIndex(mediaCtx, av1PicParam->current_display_picture);

            for (int32_t i = 0; i < av1PicParam->anchor_frames_num; i++)
                av1PicParam->anchor_frames_list[i] = GetRenderTargetIndex(mediaCtx, av1PicParam->anchor_frames_list[i]);
            for (int32_t i = 0; i < 8; i++)
                av1PicParam->ref_frame_map[i] = GetRenderTargetIndex(mediaCtx, av1PicParam->ref_frame_map[i]);
        }
        break;
    }
    case STD_AVC:
    {
        if (type == VAPictureParameterBufferType)
        {
            VAPictureParameterBufferH264 *avcPicParam = (VAPictureParameterBufferH264 *)convData;
            printf("1width : %d, height : %d, luma depth : %d, chroma depth :%d \r\n",(avcPicParam->picture_width_in_mbs_minus1+1)*16, 
                                                    (avcPicParam->picture_height_in_mbs_minus1+1),
                                                    avcPicParam->bit_depth_luma_minus8,
                                                    avcPicParam->bit_depth_chroma_minus8);
            avcPicParam->CurrPic.picture_id = GetRenderTargetIndex(mediaCtx, avcPicParam->CurrPic.picture_id);
            for (int32_t i = 0; i < 16; i++)
            {
                if (avcPicParam->ReferenceFrames[i].picture_id != VA_INVALID_SURFACE)
                    avcPicParam->ReferenceFrames[i].picture_id = GetRenderTargetIndex(mediaCtx, avcPicParam->ReferenceFrames[i].picture_id);
            }
            printf("2width : %d, height : %d, luma depth : %d, chroma depth :%d \r\n",(avcPicParam->picture_width_in_mbs_minus1+1)*16, 
                                                    (avcPicParam->picture_height_in_mbs_minus1+1),
                                                    avcPicParam->bit_depth_luma_minus8,
                                                    avcPicParam->bit_depth_chroma_minus8);
        }
        else if (type == VASliceParameterBufferType)
        {
            VASliceParameterBufferH264 *avcSliceParam = (VASliceParameterBufferH264 *)convData;

            for (uint32_t i = 0; i < 32; i++)
            {
                if ((avcSliceParam->RefPicList0[i].picture_id != VA_INVALID_SURFACE) &&
                    ((avcSliceParam->RefPicList0[i].flags & VA_PICTURE_H264_INVALID) == 0))
                {
                    avcSliceParam->RefPicList0[i].picture_id = GetRenderTargetIndex(mediaCtx, avcSliceParam->RefPicList0[i].picture_id);
                }
            }

            for (uint32_t i = 0; i < 32; i++)
            {
                if ((avcSliceParam->RefPicList1[i].picture_id != VA_INVALID_SURFACE) &&
                    ((avcSliceParam->RefPicList1[i].flags & VA_PICTURE_H264_INVALID) == 0))
                {
                    avcSliceParam->RefPicList1[i].picture_id = GetRenderTargetIndex(mediaCtx, avcSliceParam->RefPicList1[i].picture_id);
                }
            }
        }
        break;
    }
    case STD_HEVC:
    {
        bool isRext = false;
        bool isScc = false;

        if (mediaCtx->vaProfile == VAProfileHEVCMain12 ||
            mediaCtx->vaProfile == VAProfileHEVCMain422_10 ||
            mediaCtx->vaProfile == VAProfileHEVCMain422_12 ||
            mediaCtx->vaProfile == VAProfileHEVCMain444 ||
            mediaCtx->vaProfile == VAProfileHEVCMain444_10 ||
            mediaCtx->vaProfile == VAProfileHEVCMain444_12 ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain10 ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain444 ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain444_10)
        {
            isRext = true;
        }
        if (mediaCtx->vaProfile == VAProfileHEVCSccMain ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain10 ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain444 ||
            mediaCtx->vaProfile == VAProfileHEVCSccMain444_10)
        {
            isScc = true;
        };

        if (type == VAPictureParameterBufferType)
        {
            VAPictureParameterBufferHEVC *hevcPicParam = NULL;
            VAPictureParameterBufferHEVCRext *hevcRextParam = NULL;
            VAPictureParameterBufferHEVCScc *hevcSccParam = NULL;

            if (isRext)
            {
                hevcPicParam = &((VAPictureParameterBufferHEVCExtension *)convData)->base;
                hevcRextParam = &((VAPictureParameterBufferHEVCExtension *)convData)->rext;
                if (hevcRextParam)
                {
                    hevcRextParam->range_extension_pic_fields.value = hevcRextParam->range_extension_pic_fields.value;
                }

                if (isScc)
                {
                    hevcSccParam = &((VAPictureParameterBufferHEVCExtension *)convData)->scc;
                    hevcSccParam->screen_content_pic_fields.value = hevcSccParam->screen_content_pic_fields.value;
                }
            }
            else
            {
                hevcPicParam = (VAPictureParameterBufferHEVC *)convData;
            }

            hevcPicParam->CurrPic.picture_id = GetRenderTargetIndex(mediaCtx, hevcPicParam->CurrPic.picture_id);
            for (uint32_t i = 0; i < 15; i++)
            {
                if (hevcPicParam->ReferenceFrames[i].picture_id != VA_INVALID_SURFACE)
                {
                    hevcPicParam->ReferenceFrames[i].picture_id = GetRenderTargetIndex(mediaCtx, hevcPicParam->ReferenceFrames[i].picture_id);
                }
            }
        }
        break;
    }
#ifdef VA_PROFILE_AVS2_MAIN_10
    case STD_AVS2:
    {
        if (type == VAPictureParameterBufferType)
        {
            VAPictureParameterBufferAVS2 *picParam = (VAPictureParameterBufferAVS2 *)convData;
            picParam->CurrPic.surface_id = GetRenderTargetIndex(mediaCtx, picParam->CurrPic.surface_id);
            for (int32_t i = 0; i < VA_AVS2_MAX_REF_COUNT; i++)
            {
                if (picParam->ref_list[i].surface_id != VA_INVALID_SURFACE)
                    picParam->ref_list[i].surface_id = GetRenderTargetIndex(mediaCtx, picParam->ref_list[i].surface_id);
            }
        }
        break;
    }
#endif
    default:
        break;
    }

    return convData;
}

static uint32_t CalcStride(
    uint32_t width,
    FrameBufferFormat format)
{
    uint32_t lumaStride = 0;

    switch (format)
    {
    case FORMAT_420:
        lumaStride = VPU_ALIGN32(width);
        break;
    case FORMAT_420_P10_16BIT_LSB:
    case FORMAT_420_P10_16BIT_MSB:
        lumaStride = VPU_ALIGN32(VPU_ALIGN16(width) * 5) / 4;
        lumaStride = VPU_ALIGN32(lumaStride);
        break;
    default:
        break;
    }

    return lumaStride;
}

#ifdef CNM_FPGA_PLATFORM
static void AllocateLinearFrameBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    vpu_buffer_t *pVb = NULL;
    int32_t index = mediaCtx->numOfRenderTargets;
    uint32_t fbWidth = mediaCtx->linearStride;
    uint32_t fbHeight = mediaCtx->linearHeight;
    uint32_t fbLumaSize = 0;
    uint32_t fbChromaSize = 0;
    uint32_t fbSize = 0;

    fbLumaSize = fbWidth * fbHeight;
    fbChromaSize = (fbWidth / 2) * (fbHeight / 2);
    fbSize = fbLumaSize + (fbChromaSize * 2);

    if (vdi_init(mediaCtx->coreIdx) < 0)
        printf("[CNM_VPUAPI] FAIL vdi_init\n");

    pVb = &mediaCtx->linearFrameBufMem[index];
    pVb->phys_addr = 0;
    pVb->size = fbSize;

    printf("[CNM_VPUAPI] %s Allocate Linear buffer i=%d, wtlFormat=%d, stride=%d, height=%d, fbSize=%d\n", __FUNCTION__, index, mediaCtx->wtlFormat, mediaCtx->linearStride, mediaCtx->linearHeight, fbSize);
    if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, DEC_FB_LINEAR, 0) < 0)
        printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory linearBuf\n");

    mediaCtx->linearFrameBuf[index].bufY = pVb->phys_addr;
    mediaCtx->linearFrameBuf[index].bufCb = pVb->phys_addr + fbLumaSize;
    if (mediaCtx->cbcrInterleave == FALSE)
        mediaCtx->linearFrameBuf[index].bufCr = pVb->phys_addr + fbLumaSize + fbChromaSize;
    else
        mediaCtx->linearFrameBuf[index].bufCr = -1;
    
    mediaCtx->linearFrameBuf[index].width = fbWidth;
    mediaCtx->linearFrameBuf[index].height = fbHeight;

}

static void FreeLinearFrameBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    uint32_t index)
{
    printf("[CNM_VPUAPI] %s index=%d\n", __FUNCTION__, index);
    if (mediaCtx->linearFrameBufMem[index].phys_addr != 0)
    {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->linearFrameBufMem[index], DEC_FB_LINEAR, 0);
    }
    vdi_release(mediaCtx->coreIdx);
}
#endif

static void VpuApiDecPrintDebugInfo(
    VADriverContextP ctx)
{

    typedef struct
    {
        Uint32 priReason;
        Uint32 regs[0x40];
        Uint32 debugInfo0;
        Uint32 debugInfo1;
    } VPUDebugInfo;
    typedef struct CodecInstRef
    {
        Int32 inUse;
        Int32 instIndex;
        Int32 coreIdx;
        Int32 codecMode;
        Int32 codecModeAux;
        Int32 productId;
        Int32 loggingEnable;
        Uint32 isDecoder;
    } CodecInstRef;
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode cmd_ret;
    int productId;
    VPUDebugInfo debugInfo;
    uint32_t coreIdx;
    uint32_t instIndex;
    Uint32 productCode;
    CodecInstRef *pRefCodecInst = (CodecInstRef *)mediaCtx->decHandle;

    coreIdx = mediaCtx->coreIdx;
    instIndex = pRefCodecInst->instIndex;
    VLOG(INFO, "+%s core=%d, inst=%d\n", __FUNCTION__, coreIdx, instIndex);
#ifdef FAKE_VPUAPI
#else
    productId = VPU_GetProductId(mediaCtx->coreIdx);
    if (PRODUCT_ID_W6_SERIES(productId))
    {
        int i;
        for (i = 0; i < 20; i++)
        {
#define REG_BASE 0x00
#define VCPU_CUR_PC (REG_BASE + 0x0004)
#define VCPU_CUR_LR (REG_BASE + 0x0008)
            VLOG(INFO, "LR=0x%x, PC=0x%x\n", vdi_read_register(coreIdx, VCPU_CUR_LR), vdi_read_register(coreIdx, VCPU_CUR_PC));
            osal_msleep(0);
        }
    }
    else if (PRODUCT_ID_W5_SERIES(productId))
    {
        int i;
        for (i = 0; i < 20; i++)
        {
#define REG_BASE 0x00
#define VCPU_CUR_PC (REG_BASE + 0x0004)
#define VCPU_CUR_LR (REG_BASE + 0x0008)
            VLOG(INFO, "LR=0x%x, PC=0x%x\n", vdi_read_register(coreIdx, VCPU_CUR_LR), vdi_read_register(coreIdx, VCPU_CUR_PC));
            osal_msleep(0);
        }
        cmd_ret = VPU_DecGiveCommand(mediaCtx->decHandle, GET_DEBUG_INFORM, &debugInfo);
        if (cmd_ret != RETCODE_SUCCESS)
        {
            printf("-%s GET_DEBUG_INFOM command fail \n", __FUNCTION__);
            return;
        }

        VLOG(TRACE, "++ interrupt flags \n");
        VLOG(TRACE, "     pend_intrpt_idc = 0x%x \n", (debugInfo.regs[0x0c] >> 16) & 0xffff);
        VLOG(TRACE, "     multi_int_reason = 0x%x \n", debugInfo.regs[0x0c] & 0xffff);
        VLOG(TRACE, "     last interrupt reason = 0x%x \n", debugInfo.regs[0x0d]);
        VLOG(TRACE, "-- interrupt flags \n\n");

        VLOG(TRACE, "++ available core flags \n");
        VLOG(TRACE, "     STAGE_SEEK      core_avail_idc = %d \n", ((debugInfo.regs[0x0e] >> 0) & 0x0f));
        VLOG(TRACE, "     STAGE_PARSING   core_avail_idc = %d \n", ((debugInfo.regs[0x0e] >> 4) & 0x0f));
        VLOG(TRACE, "     STAGE_DEC/ENC   core_avail_idc = %d \n", ((debugInfo.regs[0x0e] >> 8) & 0x0f));
        VLOG(TRACE, "     STAGE_PACKAGING core_avail_idc = %d \n", ((debugInfo.regs[0x0e] >> 12) & 0x0f));
        VLOG(TRACE, "-- avaiable core flags \n\n");

        VLOG(TRACE, "++ the number of the allocated queue and commands in report queue\n");
        VLOG(TRACE, "     inst0={que_cnt=%d, rq_cnt=%d} inst1={que_cnt=%d, rq_cnt=%d} inst2={que_cnt=%d, rq_cnt=%d} inst3={que_cnt=%d, rq_cnt=%d}\n",
             ((((debugInfo.regs[0x06] >> 0) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x06] >> 0) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x06] >> 8) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x06] >> 8) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x06] >> 16) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x06] >> 16) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x06] >> 24) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x06] >> 24) & 0xff) >> 0) & 0xf));
        VLOG(TRACE, "     inst4={que_cnt=%d, rq_cnt=%d} inst5={que_cnt=%d, rq_cnt=%d} inst6={que_cnt=%d, rq_cnt=%d} inst7={que_cnt=%d, rq_cnt=%d}\n",
             ((((debugInfo.regs[0x07] >> 0) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x07] >> 0) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x07] >> 8) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x07] >> 8) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x07] >> 16) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x07] >> 16) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x07] >> 24) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x07] >> 24) & 0xff) >> 0) & 0xf));
        VLOG(TRACE, "     inst7={que_cnt=%d, rq_cnt=%d} inst9={que_cnt=%d, rq_cnt=%d} inst10={que_cnt=%d, rq_cnt=%d} inst11={que_cnt=%d, rq_cnt=%d}\n",
             ((((debugInfo.regs[0x08] >> 0) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x08] >> 0) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x08] >> 8) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x08] >> 8) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x08] >> 16) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x08] >> 16) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x08] >> 24) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x08] >> 24) & 0xff) >> 0) & 0xf));
        VLOG(TRACE, "     inst12={que_cnt=%d, rq_cnt=%d} inst13={que_cnt=%d, rq_cnt=%d} inst14={que_cnt=%d, rq_cnt=%d} inst15={que_cnt=%d, rq_cnt=%d}\n",
             ((((debugInfo.regs[0x09] >> 0) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x09] >> 0) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x09] >> 8) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x09] >> 8) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x09] >> 16) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x09] >> 16) & 0xff) >> 0) & 0xf),
             ((((debugInfo.regs[0x09] >> 24) & 0xff) >> 4) & 0xf), ((((debugInfo.regs[0x09] >> 24) & 0xff) >> 0) & 0xf));
        VLOG(TRACE, "-- the number of the allocated queue and commands in report queue\n\n");

        VLOG(TRACE, "++ status of instance handle\n");
        VLOG(TRACE, "     inst0 status=%d, inst1 status=%d, inst2 status=%d, inst3 status=%d \n",
             ((debugInfo.regs[0x0A] >> 0) & 0xf), ((debugInfo.regs[0x0A] >> 4) & 0xf), ((debugInfo.regs[0x0A] >> 8) & 0xf), ((debugInfo.regs[0x0A] >> 12) & 0xf));
        VLOG(TRACE, "     inst4 status=%d, inst5 status=%d, inst6 status=%d, inst7 status=%d \n",
             ((debugInfo.regs[0x0A] >> 16) & 0xf), ((debugInfo.regs[0x0A] >> 20) & 0xf), ((debugInfo.regs[0x0A] >> 24) & 0xf), ((debugInfo.regs[0x0A] >> 28) & 0xf));

        VLOG(TRACE, "     inst8 status=%d, inst9 status=%d, inst10 status=%d, inst11 status=%d \n",
             ((debugInfo.regs[0x0B] >> 0) & 0xf), ((debugInfo.regs[0x0B] >> 4) & 0xf), ((debugInfo.regs[0x0B] >> 8) & 0xf), ((debugInfo.regs[0x0B] >> 12) & 0xf));

        VLOG(TRACE, "     inst12 status=%d, inst13 status=%d, inst14 status=%d, inst15 status=%d \n",
             ((debugInfo.regs[0x0B] >> 16) & 0xf), ((debugInfo.regs[0x0B] >> 20) & 0xf), ((debugInfo.regs[0x0B] >> 24) & 0xf), ((debugInfo.regs[0x0B] >> 28) & 0xf));
        VLOG(TRACE, "-- status of instance handle\n\n");

        VLOG(TRACE, "++ active command in each stage\n");
        VLOG(TRACE, "     STAGE_SEEK, CMD_INST_ID = %d \n", (debugInfo.regs[0x10] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_QUE_CNT = %d \n", (debugInfo.regs[0x10] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_IDX = %d \n", (debugInfo.regs[0x11] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_BUF_IDX = %d \n", (debugInfo.regs[0x11] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_QUEUE_STATUS = %d \n", (debugInfo.regs[0x12] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_STATUS = %d \n", (debugInfo.regs[0x12] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_CMD = %d \n", (debugInfo.regs[0x13] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_USE_TASKBUF = %d \n", (debugInfo.regs[0x13] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_SCHED_STATUS = %d \n", (debugInfo.regs[0x14] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_SEEK, CMD_MAX_QUEUE_DEPTH = %d \n", (debugInfo.regs[0x14] >> 0) & 0xffff);

        VLOG(TRACE, "     STAGE_PARSING, CMD_INST_ID = %d \n", (debugInfo.regs[0x15] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_QUE_CNT = %d \n", (debugInfo.regs[0x15] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_IDX = %d \n", (debugInfo.regs[0x16] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_BUF_IDX = %d \n", (debugInfo.regs[0x16] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_QUEUE_STATUS = %d \n", (debugInfo.regs[0x17] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_STATUS = %d \n", (debugInfo.regs[0x17] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_CMD = %d \n", (debugInfo.regs[0x18] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_USE_TASKBUF = %d \n", (debugInfo.regs[0x18] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_SCHED_STATUS = %d \n", (debugInfo.regs[0x19] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PARSING, CMD_MAX_QUEUE_DEPTH = %d \n", (debugInfo.regs[0x19] >> 0) & 0xffff);

        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_INST_ID = %d \n", (debugInfo.regs[0x1a] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_QUE_CNT = %d \n", (debugInfo.regs[0x1a] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_IDX = %d \n", (debugInfo.regs[0x1b] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_BUF_IDX = %d \n", (debugInfo.regs[0x1b] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_QUEUE_STATUS = %d \n", (debugInfo.regs[0x1c] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_STATUS = %d \n", (debugInfo.regs[0x1c] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_CMD = %d \n", (debugInfo.regs[0x1d] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_USE_TASKBUF = %d \n", (debugInfo.regs[0x1d] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_SCHED_STATUS = %d \n", (debugInfo.regs[0x1e] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_DEC/ENCODING, CMD_MAX_QUEUE_DEPTH = %d \n", (debugInfo.regs[0x1e] >> 0) & 0xffff);

        VLOG(TRACE, "     STAGE_PACKAGING, CMD_INST_ID = %d \n", (debugInfo.regs[0x1f] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_QUE_CNT = %d \n", (debugInfo.regs[0x1f] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_IDX = %d \n", (debugInfo.regs[0x20] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_BUF_IDX = %d \n", (debugInfo.regs[0x20] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_QUEUE_STATUS = %d \n", (debugInfo.regs[0x21] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_STATUS = %d \n", (debugInfo.regs[0x21] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_CMD = %d \n", (debugInfo.regs[0x22] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_USE_TASKBUF = %d \n", (debugInfo.regs[0x22] >> 0) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_SCHED_STATUS = %d \n", (debugInfo.regs[0x23] >> 16) & 0xffff);
        VLOG(TRACE, "     STAGE_PACKAGING, CMD_MAX_QUEUE_DEPTH = %d \n", (debugInfo.regs[0x23] >> 0) & 0xffff);
        VLOG(TRACE, "-- active command in each stage\n\n");

        VLOG(TRACE, "++ queued commands in each stage\n");
        VLOG(TRACE, "     STAGE_SEEK      inst0->cmd_cnt=%d, inst1->cmt_cmd=%d, inst2->cmd_cnt=%d, inst3=>cmd_cnt=%d inst4=>cmd_cnt=%d, inst5=>cmd_cnt=%d, inst6=>cmd_cnt=%d, inst7=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x30] >> 0) & 0xf), ((debugInfo.regs[0x30] >> 4) & 0xf),
             ((debugInfo.regs[0x30] >> 8) & 0xf), ((debugInfo.regs[0x30] >> 12) & 0xf),
             ((debugInfo.regs[0x30] >> 16) & 0xf), ((debugInfo.regs[0x30] >> 20) & 0xf),
             ((debugInfo.regs[0x30] >> 24) & 0xf), ((debugInfo.regs[0x30] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_SEEK      inst8->cmd_cnt=%d, inst9->cmt_cmd=%d, inst10->cmd_cnt=%d, inst11=>cmd_cnt=%d inst12=>cmd_cnt=%d, inst13=>cmd_cnt=%d, inst14=>cmd_cnt=%d, inst15=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x31] >> 0) & 0xf), ((debugInfo.regs[0x31] >> 4) & 0xf),
             ((debugInfo.regs[0x31] >> 8) & 0xf), ((debugInfo.regs[0x31] >> 12) & 0xf),
             ((debugInfo.regs[0x31] >> 16) & 0xf), ((debugInfo.regs[0x31] >> 20) & 0xf),
             ((debugInfo.regs[0x31] >> 24) & 0xf), ((debugInfo.regs[0x31] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_PARSING      inst0->cmd_cnt=%d, inst1->cmt_cmd=%d, inst2->cmd_cnt=%d, inst3=>cmd_cnt=%d inst4=>cmd_cnt=%d, inst5=>cmd_cnt=%d, inst6=>cmd_cnt=%d, inst7=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x32] >> 0) & 0xf), ((debugInfo.regs[0x32] >> 4) & 0xf),
             ((debugInfo.regs[0x32] >> 8) & 0xf), ((debugInfo.regs[0x32] >> 12) & 0xf),
             ((debugInfo.regs[0x32] >> 16) & 0xf), ((debugInfo.regs[0x32] >> 20) & 0xf),
             ((debugInfo.regs[0x32] >> 24) & 0xf), ((debugInfo.regs[0x32] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_PARSING      inst8->cmd_cnt=%d, inst9->cmt_cmd=%d, inst10->cmd_cnt=%d, inst11=>cmd_cnt=%d inst12=>cmd_cnt=%d, inst13=>cmd_cnt=%d, inst14=>cmd_cnt=%d, inst15=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x33] >> 0) & 0xf), ((debugInfo.regs[0x33] >> 4) & 0xf),
             ((debugInfo.regs[0x33] >> 8) & 0xf), ((debugInfo.regs[0x33] >> 12) & 0xf),
             ((debugInfo.regs[0x33] >> 16) & 0xf), ((debugInfo.regs[0x33] >> 20) & 0xf),
             ((debugInfo.regs[0x33] >> 24) & 0xf), ((debugInfo.regs[0x33] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_DEC/ENCODING      inst0->cmd_cnt=%d, inst1->cmt_cmd=%d, inst2->cmd_cnt=%d, inst3=>cmd_cnt=%d inst4=>cmd_cnt=%d, inst5=>cmd_cnt=%d, inst6=>cmd_cnt=%d, inst7=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x34] >> 0) & 0xf), ((debugInfo.regs[0x34] >> 4) & 0xf),
             ((debugInfo.regs[0x34] >> 8) & 0xf), ((debugInfo.regs[0x34] >> 12) & 0xf),
             ((debugInfo.regs[0x34] >> 16) & 0xf), ((debugInfo.regs[0x34] >> 20) & 0xf),
             ((debugInfo.regs[0x34] >> 24) & 0xf), ((debugInfo.regs[0x34] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_DEC/ENCODING      inst8->cmd_cnt=%d, inst9->cmt_cmd=%d, inst10->cmd_cnt=%d, inst11=>cmd_cnt=%d inst12=>cmd_cnt=%d, inst13=>cmd_cnt=%d, inst14=>cmd_cnt=%d, inst15=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x35] >> 0) & 0xf), ((debugInfo.regs[0x35] >> 4) & 0xf),
             ((debugInfo.regs[0x35] >> 8) & 0xf), ((debugInfo.regs[0x35] >> 12) & 0xf),
             ((debugInfo.regs[0x35] >> 16) & 0xf), ((debugInfo.regs[0x35] >> 20) & 0xf),
             ((debugInfo.regs[0x35] >> 24) & 0xf), ((debugInfo.regs[0x35] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_PACKAGING      inst0->cmd_cnt=%d, inst1->cmt_cmd=%d, inst2->cmd_cnt=%d, inst3=>cmd_cnt=%d inst4=>cmd_cnt=%d, inst5=>cmd_cnt=%d, inst6=>cmd_cnt=%d, inst7=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x36] >> 0) & 0xf), ((debugInfo.regs[0x36] >> 4) & 0xf),
             ((debugInfo.regs[0x36] >> 8) & 0xf), ((debugInfo.regs[0x36] >> 12) & 0xf),
             ((debugInfo.regs[0x36] >> 16) & 0xf), ((debugInfo.regs[0x36] >> 20) & 0xf),
             ((debugInfo.regs[0x36] >> 24) & 0xf), ((debugInfo.regs[0x36] >> 28) & 0xf));
        VLOG(TRACE, "     STAGE_PACKAGING      inst8->cmd_cnt=%d, inst9->cmt_cmd=%d, inst10->cmd_cnt=%d, inst11=>cmd_cnt=%d inst12=>cmd_cnt=%d, inst13=>cmd_cnt=%d, inst14=>cmd_cnt=%d, inst15=>cmd_cnt=%d \n",
             ((debugInfo.regs[0x37] >> 0) & 0xf), ((debugInfo.regs[0x37] >> 4) & 0xf),
             ((debugInfo.regs[0x37] >> 8) & 0xf), ((debugInfo.regs[0x37] >> 12) & 0xf),
             ((debugInfo.regs[0x37] >> 16) & 0xf), ((debugInfo.regs[0x37] >> 20) & 0xf),
             ((debugInfo.regs[0x37] >> 24) & 0xf), ((debugInfo.regs[0x37] >> 28) & 0xf));
        VLOG(TRACE, "-- queued commands in each stage\n\n");

        // VLOG(TRACE, "-- vp9 vcore wtl status \n\n");
        // VLOG(TRACE, "DEBUG_SCHED_VCORE_STATUS %08x \n", (debugInfo.regs[0x38]));
        // VLOG(TRACE, "DEBUG_FBC_AVAIL          %08x \n", (debugInfo.regs[0x39]));
        // VLOG(TRACE, "DEBUG_WTL_AVAIL          %08x \n", (debugInfo.regs[0x3A]));
        // VLOG(TRACE, "-- vp9 vcore wtl status \n\n");
    }
#endif
    VLOG(INFO, "-%s core=%d, inst=%d\n", __FUNCTION__, coreIdx, instIndex);
}
static void VpuApiAddSurfaceInfo(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VASurfaceID surfaceId,
    uint32_t width,
    uint32_t height)
{
    if (mediaCtx->numOfRenderTargets == 0)
    {
        DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surfaceId);
        FrameBufferFormat wtlFormat = FORMAT_420;
        bool cbcrOrder = FALSE;
        bool cbcrInterleave = FALSE;
        bool nv21 = FALSE;

        switch (mediaSurface->format)
        {
        case Media_Format_NV12:
            wtlFormat = FORMAT_420;
            cbcrInterleave = TRUE;
            nv21 = FALSE;
            cbcrOrder = FALSE;
            break;
        case Media_Format_NV21:
            wtlFormat = FORMAT_420;
            cbcrInterleave = TRUE;
            nv21 = TRUE;
            cbcrOrder = FALSE;
            break;
        case Media_Format_I420:
            wtlFormat = FORMAT_420;
            cbcrInterleave = FALSE;
            nv21 = FALSE;
            cbcrOrder = FALSE;
            break;
        case Media_Format_YV12:
            wtlFormat = FORMAT_420;
            cbcrInterleave = FALSE;
            nv21 = FALSE;
            cbcrOrder = TRUE;
            break;
        case Media_Format_P010:
            wtlFormat = FORMAT_420_P10_16BIT_MSB;
            cbcrInterleave = TRUE;
            nv21 = FALSE;
            cbcrOrder = FALSE;
            break;
        default:
            break;
        }

        mediaCtx->wtlFormat = wtlFormat;
        mediaCtx->cbcrOrder = cbcrOrder;
        mediaCtx->cbcrInterleave = cbcrInterleave;
        mediaCtx->nv21 = nv21;
        mediaCtx->linearStride = mediaSurface->iPitch;
        mediaCtx->linearHeight = VPU_ALIGN16(mediaSurface->iHeight);
        mediaCtx->encSrcWidth = width;
        mediaCtx->encSrcHeight = height;
    }

#ifdef CNM_FPGA_PLATFORM
    AllocateLinearFrameBuffer(mediaCtx);
#endif
    mediaCtx->renderTargets[mediaCtx->numOfRenderTargets] = surfaceId;
    mediaCtx->numOfRenderTargets++;
}

static void VpuApiRemoveSurfaceInfo(
    PDDI_MEDIA_CONTEXT mediaCtx,
    VASurfaceID surfaceId)
{
    uint32_t index = GetRenderTargetIndex(mediaCtx, surfaceId);
    if (index != -1)
    {
#ifdef CNM_FPGA_PLATFORM
        FreeLinearFrameBuffer(mediaCtx, index);
#endif
        mediaCtx->renderTargets[index] = VA_INVALID_SURFACE;
        mediaCtx->numOfRenderTargets--;
        if (mediaCtx->numOfRenderTargets < 0)
        { // if DdiMeia_DestroyContext is called before.
            mediaCtx->numOfRenderTargets = 0;
        }
    }
}

static VAStatus VpuApiInit(
    int32_t coreIdx)
{
    Int32 productId;
    Uint32 sizeInWord;
    Uint16 *pusBitCode;
    char fwPath[100];
    RetCode ret = RETCODE_SUCCESS;

#if defined(SUPPORT_CONF_TEST) || (defined(WAVE6_CODEC_NATIVE_40BIT) && defined(CNM_SIM_DPI_INTERFACE))
    InitQC();
#endif
    vdi_init(coreIdx);
    vdi_vaapi_driver_lock(coreIdx);

    // if (!VPU_IsInit(coreIdx))
    {
        printf("[CNM_VPUAPI] Start HW INIT. GREGORY\n");
#ifdef CNM_FPGA_PLATFORM
        vdi_hw_reset(coreIdx);
        osal_msleep(1000); // Waiting for stable state
        if (vdi_set_timing_opt(coreIdx) == HPI_SET_TIMING_MAX)
        {
            printf("[CNM_VPUAPI] Failed to optimize HPI timing\n");
        }
        vdi_hw_reset(coreIdx);
        osal_msleep(1000); // Waiting for stable state
#endif
    }

#ifdef FAKE_VPUAPI
    productId = FAKE_PRODUCT_ID;
#else
    productId = VPU_GetProductId(coreIdx);
    if (productId == -1)
    {
        printf("[CNM_VPUAPI] Failed to get product ID. %d\n", productId);
        vdi_vaapi_driver_unlock(coreIdx);
        vdi_release(coreIdx);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
#endif

    printf("[CNM_VPUAPI] Product ID : %d\n", productId);

    switch (productId)
    {
    case PRODUCT_ID_980:
        strcpy(fwPath, "/usr/local/lib/coda980.out");
        break;
    case PRODUCT_ID_517:
        strcpy(fwPath, "/usr/local/lib/vincent.bin");
        break;
    case PRODUCT_ID_627:
    case PRODUCT_ID_637:
        strcpy(fwPath, "/usr/local/lib/seurat.bin");
        break;
    default:
        printf("[CNM_VPUAPI] Unknown product ID. %d\n", productId);
        break;
    }

    if (LoadFirmware(productId, (Uint8 **)&pusBitCode, &sizeInWord, fwPath) < 0)
    {
        printf("[CNM_VPUAPI] Failed to load firmware: %s.\n", fwPath);
        vdi_vaapi_driver_unlock(coreIdx);
        vdi_release(coreIdx);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    ret = VPU_InitWithBitcode(coreIdx, (const Uint16 *)pusBitCode, sizeInWord);
    if (ret != RETCODE_CALLED_BEFORE && ret != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] Failed to boot up VPU.\n");
        vdi_vaapi_driver_unlock(coreIdx);
        vdi_release(coreIdx);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    vdi_vaapi_driver_unlock(coreIdx);
    vdi_release(coreIdx);

    printf("[CNM_VPUAPI] Success boot up VPU\n");

    return VA_STATUS_SUCCESS;
}

static void VpuApiDeInit(
    int32_t coreIdx)
{
    VPU_DeInit(coreIdx);
    printf("[CNM_VPUAPI] Success DeInit VPU\n");
}


static RetCode AllocateAuxBuffer(PDDI_MEDIA_CONTEXT mediaCtx, DecInitialInfo *seqInfo, 
                        DecAuxBufferSizeInfo sizeInfo, AuxBufferType type, MemTypes memTypes)
{
    AuxBufferInfo auxBufferInfo;
    AuxBuffer bufArr[MAX_REG_FRAME];
    Uint32 i;
    RetCode ret_code = RETCODE_FAILURE;

    sizeInfo.type = type;
    auxBufferInfo.type = type;
    auxBufferInfo.num = seqInfo->minFrameBufferCount;

    if (type == AUX_BUF_DEF_CDF     ||
        type == AUX_BUF_MV_COL_PRE_ENT ||
        type == AUX_BUF_SEG_MAP) {
        auxBufferInfo.num = 1;
    }
    else if (type == AUX_BUF_MV_COL) {
        auxBufferInfo.num = seqInfo->reqMvColBufferCount;
    }

    osal_memset(bufArr, 0x00, sizeof(bufArr));
    auxBufferInfo.bufArray = bufArr;

    for (i = 0; i < auxBufferInfo.num; i++) {
        if ((ret_code = VPU_DecGetAuxBufSize(mediaCtx->decHandle, sizeInfo, &mediaCtx->vbAux[type][i].size)) != RETCODE_SUCCESS) {
            break;
        } else {
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbAux[type][i], memTypes, 0) < 0) {
                ret_code = RETCODE_INSUFFICIENT_RESOURCE;
                break;
            }
            auxBufferInfo.bufArray[i].index = i;
            auxBufferInfo.bufArray[i].addr = mediaCtx->vbAux[type][i].phys_addr;
            auxBufferInfo.bufArray[i].size = mediaCtx->vbAux[type][i].size;
        }
    }

    if (ret_code == RETCODE_SUCCESS) {
        // host to API, to used
        ret_code = VPU_DecRegisterAuxBuffer(mediaCtx->decHandle, auxBufferInfo);
    }

    return ret_code;
}

#define MAX_RES_WIDTH 8192
#define MAX_RES_HEIGHT 8192

static VAStatus AllocateDecFrameBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    DecInitialInfo seqInfo)
{
    DecHandle hdl = mediaCtx->decHandle;
    DecOpenParam decOP = mediaCtx->decOP;
    vpu_buffer_t *pVb = NULL;
    FrameBufferAllocInfo fbAllocInfo;
    FrameBuffer frameBuf[VPUAPI_MAX_FB_NUM];
    FrameBufferFormat format = FORMAT_420;
    int32_t fbHeight = 0;
    int32_t fbStride = 0;
    int32_t fbSize = 0;
    int32_t fbCount = 0;
    RetCode retCode;

    osal_memset(&fbAllocInfo, 0x00, sizeof(FrameBufferAllocInfo));
    osal_memset(frameBuf, 0x00, sizeof(FrameBuffer) * VPUAPI_MAX_FB_NUM);

    format = (seqInfo.lumaBitdepth > 8 || seqInfo.chromaBitdepth > 8) ? FORMAT_420_P10_16BIT_MSB : FORMAT_420;
    if (decOP.bitstreamFormat == STD_VP9 || decOP.bitstreamFormat == STD_AV1)
    {
        fbHeight = VPU_ALIGN64(seqInfo.picHeight);
        fbStride = CalcStride(VPU_ALIGN64(seqInfo.picWidth), format);
    }
    else
    {
        fbHeight = VPU_ALIGN32(seqInfo.picHeight);
        fbStride = CalcStride(seqInfo.picWidth, format);
    }
    fbSize = VPU_GetFrameBufSize(hdl, mediaCtx->coreIdx, fbStride, fbHeight, COMPRESSED_FRAME_MAP, format, mediaCtx->cbcrInterleave, NULL);
    fbCount = seqInfo.minFrameBufferCount;

    fbAllocInfo.format = format;
    fbAllocInfo.cbcrInterleave = mediaCtx->cbcrInterleave;
    fbAllocInfo.mapType = COMPRESSED_FRAME_MAP;
    fbAllocInfo.stride = fbStride;
    fbAllocInfo.height = fbHeight;
    fbAllocInfo.lumaBitDepth = seqInfo.lumaBitdepth;
    fbAllocInfo.chromaBitDepth = seqInfo.chromaBitdepth;
    fbAllocInfo.num = fbCount;
    fbAllocInfo.endian = decOP.frameEndian;
    fbAllocInfo.type = FB_TYPE_CODEC;

    for (int32_t index = 0; index < fbCount; index++)
    {
        pVb = &mediaCtx->frameBufMem[index];
        pVb->size = fbSize;
        printf("[CNM_VPUAPI] %s Allocate FBC buffer i=%d, bitDepth=%d, format=%d, fbSize=%d\n", __FUNCTION__, index, seqInfo.lumaBitdepth, format, fbSize);
        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, DEC_FBC, 0) < 0)
        {
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory frameBuf\n");
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        frameBuf[index].bufY = pVb->phys_addr;
        frameBuf[index].bufCb = -1;
        frameBuf[index].bufCr = -1;
        frameBuf[index].updateFbInfo = TRUE;
    }

    if ((retCode = VPU_DecAllocateFrameBuffer(hdl, fbAllocInfo, frameBuf)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] Failed to allocate frame buffer retCode=0x%x\n", retCode);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    for (int32_t index = fbCount; index < fbCount + mediaCtx->numOfRenderTargets; index++)
    {
        frameBuf[index].bufY = -1;
        frameBuf[index].bufCb = -1;
        frameBuf[index].bufCr = -1;
        frameBuf[index].mapType = LINEAR_FRAME_MAP;
        frameBuf[index].cbcrInterleave = mediaCtx->cbcrInterleave;
        frameBuf[index].nv21 = mediaCtx->nv21;
        frameBuf[index].format = mediaCtx->wtlFormat;
        frameBuf[index].stride = mediaCtx->linearStride;
        frameBuf[index].height = mediaCtx->linearHeight;
        frameBuf[index].endian = decOP.frameEndian;
        frameBuf[index].lumaBitDepth = (mediaCtx->wtlFormat == FORMAT_420) ? 8 : 10;
        frameBuf[index].chromaBitDepth = (mediaCtx->wtlFormat == FORMAT_420) ? 8 : 10;
    }

    DecAuxBufferSizeInfo sizeInfo = {0,};
    sizeInfo.width = seqInfo.picWidth;
    sizeInfo.height = seqInfo.picHeight;

    if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_FBC_Y_TBL, DEC_FBCY_TBL)) != RETCODE_SUCCESS) {
        printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_FBC_Y_TBL, return code : %d \n", retCode);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_FBC_C_TBL, DEC_FBCC_TBL)) != RETCODE_SUCCESS) {
        printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_FBC_C_TBL, return code : %d \n", retCode);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_MV_COL, DEC_MV)) != RETCODE_SUCCESS) {
        printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_MV_COL, return code : %d \n",  retCode);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if (decOP.bitstreamFormat == STD_AV1) {
        if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_DEF_CDF, DEC_DEF_CDF)) != RETCODE_SUCCESS) {
            printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_DEF_CDF, return code : %d \n", retCode);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    } else if (decOP.bitstreamFormat == STD_VP9) {
        sizeInfo.width = MAX_RES_WIDTH;
        sizeInfo.height = MAX_RES_HEIGHT;
        if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_SEG_MAP, DEC_SEG_MAP)) != RETCODE_SUCCESS) {
            printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_SEG_MAP, return code : %d \n", retCode);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    }

    if (decOP.bitstreamFormat == STD_AV1 || decOP.bitstreamFormat == STD_VP9) {
        sizeInfo.width = MAX_RES_WIDTH;
        sizeInfo.height = MAX_RES_HEIGHT;
        if ((retCode = AllocateAuxBuffer(mediaCtx, &seqInfo, sizeInfo, AUX_BUF_MV_COL_PRE_ENT, DEC_ETC)) != RETCODE_SUCCESS) {
            printf("[CNM_VPUAPI] Failed to allocate AUX_BUF_MV_COL_PRE_ENT, return code : %d \n",  retCode);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        
    }

#ifdef FAKE_VPUAPI
#else
    if ((retCode = VPU_DecRegisterFrameBufferEx(hdl, frameBuf, fbCount, mediaCtx->numOfRenderTargets, fbStride, fbHeight, COMPRESSED_FRAME_MAP)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] Failed to VPU_DecRegisterFrameBuffer retCode=0x%x fbc : fbCount=%d fbStride=%d, fbHeight=%d, linear : numOfRenderTargets=%d, stride=%d, bitDepth=%d, wtl_format=%d \n", retCode, fbCount, fbStride, fbHeight, mediaCtx->numOfRenderTargets, mediaCtx->linearStride, (mediaCtx->wtlFormat == FORMAT_420) ? 8 : 10, mediaCtx->wtlFormat);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
#endif
    printf("[CNM_VPUAPI] Success VPU_DecRegisterFrameBuffer fbc : fbCount=%d fbStride=%d, fbHeight=%d linear : numOfRenderTargets=%d, stride=%d, bitDepth=%d, wtl_format=%d \n", fbCount, fbStride, fbHeight, mediaCtx->numOfRenderTargets, mediaCtx->linearStride, (mediaCtx->wtlFormat == FORMAT_420) ? 8 : 10, mediaCtx->wtlFormat);

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiDecCreateBuffer(
    VADriverContextP ctx,
    void *ctxPtr,
    VABufferType type,
    uint32_t size,
    uint32_t numElements,
    void *data,
    VABufferID *bufId)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    VAStatus va = VA_STATUS_SUCCESS;
    MOS_STATUS mos = MOS_STATUS_SUCCESS;
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufferHeapElement;
    DDI_MEDIA_BUFFER *buf;

    buf = (DDI_MEDIA_BUFFER *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_BUFFER));
    if (nullptr == buf)
    {
        va = VA_STATUS_ERROR_ALLOCATION_FAILED;
        return va;
    }

    buf->uiNumElements = numElements;
    buf->iSize = size * numElements;
    buf->uiType = type;
    buf->format = Media_Format_CPU;
    buf->uiOffset = 0;
    buf->pMediaCtx = mediaCtx;

    va = DdiMediaUtil_CreateBuffer(buf, mediaCtx->pDrmBufMgr);
    if (va != VA_STATUS_SUCCESS)
    {
        MOS_FreeMemory(buf);
        return va;
    }

    bufferHeapElement = DdiMediaUtil_AllocPMediaBufferFromHeap(mediaCtx->pBufferHeap);
    if (nullptr == bufferHeapElement)
    {
        DdiMediaUtil_FreeBuffer(buf);
        MOS_FreeMemory(buf);
        va = VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        return va;
    }

    bufferHeapElement->pBuffer = buf;
    bufferHeapElement->pCtx = ctxPtr;
    bufferHeapElement->uiCtxType = DDI_MEDIA_CONTEXT_TYPE_DECODER;

    *bufId = bufferHeapElement->uiVaBufferID;
    mediaCtx->uiNumBufs++;

    if (nullptr == data)
        return va;

    mos = MOS_SecureMemcpy((void *)(buf->pData + buf->uiOffset), size * numElements, data, size * numElements);
    if (mos != MOS_STATUS_SUCCESS)
    {
        va = VA_STATUS_ERROR_OPERATION_FAILED;
        return va;
    }

    return va;
}

static VAStatus VpuApiDecRenderPicture(
    VADriverContextP ctx,
    VABufferID *buffers,
    int32_t numBuffers)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    VAStatus va = VA_STATUS_SUCCESS;
    void *data = nullptr;

    if (!buffers)
    {
        va = VA_STATUS_ERROR_INVALID_BUFFER;
        return va;
    }

    for (int32_t idx = 0; idx < numBuffers; idx++)
    {
        if (buffers[idx] == VA_INVALID_ID)
        {
            va = VA_STATUS_ERROR_INVALID_BUFFER;
            return va;
        }

        DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buffers[idx]);
        if (buf == nullptr)
        {
            va = VA_STATUS_ERROR_INVALID_BUFFER;
            return va;
        }

        VABufferType type = (VABufferType)buf->uiType;
        uint32_t size = buf->iSize;

        DdiMedia_MapBuffer(ctx, buffers[idx], &data);
        if (data == nullptr)
        {
            va = VA_STATUS_ERROR_INVALID_BUFFER;
            return va;
        }

        switch ((int32_t)type)
        {
        case VASliceParameterBufferType:
            mediaCtx->numOfSlice++;
        case VAPictureParameterBufferType:
        case VAIQMatrixBufferType:
        case VASubsetsParameterBufferType:
        {
            void *convData = NULL;
            vpu_buffer_t paramBuf = mediaCtx->paramBuf[mediaCtx->bufIdx];
            convData = ConvertBufferData(mediaCtx, type, size, (uint8_t *)data);
            if (convData)
            {
                if (paramBuf.size < mediaCtx->paramSize + VPU_ALIGN16(size))
                {
                    printf("[CNM_VPUAPI] Internal paramBuf overflow. (internal:0x%x < input:0x%x)\n", paramBuf.size, mediaCtx->paramSize + VPU_ALIGN16(size));
                    va = VA_STATUS_ERROR_OPERATION_FAILED;
                    free(convData);
                    return va;
                }

                vdi_write_memory(mediaCtx->coreIdx, paramBuf.phys_addr + mediaCtx->paramSize, (Uint8 *)convData, VPU_ALIGN16(size), VDI_LITTLE_ENDIAN);
                mediaCtx->paramSize += VPU_ALIGN16(size);
                free(convData);

                printf("[CNM_VPUAPI] type: %d, size: %d/%d, paramSize:%d bufIdx:%d\n",(int32_t)type, size,VPU_ALIGN16(size),mediaCtx->paramSize,mediaCtx->bufIdx);
            }
        }
        break;
        case VASliceDataBufferType:
        {
            vpu_buffer_t bsBuf = mediaCtx->bsBuf[mediaCtx->bufIdx];
            if (bsBuf.size < mediaCtx->bsSize + size)
            {
                printf("[CNM_VPUAPI] Internal bsBuf overflow. (internal:0x%x < input:0x%x)\n", bsBuf.size, mediaCtx->bsSize + size);
                va = VA_STATUS_ERROR_OPERATION_FAILED;
                return va;
            }

            vdi_write_memory(mediaCtx->coreIdx, bsBuf.phys_addr + mediaCtx->bsSize, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
            mediaCtx->bsSize += size;
            break;
        }
        }

        DdiMedia_UnmapBuffer(ctx, buffers[idx]);
    }

    return va;
}

static BOOL VpuApiAllocateworkBuffer(int32_t coreIdx, vpu_buffer_t *vbWork) {
    vbWork->phys_addr = 0;
    vbWork->size = (Uint32)WAVE637DEC_WORKBUF_SIZE_FOR_CQ;

    if (vdi_allocate_dma_memory(coreIdx, vbWork, DEC_WORK, 0) < 0)
        return FALSE;

    return TRUE;
}

static BOOL VpuApiAllocateTempBuffer(int32_t coreIdx, vpu_buffer_t *vbTemp)
{
    vbTemp->phys_addr = 0;
    vbTemp->size  = VPU_ALIGN4096(WAVE6_TEMPBUF_SIZE_FOR_CQ);
    
    if (vdi_allocate_dma_memory(coreIdx, vbTemp, DEC_TEMP, 0) < 0)
        return FALSE;

    return TRUE;
}

static BOOL VpuApiAllocateArTableBuffer(int32_t coreIdx, vpu_buffer_t *vbAr)
{
#define WAVE6_AR_TABLE_BUF_SIZE 1024 //adative round
    vbAr->phys_addr = 0;
    vbAr->size      = VPU_ALIGN4096(WAVE6_AR_TABLE_BUF_SIZE);
    if (vdi_allocate_dma_memory(coreIdx, vbAr, ENC_AR, 0) < 0)
        return FALSE;
    return TRUE;
}

static BOOL VpuApiAllocateSecAxiBuffer(int32_t coreIdx, vpu_buffer_t *vbSecAxi)
{
    if (vdi_get_sram_memory(coreIdx, vbSecAxi) < 0)
        return FALSE;

    return TRUE;
}

static BOOL Wave6xxSetLinearFrameBufferInfo(
    DecHandle decHandle,
    PDDI_MEDIA_CONTEXT mediaCtx,
    FrameBuffer* frame_buffer,
    DecInitialInfo   seqInfo)
{
    Uint32 framebufSize;
    Uint32 coreIndex;
    FrameBufferAllocInfo fbAllocInfo;
    RetCode ret;
    size_t linearStride;
    size_t picWidth;
    size_t picHeight;
    size_t fbHeight;

    TiledMapType mapType = LINEAR_FRAME_MAP;
    coreIndex = mediaCtx->coreIdx;

    osal_memset(&fbAllocInfo, 0x00, sizeof(FrameBufferAllocInfo));
    FrameBufferFormat outFormat = mediaCtx->wtlFormat;
    picWidth  = frame_buffer->width;
    picHeight = (STD_AVC == mediaCtx->decOP.bitstreamFormat) ? VPU_ALIGN16(frame_buffer->height) : VPU_ALIGN8(frame_buffer->height);
    fbHeight  = picHeight;

    linearStride = VPU_GetFrameBufStride(decHandle, picWidth, picHeight, outFormat, mediaCtx->cbcrInterleave, (TiledMapType)mapType);
    framebufSize = VPU_GetFrameBufSize(decHandle, coreIndex, linearStride, fbHeight, (TiledMapType)mapType, outFormat, mediaCtx->cbcrInterleave, NULL);
    frame_buffer->updateFbInfo = TRUE;
    frame_buffer->size  = framebufSize;

    fbAllocInfo.cbcrInterleave  = mediaCtx->cbcrInterleave;
    fbAllocInfo.nv21    = mediaCtx->nv21;
    fbAllocInfo.format  = outFormat;
    fbAllocInfo.num     = 1;
    fbAllocInfo.mapType = (TiledMapType)mapType;
    fbAllocInfo.stride  = linearStride;
    fbAllocInfo.height  = fbHeight;
    fbAllocInfo.endian  = mediaCtx->decOP.frameEndian;
    fbAllocInfo.type = FB_TYPE_CODEC;

    printf("[CNM_VPUAPI] width:%d, height:%d linearStrie:%ld, fbheight:%ld, framebufSize:%d cbcrInterleave:%d nv21:%d\n", 
                                                                        frame_buffer->width, 
                                                                        frame_buffer->height, 
                                                                        linearStride, 
                                                                        fbHeight, 
                                                                        framebufSize,
                                                                        mediaCtx->cbcrInterleave,
                                                                        mediaCtx->nv21);

    ret = VPU_DecAllocateFrameBuffer(decHandle, fbAllocInfo, frame_buffer);
    if (ret != RETCODE_SUCCESS) {
        VLOG(ERR, "%s:%d failed to VPU_DecAllocateFrameBuffer() ret:%d\n",
            __FUNCTION__, __LINE__, ret);
        return FALSE;
    }

    return TRUE;
}


static VAStatus VpuApiDecOpen(
    VADriverContextP ctx,
    VAConfigID configId,
    int32_t pictureWidth,
    int32_t pictureHeight,
    VAContextID *context)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT contextHeapElement;
    VAStatus va = VA_STATUS_SUCCESS;
    PDDI_DECODE_CONTEXT decCtx = (PDDI_DECODE_CONTEXT)MOS_AllocAndZeroMemory(sizeof(DDI_DECODE_CONTEXT));
    RetCode retCode;
    CodStd bitFormat = STD_HEVC;
    VAProfile profile;
    VAEntrypoint entrypoint;
    bool findValidConfigId = false;
    uint32_t i = 0;

    for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
    {
        if (s_vpuApiAttrs[i].actual_profile == VAProfileNone)
        {
            continue;
        }
        if (s_vpuApiAttrs[i].configId == configId)
        {
            findValidConfigId = true;
            break;
        }
    }
    if (findValidConfigId == false)
    {
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    profile = s_vpuApiAttrs[i].actual_profile;
    entrypoint = s_vpuApiAttrs[i].actual_entrypoint;

    if (pictureWidth > VPUAPI_MAX_PIC_WIDTH || pictureHeight > VPUAPI_MAX_PIC_HEIGHT)
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;

    switch (profile)
    {
#ifdef VA_PROFILE_H264_HIGH_10
    case VAProfileH264High10:
#endif
    case VAProfileH264High:
    case VAProfileH264Main:
    case VAProfileH264ConstrainedBaseline:
        bitFormat = STD_AVC;
        break;
    case VAProfileVP9Profile0:
    case VAProfileVP9Profile2:
        bitFormat = STD_VP9;
        break;
    case VAProfileHEVCMain:
    case VAProfileHEVCMain10:
        bitFormat = STD_HEVC;
        break;
    case VAProfileAV1Profile0:
    case VAProfileAV1Profile1:
        bitFormat = STD_AV1;
        break;
#ifdef VA_PROFILE_AVS2_MAIN_10
    case VAProfileAVS2Main:
    case VAProfileAVS2Main10:
        bitFormat = STD_AVS2;
        break;
#endif
    default:
        printf("[CNM_VPUAPI] not supported profile=0x%x\n", profile);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    DdiMediaUtil_InitMutex(&mediaCtx->vpuapiMutex);
    mediaCtx->pictureWidth = pictureWidth;
    mediaCtx->pictureHeight = pictureHeight;
    mediaCtx->decOP.bitstreamFormat = bitFormat;
    mediaCtx->decOP.coreIdx = mediaCtx->coreIdx;
    mediaCtx->decOP.bitstreamMode = BS_MODE_PIC_END;
    mediaCtx->decOP.wtlEnable = TRUE;
    mediaCtx->decOP.wtlMode = FF_FRAME;
    mediaCtx->decOP.streamEndian = VDI_LITTLE_ENDIAN;
    mediaCtx->decOP.frameEndian = VDI_LITTLE_ENDIAN;
    mediaCtx->decOP.cbcrInterleave = mediaCtx->cbcrInterleave;
    mediaCtx->decOP.nv21 = mediaCtx->nv21;
    mediaCtx->decOP.vaEnable = TRUE;
    mediaCtx->decOP.bitstreamBuffer = 0;     // if bitstreamMode is BS_MODE_PIC_END. this value should be 0
    mediaCtx->decOP.bitstreamBufferSize = 0; // if bitstreamMode is BS_MODE_PIC_END. this value should be 0
   
    mediaCtx->decOP.numUseVcore = 1;
    mediaCtx->decOP.vcoreCoreIdc = 1;
#ifdef FAKE_VPUAPI
    mediaCtx->decHandle = s_decHandle;
#else
    mediaCtx->decOP.displayMode = DISP_MODE_DEC_ORDER; 

    VpuApiAllocateworkBuffer(mediaCtx->coreIdx, &mediaCtx->vbWork);
    mediaCtx->decOP.instBuffer.workBufBase = mediaCtx->vbWork.phys_addr;
    mediaCtx->decOP.instBuffer.workBufSize = mediaCtx->vbWork.size;

    VpuApiAllocateTempBuffer(mediaCtx->coreIdx, &mediaCtx->vbTemp);
    mediaCtx->decOP.instBuffer.tempBufBase = mediaCtx->vbTemp.phys_addr;
    mediaCtx->decOP.instBuffer.tempBufSize = mediaCtx->vbTemp.size;

    VpuApiAllocateSecAxiBuffer(mediaCtx->coreIdx, &mediaCtx->vbSecAxi);
#ifdef SUPPORT_MULTI_VCORE
    mediaCtx->decOP.instBuffer.secAxiBufBaseCore0 = mediaCtx->vbSecAxi.phys_addr;
    mediaCtx->decOP.instBuffer.secAxiBufSizeCore0 = mediaCtx->vbSecAxi.size/2;
    mediaCtx->decOP.instBuffer.secAxiBufBaseCore1 = mediaCtx->vbSecAxi.phys_addr;
    mediaCtx->decOP.instBuffer.secAxiBufSizeCore1 = mediaCtx->vbSecAxi.size/2;
#else
    mediaCtx->decOP.instBuffer.secAxiBufBaseCore0 = mediaCtx->vbSecAxi.phys_addr;
    mediaCtx->decOP.instBuffer.secAxiBufSizeCore0 = mediaCtx->vbSecAxi.size;
#endif

    retCode = VPU_DecOpen(&mediaCtx->decHandle, &mediaCtx->decOP);
    if (retCode != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] %s Failed to Open decoder instance retCode=0x%x\n", __FUNCTION__, retCode);
        printf("    >>> bitstreamFormat=%d, bitstreamMode=%d, coreIdx=%d \n", mediaCtx->decOP.bitstreamFormat, mediaCtx->decOP.bitstreamMode, mediaCtx->decOP.coreIdx);
        va = VA_STATUS_ERROR_OPERATION_FAILED;
        return va;
    }
    retCode = VPU_DecGiveCommand(mediaCtx->decHandle, DEC_SET_WTL_FRAME_FORMAT, &mediaCtx->wtlFormat);
    if (retCode != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] %s Failed to DEC_SET_WTL_FRAME_FORMAT command retCode=0x%x\n", __FUNCTION__, retCode);
        va = VA_STATUS_ERROR_OPERATION_FAILED;
        return va;
    }
#endif

    DdiMediaUtil_LockMutex(&mediaCtx->DecoderMutex);
    contextHeapElement = DdiMediaUtil_AllocPVAContextFromHeap(mediaCtx->pDecoderCtxHeap);
    if (nullptr == contextHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->DecoderMutex);
        va = VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        return va;
    }

    contextHeapElement->pVaContext = (void *)decCtx;
    mediaCtx->uiNumDecoders++;
    *context = (VAContextID)(contextHeapElement->uiVaContextID + DDI_MEDIA_VACONTEXTID_OFFSET_DECODER);
    DdiMediaUtil_UnLockMutex(&mediaCtx->DecoderMutex);

    printf("[CNM_VPUAPI] Success Open decoder instance: format %d\n", mediaCtx->decOP.bitstreamFormat);

    mediaCtx->vaProfile = profile;
    for (i = 0; i < VPUAPI_MAX_BUF_NUM; i++)
    {
        if (mediaCtx->paramBuf[i].phys_addr == 0)
        {
            mediaCtx->paramBuf[i].size = 0xA00000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->paramBuf[i], DEC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory paramBuf[%d]\n", i);
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }
        if (mediaCtx->bsBuf[i].phys_addr == 0)
        {
            mediaCtx->bsBuf[i].size = 0xA00000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->bsBuf[i], DEC_BS, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory bsBuf[%d]\n", i);
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }
    }
    for (i = 0; i < VPUAPI_MAX_FB_NUM; i++)
    {
        mediaCtx->usedRenderTargets[i] = VPUAPI_UNKNOWN_SURFACE_ID;
    }
#ifdef CNM_FPGA_PLATFORM
#ifdef CNM_VPUAPI_INTERFACE_DEBUG
    mediaCtx->fpYuvDebug = fopen("/Stream/work/gregory/output.yuv", "wb");
    if (!mediaCtx->fpYuvDebug)
    {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
#endif
#endif
    return va;
}

static VAStatus VpuApiDecCheckValidity(
    VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    if (mediaCtx->numOfSlice == 0 || mediaCtx->paramSize == 0 || mediaCtx->bsSize == 0)
    {
        printf("[CNM_VPUAPI] FAIL VpuApiDecCheckValidity \n");
        printf("    >>> invalid vaapi parameter numOfSlice=%d, paramSize=%d, bsSize=%d \n", mediaCtx->numOfSlice, mediaCtx->paramSize, mediaCtx->bsSize);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiDecSeqInit(
    VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode ret = RETCODE_SUCCESS;
    DecHandle hdl = mediaCtx->decHandle;
    DecInitialInfo seqInfo;
    BOOL seqInited = FALSE;
    Int32 intrFlag = 0;

    if (mediaCtx->seqInited == TRUE)
    {
        return VA_STATUS_SUCCESS;
    }
#ifdef FAKE_VPUAPI
    mediaCtx->seqInited = TRUE;
#else
    VPU_DecSetRdPtr(hdl, mediaCtx->bsBuf[mediaCtx->bufIdx].phys_addr, TRUE);
    VPU_DecUpdateBitstreamBuffer(hdl, mediaCtx->bsSize);
    VPU_DecGiveCommand(hdl, DEC_SET_VAAPI_FIRST_PARAM_BUFFER, &mediaCtx->paramBuf[mediaCtx->bufIdx].phys_addr);
    VPU_DecGiveCommand(hdl, DEC_SET_VAAPI_WIDTH, &mediaCtx->pictureWidth);
    VPU_DecGiveCommand(hdl, DEC_SET_VAAPI_HEIGHT, &mediaCtx->pictureHeight);

{
    uint8_t *pParam;
    int32_t size = sizeof(VAPictureParameterBufferH264);
    
    size = VPU_ALIGN16(size);
    pParam = (uint8_t *)osal_malloc(size);
    vdi_read_memory(mediaCtx->coreIdx, mediaCtx->paramBuf[mediaCtx->bufIdx].phys_addr, pParam, size, VDI_LITTLE_ENDIAN);
    VAPictureParameterBufferH264 *h264 = (VAPictureParameterBufferH264 *)pParam;

    printf("[CNM_VPUAPI] width:%d, height:%d, luma:%d, chroma:%d\n",(h264->picture_width_in_mbs_minus1+1)*16, 
                                                                    (h264->picture_height_in_mbs_minus1+1)*16,
                                                                    h264->bit_depth_luma_minus8,
                                                                    h264->bit_depth_chroma_minus8);
    free(pParam);
}


    if ((ret = VPU_DecIssueSeqInit(hdl)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_DecIssueSeqInit: 0x%x\n", ret);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
    intrFlag = VPU_WaitInterruptEx(hdl, VPU_DEC_TIMEOUT);
    if (intrFlag == -1)
    {
        printf("[CNM_VPUAPI] FAIL VPU_WaitInterruptEx for VPU_DecIssueSeqInit: timeout=%dms\n", VPU_DEC_TIMEOUT);
        VPU_DecCompleteSeqInit(hdl, &seqInfo);
        VpuApiDecPrintDebugInfo(ctx);
        return VA_STATUS_ERROR_TIMEDOUT;
    }

    VPU_ClearInterruptEx(hdl, intrFlag);
    if (intrFlag & (1 << INT_WAVE5_INIT_SEQ))
    {
        seqInited = TRUE;
    }
    else
    {
        printf("[CNM_VPUAPI] VPU_DecIssueSeqInit is done. but intrFlag=0x%x is wrong\n", intrFlag);
        return VA_STATUS_ERROR_DECODING_ERROR;
    }

    if ((ret = VPU_DecCompleteSeqInit(hdl, &seqInfo)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_DecCompleteSeqInit: 0x%x\n", seqInfo.seqInitErrReason);
        mediaCtx->seqInited = seqInited;
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    if (seqInited == FALSE)
    {
        mediaCtx->seqInited = seqInited;
        return VA_STATUS_ERROR_DECODING_ERROR;
    }

    mediaCtx->chromaFormatIDC = seqInfo.chromaFormatIDC;
    mediaCtx->seqInited = seqInited;

    printf("[CNM_VPUAPI] SUCCESS VpuApiDecSeqInit\n");
    printf("[CNM_VPUAPI] >>> width : %d | height : %d\n", seqInfo.picWidth, seqInfo.picHeight);
    printf("[CNM_VPUAPI] >>> crop left : %d | top : %d | right : %d | bottom : %d\n", seqInfo.picCropRect.left, seqInfo.picCropRect.top, seqInfo.picCropRect.right, seqInfo.picCropRect.bottom);
    printf("[CNM_VPUAPI] >>> lumaBitdepth : %d | chromaBitdepth : %d | chromaFormatIDC : %d \n", seqInfo.lumaBitdepth, seqInfo.chromaBitdepth, seqInfo.chromaFormatIDC);
    printf("[CNM_VPUAPI] >>> minFrameBufferCount : %d\n", seqInfo.minFrameBufferCount);
    printf("[CNM_VPUAPI] >>> numOfRenderTargets : %d\n", mediaCtx->numOfRenderTargets);
#endif
    if (AllocateDecFrameBuffer(mediaCtx, seqInfo) != VA_STATUS_SUCCESS)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    for(int i=0; i<mediaCtx->numOfRenderTargets; i++) {
        if(Wave6xxSetLinearFrameBufferInfo(hdl,mediaCtx,&mediaCtx->linearFrameBuf[i],seqInfo) != TRUE) {
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    }

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiDecGetResult(
    VADriverContextP ctx,
    VASurfaceID surfaceID)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode ret = RETCODE_SUCCESS;
    DecHandle hdl = mediaCtx->decHandle;
    DecOutputInfo outputInfo;
    Int32 intrFlag = 0;

    osal_memset(&outputInfo, 0x00, sizeof(DecOutputInfo));
printf("VpuApiDecGetResult %d \r\n",__LINE__);
    DdiMediaUtil_LockMutex(&mediaCtx->vpuapiMutex);

    if (surfaceID == VPUAPI_UNKNOWN_SURFACE_ID)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
        return VA_STATUS_SUCCESS;
    }

#ifdef FAKE_VPUAPI
#else
printf("VpuApiDecGetResult %d \r\n",__LINE__);
    intrFlag = VPU_WaitInterruptEx(hdl, VPU_DEC_TIMEOUT);
    if (intrFlag == -1)
    {
        printf("[CNM_VPUAPI] FAIL VPU_WaitInterruptEx for VPU_DecStartOneFrame : timeout=%dms\n", VPU_DEC_TIMEOUT);
        VpuApiDecPrintDebugInfo(ctx);
        VPU_DecGetOutputInfo(hdl, &outputInfo);
        DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
        return VA_STATUS_ERROR_TIMEDOUT;
    }
printf("VpuApiDecGetResult %d \r\n",__LINE__);

    VPU_ClearInterruptEx(hdl, intrFlag);
    if (intrFlag & (1 << INT_WAVE5_DEC_PIC))
    {
    }
    else
    {
        printf("[CNM_VPUAPI] VPU_DecStartOneFrame is done. but intrFlag=0x%x is wrong\n", intrFlag);
        DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
printf("VpuApiDecGetResult %d \r\n",__LINE__);

    if ((ret = VPU_DecGetOutputInfo(hdl, &outputInfo)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_DecGetOutputInfo: 0x%x, errorReason: 0x%x\n", ret, outputInfo.errorReason);
        DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
printf("VpuApiDecGetResult %d \r\n",__LINE__);

    if (outputInfo.decodingSuccess == 0)
    {
        printf("[CNM_VPUAPI] Decoding Fail: %d, Warning Info: %d\n", outputInfo.decodingSuccess, outputInfo.warnInfo);
    }
printf("VpuApiDecGetResult %d \r\n",__LINE__);

    if (outputInfo.indexFrameDecoded < 0)
    {
        printf("[CNM_VPUAPI] DECODE FAIL indexFrameDecoded %d indexFrameDisplay %d.\n", outputInfo.indexFrameDecoded, outputInfo.indexFrameDisplay);
        DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
printf("VpuApiDecGetResult %d \r\n",__LINE__);

    if (outputInfo.numOfErrMBs > 0)
    {
        printf("[CNM_VPUAPI] Warning Error Block: %d\n", outputInfo.numOfErrMBs);
    }
    printf("[CNM_VPUAPI] IDX %d | SURFACE %d | PIC %d | NAL %d | BufAddrY : 0x%lx | BufAddrCb : 0x%lx | BufAddrCr : 0x%lx | vaParamAddr : 0x%lx | BYTEPOS 0x%lx ~ 0x%lx | CONSUME : %ld | DISP %dx%d\n",
           mediaCtx->decIdx,
           surfaceID,
           outputInfo.picType, outputInfo.nalType,
           outputInfo.vaDecodeBufAddrY, outputInfo.vaDecodeBufAddrCb, outputInfo.vaDecodeBufAddrCr, outputInfo.vaParamAddr,
           outputInfo.bytePosFrameStart, outputInfo.bytePosFrameEnd, outputInfo.bytePosFrameEnd - outputInfo.bytePosFrameStart,
           outputInfo.decPicWidth, outputInfo.decPicHeight);

    mediaCtx->decIdx++;
#ifdef CNM_FPGA_PLATFORM
    {
        BOOL cbcrInterleave = mediaCtx->cbcrInterleave;
        uint32_t lumaSize = 0, chromaSize = 0;
        uint32_t fourcc = 0;
        uint32_t lumaStride = 0;
        uint32_t chromaUStride = 0;
        uint32_t chromaVStride = 0;
        uint32_t lumaOffset = 0;
        uint32_t chromaUOffset = 0;
        uint32_t chromaVOffset = 0;
        uint32_t bufferName = 0;
        void *buffer = NULL;
        uint8_t *surfaceOffset;
        uint8_t *dataY;
        uint8_t *dataCb;
        uint8_t *dataCr;

        lumaSize = mediaCtx->linearStride * mediaCtx->linearHeight;
        if (cbcrInterleave == FALSE)
            chromaSize = (mediaCtx->linearStride / 2) * (mediaCtx->linearHeight / 2);
        else
            chromaSize = (mediaCtx->linearStride) * (mediaCtx->linearHeight / 2);

#ifdef CNM_VPUAPI_INTERFACE_DEBUG
        printf("[CNM_VPUAPI] Dump Linear Frame WxH : %dx%d, wtl_format=%d\n", mediaCtx->linearStride, mediaCtx->linearHeight, mediaCtx->wtlFormat);
        printf("[CNM_VPUAPI] Dump lumaSize : %d\n", lumaSize);
        printf("[CNM_VPUAPI] Dump chromaSize : %d\n", chromaSize);
        printf("[CNM_VPUAPI] Dump BufAddrY: 0x%lx\n", outputInfo.vaDecodeBufAddrY);
        printf("[CNM_VPUAPI] Dump BufAddrCb: 0x%lx\n", outputInfo.vaDecodeBufAddrCb);
        printf("[CNM_VPUAPI] Dump BufAddrCr: 0x%lx\n", outputInfo.vaDecodeBufAddrCr);
#endif

        dataY = (uint8_t *)osal_malloc(lumaSize);
        dataCb = (uint8_t *)osal_malloc(chromaSize);
        dataCr = (uint8_t *)osal_malloc(chromaSize);

#ifdef CNM_VPUAPI_INTERFACE_DEBUG
        if (mediaCtx->fpYuvDebug)
        {
            printf("[CNM_VPUAPI]+Dump YuvDebug: fp=%p, lumaSize=%d, chromSize=%d\n", mediaCtx->fpYuvDebug, lumaSize, chromaSize);
            vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrY, dataY, lumaSize, VDI_LITTLE_ENDIAN);
            fwrite((void *)dataY, 1, lumaSize, mediaCtx->fpYuvDebug);
            vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrCb, dataCb, chromaSize, VDI_LITTLE_ENDIAN);
            fwrite((void *)dataCb, 1, chromaSize, mediaCtx->fpYuvDebug);
            if (cbcrInterleave == FALSE)
            {
                vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrCr, dataCr, chromaSize, VDI_LITTLE_ENDIAN);
                fwrite((void *)dataCr, 1, chromaSize, mediaCtx->fpYuvDebug);
            }
            printf("[CNM_VPUAPI]-Dump YuvDebug: fp=%p, lumaSize=%d, chromSize=%d\n", mediaCtx->fpYuvDebug, lumaSize, chromaSize);
        }
#endif
        vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrY, dataY, lumaSize, VDI_LITTLE_ENDIAN);
        vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrCb, dataCb, chromaSize, VDI_LITTLE_ENDIAN);
        if (cbcrInterleave == FALSE)
            vdi_read_memory(mediaCtx->coreIdx, outputInfo.vaDecodeBufAddrCr, dataCr, chromaSize, VDI_LITTLE_ENDIAN);

        DdiMedia_LockSurface(ctx, surfaceID,
                             &fourcc,
                             &lumaStride, &chromaUStride, &chromaVStride,
                             &lumaOffset, &chromaUOffset, &chromaVOffset,
                             &bufferName, &buffer);
        printf("[CNM_VPUAPI] Surface Info: fourcc=0x%x, lumaStride=%d, chromaUStride=%d, lumaOffset=%d, chromalUoffset=%d, chromaVOffset=%d\n", fourcc, lumaStride, chromaUStride, lumaOffset, chromaUOffset, chromaVOffset);

        surfaceOffset = (unsigned char *)buffer + lumaOffset;
        memcpy(surfaceOffset, dataY, lumaSize);

        surfaceOffset = (unsigned char *)buffer + chromaUOffset;
        memcpy(surfaceOffset, dataCb, chromaSize);
        if (cbcrInterleave == FALSE)
        {
            surfaceOffset = (unsigned char *)buffer + chromaVOffset;
            memcpy(surfaceOffset, dataCr, chromaSize);
        }

        DdiMedia_UnlockSurface(ctx, surfaceID);

        osal_free(dataY);
        osal_free(dataCb);
        osal_free(dataCr);
    }
#endif

#endif
    DdiMediaUtil_UnLockMutex(&mediaCtx->vpuapiMutex);

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiDecPic(
    VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode ret = RETCODE_SUCCESS;
    DecHandle hdl = mediaCtx->decHandle;
    DecParam param;

    osal_memset(&param, 0x00, sizeof(DecParam));

    printf("VpuApiDecPic 1 \r\n");
    while (FindUsedRenderTarget(mediaCtx, mediaCtx->renderTarget))
    {
        printf("VpuApiDecPic 1-1 \r\n");
        VAStatus vaStatus = VpuApiDecGetResult(ctx, GetUsedRenderTarget(mediaCtx));
        if (vaStatus != VA_STATUS_SUCCESS)
            return vaStatus;
    }
    printf("VpuApiDecPic 2 \r\n");
#ifdef FAKE_VPUAPI
#else
    VPU_DecSetRdPtr(hdl, mediaCtx->bsBuf[mediaCtx->bufIdx].phys_addr, TRUE);
    VPU_DecUpdateBitstreamBuffer(hdl, mediaCtx->bsSize);
#endif

    param.vaParamAddr = mediaCtx->paramBuf[mediaCtx->bufIdx].phys_addr;
    param.vaSliceNum = mediaCtx->numOfSlice;
    param.vaRenderTarget = GetRenderTargetIndex(mediaCtx, mediaCtx->renderTarget);
#ifdef CNM_FPGA_PLATFORM
    param.vaDecodeBufAddrY = mediaCtx->linearFrameBuf[param.vaRenderTarget].bufY;
    param.vaDecodeBufAddrCb = mediaCtx->linearFrameBuf[param.vaRenderTarget].bufCb;
    param.vaDecodeBufAddrCr = mediaCtx->linearFrameBuf[param.vaRenderTarget].bufCr;


{
    uint8_t *pParam;
    int32_t size = sizeof(VAPictureParameterBufferH264);
    
    size = VPU_ALIGN16(size);
    pParam = (uint8_t *)osal_malloc(size);
    vdi_read_memory(mediaCtx->coreIdx, mediaCtx->paramBuf[mediaCtx->bufIdx].phys_addr, pParam, size, VDI_LITTLE_ENDIAN);
    VAPictureParameterBufferH264 *h264 = (VAPictureParameterBufferH264 *)pParam;

    printf("[CNM_VPUAPI] vaparam addr:%lx, width:%d, height:%d, luma:%d, chroma:%d\n",mediaCtx->paramBuf[mediaCtx->bufIdx].phys_addr,
                                                                    (h264->picture_width_in_mbs_minus1+1)*16, 
                                                                    (h264->picture_height_in_mbs_minus1+1)*16,
                                                                    h264->bit_depth_luma_minus8,
                                                                    h264->bit_depth_chroma_minus8);
    free(pParam);
}

    mediaCtx->linearFrameBuf[param.vaRenderTarget].chromaFormatIDC = mediaCtx->chromaFormatIDC;
    printf("[CNM_VPUAPI] vaRenderTarget:%d chromaFormatIDC:%d \n",param.vaRenderTarget, mediaCtx->chromaFormatIDC);
    ret = VPU_DecRegisterDisplayBuffer(hdl, mediaCtx->linearFrameBuf[param.vaRenderTarget]);
    if (ret != RETCODE_SUCCESS) {
        printf("[CNM_VPUAPI] failed to register display buffer : %d,  vaRenderTarget:%d , bufIdx:%d \n", ret,param.vaRenderTarget,mediaCtx->bufIdx);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
#else
    printf("[CNM_VPUAPI] customer needs to get physical address from render_target surface=0x%x", mediaCtx->renderTarget);
    printf("[CNM_VPUAPI] customer needs to set param.vaDecodeBufAddrY and param.vaDecodeBufAddrCb and param.vaDecodeBufAddrCr to Physical address that VPU can access.\n");
#endif

#ifdef CNM_VPUAPI_INTERFACE_DEBUG
    printf("[CNM_VPUAPI] bsBuf: 0x%lx\n", mediaCtx->bsBuf[mediaCtx->bufIdx].phys_addr);
    printf("[CNM_VPUAPI] bsSize: %d\n", mediaCtx->bsSize);
    printf("[CNM_VPUAPI] paramBuf: 0x%lx\n", param.vaParamAddr);
    printf("[CNM_VPUAPI] numOfSlice: %d\n", param.vaSliceNum);
    printf("[CNM_VPUAPI] RenderTarget: %d\n", param.vaRenderTarget);
    printf("[CNM_VPUAPI] BufAddrY: 0x%lx\n", param.vaDecodeBufAddrY);
    printf("[CNM_VPUAPI] BufAddrCb: 0x%lx\n", param.vaDecodeBufAddrCb);
    printf("[CNM_VPUAPI] BufAddrCr: 0x%lx\n", param.vaDecodeBufAddrCr);
#endif

#ifdef FAKE_VPUAPI
#else
    while ((ret = VPU_DecStartOneFrame(hdl, &param)) == RETCODE_QUEUEING_FAILURE)
    {
        VAStatus vaStatus = VpuApiDecGetResult(ctx, GetUsedRenderTarget(mediaCtx));
        if (vaStatus != VA_STATUS_SUCCESS)
            return vaStatus;

        VPU_DecSetRdPtr(hdl, mediaCtx->bsBuf[mediaCtx->bufIdx].phys_addr, TRUE);
        VPU_DecUpdateBitstreamBuffer(hdl, mediaCtx->bsSize);
    }

    if (ret != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_DecStartOneFrame: 0x%x\n", ret);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
#endif

    mediaCtx->numOfSlice = 0;
    mediaCtx->paramSize = 0;
    mediaCtx->bsSize = 0;
    mediaCtx->bufIdx = (mediaCtx->bufIdx == COMMAND_QUEUE_DEPTH) ? 0 : mediaCtx->bufIdx + 1;

    SetUsedRenderTarget(mediaCtx, mediaCtx->renderTarget);

    return VA_STATUS_SUCCESS;
}

static void VpuApiDecClose(
    VADriverContextP ctx,
    VAContextID context)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    uint32_t ctxType;
    PDDI_DECODE_CONTEXT decCtx = (PDDI_DECODE_CONTEXT)DdiMedia_GetContextFromContextID(ctx, context, &ctxType);
    uint32_t decIndex = (uint32_t)context & DDI_MEDIA_MASK_VACONTEXTID;
    VASurfaceID renderTarget = GetUsedRenderTarget(mediaCtx);

    while (renderTarget != VPUAPI_UNKNOWN_SURFACE_ID)
    {
        VAStatus vaStatus = VpuApiDecGetResult(ctx, renderTarget);
        if (vaStatus != VA_STATUS_SUCCESS)
            break;

        renderTarget = GetUsedRenderTarget(mediaCtx);
    }
#ifdef FAKE_VPUAPI
#else
    VPU_DecUpdateBitstreamBuffer(mediaCtx->decHandle, STREAM_END_SIZE);
#endif

    for (int32_t index = 0; index < VPUAPI_MAX_FB_NUM; index++)
    {
        if (mediaCtx->frameBufMem[index].phys_addr != 0)
            vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->frameBufMem[index], DEC_FBC, 0);
    }

    for (int32_t index = 0; index < VPUAPI_MAX_BUF_NUM; index++)
    {
        if (mediaCtx->paramBuf[index].phys_addr != 0)
            vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->paramBuf[index], DEC_ETC, 0);
        if (mediaCtx->bsBuf[index].phys_addr != 0)
            vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->bsBuf[index], DEC_BS, 0);
    }
#ifdef FAKE_VPUAPI
#else
    if (VPU_DecClose(mediaCtx->decHandle) == RETCODE_VPU_STILL_RUNNING)
    {
        QueueStatusInfo qStatus;
        VPU_DecGiveCommand(mediaCtx->decHandle, DEC_GET_QUEUE_STATUS, &qStatus);
        printf("[CNM_VPUAPI] VPU_DecClose fail RETCODE_VPU_STILL_RUNNING instanceQueueCount=%d, reportQueueCount=%d\n", qStatus.instanceQueueCount, qStatus.reportQueueCount);
    }
#endif
    mediaCtx->seqInited = FALSE;
    mediaCtx->numOfRenderTargets = 0;

#ifdef CNM_FPGA_PLATFORM
#ifdef CNM_VPUAPI_INTERFACE_DEBUG
    if (mediaCtx->fpYuvDebug)
    {
        fclose(mediaCtx->fpYuvDebug);
    }
#endif
#endif

    if(mediaCtx->vbWork.size) {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbWork, DEC_WORK, 0);
        mediaCtx->vbWork.size = 0;
        mediaCtx->vbWork.phys_addr = 0UL;
    }

    if(mediaCtx->vbTemp.size) {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbTemp, DEC_TEMP, 0);
        mediaCtx->vbTemp.size = 0;
        mediaCtx->vbTemp.phys_addr = 0UL;        
    }

    DdiMediaUtil_DestroyMutex(&mediaCtx->vpuapiMutex);

    DdiMediaUtil_LockMutex(&mediaCtx->DecoderMutex);
    DdiMediaUtil_ReleasePVAContextFromHeap(mediaCtx->pDecoderCtxHeap, decIndex);
    mediaCtx->uiNumDecoders--;
    DdiMediaUtil_UnLockMutex(&mediaCtx->DecoderMutex);
    MOS_FreeMemory(decCtx);

    printf("[CNM_VPUAPI] Success Close decoder instance\n");
}

static void AllocateEncInternalBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    int32_t index)
{
    vpu_buffer_t *pVb = NULL;
    uint32_t fbWidth = mediaCtx->linearStride;
    uint32_t fbHeight = mediaCtx->linearHeight;
    uint32_t fbSize = 0;

    pVb = &mediaCtx->fbcYOffsetBufMem[index];
    if (pVb->phys_addr == 0)
    {
        fbSize = WAVE6_FBC_LUMA_TABLE_SIZE(fbWidth, fbHeight);
        pVb->size = ((fbSize + 4095) & ~4095) + 4096;

        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_FBCY_TBL, 0) < 0)
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory fbcYOffsetBuf\n");
    }

    pVb = &mediaCtx->fbcCOffsetBufMem[index];
    if (pVb->phys_addr == 0)
    {
        fbSize = WAVE6_FBC_CHROMA_TABLE_SIZE(fbWidth, fbHeight);
        pVb->size = ((fbSize + 4095) & ~4095) + 4096;

        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_FBCC_TBL, 0) < 0)
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory fbcCOffsetBuf\n");
    }

    pVb = &mediaCtx->mvColBufMem[index];
    if (pVb->phys_addr == 0)
    {
        if (mediaCtx->vaProfile == VAProfileHEVCMain || mediaCtx->vaProfile == VAProfileHEVCMain10)
            fbSize = WAVE6_ENC_HEVC_MVCOL_BUF_SIZE(fbWidth, fbHeight);
        else if (mediaCtx->vaProfile == VAProfileAV1Profile0 || mediaCtx->vaProfile == VAProfileAV1Profile1)
            fbSize = WAVE6_ENC_AV1_MVCOL_BUF_SIZE;
        else
            fbSize = WAVE6_ENC_AVC_MVCOL_BUF_SIZE(fbWidth, fbHeight);
        pVb->size = ((fbSize + 4095) & ~4095) + 4096;

        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_MV, 0) < 0)
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory mvColBuf\n");
    }

    pVb = &mediaCtx->subSampledBufMem[index];
    if (pVb->phys_addr == 0)
    {
        fbSize = WAVE6_ENC_SUBSAMPLED_SIZE(fbWidth, fbHeight);
        pVb->size = ((fbSize + 4095) & ~4095) + 4096;

        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_SUBSAMBUF, 0) < 0)
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory subSampledBuf\n");
    }

    if (mediaCtx->miscParamEnable & (1 << VAEncMiscParameterTypeROI))
    {
        pVb = &mediaCtx->roiBufMem[index];
        if (pVb->phys_addr == 0)
        {
            fbSize = (mediaCtx->encOP.bitstreamFormat == STD_AVC) ? MAX_MB_NUM : MAX_CTU_NUM * 4;
            pVb->size = ((fbSize + 4095) & ~4095) + 4096;

            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_ETC, 0) < 0)
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory roiBufMem\n");
        }
    }
}

static void FreeEncInternalBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    uint32_t index)
{
    if (mediaCtx->fbcYOffsetBufMem[index].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->fbcYOffsetBufMem[index], ENC_FBCY_TBL, 0);
    if (mediaCtx->fbcCOffsetBufMem[index].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->fbcCOffsetBufMem[index], ENC_FBCC_TBL, 0);
    if (mediaCtx->mvColBufMem[index].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->mvColBufMem[index], ENC_MV, 0);
    if (mediaCtx->subSampledBufMem[index].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->subSampledBufMem[index], ENC_SUBSAMBUF, 0);
    if (mediaCtx->roiBufMem[index].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->roiBufMem[index], ENC_ETC, 0);
}

static int8_t ClipQpValue(int8_t qp)
{
    int8_t max_qp = 31;
    int8_t min_qp = -32;

    if (qp < min_qp)
        return min_qp;
    else if (qp > max_qp)
        return max_qp;
    else
        return qp;
}

static int8_t ConvertPriorityToQpValue(int8_t priority)
{
    int8_t max_qp = 12;
    int8_t min_qp = -12;
    int8_t qp = priority * 4;

    if (qp < min_qp)
        return min_qp;
    else if (qp > max_qp)
        return max_qp;
    else
        return qp;
}

static void UpdateEncROIBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    int32_t index)
{
    VAEncMiscParameterBufferROI *vaEncMiscParamROI;
    uint8_t *miscParamBufData;
    uint32_t ctuSize;
    uint32_t mapWidth, mapHeight, mapSize;
    int8_t roiIndex;
    uint8_t mapQp[MAX_MB_NUM];
    vpu_buffer_t pVb = mediaCtx->miscParamBuf[VAEncMiscParameterTypeROI];

    osal_memset(mapQp, 0x00, MAX_MB_NUM);

    miscParamBufData = (uint8_t *)osal_malloc(pVb.size);

    vdi_read_memory(mediaCtx->coreIdx, pVb.phys_addr, miscParamBufData, pVb.size, VDI_LITTLE_ENDIAN);

    vaEncMiscParamROI = (VAEncMiscParameterBufferROI *)miscParamBufData;

    if (mediaCtx->encOP.bitstreamFormat == STD_AVC)
    {
        ctuSize = 16;
        mapWidth = VPU_ALIGN32(VPU_ALIGN16(mediaCtx->encOP.picWidth) / ctuSize);
        mapHeight = VPU_ALIGN16(mediaCtx->encOP.picHeight) / ctuSize;
        mapSize = mapWidth * mapHeight;
    }
    else
    {
        ctuSize = 64;
        mapWidth = VPU_ALIGN8(VPU_ALIGN64(mediaCtx->encOP.picWidth) / ctuSize) * 4;
        mapHeight = VPU_ALIGN64(mediaCtx->encOP.picHeight) / ctuSize;
        mapSize = mapWidth * mapHeight;
    }

    for (roiIndex = vaEncMiscParamROI->num_roi - 1; roiIndex >= 0; roiIndex--)
    {
        VAEncROI roi = vaEncMiscParamROI->roi[roiIndex];
        uint32_t roiX, roiY;
        uint32_t roiWidth, roiHeight;

        if (mediaCtx->encOP.bitstreamFormat == STD_AVC)
        {
            roiX = roi.roi_rectangle.x / ctuSize;
            roiY = roi.roi_rectangle.y / ctuSize;
            roiWidth = (roi.roi_rectangle.width + ctuSize - 1) / ctuSize;
            roiHeight = (roi.roi_rectangle.height + ctuSize - 1) / ctuSize;
        }
        else
        {
            roiX = (roi.roi_rectangle.x / ctuSize) * 4;
            roiY = roi.roi_rectangle.y / ctuSize;
            roiWidth = ((roi.roi_rectangle.width + ctuSize - 1) / ctuSize) * 4;
            roiHeight = (roi.roi_rectangle.height + ctuSize - 1) / ctuSize;
        }

        for (uint8_t y = roiY; y <= roiHeight + roiY; y++)
        {
            for (uint8_t x = roiX; x <= roiWidth + roiX; x++)
            {
                if (vaEncMiscParamROI->roi_flags.bits.roi_value_is_qp_delta)
                {
                    mapQp[y * mapWidth + x] = ClipQpValue(roi.roi_value);
                }
                else
                { // roi_value is priority.
                    mapQp[y * mapWidth + x] = ConvertPriorityToQpValue(roi.roi_value);
                }
            }
        }
    }

    vdi_write_memory(mediaCtx->coreIdx, mediaCtx->roiBufMem[index].phys_addr, mapQp, mapSize, VDI_128BIT_BIG_ENDIAN);

    osal_free(miscParamBufData);
}

static VAStatus AllocateEncFrameBuffer(
    PDDI_MEDIA_CONTEXT mediaCtx,
    EncInitialInfo seqInfo)
{
    EncHandle hdl = mediaCtx->encHandle;
    EncOpenParam encOP = mediaCtx->encOP;
    FrameBuffer frameBuf[VPUAPI_MAX_FB_NUM];
    FrameBufferFormat format = FORMAT_420;
    int32_t fbHeight = 0;
    int32_t fbStride = 0;
    int32_t fbCount = 0;
    RetCode retCode;

    osal_memset(frameBuf, 0x00, sizeof(FrameBuffer) * VPUAPI_MAX_FB_NUM);

    format = (mediaCtx->encOP.EncStdParam.wave6Param.internalBitDepth > 8) ? FORMAT_420_P10_16BIT_MSB : FORMAT_420;
    if (encOP.bitstreamFormat == STD_AVC)
    {
        fbHeight = VPU_ALIGN16(encOP.picHeight);
        fbStride = CalcStride(VPU_ALIGN16(encOP.picWidth), format);
    }
    else
    {
        fbHeight = VPU_ALIGN8(encOP.picHeight);
        fbStride = CalcStride(VPU_ALIGN8(encOP.picWidth), format);
    }
    fbCount = seqInfo.minFrameBufferCount;

    frameBuf[0].bufY = -1;
    frameBuf[0].bufCb = -1;
    frameBuf[0].bufCr = -1;
    frameBuf[0].mapType = COMPRESSED_FRAME_MAP;
    frameBuf[0].cbcrInterleave = mediaCtx->cbcrInterleave;
    frameBuf[0].nv21 = mediaCtx->nv21;
    frameBuf[0].format = format;
    frameBuf[0].stride = fbStride;
    frameBuf[0].height = fbHeight;
    frameBuf[0].endian = encOP.frameEndian;
    frameBuf[0].lumaBitDepth = mediaCtx->encOP.EncStdParam.wave6Param.internalBitDepth;
    frameBuf[0].chromaBitDepth = mediaCtx->encOP.EncStdParam.wave6Param.internalBitDepth;
#ifdef FAKE_VPUAPI
#else
    if ((retCode = VPU_EncRegisterFrameBuffer(hdl, frameBuf, fbCount, fbStride, fbHeight, COMPRESSED_FRAME_MAP)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] Failed to register frame buffer retCode=0x%x\n", retCode);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
#endif
    printf("[CNM_VPUAPI] Success VPU_EncRegisterFrameBufferEx fbc : fbCount=%d fbStride=%d\n", fbCount, fbStride);

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiEncOpen(
    VADriverContextP ctx,
    VAConfigID configId,
    int32_t pictureWidth,
    int32_t pictureHeight,
    VAContextID *context)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT contextHeapElement;
    VAStatus va = VA_STATUS_SUCCESS;
    PDDI_ENCODE_CONTEXT encCtx = (PDDI_ENCODE_CONTEXT)MOS_AllocAndZeroMemory(sizeof(DDI_ENCODE_CONTEXT));
    CodStd bitFormat = STD_HEVC;
    VAProfile profile;
    VAEntrypoint entrypoint;
    bool findValidConfigId = false;
    bool enRateControl = false;
    uint32_t rcMode = 0;
    uint32_t i = 0;
    uint32_t bitDepth = 8;

    for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
    {
        if (s_vpuApiAttrs[i].actual_profile == VAProfileNone)
        {
            continue;
        }
        if (s_vpuApiAttrs[i].configId == configId)
        {
            findValidConfigId = true;
            break;
        }
    }
    if (findValidConfigId == false)
    {
        return VA_STATUS_ERROR_INVALID_CONFIG;
    }
    profile = s_vpuApiAttrs[i].actual_profile;
    entrypoint = s_vpuApiAttrs[i].actual_entrypoint;

    if (pictureWidth > VPUAPI_MAX_PIC_WIDTH || pictureHeight > VPUAPI_MAX_PIC_HEIGHT)
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;
    if (pictureWidth < VPUAPI_MIN_ENC_PIC_WIDTH || pictureHeight < VPUAPI_MIN_ENC_PIC_HEIGHT)
        return VA_STATUS_ERROR_RESOLUTION_NOT_SUPPORTED;

    switch (profile)
    {
#ifdef VA_PROFILE_H264_HIGH_10
    case VAProfileH264High10:
        bitDepth = 10;
#endif
    case VAProfileH264High:
    case VAProfileH264Main:
    case VAProfileH264ConstrainedBaseline:
        bitFormat = STD_AVC;
        break;
    case VAProfileHEVCMain10:
        bitDepth = 10;
    case VAProfileHEVCMain:
        bitFormat = STD_HEVC;
        break;
    case VAProfileAV1Profile0:
    case VAProfileAV1Profile1:
        bitFormat = STD_AV1;
        break;
    default:
        printf("[CNM_VPUAPI] not supported profile=0x%x\n", profile);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    mediaCtx->rcMode = (mediaCtx->rcMode == 0) ? VA_RC_CQP : mediaCtx->rcMode;
    switch (mediaCtx->rcMode)
    {
    case VA_RC_CBR:
        enRateControl = true;
        rcMode = 1;
        break;
    case VA_RC_VBR:
        enRateControl = true;
        rcMode = 0;
        break;
    case VA_RC_CQP:
        enRateControl = false;
        rcMode = 0;
        break;
    default:
        printf("[CNM_VPUAPI] not supported rcMode=0x%x\n", mediaCtx->rcMode);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    VpuApiAllocateworkBuffer(mediaCtx->coreIdx, &mediaCtx->vbWork);
    mediaCtx->encOP.instBuffer.workBufBase = mediaCtx->vbWork.phys_addr;
    mediaCtx->encOP.instBuffer.workBufSize = mediaCtx->vbWork.size;

    VpuApiAllocateTempBuffer(mediaCtx->coreIdx, &mediaCtx->vbTemp);
    mediaCtx->encOP.instBuffer.tempBufBase = mediaCtx->vbTemp.phys_addr;
    mediaCtx->encOP.instBuffer.tempBufSize = mediaCtx->vbTemp.size;

    VpuApiAllocateSecAxiBuffer(mediaCtx->coreIdx, &mediaCtx->vbSecAxi);    

    VpuApiAllocateArTableBuffer(mediaCtx->coreIdx,&mediaCtx->vbAr);
    mediaCtx->encOP.instBuffer.arTblBufBase = mediaCtx->vbAr.phys_addr;

    mediaCtx->encOP.numUseVcore = 1;
    mediaCtx->encOP.vcoreCoreIdc = 1;

    mediaCtx->encOP.bitstreamFormat = bitFormat;
    mediaCtx->encOP.srcFormat = mediaCtx->wtlFormat;
    mediaCtx->encOP.EncStdParam.wave6Param.internalBitDepth = bitDepth;
    mediaCtx->encOP.EncStdParam.wave6Param.enRateControl = enRateControl;
    mediaCtx->encOP.EncStdParam.wave6Param.rcMode = rcMode;
    mediaCtx->encOP.coreIdx = mediaCtx->coreIdx;
    mediaCtx->encOP.picWidth = pictureWidth;
    mediaCtx->encOP.picHeight = pictureHeight;
    mediaCtx->encOP.streamEndian = VDI_LITTLE_ENDIAN;
    mediaCtx->encOP.frameEndian = VDI_LITTLE_ENDIAN;
    /* In case of 10bit, Intput source data format is p010le. */
    /* To read p010le format correctly, change the source endian value to BYTE_SWAP. */
    mediaCtx->encOP.sourceEndian = (mediaCtx->wtlFormat == FORMAT_420) ? VDI_LITTLE_ENDIAN : VDI_128BIT_LE_BYTE_SWAP;
    mediaCtx->encOP.cbcrOrder = mediaCtx->cbcrOrder;
    mediaCtx->encOP.cbcrInterleave = mediaCtx->cbcrInterleave;
    mediaCtx->encOP.nv21 = mediaCtx->nv21;
    mediaCtx->encOP.vaEnable = TRUE;

#ifdef FAKE_VPUAPI
    mediaCtx->encHandle = s_EncHandle;
#else
    if (VPU_EncOpen(&mediaCtx->encHandle, &mediaCtx->encOP) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] Failed to Open encoder instance\n");
        va = VA_STATUS_ERROR_OPERATION_FAILED;
        return va;
    }
#endif
    DdiMediaUtil_LockMutex(&mediaCtx->EncoderMutex);
    contextHeapElement = DdiMediaUtil_AllocPVAContextFromHeap(mediaCtx->pEncoderCtxHeap);
    if (nullptr == contextHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->EncoderMutex);
        va = VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        return va;
    }

    contextHeapElement->pVaContext = (void *)encCtx;
    mediaCtx->uiNumEncoders++;
    *context = (VAContextID)(contextHeapElement->uiVaContextID + DDI_MEDIA_VACONTEXTID_OFFSET_ENCODER);
    DdiMediaUtil_UnLockMutex(&mediaCtx->EncoderMutex);

    mediaCtx->pCodedBufferSegment = (VACodedBufferSegment *)MOS_AllocAndZeroMemory(sizeof(VACodedBufferSegment));
    if (mediaCtx->pCodedBufferSegment == nullptr)
    {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    mediaCtx->pCodedBufferSegment->next = nullptr;

    printf("[CNM_VPUAPI] Success Open encoder instance: format %d\n", mediaCtx->encOP.bitstreamFormat);

    mediaCtx->vaProfile = profile;
    if (mediaCtx->bsBuf[0].phys_addr == 0)
    {
        mediaCtx->bsBuf[0].size = 0x100000;
        if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->bsBuf[0], ENC_BS, 0) < 0)
        {
            printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory bsBuf[0]\n");
            va = VA_STATUS_ERROR_ALLOCATION_FAILED;
            return va;
        }
    }
    return va;
}

static void VpuApiEncClose(
    VADriverContextP ctx,
    VAContextID context)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    uint32_t ctxType;
    PDDI_ENCODE_CONTEXT encCtx = (PDDI_ENCODE_CONTEXT)DdiMedia_GetContextFromContextID(ctx, context, &ctxType);
    uint32_t encIndex = (uint32_t)context & DDI_MEDIA_MASK_VACONTEXTID;

    for (int32_t index = 0; index < VPUAPI_MAX_FB_NUM; index++)
    {
        FreeEncInternalBuffer(mediaCtx, index);
    }

    if (mediaCtx->bsBuf[0].phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->bsBuf[0], ENC_BS, 0);
    if (mediaCtx->seqParamBuf.phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->seqParamBuf, ENC_ETC, 0);
    if (mediaCtx->picParamBuf.phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->picParamBuf, ENC_ETC, 0);
    if (mediaCtx->sliceParamBuf.phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->sliceParamBuf, ENC_ETC, 0);
    if (mediaCtx->packedParamBuf.phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->packedParamBuf, ENC_ETC, 0);
    if (mediaCtx->packedDataBuf.phys_addr != 0)
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->packedDataBuf, ENC_ETC, 0);
    for (int32_t index = 0; index < VPUAPI_MAX_MISC_TYPE_NUM; index++)
    {
        if (mediaCtx->miscParamBuf[index].phys_addr != 0)
            vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->miscParamBuf[index], ENC_ETC, 0);
    }
    
#ifdef FAKE_VPUAPI
#else
    VPU_EncClose(mediaCtx->encHandle);

    if (mediaCtx->vbWork.size) {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbWork, ENC_WORK, 0);
        mediaCtx->vbWork.size = 0;
        mediaCtx->vbWork.phys_addr = 0UL;
    }
    if (mediaCtx->vbTemp.size) {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbTemp, ENC_TEMP, 0);
        mediaCtx->vbTemp.size = 0;
        mediaCtx->vbTemp.phys_addr = 0UL;
    }
    if (mediaCtx->vbAr.size) {
        vdi_free_dma_memory(mediaCtx->coreIdx, &mediaCtx->vbAr, ENC_AR, 0);
        mediaCtx->vbAr.size = 0;
        mediaCtx->vbAr.phys_addr = 0UL;
    }

#endif
    mediaCtx->seqInited = FALSE;
    mediaCtx->numOfRenderTargets = 0;

    MOS_FreeMemory(mediaCtx->pCodedBufferSegment);
    mediaCtx->pCodedBufferSegment = nullptr;

    DdiMediaUtil_LockMutex(&mediaCtx->EncoderMutex);
    DdiMediaUtil_ReleasePVAContextFromHeap(mediaCtx->pDecoderCtxHeap, encIndex);
    mediaCtx->uiNumEncoders--;
    DdiMediaUtil_UnLockMutex(&mediaCtx->EncoderMutex);

    MOS_FreeMemory(encCtx);

    printf("[CNM_VPUAPI] Success Close encoder instance\n");
}

static VAStatus VpuApiEncCreateBuffer(
    VADriverContextP ctx,
    void *ctxPtr,
    VABufferType type,
    uint32_t size,
    uint32_t numElements,
    void *data,
    VABufferID *bufId)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    VAStatus va = VA_STATUS_SUCCESS;
    MOS_STATUS mos = MOS_STATUS_SUCCESS;
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufferHeapElement;
    DDI_MEDIA_BUFFER *buf;
    uint32_t offset;

    buf = (DDI_MEDIA_BUFFER *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_BUFFER));
    if (nullptr == buf)
    {
        va = VA_STATUS_ERROR_ALLOCATION_FAILED;
        return va;
    }

    buf->uiNumElements = numElements;
    buf->iSize = size * numElements;
    buf->uiType = type;
    buf->format = Media_Format_CPU;
    buf->uiOffset = 0;
    buf->pMediaCtx = mediaCtx;

    va = DdiMediaUtil_CreateBuffer(buf, mediaCtx->pDrmBufMgr);
    if (va != VA_STATUS_SUCCESS)
    {
        MOS_FreeMemory(buf);
        return va;
    }

    bufferHeapElement = DdiMediaUtil_AllocPMediaBufferFromHeap(mediaCtx->pBufferHeap);
    if (nullptr == bufferHeapElement)
    {
        DdiMediaUtil_FreeBuffer(buf);
        MOS_FreeMemory(buf);
        va = VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
        return va;
    }

    bufferHeapElement->pBuffer = buf;
    bufferHeapElement->pCtx = ctxPtr;
    bufferHeapElement->uiCtxType = DDI_MEDIA_CONTEXT_TYPE_ENCODER;

    *bufId = bufferHeapElement->uiVaBufferID;
    mediaCtx->uiNumBufs++;

    if (type != VAEncCodedBufferType)
    {
        if (nullptr == data)
            return va;

        mos = MOS_SecureMemcpy((void *)(buf->pData + buf->uiOffset), size * numElements, data, size * numElements);
        if (mos != MOS_STATUS_SUCCESS)
        {
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }
    }

    printf("[CNM_VPUAPI] VA Buffer Type : %d, uiNumBufs : %d \n",(int32_t)type, mediaCtx->uiNumBufs);

    switch ((int32_t)type)
    {
    case VAEncCodedBufferType:
        mediaCtx->encodedBufferId = *bufId;
        break;
    case VAEncMiscParameterBufferType:
    {
        VAEncMiscParameterBuffer *miscParamBuf = (VAEncMiscParameterBuffer *)data;
        vpu_buffer_t *pVb = &mediaCtx->miscParamBuf[(int32_t)miscParamBuf->type];

        if (pVb->phys_addr == 0)
        {
            pVb->size = VPU_ALIGN16(size);
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, pVb, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory VAEncMiscParameterBuffer\n");
                va = VA_STATUS_ERROR_OPERATION_FAILED;
                return va;
            }
        }
        vdi_write_memory(mediaCtx->coreIdx, pVb->phys_addr, (Uint8 *)miscParamBuf->data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->miscParamEnable |= (1 << (int32_t)miscParamBuf->type);
        printf("[CNM_VPUAPI] VAEncMiscParameterBuffer type : %d, miscParamEnable: %08X \n",(int32_t)miscParamBuf->type, mediaCtx->miscParamEnable);
        break;
    }
    case VAEncSequenceParameterBufferType:
        if (mediaCtx->seqParamBuf.phys_addr == 0)
        {
            mediaCtx->seqParamBuf.size = 0x1000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->seqParamBuf, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory seqParamBuf\n");
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }

        offset = mediaCtx->seqParamNum * VPU_ALIGN16(size);
        if (mediaCtx->seqParamBuf.size < offset + VPU_ALIGN16(size))
        {
            printf("[CNM_VPUAPI] Internal seqParamBuf overflow. (internal:0x%x < input:0x%x)\n", mediaCtx->seqParamBuf.size, offset + VPU_ALIGN16(size));
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->seqParamBuf.phys_addr + offset, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->seqParamNum++;
        break;
    case VAEncPictureParameterBufferType:
        if (mediaCtx->picParamBuf.phys_addr == 0)
        {
            mediaCtx->picParamBuf.size = 0x1000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->picParamBuf, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory picParamBuf\n");
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }

        offset = mediaCtx->picParamNum * VPU_ALIGN16(size);
        if (mediaCtx->picParamBuf.size < offset + VPU_ALIGN16(size))
        {
            printf("[CNM_VPUAPI] Internal picParamBuf overflow. (internal:0x%x < input:0x%x)\n", mediaCtx->picParamBuf.size, offset + VPU_ALIGN16(size));
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->picParamBuf.phys_addr + offset, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->picParamNum++;

        if (mediaCtx->encOP.bitstreamFormat == STD_AVC)
        {
            VAEncPictureParameterBufferH264 *va_pps = (VAEncPictureParameterBufferH264 *)data;
            mediaCtx->reconTarget = va_pps->CurrPic.picture_id;
        }
        else if (mediaCtx->encOP.bitstreamFormat == STD_HEVC)
        {
            VAEncPictureParameterBufferHEVC *va_pps = (VAEncPictureParameterBufferHEVC *)data;
            mediaCtx->reconTarget = va_pps->decoded_curr_pic.picture_id;
        }
        else if (mediaCtx->encOP.bitstreamFormat == STD_AV1)
        {
            VAEncPictureParameterBufferAV1 *va_pps = (VAEncPictureParameterBufferAV1 *)data;
            mediaCtx->reconTarget = va_pps->reconstructed_frame;
        }
        break;
    case VAEncSliceParameterBufferType:
        if (mediaCtx->sliceParamBuf.phys_addr == 0)
        {
            mediaCtx->sliceParamBuf.size = 0x70000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->sliceParamBuf, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory sliceParamBuf\n");
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }

        offset = mediaCtx->sliceParamNum * VPU_ALIGN16(size);
        if (mediaCtx->sliceParamBuf.size < offset + VPU_ALIGN16(size))
        {
            printf("[CNM_VPUAPI] Internal sliceParamBuf overflow. (internal:0x%x < input:0x%x)\n", mediaCtx->sliceParamBuf.size, offset + VPU_ALIGN16(size));
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->sliceParamBuf.phys_addr + offset, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->sliceParamNum++;
        break;
    case VAEncPackedHeaderParameterBufferType:
        if (mediaCtx->packedParamBuf.phys_addr == 0)
        {
            mediaCtx->packedParamBuf.size = 0x2000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->packedParamBuf, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory packedParamBuf\n");
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }

        offset = mediaCtx->packedParamSize;
        if (mediaCtx->packedParamBuf.size < offset + VPU_ALIGN16(size))
        {
            printf("[CNM_VPUAPI] Internal packedParamBuf overflow. (internal:0x%x < input:0x%x)\n", mediaCtx->packedParamBuf.size, offset + VPU_ALIGN16(size));
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->packedParamBuf.phys_addr + offset, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->packedParamSize += VPU_ALIGN16(size);

        {
            VAEncPackedHeaderParameterBuffer *encPackedHeaderParamBuf = (VAEncPackedHeaderParameterBuffer *)data;
            printf("[CNM_VPUAPI] VAEncPackedHeaderParameterBuffer type : %d \n",encPackedHeaderParamBuf->type);
            switch (encPackedHeaderParamBuf->type)
            {
            case VAEncPackedHeaderSequence:
                mediaCtx->packedSeqParamNum++;
                break;
            case VAEncPackedHeaderPicture:
                mediaCtx->packedPicParamNum++;
                break;
            case VAEncPackedHeaderSlice:
                mediaCtx->packedSliceParamNum++;
                break;
            case VAEncPackedHeaderRawData:
                mediaCtx->packedSeiParamNum++;
                break;
            }
        }
        break;
    case VAEncPackedHeaderDataBufferType:
        if (mediaCtx->packedDataBuf.phys_addr == 0)
        {
            mediaCtx->packedDataBuf.size = 0x10000;
            if (vdi_allocate_dma_memory(mediaCtx->coreIdx, &mediaCtx->packedDataBuf, ENC_ETC, 0) < 0)
            {
                printf("[CNM_VPUAPI] FAIL vdi_allocate_dma_memory packedDataBuf\n");
                va = VA_STATUS_ERROR_ALLOCATION_FAILED;
                return va;
            }
        }

        offset = mediaCtx->packedDataSize;
        if (mediaCtx->packedDataBuf.size < offset + VPU_ALIGN256(size))
        {
            printf("[CNM_VPUAPI] Internal packedDataBuf overflow. (internal:0x%x < input:0x%x)\n", mediaCtx->packedDataBuf.size, offset + VPU_ALIGN256(size));
            va = VA_STATUS_ERROR_OPERATION_FAILED;
            return va;
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->packedDataBuf.phys_addr + offset, (Uint8 *)data, size, VDI_LITTLE_ENDIAN);
        mediaCtx->packedDataSize += VPU_ALIGN256(size);
        break;
    default:
        break;
    }

    return va;
}

static VAStatus VpuApiEncSeqInit(
    VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode ret = RETCODE_SUCCESS;
    EncHandle hdl = mediaCtx->encHandle;
    EncInitialInfo seqInfo;
    VaapiInfo vaInfo;
    BOOL seqInited = FALSE;
    Int32 intrFlag = 0;

    osal_memset(&vaInfo, 0x00, sizeof(VaapiInfo));

    if (mediaCtx->seqInited == TRUE)
        return VA_STATUS_SUCCESS;

#ifdef FAKE_VPUAPI
    mediaCtx->seqInited = TRUE;
#else
    vaInfo.seqParamNum = mediaCtx->seqParamNum;
    vaInfo.picParamNum = mediaCtx->picParamNum;
    vaInfo.miscEnable = mediaCtx->miscParamEnable;
    vaInfo.seqParamBufAddr = mediaCtx->seqParamBuf.phys_addr;
    vaInfo.picParamBufAddr = mediaCtx->picParamBuf.phys_addr;
    vaInfo.miscFrameRateBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeFrameRate].phys_addr;
    vaInfo.miscRateControlBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeRateControl].phys_addr;
    vaInfo.miscHrdBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeHRD].phys_addr;
    VPU_EncGiveCommand(hdl, ENC_SET_VAAPI_INFO, &vaInfo);

    if ((ret = VPU_EncIssueSeqInit(hdl)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_EncIssueSeqInit: 0x%x\n", ret);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
    intrFlag = VPU_WaitInterruptEx(hdl, VPU_ENC_TIMEOUT);
    if (intrFlag == -1)
    {
        printf("[CNM_VPUAPI] FAIL VPU_WaitInterruptEx for VPU_EncIssueSeqInit: timeout=%dms\n", VPU_ENC_TIMEOUT);
        VPU_EncCompleteSeqInit(hdl, &seqInfo);
        return VA_STATUS_ERROR_TIMEDOUT;
    }

    VPU_ClearInterruptEx(hdl, intrFlag);
    if (intrFlag & (1 << INT_WAVE5_ENC_SET_PARAM))
    {
        seqInited = TRUE;
    }
    else
    {
        printf("[CNM_VPUAPI] VPU_EncIssueSeqInit is done. but intrFlag=0x%x is wrong\n", intrFlag);
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }

    if ((ret = VPU_EncCompleteSeqInit(hdl, &seqInfo)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_EncCompleteSeqInit: 0x%x\n", seqInfo.seqInitErrReason);
        mediaCtx->seqInited = seqInited;
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }
    if (seqInited == FALSE)
    {
        mediaCtx->seqInited = seqInited;
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }

    mediaCtx->seqInited = seqInited;

    printf("[CNM_VPUAPI] SUCCESS VpuApiEncSeqInit\n");
    printf("[CNM_VPUAPI] >>> width : %d | height : %d\n", mediaCtx->encOP.picWidth, mediaCtx->encOP.picHeight);
    printf("[CNM_VPUAPI] >>> bitdepth : %d\n", mediaCtx->encOP.EncStdParam.wave6Param.internalBitDepth);
    printf("[CNM_VPUAPI] >>> minFrameBufferCount : %d\n", seqInfo.minFrameBufferCount);
#endif

    if (AllocateEncFrameBuffer(mediaCtx, seqInfo) != VA_STATUS_SUCCESS)
        return VA_STATUS_ERROR_ALLOCATION_FAILED;

    return VA_STATUS_SUCCESS;
}

static VAStatus VpuApiEncPic(
    VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    RetCode ret = RETCODE_SUCCESS;
    FrameBuffer srcFb;
    EncHandle hdl = mediaCtx->encHandle;
    EncParam param;
    EncOutputInfo outputInfo;
    VaapiInfo vaInfo;
    int32_t renderTargetIdx;
    int32_t reconTargetIdx;
    Int32 intrFlag = 0;

    osal_memset(&param, 0x00, sizeof(EncParam));
    osal_memset(&outputInfo, 0x00, sizeof(EncOutputInfo));
    osal_memset(&srcFb, 0x00, sizeof(FrameBuffer));
    osal_memset(&vaInfo, 0x00, sizeof(VaapiInfo));

    renderTargetIdx = GetRenderTargetIndex(mediaCtx, mediaCtx->renderTarget);
    reconTargetIdx = GetRenderTargetIndex(mediaCtx, mediaCtx->reconTarget);

    AllocateEncInternalBuffer(mediaCtx, reconTargetIdx);

    vaInfo.seqParamNum = mediaCtx->seqParamNum;
    vaInfo.picParamNum = mediaCtx->picParamNum;
    vaInfo.sliceParamNum = mediaCtx->sliceParamNum;
    vaInfo.packedSeqParamNum = mediaCtx->packedSeqParamNum;
    vaInfo.packedPicParamNum = mediaCtx->packedPicParamNum;
    vaInfo.packedSliceParamNum = mediaCtx->packedSliceParamNum;
    vaInfo.packedSeiParamNum = mediaCtx->packedSeiParamNum;
    vaInfo.miscEnable = mediaCtx->miscParamEnable;
    vaInfo.seqParamBufAddr = mediaCtx->seqParamBuf.phys_addr;
    vaInfo.picParamBufAddr = mediaCtx->picParamBuf.phys_addr;
    vaInfo.sliceParamBufAddr = mediaCtx->sliceParamBuf.phys_addr;
    vaInfo.packedParamBufAddr = mediaCtx->packedParamBuf.phys_addr;
    vaInfo.packedParamDataBufAddr = mediaCtx->packedDataBuf.phys_addr;
    vaInfo.miscFrameRateBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeFrameRate].phys_addr;
    vaInfo.miscRateControlBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeRateControl].phys_addr;
    vaInfo.miscHrdBufAddr = mediaCtx->miscParamBuf[VAEncMiscParameterTypeHRD].phys_addr;
    vaInfo.fbcYOffsetBufAddr = mediaCtx->fbcYOffsetBufMem[reconTargetIdx].phys_addr;
    vaInfo.fbcCOffsetBufAddr = mediaCtx->fbcCOffsetBufMem[reconTargetIdx].phys_addr;
    vaInfo.mvColBufAddr = mediaCtx->mvColBufMem[reconTargetIdx].phys_addr;
    vaInfo.subSampledBufAddr = mediaCtx->subSampledBufMem[reconTargetIdx].phys_addr;
#ifdef CNM_FPGA_PLATFORM
    vaInfo.fbcYBufAddr = mediaCtx->linearFrameBuf[reconTargetIdx].bufY;
    vaInfo.fbcCBufAddr = mediaCtx->linearFrameBuf[reconTargetIdx].bufCb;
#else
    printf("[CNM_VPUAPI] customer needs to get physical address from recon_target surface=0x%x", mediaCtx->reconTarget);
    printf("[CNM_VPUAPI] customer needs to set fbcYBufAddr and fbcCBufAddr to Physical address that VPU can access.\n");
#endif

    printf("[CNM_VPUAPI] seqParamNum: %d\n", vaInfo.seqParamNum);
    printf("[CNM_VPUAPI] picParamNum: %d\n", vaInfo.picParamNum);
    printf("[CNM_VPUAPI] sliceParamNum: %d\n", vaInfo.sliceParamNum);
    printf("[CNM_VPUAPI] packedSeqParamNum: %d\n", vaInfo.packedSeqParamNum);
    printf("[CNM_VPUAPI] packedPicParamNum: %d\n", vaInfo.packedPicParamNum);
    printf("[CNM_VPUAPI] packedSliceParamNum: %d\n", vaInfo.packedSliceParamNum);
    printf("[CNM_VPUAPI] packedSeiParamNum: %d\n", vaInfo.packedSeiParamNum);
    printf("[CNM_VPUAPI] miscEnable: %d\n", vaInfo.miscEnable);
    printf("[CNM_VPUAPI] seqParamBufAddr: 0x%lx\n", vaInfo.seqParamBufAddr);
    printf("[CNM_VPUAPI] picParamBufAddr: 0x%lx\n", vaInfo.picParamBufAddr);
    printf("[CNM_VPUAPI] sliceParamBufAddr: 0x%lx\n", vaInfo.sliceParamBufAddr);
    printf("[CNM_VPUAPI] packedParamBufAddr: 0x%lx\n", vaInfo.packedParamBufAddr);
    printf("[CNM_VPUAPI] packedParamDataBufAddr: 0x%lx\n", vaInfo.packedParamDataBufAddr);
    printf("[CNM_VPUAPI] miscFrameRateBufAddr: 0x%lx\n", vaInfo.miscFrameRateBufAddr);
    printf("[CNM_VPUAPI] miscRateControlBufAddr: 0x%lx\n", vaInfo.miscRateControlBufAddr);
    printf("[CNM_VPUAPI] miscHrdBufAddr: 0x%lx\n", vaInfo.miscHrdBufAddr);
    printf("[CNM_VPUAPI] reconTarget: %d\n", reconTargetIdx);
    printf("[CNM_VPUAPI] fbcYBufAddr: 0x%lx\n", vaInfo.fbcYBufAddr);
    printf("[CNM_VPUAPI] fbcCBufAddr: 0x%lx\n", vaInfo.fbcCBufAddr);
    printf("[CNM_VPUAPI] fbcYOffsetBufAddr: 0x%lx\n", vaInfo.fbcYOffsetBufAddr);
    printf("[CNM_VPUAPI] fbcCOffsetBufAddr: 0x%lx\n", vaInfo.fbcCOffsetBufAddr);
    printf("[CNM_VPUAPI] mvColBufAddr: 0x%lx\n", vaInfo.mvColBufAddr);
    printf("[CNM_VPUAPI] subSampledBufAddr: 0x%lx\n", vaInfo.subSampledBufAddr);
#ifdef FAKE_VPUAPI
    mediaCtx->encIdx++;
#else
    VPU_EncGiveCommand(hdl, ENC_SET_VAAPI_INFO, &vaInfo);

    srcFb.stride = mediaCtx->linearStride;
#ifdef CNM_FPGA_PLATFORM
    {
        BOOL cbcrInterleave = mediaCtx->cbcrInterleave;
        uint32_t lumaSize = 0, chromaSize = 0;
        uint32_t fourcc = 0;
        uint32_t lumaStride = 0;
        uint32_t chromaUStride = 0;
        uint32_t chromaVStride = 0;
        uint32_t lumaOffset = 0;
        uint32_t chromaUOffset = 0;
        uint32_t chromaVOffset = 0;
        uint32_t bufferName = 0;
        void *buffer = NULL;
        uint8_t *surfaceOffset;
        uint8_t *dataY;
        uint8_t *dataCb;
        uint8_t *dataCr;
        uint8_t multiple = (mediaCtx->wtlFormat == FORMAT_420_P10_16BIT_MSB) ? 2 : 1;

        lumaSize = mediaCtx->linearStride * mediaCtx->linearHeight;
        if (cbcrInterleave == FALSE)
            chromaSize = (mediaCtx->linearStride / 2) * (mediaCtx->linearHeight / 2);
        else
            chromaSize = (mediaCtx->linearStride) * (mediaCtx->linearHeight / 2);

        dataY = (uint8_t *)osal_malloc(lumaSize);
        dataCb = (uint8_t *)osal_malloc(chromaSize);
        dataCr = (uint8_t *)osal_malloc(chromaSize);

        memset(dataY, 0, lumaSize);
        memset(dataCb, 0, chromaSize);
        memset(dataCr, 0, chromaSize);

        DdiMedia_LockSurface(ctx, mediaCtx->renderTarget,
                             &fourcc,
                             &lumaStride, &chromaUStride, &chromaVStride,
                             &lumaOffset, &chromaUOffset, &chromaVOffset,
                             &bufferName, &buffer);
        printf("[CNM_VPUAPI] Surface Info: fourcc=0x%x, lumaStride=%d, chromaUStride=%d, lumaOffset=%d, chromalUoffset=%d, chromaVOffset=%d\n", fourcc, lumaStride, chromaUStride, lumaOffset, chromaUOffset, chromaVOffset);

        surfaceOffset = (unsigned char *)buffer + lumaOffset;
        for (int i = 0; i < mediaCtx->encSrcHeight; i++)
            memcpy(dataY + (lumaStride * i), surfaceOffset + (lumaStride * i), mediaCtx->encSrcWidth * multiple);

        if (cbcrInterleave == FALSE)
        {
            surfaceOffset = (unsigned char *)buffer + chromaUOffset;
            for (int i = 0; i < mediaCtx->encSrcHeight / 2; i++)
                memcpy(dataCb + (chromaUStride * i), surfaceOffset + (chromaUStride * i), (mediaCtx->encSrcWidth * multiple / 2));

            surfaceOffset = (unsigned char *)buffer + chromaVOffset;
            for (int i = 0; i < mediaCtx->encSrcHeight / 2; i++)
                memcpy(dataCr + (chromaVStride * i), surfaceOffset + (chromaVStride * i), (mediaCtx->encSrcWidth * multiple / 2));
        }
        else
        {
            surfaceOffset = (unsigned char *)buffer + chromaUOffset;
            for (int i = 0; i < mediaCtx->encSrcHeight / 2; i++)
                memcpy(dataCb + (chromaUStride * i), surfaceOffset + (chromaUStride * i), mediaCtx->encSrcWidth * multiple);
        }

        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->linearFrameBuf[renderTargetIdx].bufY, dataY, lumaSize, VDI_LITTLE_ENDIAN);
        vdi_write_memory(mediaCtx->coreIdx, mediaCtx->linearFrameBuf[renderTargetIdx].bufCb, dataCb, chromaSize, VDI_LITTLE_ENDIAN);
        if (cbcrInterleave == FALSE)
            vdi_write_memory(mediaCtx->coreIdx, mediaCtx->linearFrameBuf[renderTargetIdx].bufCr, dataCr, chromaSize, VDI_LITTLE_ENDIAN);

        DdiMedia_UnlockSurface(ctx, mediaCtx->renderTarget);

        osal_free(dataY);
        osal_free(dataCb);
        osal_free(dataCr);
    }

    srcFb.bufY = mediaCtx->linearFrameBuf[renderTargetIdx].bufY;
    srcFb.bufCb = mediaCtx->linearFrameBuf[renderTargetIdx].bufCb;
    srcFb.bufCr = mediaCtx->linearFrameBuf[renderTargetIdx].bufCr;
#else
    printf("[CNM_VPUAPI] customer needs to get physical address from render_target surface=0x%x", mediaCtx->renderTarget);
    printf("[CNM_VPUAPI] customer needs to set srcFbY and srcFbCb and srcFbCr to Physical address that VPU can access.\n");
#endif
    param.sourceFrame = &srcFb;
    param.picStreamBufferAddr = mediaCtx->bsBuf[0].phys_addr;
    param.picStreamBufferSize = mediaCtx->bsBuf[0].size;
    if(mediaCtx->encOP.bitstreamFormat == STD_AVC) {
        param.cuSizeMode = 7;
    }

    printf("[CNM_VPUAPI] bsBuf: 0x%lx\n", param.picStreamBufferAddr);
    printf("[CNM_VPUAPI] bsSize: %d\n", param.picStreamBufferSize);
    printf("[CNM_VPUAPI] renderTarget: %d\n", renderTargetIdx);
    printf("[CNM_VPUAPI] srcStride: %d\n", param.sourceFrame->stride);
    printf("[CNM_VPUAPI] srcBufY: 0x%lx\n", param.sourceFrame->bufY);
    printf("[CNM_VPUAPI] srcBufCb: 0x%lx\n", param.sourceFrame->bufCb);
    printf("[CNM_VPUAPI] srcBufCr: 0x%lx\n", param.sourceFrame->bufCr);

    if (mediaCtx->miscParamEnable & (1 << VAEncMiscParameterTypeROI))
    {
        UpdateEncROIBuffer(mediaCtx, reconTargetIdx);
        param.customMapOpt.customRoiMapEnable = TRUE;
        param.customMapOpt.customMapAddr = mediaCtx->roiBufMem[reconTargetIdx].phys_addr;
    }
    mediaCtx->encIdx++;
    if ((ret = VPU_EncStartOneFrame(hdl, &param)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_EncStartOneFrame: 0x%x\n", ret);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }

    intrFlag = VPU_WaitInterruptEx(hdl, VPU_ENC_TIMEOUT);
    if (intrFlag == -1)
    {
        printf("[CNM_VPUAPI] FAIL VPU_WaitInterruptEx for VPU_EncStartOneFrame : timeout=%dms\n", VPU_ENC_TIMEOUT);
        VPU_EncGetOutputInfo(hdl, &outputInfo);
        return VA_STATUS_ERROR_TIMEDOUT;
    }

    VPU_ClearInterruptEx(hdl, intrFlag);
    if (intrFlag & (1 << INT_WAVE6_ENC_PIC))
    {
    }
    else
    {
        printf("[CNM_VPUAPI] VPU_EncStartOneFrame is done. but intrFlag=0x%x is wrong\n", intrFlag);
        return VA_STATUS_ERROR_ENCODING_ERROR;
    }

    if ((ret = VPU_EncGetOutputInfo(hdl, &outputInfo)) != RETCODE_SUCCESS)
    {
        printf("[CNM_VPUAPI] FAIL VPU_EncGetOutputInfo: 0x%x\n", outputInfo.errorReason);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }

    printf("[CNM_VPUAPI] IDX %d | PIC %d | RDPTR : 0x%lx | WRPTR : 0x%lx | BYTES : 0x%x\n",
           mediaCtx->encIdx,
           outputInfo.picType, outputInfo.rdPtr, outputInfo.wrPtr, outputInfo.bitstreamSize);

    mediaCtx->encIdx++;

    if (outputInfo.bitstreamSize > 0)
    {
        uint8_t *encodedData;
        uint8_t *encodedBufPtr;
        DDI_MEDIA_BUFFER *encodedBuf = DdiMedia_GetBufferFromVABufferID(mediaCtx, mediaCtx->encodedBufferId);

        encodedData = (uint8_t *)osal_malloc(outputInfo.bitstreamSize);

        vdi_read_memory(mediaCtx->coreIdx, outputInfo.rdPtr, encodedData, outputInfo.bitstreamSize, VDI_LITTLE_ENDIAN);

        encodedBufPtr = (uint8_t *)DdiMediaUtil_LockBuffer(encodedBuf, MOS_LOCKFLAG_WRITEONLY);
        memcpy(encodedBufPtr, encodedData, outputInfo.bitstreamSize);
        encodedBuf->iSize = outputInfo.bitstreamSize;
        DdiMediaUtil_UnlockBuffer(encodedBuf);

        osal_free(encodedData);
    }
#endif
    printf("[CNM_VPUAPI] SUCCESS VpuApiEncPic\n");

    mediaCtx->seqParamNum = 0;
    mediaCtx->picParamNum = 0;
    mediaCtx->sliceParamNum = 0;
    mediaCtx->packedSeqParamNum = 0;
    mediaCtx->packedPicParamNum = 0;
    mediaCtx->packedSliceParamNum = 0;
    mediaCtx->packedSeiParamNum = 0;
    mediaCtx->packedParamSize = 0;
    mediaCtx->packedDataSize = 0;
    mediaCtx->miscParamEnable = 0;

    return VA_STATUS_SUCCESS;
}
#endif

static PDDI_MEDIA_CONTEXT DdiMedia_CreateMediaDriverContext()
{
    PDDI_MEDIA_CONTEXT mediaCtx;

    mediaCtx = (PDDI_MEDIA_CONTEXT)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_CONTEXT));

    return mediaCtx;
}

// refine this for decoder later
static bool DdiMedia_ReleaseSliceControlBuffer(
    uint32_t ctxType,
    void *ctx,
    DDI_MEDIA_BUFFER *buf)
{
    DDI_UNUSED(buf);

    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
    {
        PDDI_DECODE_CONTEXT decCtx;

        decCtx = DdiDecode_GetDecContextFromPVOID(ctx);
        DDI_CODEC_COM_BUFFER_MGR *bufMgr = &(decCtx->BufMgr);

        switch (decCtx->wMode)
        {
        case CODECHAL_DECODE_MODE_AVCVLD:
            if (decCtx->bShortFormatInUse)
            {
                if (bufMgr->Codec_Param.Codec_Param_H264.pVASliceParaBufH264Base == nullptr)
                {
                    return false;
                }
            }
            else
            {
                if (bufMgr->Codec_Param.Codec_Param_H264.pVASliceParaBufH264 == nullptr)
                {
                    return false;
                }
            }
            break;
        case CODECHAL_DECODE_MODE_MPEG2VLD:
            if (bufMgr->Codec_Param.Codec_Param_MPEG2.pVASliceParaBufMPEG2 == nullptr)
            {
                return false;
            }
            break;
        case CODECHAL_DECODE_MODE_VC1VLD:
            if (bufMgr->Codec_Param.Codec_Param_VC1.pVASliceParaBufVC1 == nullptr)
            {
                return false;
            }
            break;
        case CODECHAL_DECODE_MODE_JPEG:
            if (bufMgr->Codec_Param.Codec_Param_JPEG.pVASliceParaBufJPEG == nullptr)
            {
                return false;
            }
            break;
        case CODECHAL_DECODE_MODE_VP8VLD:
            if (bufMgr->Codec_Param.Codec_Param_VP8.pVASliceParaBufVP8 == nullptr)
            {
                return false;
            }
            break;
        case CODECHAL_DECODE_MODE_HEVCVLD:
            if (decCtx->bShortFormatInUse)
            {
                if (bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufBaseHEVC == nullptr)
                {
                    return false;
                }
            }
            else
            {
                if (bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufHEVC == nullptr &&
                    bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufHEVCRext == nullptr)
                {
                    return false;
                }
            }
            break;
        case CODECHAL_DECODE_MODE_VP9VLD:
            if (bufMgr->Codec_Param.Codec_Param_VP9.pVASliceParaBufVP9 == nullptr)
            {
                return false;
            }
            break;
        default:
            return false;
        }
        break;
    }
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        break;
    default:
        break;
    }

    return true;
}

static bool DdiMedia_ReleaseBpBuffer(
    DDI_CODEC_COM_BUFFER_MGR *bufMgr,
    DDI_MEDIA_BUFFER *buf)
{
    DDI_UNUSED(bufMgr);
    DDI_UNUSED(buf);
    return true;
}

static bool DdiMedia_ReleaseBsBuffer(
    DDI_CODEC_COM_BUFFER_MGR *bufMgr,
    DDI_MEDIA_BUFFER *buf)
{
    if ((bufMgr == nullptr) || (buf == nullptr))
    {
        return true;
    }

    if (buf->format == Media_Format_CPU)
    {
        for (uint32_t i = 0; i < bufMgr->dwNumSliceData; i++)
        {
            if (bufMgr->pSliceData[i].pBaseAddress == buf->pData)
            {
                DdiMediaUtil_FreeBuffer(buf);
                bufMgr->pSliceData[i].pBaseAddress = nullptr;
                if (bufMgr->pSliceData[i].pMappedGPUBuffer != nullptr)
                {
                    DdiMediaUtil_UnlockBuffer(bufMgr->pSliceData[i].pMappedGPUBuffer);
                    if (bufMgr->pSliceData[i].pMappedGPUBuffer->bMapped == false)
                    {
                        DdiMediaUtil_FreeBuffer(bufMgr->pSliceData[i].pMappedGPUBuffer);
                        MOS_FreeMemory(bufMgr->pSliceData[i].pMappedGPUBuffer);
                    }
                }
                MOS_ZeroMemory((void *)(&(bufMgr->pSliceData[i])), sizeof(bufMgr->pSliceData[0]));
                bufMgr->dwNumSliceData--;
                return true;
            }
        }
        return false;
    }
    else
    {
        if (bufMgr->dwNumSliceData)
            bufMgr->dwNumSliceData--;
    }
    return true;
}

static uint32_t DdiMedia_CreateRenderTarget(
    PDDI_MEDIA_CONTEXT mediaDrvCtx,
    DDI_MEDIA_FORMAT mediaFormat,
    uint32_t width,
    uint32_t height,
    DDI_MEDIA_SURFACE_DESCRIPTOR *surfDesc,
    uint32_t surfaceUsageHint,
    int memType)
{
    DdiMediaUtil_LockMutex(&mediaDrvCtx->SurfaceMutex);

    PDDI_MEDIA_SURFACE_HEAP_ELEMENT surfaceElement = DdiMediaUtil_AllocPMediaSurfaceFromHeap(mediaDrvCtx->pSurfaceHeap);
    if (nullptr == surfaceElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaDrvCtx->SurfaceMutex);
        return VA_INVALID_ID;
    }

    surfaceElement->pSurface = (DDI_MEDIA_SURFACE *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_SURFACE));
    if (nullptr == surfaceElement->pSurface)
    {
        DdiMediaUtil_ReleasePMediaSurfaceFromHeap(mediaDrvCtx->pSurfaceHeap, surfaceElement->uiVaSurfaceID);
        DdiMediaUtil_UnLockMutex(&mediaDrvCtx->SurfaceMutex);
        return VA_INVALID_ID;
    }

    surfaceElement->pSurface->pMediaCtx = mediaDrvCtx;
    surfaceElement->pSurface->iWidth = width;
    surfaceElement->pSurface->iHeight = height;
    surfaceElement->pSurface->pSurfDesc = surfDesc;
    surfaceElement->pSurface->format = mediaFormat;
    surfaceElement->pSurface->uiLockedBufID = VA_INVALID_ID;
    surfaceElement->pSurface->uiLockedImageID = VA_INVALID_ID;
    surfaceElement->pSurface->surfaceUsageHint = surfaceUsageHint;
    surfaceElement->pSurface->memType = memType;

    if (DdiMediaUtil_CreateSurface(surfaceElement->pSurface, mediaDrvCtx) != VA_STATUS_SUCCESS)
    {
        MOS_FreeMemory(surfaceElement->pSurface);
        DdiMediaUtil_ReleasePMediaSurfaceFromHeap(mediaDrvCtx->pSurfaceHeap, surfaceElement->uiVaSurfaceID);
        DdiMediaUtil_UnLockMutex(&mediaDrvCtx->SurfaceMutex);
        return VA_INVALID_ID;
    }

    mediaDrvCtx->uiNumSurfaces++;
    uint32_t surfaceID = surfaceElement->uiVaSurfaceID;
    DdiMediaUtil_UnLockMutex(&mediaDrvCtx->SurfaceMutex);
    return surfaceID;
}

VAStatus
DdiMedia_HybridQueryBufferAttributes(
    VADisplay dpy,
    VAContextID context,
    VABufferType bufferType,
    void *outputData,
    uint32_t *outputDataLen)
{
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Set Frame ID
//!
//! \param  [in] dpy
//!         VA display
//! \param  [in] surface
//!         VA surface ID
//! \param  [in] frame_id
//!         Frame ID
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_SetFrameID(
    VADisplay dpy,
    VASurfaceID surface,
    uint32_t frame_id)
{
    VADriverContextP ctx = (((VADisplayContextP)dpy)->pDriverContext);
    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface.", VA_STATUS_ERROR_INVALID_PARAMETER);

    mediaSurface->frame_idx = frame_id;

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Convert media format to OS format
//!
//! \param  [in] format
//!         Ddi media format
//!
//! \return Os format if call sucesss,else
//!     VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT if fail
//!
int32_t DdiMedia_MediaFormatToOsFormat(DDI_MEDIA_FORMAT format)
{
    switch (format)
    {
    case Media_Format_X8R8G8B8:
        return VA_FOURCC_XRGB;
    case Media_Format_X8B8G8R8:
        return VA_FOURCC_XBGR;
    case Media_Format_A8B8G8R8:
        return VA_FOURCC_ABGR;
    case Media_Format_R10G10B10A2:
        return VA_FOURCC_A2B10G10R10;
    case Media_Format_R8G8B8A8:
        return VA_FOURCC_RGBA;
    case Media_Format_A8R8G8B8:
        return VA_FOURCC_ARGB;
    case Media_Format_B10G10R10A2:
        return VA_FOURCC_A2R10G10B10;
    case Media_Format_R10G10B10X2:
        return VA_FOURCC_X2B10G10R10;
    case Media_Format_B10G10R10X2:
        return VA_FOURCC_X2R10G10B10;
    case Media_Format_R5G6B5:
        return VA_FOURCC_R5G6B5;
    case Media_Format_R8G8B8:
        return VA_FOURCC_R8G8B8;
    case Media_Format_NV12:
        return VA_FOURCC_NV12;
    case Media_Format_NV21:
        return VA_FOURCC_NV21;
    case Media_Format_YUY2:
        return VA_FOURCC_YUY2;
    case Media_Format_UYVY:
        return VA_FOURCC_UYVY;
    case Media_Format_YV12:
        return VA_FOURCC_YV12;
    case Media_Format_IYUV:
        return VA_FOURCC_IYUV;
    case Media_Format_I420:
        return VA_FOURCC_I420;
    case Media_Format_400P:
        return VA_FOURCC('4', '0', '0', 'P');
    case Media_Format_IMC3:
        return VA_FOURCC_IMC3;
    case Media_Format_422H:
        return VA_FOURCC_422H;
    case Media_Format_422V:
        return VA_FOURCC_422V;
    case Media_Format_411P:
        return VA_FOURCC_411P;
    case Media_Format_444P:
        return VA_FOURCC_444P;
    case Media_Format_RGBP:
        return VA_FOURCC_RGBP;
    case Media_Format_BGRP:
        return VA_FOURCC_BGRP;
    case Media_Format_Buffer:
        return VA_FOURCC_P208;
    case Media_Format_P010:
        return VA_FOURCC_P010;
    case Media_Format_P012:
        return VA_FOURCC_P012;
    case Media_Format_P016:
        return VA_FOURCC_P016;
    case Media_Format_Y210:
        return VA_FOURCC_Y210;
#if VA_CHECK_VERSION(1, 9, 0)
    case Media_Format_Y212:
        return VA_FOURCC_Y212;
#endif
    case Media_Format_Y216:
        return VA_FOURCC_Y216;
    case Media_Format_AYUV:
        return VA_FOURCC_AYUV;
    case Media_Format_Y410:
        return VA_FOURCC_Y410;
#if VA_CHECK_VERSION(1, 9, 0)
    case Media_Format_Y412:
        return VA_FOURCC_Y412;
#endif
    case Media_Format_Y416:
        return VA_FOURCC_Y416;
    case Media_Format_Y8:
        return VA_FOURCC_Y8;
    case Media_Format_Y16S:
        return VA_FOURCC_Y16;
    case Media_Format_Y16U:
        return VA_FOURCC_Y16;
    case Media_Format_VYUY:
        return VA_FOURCC_VYUY;
    case Media_Format_YVYU:
        return VA_FOURCC_YVYU;
    case Media_Format_A16R16G16B16:
        return VA_FOURCC_ARGB64;
    case Media_Format_A16B16G16R16:
        return VA_FOURCC_ABGR64;

    default:
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }
}

//!
//! \brief  Convert Os format to media format
//!
//! \param  [in] fourcc
//!         FourCC
//! \param  [in] rtformatType
//!         Rt format type
//!
//! \return DDI_MEDIA_FORMAT
//!     Ddi media format
//!
DDI_MEDIA_FORMAT DdiMedia_OsFormatToMediaFormat(int32_t fourcc, int32_t rtformatType)
{
    switch (fourcc)
    {
    case VA_FOURCC_A2R10G10B10:
        return Media_Format_B10G10R10A2;
    case VA_FOURCC_A2B10G10R10:
        return Media_Format_R10G10B10A2;
    case VA_FOURCC_X2R10G10B10:
        return Media_Format_B10G10R10X2;
    case VA_FOURCC_X2B10G10R10:
        return Media_Format_R10G10B10X2;
    case VA_FOURCC_BGRA:
    case VA_FOURCC_ARGB:
#ifdef VA_RT_FORMAT_RGB32_10BPP
        if (VA_RT_FORMAT_RGB32_10BPP == rtformatType)
        {
            return Media_Format_B10G10R10A2;
        }
#endif
        return Media_Format_A8R8G8B8;
    case VA_FOURCC_RGBA:
#ifdef VA_RT_FORMAT_RGB32_10BPP
        if (VA_RT_FORMAT_RGB32_10BPP == rtformatType)
        {
            return Media_Format_R10G10B10A2;
        }
#endif
        return Media_Format_R8G8B8A8;
    case VA_FOURCC_ABGR:
#ifdef VA_RT_FORMAT_RGB32_10BPP
        if (VA_RT_FORMAT_RGB32_10BPP == rtformatType)
        {
            return Media_Format_R10G10B10A2;
        }
#endif
        return Media_Format_A8B8G8R8;
    case VA_FOURCC_BGRX:
    case VA_FOURCC_XRGB:
        return Media_Format_X8R8G8B8;
    case VA_FOURCC_XBGR:
    case VA_FOURCC_RGBX:
        return Media_Format_X8B8G8R8;
    case VA_FOURCC_R5G6B5:
        return Media_Format_R5G6B5;
    case VA_FOURCC_R8G8B8:
        return Media_Format_R8G8B8;
    case VA_FOURCC_NV12:
        return Media_Format_NV12;
    case VA_FOURCC_NV21:
        return Media_Format_NV21;
    case VA_FOURCC_YUY2:
        return Media_Format_YUY2;
    case VA_FOURCC_UYVY:
        return Media_Format_UYVY;
    case VA_FOURCC_YV12:
        return Media_Format_YV12;
    case VA_FOURCC_IYUV:
        return Media_Format_IYUV;
    case VA_FOURCC_I420:
        return Media_Format_I420;
    case VA_FOURCC_422H:
        return Media_Format_422H;
    case VA_FOURCC_422V:
        return Media_Format_422V;
    case VA_FOURCC('4', '0', '0', 'P'):
    case VA_FOURCC_Y800:
        return Media_Format_400P;
    case VA_FOURCC_411P:
        return Media_Format_411P;
    case VA_FOURCC_IMC3:
        return Media_Format_IMC3;
    case VA_FOURCC_444P:
        return Media_Format_444P;
    case VA_FOURCC_BGRP:
        return Media_Format_BGRP;
    case VA_FOURCC_RGBP:
        return Media_Format_RGBP;
    case VA_FOURCC_P208:
        return Media_Format_Buffer;
    case VA_FOURCC_P010:
        return Media_Format_P010;
    case VA_FOURCC_P012:
        return Media_Format_P012;
    case VA_FOURCC_P016:
        return Media_Format_P016;
    case VA_FOURCC_Y210:
        return Media_Format_Y210;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y212:
        return Media_Format_Y212;
#endif
    case VA_FOURCC_Y216:
        return Media_Format_Y216;
    case VA_FOURCC_AYUV:
        return Media_Format_AYUV;
    case VA_FOURCC_Y410:
        return Media_Format_Y410;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y412:
        return Media_Format_Y412;
#endif
    case VA_FOURCC_Y416:
        return Media_Format_Y416;
    case VA_FOURCC_Y8:
        return Media_Format_Y8;
    case VA_FOURCC_Y16:
        return Media_Format_Y16S;
    case VA_FOURCC_VYUY:
        return Media_Format_VYUY;
    case VA_FOURCC_YVYU:
        return Media_Format_YVYU;
    case VA_FOURCC_ARGB64:
        return Media_Format_A16R16G16B16;
    case VA_FOURCC_ABGR64:
        return Media_Format_A16B16G16R16;

    default:
        return Media_Format_Count;
    }
}

static VAStatus DdiMedia_GetChromaPitchHeight(
    uint32_t fourcc,
    uint32_t pitch,
    uint32_t height,
    uint32_t *chromaPitch,
    uint32_t *chromaHeight)
{
    DDI_CHK_NULL(chromaPitch, "nullptr chromaPitch", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(chromaHeight, "nullptr chromaHeight", VA_STATUS_ERROR_INVALID_PARAMETER);

    switch (fourcc)
    {
    case VA_FOURCC_NV12:
    case VA_FOURCC_P010:
    case VA_FOURCC_P012:
    case VA_FOURCC_P016:
        *chromaHeight = MOS_ALIGN_CEIL(height, 2) / 2;
        *chromaPitch = pitch;
        break;
    case VA_FOURCC_I420:
    case VA_FOURCC_YV12:
        *chromaHeight = MOS_ALIGN_CEIL(height, 2) / 2;
        *chromaPitch = MOS_ALIGN_CEIL(pitch, 2) / 2;
        break;
    case VA_FOURCC_411P:
    case VA_FOURCC_422H:
    case VA_FOURCC_444P:
        *chromaHeight = height;
        *chromaPitch = pitch;
        break;
    case VA_FOURCC_422V:
    case VA_FOURCC_IMC3:
        *chromaHeight = MOS_ALIGN_CEIL(height, 2) / 2;
        *chromaPitch = pitch;
        break;
    default:
        *chromaPitch = 0;
        *chromaHeight = 0;
    }

    return VA_STATUS_SUCCESS;
}

#if !defined(ANDROID) && defined(X11_FOUND)

#define X11_LIB_NAME "libX11.so.6"

/*
 * Close opened libX11.so lib, free related function table.
 */
static void DdiMedia_DestroyX11Connection(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    if (nullptr == mediaCtx || nullptr == mediaCtx->X11FuncTable)
    {
        return;
    }

    MOS_FreeLibrary(mediaCtx->X11FuncTable->pX11LibHandle);
    MOS_FreeMemory(mediaCtx->X11FuncTable);
    mediaCtx->X11FuncTable = nullptr;

    return;
}

/*
 * dlopen libX11.so, setup the function table, which is used by
 * DdiCodec_PutSurface (Linux) so far.
 */
static VAStatus DdiMedia_ConnectX11(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    mediaCtx->X11FuncTable = (PDDI_X11_FUNC_TABLE)MOS_AllocAndZeroMemory(sizeof(DDI_X11_FUNC_TABLE));
    DDI_CHK_NULL(mediaCtx->X11FuncTable, "Allocation Failed for X11FuncTable", VA_STATUS_ERROR_ALLOCATION_FAILED);

    HMODULE h_module = nullptr;
    MOS_STATUS mos_status = MOS_LoadLibrary(X11_LIB_NAME, &h_module);
    if (MOS_STATUS_SUCCESS != mos_status || nullptr == h_module)
    {
        DdiMedia_DestroyX11Connection(mediaCtx);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    mediaCtx->X11FuncTable->pX11LibHandle = h_module;

    mediaCtx->X11FuncTable->pfnXCreateGC =
        MOS_GetProcAddress(h_module, "XCreateGC");
    mediaCtx->X11FuncTable->pfnXFreeGC =
        MOS_GetProcAddress(h_module, "XFreeGC");
    mediaCtx->X11FuncTable->pfnXCreateImage =
        MOS_GetProcAddress(h_module, "XCreateImage");
    mediaCtx->X11FuncTable->pfnXDestroyImage =
        MOS_GetProcAddress(h_module, "XDestroyImage");
    mediaCtx->X11FuncTable->pfnXPutImage =
        MOS_GetProcAddress(h_module, "XPutImage");

    if (nullptr == mediaCtx->X11FuncTable->pfnXCreateGC ||
        nullptr == mediaCtx->X11FuncTable->pfnXFreeGC ||
        nullptr == mediaCtx->X11FuncTable->pfnXCreateImage ||
        nullptr == mediaCtx->X11FuncTable->pfnXDestroyImage ||
        nullptr == mediaCtx->X11FuncTable->pfnXPutImage)
    {
        DdiMedia_DestroyX11Connection(mediaCtx);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    return VA_STATUS_SUCCESS;
}
#endif

/////////////////////////////////////////////////////////////////////////////
//! \Free allocated surfaceheap elements
//! \params
//! [in] PDDI_MEDIA_CONTEXT
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeSurfaceHeapElements(PDDI_MEDIA_CONTEXT mediaCtx)
{
    if (nullptr == mediaCtx)
        return;
    PDDI_MEDIA_HEAP surfaceHeap = mediaCtx->pSurfaceHeap;

    if (nullptr == surfaceHeap)
        return;

    PDDI_MEDIA_SURFACE_HEAP_ELEMENT mediaSurfaceHeapBase = (PDDI_MEDIA_SURFACE_HEAP_ELEMENT)surfaceHeap->pHeapBase;
    if (nullptr == mediaSurfaceHeapBase)
        return;

    int32_t surfaceNums = mediaCtx->uiNumSurfaces;
    for (int32_t elementId = 0; elementId < surfaceNums; elementId++)
    {
        PDDI_MEDIA_SURFACE_HEAP_ELEMENT mediaSurfaceHeapElmt = &mediaSurfaceHeapBase[elementId];
        if (nullptr == mediaSurfaceHeapElmt->pSurface)
            continue;

        DdiMediaUtil_FreeSurface(mediaSurfaceHeapElmt->pSurface);
        MOS_FreeMemory(mediaSurfaceHeapElmt->pSurface);
        DdiMediaUtil_ReleasePMediaSurfaceFromHeap(surfaceHeap, mediaSurfaceHeapElmt->uiVaSurfaceID);
        mediaCtx->uiNumSurfaces--;
    }
}

/////////////////////////////////////////////////////////////////////////////
//! \Free allocated bufferheap elements
//! \params
//! [in] VADriverContextP
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeBufferHeapElements(VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (nullptr == mediaCtx)
        return;

    PDDI_MEDIA_HEAP bufferHeap = mediaCtx->pBufferHeap;
    if (nullptr == bufferHeap)
        return;

    PDDI_MEDIA_BUFFER_HEAP_ELEMENT mediaBufferHeapBase = (PDDI_MEDIA_BUFFER_HEAP_ELEMENT)bufferHeap->pHeapBase;
    if (nullptr == mediaBufferHeapBase)
        return;

    int32_t bufNums = mediaCtx->uiNumBufs;
    for (int32_t elementId = 0; bufNums > 0; ++elementId)
    {
        PDDI_MEDIA_BUFFER_HEAP_ELEMENT mediaBufferHeapElmt = &mediaBufferHeapBase[elementId];
        if (nullptr == mediaBufferHeapElmt->pBuffer)
            continue;
        DdiMedia_DestroyBuffer(ctx, mediaBufferHeapElmt->uiVaBufferID);
        // Ensure the non-empty buffer to be destroyed.
        --bufNums;
    }
}

/////////////////////////////////////////////////////////////////////////////
//! \Free allocated Imageheap elements
//! \params
//! [in] VADriverContextP
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeImageHeapElements(VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (nullptr == mediaCtx)
        return;

    PDDI_MEDIA_HEAP imageHeap = mediaCtx->pImageHeap;
    if (nullptr == imageHeap)
        return;

    PDDI_MEDIA_IMAGE_HEAP_ELEMENT mediaImageHeapBase = (PDDI_MEDIA_IMAGE_HEAP_ELEMENT)imageHeap->pHeapBase;
    if (nullptr == mediaImageHeapBase)
        return;

    int32_t imageNums = mediaCtx->uiNumImages;
    for (int32_t elementId = 0; elementId < imageNums; ++elementId)
    {
        PDDI_MEDIA_IMAGE_HEAP_ELEMENT mediaImageHeapElmt = &mediaImageHeapBase[elementId];
        if (nullptr == mediaImageHeapElmt->pImage)
            continue;
        DdiMedia_DestroyImage(ctx, mediaImageHeapElmt->uiVaImageID);
    }
}

/////////////////////////////////////////////////////////////////////////////
//! \Execute free allocated bufferheap elements for FreeContextHeapElements function
//! \params
//! [in] VADriverContextP
//! [in] PDDI_MEDIA_HEAP
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeContextHeap(VADriverContextP ctx, PDDI_MEDIA_HEAP contextHeap, int32_t vaContextOffset, int32_t ctxNums)
{
    PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT mediaContextHeapBase = (PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT)contextHeap->pHeapBase;
    if (nullptr == mediaContextHeapBase)
        return;

    for (int32_t elementId = 0; elementId < ctxNums; ++elementId)
    {
        PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT mediaContextHeapElmt = &mediaContextHeapBase[elementId];
        if (nullptr == mediaContextHeapElmt->pVaContext)
            continue;
        VAContextID vaCtxID = (VAContextID)(mediaContextHeapElmt->uiVaContextID + vaContextOffset);
        DdiMedia_DestroyContext(ctx, vaCtxID);
    }
}

/////////////////////////////////////////////////////////////////////////////
//! \Free allocated contextheap elements
//! \params
//! [in] VADriverContextP
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeContextHeapElements(VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (nullptr == mediaCtx)
        return;

    // Free EncoderContext
    PDDI_MEDIA_HEAP encoderContextHeap = mediaCtx->pEncoderCtxHeap;
    int32_t encCtxNums = mediaCtx->uiNumEncoders;
    if (nullptr != encoderContextHeap)
        DdiMedia_FreeContextHeap(ctx, encoderContextHeap, DDI_MEDIA_VACONTEXTID_OFFSET_ENCODER, encCtxNums);

    // Free DecoderContext
    PDDI_MEDIA_HEAP decoderContextHeap = mediaCtx->pDecoderCtxHeap;
    int32_t decCtxNums = mediaCtx->uiNumDecoders;
    if (nullptr != decoderContextHeap)
        DdiMedia_FreeContextHeap(ctx, decoderContextHeap, DDI_MEDIA_VACONTEXTID_OFFSET_DECODER, decCtxNums);

    // Free VpContext
    PDDI_MEDIA_HEAP vpContextHeap = mediaCtx->pVpCtxHeap;
    int32_t vpctxNums = mediaCtx->uiNumVPs;
    if (nullptr != vpContextHeap)
        DdiMedia_FreeContextHeap(ctx, vpContextHeap, DDI_MEDIA_VACONTEXTID_OFFSET_VP, vpctxNums);

    // Free ProtContext
    PDDI_MEDIA_HEAP protContextHeap = mediaCtx->pProtCtxHeap;
    int32_t protCtxNums = mediaCtx->uiNumProts;
    if (nullptr != protContextHeap)
        DdiMedia_FreeProtectedSessionHeap(ctx, protContextHeap, DDI_MEDIA_VACONTEXTID_OFFSET_PROT, protCtxNums);

    // Free MfeContext
    PDDI_MEDIA_HEAP mfeContextHeap = mediaCtx->pMfeCtxHeap;
    int32_t mfeCtxNums = mediaCtx->uiNumMfes;
    if (nullptr != mfeContextHeap)
        DdiMedia_FreeContextHeap(ctx, mfeContextHeap, DDI_MEDIA_VACONTEXTID_OFFSET_MFE, mfeCtxNums);

    // Free media memory decompression data structure
    if (!mediaCtx->m_apoMosEnabled && mediaCtx->pMediaMemDecompState)
    {
        MediaMemDecompBaseState *mediaMemCompState =
            static_cast<MediaMemDecompBaseState *>(mediaCtx->pMediaMemDecompState);
        MOS_Delete(mediaMemCompState);
    }
    mediaCtx->pMediaMemDecompState = nullptr;
}

/////////////////////////////////////////////////////////////////////////////
//! \Free allocated ContextCM elements
//! \params
//! [in] VADriverContextP
//! [out] none
//! \returns
/////////////////////////////////////////////////////////////////////////////
static void DdiMedia_FreeContextCMElements(VADriverContextP ctx)
{
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (nullptr == mediaCtx)
        return;

    int32_t cmnums = mediaCtx->uiNumCMs;
    for (int32_t elementId = 0; elementId < cmnums; elementId++)
    {
        VAContextID vaCtxID = elementId + DDI_MEDIA_VACONTEXTID_OFFSET_CM;
        DdiDestroyContextCM(ctx, vaCtxID);
    }
}

//!
//! \brief  Get VA image from VA image ID
//!
//! \param  [in] mediaCtx
//!         Pointer to ddi media context
//! \param  [in] imageID
//!         VA image ID
//!
//! \return VAImage*
//!     Pointer to VAImage
//!
VAImage *DdiMedia_GetVAImageFromVAImageID(PDDI_MEDIA_CONTEXT mediaCtx, VAImageID imageID)
{
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", nullptr);

    uint32_t i = (uint32_t)imageID;
    DDI_CHK_LESS(i, mediaCtx->pImageHeap->uiAllocatedHeapElements, "invalid image id", nullptr);
    DdiMediaUtil_LockMutex(&mediaCtx->ImageMutex);
    PDDI_MEDIA_IMAGE_HEAP_ELEMENT imageElement = (PDDI_MEDIA_IMAGE_HEAP_ELEMENT)mediaCtx->pImageHeap->pHeapBase;
    imageElement += i;
    VAImage *vaImage = imageElement->pImage;
    DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);

    return vaImage;
}

//!
//! \brief  Get ctx from VA buffer ID
//!
//! \param  [in] mediaCtx
//!         pddi media context
//! \param  [in] bufferID
//!         VA Buffer ID
//!
//! \return void*
//!     Pointer to buffer heap element context
//!
void *DdiMedia_GetCtxFromVABufferID(PDDI_MEDIA_CONTEXT mediaCtx, VABufferID bufferID)
{
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", nullptr);

    uint32_t i = (uint32_t)bufferID;
    DDI_CHK_LESS(i, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "invalid buffer id", nullptr);
    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufHeapElement = (PDDI_MEDIA_BUFFER_HEAP_ELEMENT)mediaCtx->pBufferHeap->pHeapBase;
    bufHeapElement += i;
    void *temp = bufHeapElement->pCtx;
    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);

    return temp;
}

//!
//! \brief  Get ctx type from VA buffer ID
//!
//! \param  [in] mediaCtx
//!         Pointer to ddi media context
//! \param  [in] bufferID
//!         VA buffer ID
//!
//! \return uint32_t
// 1     Context type
//!
uint32_t DdiMedia_GetCtxTypeFromVABufferID(PDDI_MEDIA_CONTEXT mediaCtx, VABufferID bufferID)
{
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", DDI_MEDIA_CONTEXT_TYPE_NONE);

    uint32_t i = (uint32_t)bufferID;
    DDI_CHK_LESS(i, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "invalid buffer id", DDI_MEDIA_CONTEXT_TYPE_NONE);
    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufHeapElement = (PDDI_MEDIA_BUFFER_HEAP_ELEMENT)mediaCtx->pBufferHeap->pHeapBase;
    bufHeapElement += i;
    uint32_t ctxType = bufHeapElement->uiCtxType;
    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);

    return ctxType;
}

//!
//! \brief  Destroy image from VA image ID
//!
//! \param  [in] mediaCtx
//!         Pointer to ddi media context
//! \param  [in] imageID
//!     VA image ID
//!
//! \return bool
//!     True if destroy image from VA image ID, else fail
//!
bool DdiMedia_DestroyImageFromVAImageID(PDDI_MEDIA_CONTEXT mediaCtx, VAImageID imageID)
{
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", false);

    DdiMediaUtil_LockMutex(&mediaCtx->ImageMutex);
    DdiMediaUtil_ReleasePVAImageFromHeap(mediaCtx->pImageHeap, (uint32_t)imageID);
    mediaCtx->uiNumImages--;
    DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);
    return true;
}
#ifdef _MMC_SUPPORTED
//!
//! \brief  Decompress internal media memory
//!
//! \param  [in] mosCtx
//!         Pointer to mos context
//! \param  [in] osResource
//!         Pointer mos resource
//!
void DdiMedia_MediaMemoryDecompressInternal(PMOS_CONTEXT mosCtx, PMOS_RESOURCE osResource)
{
    DDI_CHK_NULL(mosCtx, "nullptr mosCtx", );
    DDI_CHK_NULL(osResource, "nullptr osResource", );
    DDI_ASSERT(osResource);

    MediaMemDecompBaseState *mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(*mosCtx->ppMediaMemDecompState);

    if (mosCtx->m_apoMosEnabled && !mediaMemDecompState)
    {
        DDI_CHK_NULL(mediaMemDecompState, "nullptr mediaMemDecompState", );
    }

    if (!mediaMemDecompState)
    {
        mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(MmdDevice::CreateFactory(mosCtx));
        *mosCtx->ppMediaMemDecompState = mediaMemDecompState;
    }

    if (mediaMemDecompState)
    {
        mediaMemDecompState->MemoryDecompress(osResource);
    }
    else
    {
        DDI_ASSERTMESSAGE("Invalid memory decompression state.");
    }
}

//!
//! \brief  copy internal media surface to another surface
//!
//! \param  [in] mosCtx
//!         Pointer to mos context
//! \param  [in] inputOsResource
//!         Pointer input mos resource
//! \param  [in] outputOsResource
//!         Pointer output mos resource
//! \param  [in] boutputcompressed
//!         output can be compressed or not
//!
void DdiMedia_MediaMemoryCopyInternal(PMOS_CONTEXT mosCtx, PMOS_RESOURCE inputOsResource, PMOS_RESOURCE outputOsResource, bool boutputcompressed)
{
    DDI_CHK_NULL(mosCtx, "nullptr mosCtx", );
    DDI_CHK_NULL(inputOsResource, "nullptr input osResource", );
    DDI_CHK_NULL(outputOsResource, "nullptr output osResource", );
    DDI_ASSERT(inputOsResource);
    DDI_ASSERT(outputOsResource);

    MediaMemDecompBaseState *mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(*mosCtx->ppMediaMemDecompState);

    if (mosCtx->m_apoMosEnabled && !mediaMemDecompState)
    {
        DDI_CHK_NULL(mediaMemDecompState, "nullptr mediaMemDecompState", );
    }

    if (!mediaMemDecompState)
    {
        mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(MmdDevice::CreateFactory(mosCtx));
        *mosCtx->ppMediaMemDecompState = mediaMemDecompState;
    }

    if (mediaMemDecompState)
    {
        mediaMemDecompState->MediaMemoryCopy(inputOsResource, outputOsResource, boutputcompressed);
    }
    else
    {
        DDI_ASSERTMESSAGE("Invalid memory decompression state.");
    }
}

//!
//! \brief  copy internal media surface/buffer to another surface/buffer
//!
//! \param  [in] mosCtx
//!         Pointer to mos context
//! \param  [in] inputOsResource
//!         Pointer input mos resource
//! \param  [in] outputOsResource
//!         Pointer output mos resource
//! \param  [in] boutputcompressed
//!         output can be compressed or not
//! \param  [in] copyWidth
//!         The 2D surface Width
//! \param  [in] copyHeight
//!         The 2D surface height
//! \param  [in] copyInputOffset
//!         The offset of copied surface from
//! \param  [in] copyOutputOffset
//!         The offset of copied to
//!
void DdiMedia_MediaMemoryCopy2DInternal(PMOS_CONTEXT mosCtx, PMOS_RESOURCE inputOsResource, PMOS_RESOURCE outputOsResource, uint32_t copyWidth, uint32_t copyHeight, uint32_t copyInputOffset, uint32_t copyOutputOffset, uint32_t bpp, bool boutputcompressed)
{
    DDI_CHK_NULL(mosCtx, "nullptr mosCtx", );
    DDI_CHK_NULL(inputOsResource, "nullptr input osResource", );
    DDI_CHK_NULL(outputOsResource, "nullptr output osResource", );
    DDI_ASSERT(inputOsResource);
    DDI_ASSERT(outputOsResource);

    MediaMemDecompBaseState *mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(*mosCtx->ppMediaMemDecompState);

    if (mosCtx->m_apoMosEnabled && !mediaMemDecompState)
    {
        DDI_CHK_NULL(mediaMemDecompState, "nullptr mediaMemDecompState", );
    }

    if (!mediaMemDecompState)
    {
        mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(MmdDevice::CreateFactory(mosCtx));
        *mosCtx->ppMediaMemDecompState = mediaMemDecompState;
    }

    if (mediaMemDecompState)
    {
        mediaMemDecompState->MediaMemoryCopy2D(
            inputOsResource,
            outputOsResource,
            copyWidth,
            copyHeight,
            copyInputOffset,
            copyOutputOffset,
            bpp,
            boutputcompressed);
    }
    else
    {
        DDI_ASSERTMESSAGE("Invalid memory decompression state.");
    }
}

//!
//! \brief  Tile/Linear format conversion for media surface/buffer
//!
//! \param  [in] mosCtx
//!         Pointer to mos context
//! \param  [in] inputOsResource
//!         Pointer input mos resource
//! \param  [in] outputOsResource
//!         Pointer output mos resource
//! \param  [in] copyWidth
//!         The 2D surface Width
//! \param  [in] copyHeight
//!         The 2D surface height
//! \param  [in] copyInputOffset
//!         The offset of copied surface from
//! \param  [in] copyOutputOffset
//!         The offset of copied to
//! \param  [in] isTileToLinear
//!         Convertion direction, true: tile->linear, false: linear->tile
//! \param  [in] outputCompressed
//!         output can be compressed or not
//!
VAStatus DdiMedia_MediaMemoryTileConvertInternal(
    PMOS_CONTEXT mosCtx,
    PMOS_RESOURCE inputOsResource,
    PMOS_RESOURCE outputOsResource,
    uint32_t copyWidth,
    uint32_t copyHeight,
    uint32_t copyInputOffset,
    uint32_t copyOutputOffset,
    bool isTileToLinear,
    bool outputCompressed)
{
    DDI_CHK_NULL(mosCtx, "nullptr mosCtx", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(inputOsResource, "nullptr input osResource", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(outputOsResource, "nullptr output osResource", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_ASSERT(inputOsResource);
    DDI_ASSERT(outputOsResource);

    VAStatus vaStatus = VA_STATUS_SUCCESS;

    MediaMemDecompBaseState *mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(*mosCtx->ppMediaMemDecompState);

    if (mosCtx->m_apoMosEnabled && !mediaMemDecompState)
    {
        DDI_CHK_NULL(mediaMemDecompState, "nullptr mediaMemDecompState", VA_STATUS_ERROR_INVALID_PARAMETER);
    }

    if (!mediaMemDecompState)
    {
        mediaMemDecompState = static_cast<MediaMemDecompBaseState *>(MmdDevice::CreateFactory(mosCtx));
        *mosCtx->ppMediaMemDecompState = mediaMemDecompState;
    }

    DDI_CHK_NULL(mediaMemDecompState, "Invalid memory decompression state", VA_STATUS_ERROR_INVALID_PARAMETER);

    MOS_STATUS mosStatus = mediaMemDecompState->MediaMemoryTileConvert(
        inputOsResource,
        outputOsResource,
        copyWidth,
        copyHeight,
        copyInputOffset,
        copyOutputOffset,
        isTileToLinear,
        outputCompressed);
    if (mosStatus != MOS_STATUS_SUCCESS)
    {
        vaStatus = VA_STATUS_ERROR_UNKNOWN;
    }

    return vaStatus;
}
#endif

//!
//! \brief  Decompress a compressed surface.
//!
//! \param  [in]     mediaCtx
//!     Pointer to ddi media context
//! \param  [in]     mediaSurface
//!     Ddi media surface
//!
//! \return     VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_MediaMemoryDecompress(PDDI_MEDIA_CONTEXT mediaCtx, DDI_MEDIA_SURFACE *mediaSurface)
{
    DDI_CHK_NULL(mediaCtx, "Null mediaCtx.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(mediaSurface->pGmmResourceInfo, "nullptr mediaSurface->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_PARAMETER);

    VAStatus vaStatus = VA_STATUS_SUCCESS;
    GMM_RESOURCE_FLAG GmmFlags;

    MOS_ZeroMemory(&GmmFlags, sizeof(GmmFlags));
    GmmFlags = mediaSurface->pGmmResourceInfo->GetResFlags();

    if (((GmmFlags.Gpu.MMC ||
          GmmFlags.Gpu.CCS) &&
         GmmFlags.Info.MediaCompressed) ||
        mediaSurface->pGmmResourceInfo->IsMediaMemoryCompressed(0))
    {
#ifdef _MMC_SUPPORTED
        MOS_CONTEXT mosCtx;
        MOS_RESOURCE surface;
        DdiCpInterface *pCpDdiInterface;

        MOS_ZeroMemory(&mosCtx, sizeof(mosCtx));
        MOS_ZeroMemory(&surface, sizeof(surface));

        mosCtx.bufmgr = mediaCtx->pDrmBufMgr;
        mosCtx.m_gpuContextMgr = mediaCtx->m_gpuContextMgr;
        mosCtx.m_cmdBufMgr = mediaCtx->m_cmdBufMgr;
        mosCtx.fd = mediaCtx->fd;
        mosCtx.iDeviceId = mediaCtx->iDeviceId;
        mosCtx.SkuTable = mediaCtx->SkuTable;
        mosCtx.WaTable = mediaCtx->WaTable;
        mosCtx.gtSystemInfo = *mediaCtx->pGtSystemInfo;
        mosCtx.platform = mediaCtx->platform;

        mosCtx.ppMediaMemDecompState = &mediaCtx->pMediaMemDecompState;
        mosCtx.pfnMemoryDecompress = mediaCtx->pfnMemoryDecompress;
        mosCtx.pfnMediaMemoryCopy = mediaCtx->pfnMediaMemoryCopy;
        mosCtx.pfnMediaMemoryCopy2D = mediaCtx->pfnMediaMemoryCopy2D;
        mosCtx.gtSystemInfo = *mediaCtx->pGtSystemInfo;
        mosCtx.m_auxTableMgr = mediaCtx->m_auxTableMgr;
        mosCtx.pGmmClientContext = mediaCtx->pGmmClientContext;

        mosCtx.m_osDeviceContext = mediaCtx->m_osDeviceContext;
        mosCtx.m_apoMosEnabled = mediaCtx->m_apoMosEnabled;

        pCpDdiInterface = Create_DdiCpInterface(mosCtx);

        if (nullptr == pCpDdiInterface)
        {
            vaStatus = VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        else
        {
            DdiMediaUtil_LockMutex(&mediaCtx->SurfaceMutex);

            DdiMedia_MediaSurfaceToMosResource(mediaSurface, &surface);
            DdiMedia_MediaMemoryDecompressInternal(&mosCtx, &surface);

            DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);

            if (pCpDdiInterface)
            {
                Delete_DdiCpInterface(pCpDdiInterface);
                pCpDdiInterface = NULL;
            }
        }
#else
        vaStatus = VA_STATUS_ERROR_INVALID_SURFACE;
        DDI_ASSERTMESSAGE("MMC unsupported! [%d].", vaStatus);
#endif
    }

    return vaStatus;
}

/*
 * Initialize the library
 */

// Global mutex
MEDIA_MUTEX_T GlobalMutex = MEDIA_MUTEX_INITIALIZER;

//!
//! \brief  Initialize
//!
//! \param  [in] mediaCtx
//!         Pointer to DDI media driver context
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
static VAStatus DdiMedia_HeapInitialize(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    DDI_CHK_NULL(mediaCtx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    // Heap initialization here
    mediaCtx->pSurfaceHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr pSurfaceHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pSurfaceHeap->uiHeapElementSize = sizeof(DDI_MEDIA_SURFACE_HEAP_ELEMENT);

    mediaCtx->pBufferHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr BufferHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pBufferHeap->uiHeapElementSize = sizeof(DDI_MEDIA_BUFFER_HEAP_ELEMENT);

    mediaCtx->pImageHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pImageHeap, "nullptr ImageHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pImageHeap->uiHeapElementSize = sizeof(DDI_MEDIA_IMAGE_HEAP_ELEMENT);

    mediaCtx->pDecoderCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pDecoderCtxHeap, "nullptr DecoderCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pDecoderCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    mediaCtx->pEncoderCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pEncoderCtxHeap, "nullptr EncoderCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pEncoderCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    mediaCtx->pVpCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pVpCtxHeap, "nullptr VpCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pVpCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    mediaCtx->pProtCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pProtCtxHeap, "nullptr pProtCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pProtCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    mediaCtx->pCmCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pCmCtxHeap, "nullptr CmCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pCmCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    mediaCtx->pMfeCtxHeap = (DDI_MEDIA_HEAP *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_HEAP));
    DDI_CHK_NULL(mediaCtx->pMfeCtxHeap, "nullptr MfeCtxHeap", VA_STATUS_ERROR_ALLOCATION_FAILED);
    mediaCtx->pMfeCtxHeap->uiHeapElementSize = sizeof(DDI_MEDIA_VACONTEXT_HEAP_ELEMENT);

    // init the mutexs
    DdiMediaUtil_InitMutex(&mediaCtx->SurfaceMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->BufferMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->ImageMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->DecoderMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->EncoderMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->VpMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->ProtMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->CmMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->MfeMutex);

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Initialize
//!
//! \param  [in] mediaCtx
//!         Pointer to DDI media driver context
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
static VAStatus DdiMedia_HeapDestroy(
    PDDI_MEDIA_CONTEXT mediaCtx)
{
    DDI_CHK_NULL(mediaCtx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    // destroy heaps
    MOS_FreeMemory(mediaCtx->pSurfaceHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pSurfaceHeap);

    MOS_FreeMemory(mediaCtx->pBufferHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pBufferHeap);

    MOS_FreeMemory(mediaCtx->pImageHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pImageHeap);

    MOS_FreeMemory(mediaCtx->pDecoderCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pDecoderCtxHeap);

    MOS_FreeMemory(mediaCtx->pEncoderCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pEncoderCtxHeap);

    MOS_FreeMemory(mediaCtx->pVpCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pVpCtxHeap);

    MOS_FreeMemory(mediaCtx->pProtCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pProtCtxHeap);

    MOS_FreeMemory(mediaCtx->pCmCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pCmCtxHeap);

    MOS_FreeMemory(mediaCtx->pMfeCtxHeap->pHeapBase);
    MOS_FreeMemory(mediaCtx->pMfeCtxHeap);
    // destroy the mutexs
    DdiMediaUtil_DestroyMutex(&mediaCtx->SurfaceMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->BufferMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->ImageMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->DecoderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->EncoderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->VpMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->ProtMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->CmMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->MfeMutex);

    // resource checking
    if (mediaCtx->uiNumSurfaces != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the surfaces.");
    }
    if (mediaCtx->uiNumBufs != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the buffers.");
    }
    if (mediaCtx->uiNumImages != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the images.");
    }
    if (mediaCtx->uiNumDecoders != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the decoders.");
    }
    if (mediaCtx->uiNumEncoders != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the encoders.");
    }
    if (mediaCtx->uiNumVPs != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the VPs.");
    }
    if (mediaCtx->uiNumProts != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the Prots.");
    }
    if (mediaCtx->uiNumCMs != 0)
    {
        DDI_ASSERTMESSAGE("APP does not destroy all the CMs.");
    }
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Free for media context
//!
//! \param  [in] mediaCtx
//!         Pointer to ddi media context
//!
void FreeForMediaContext(PDDI_MEDIA_CONTEXT mediaCtx)
{
    DdiMediaUtil_UnLockMutex(&GlobalMutex);

    if (mediaCtx)
    {
        mediaCtx->SkuTable.reset();
        mediaCtx->WaTable.reset();
        MOS_FreeMemory(mediaCtx->pSurfaceHeap);
        MOS_FreeMemory(mediaCtx->pBufferHeap);
        MOS_FreeMemory(mediaCtx->pImageHeap);
        MOS_FreeMemory(mediaCtx->pDecoderCtxHeap);
        MOS_FreeMemory(mediaCtx->pEncoderCtxHeap);
        MOS_FreeMemory(mediaCtx->pVpCtxHeap);
        MOS_FreeMemory(mediaCtx->pProtCtxHeap);
        MOS_FreeMemory(mediaCtx->pMfeCtxHeap);
        MOS_FreeMemory(mediaCtx);
    }

    return;
}

void DestroyMediaContextMutex(PDDI_MEDIA_CONTEXT mediaCtx)
{
    // destroy the mutexs
    DdiMediaUtil_DestroyMutex(&mediaCtx->SurfaceMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->BufferMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->ImageMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->DecoderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->EncoderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->VpMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->CmMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->MfeMutex);
#if !defined(ANDROID) && defined(X11_FOUND)
    DdiMediaUtil_DestroyMutex(&mediaCtx->PutSurfaceRenderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->PutSurfaceSwapBufferMutex);
#endif
    return;
}

VAStatus DdiMedia_GetDeviceFD(
    VADriverContextP ctx,
    int32_t *pDevicefd)
{
    struct drm_state *pDRMState = (struct drm_state *)ctx->drm_state;
    DDI_CHK_NULL(pDRMState, "nullptr pDRMState", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(pDevicefd, "nullptr pDevicefd", VA_STATUS_ERROR_INVALID_CONTEXT);

    // If libva failes to open the graphics card, try to open it again within Media Driver
    if (pDRMState->fd < 0 || pDRMState->fd == 0)
    {
        DDI_ASSERTMESSAGE("DDI:LIBVA Wrapper doesn't pass file descriptor for graphics adaptor, trying to open the graphics... ");
        pDRMState->fd = DdiMediaUtil_OpenGraphicsAdaptor((char *)DEVICE_NAME);
        if (pDRMState->fd < 0)
        {
            DDI_ASSERTMESSAGE("DDI: Still failed to open the graphic adaptor, return failure");
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
    }
    *pDevicefd = pDRMState->fd;

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia__Initialize(
    VADriverContextP ctx,
    int32_t *major_version,
    int32_t *minor_version)
{
#if !defined(ANDROID) && defined(X11_FOUND)
    // ATRACE code in <cutils/trace.h> started from KitKat, version = 440
    // ENABLE_TRACE is defined only for eng build so release build won't leak perf data
    // thus trace code enabled only on KitKat (and newer) && eng build
#if ANDROID_VERSION > 439 && defined(ENABLE_ATRACE)
    char switch_value[PROPERTY_VALUE_MAX];

    property_get("debug.DdiCodec_.umd", switch_value, "0");
    atrace_switch = atoi(switch_value);
#endif
#endif

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    bool apoDdiEnabled = false;
    int32_t devicefd = 0;
    if (DdiMedia_GetDeviceFD(ctx, &devicefd) != VA_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Unable to get device FD");
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if (MediaLibvaInterface::LoadFunction(ctx) != VA_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Failed to load function pointer");
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    if (apoDdiEnabled)
    {
        return MediaLibvaInterfaceNext::Initialize(ctx, devicefd, major_version, minor_version);
    }
    else
    {
        return DdiMedia_InitMediaContext(ctx, devicefd, major_version, minor_version);
    }
}

VAStatus DdiMedia_InitMediaContext(
    VADriverContextP ctx,
    int32_t devicefd,
    int32_t *major_version,
    int32_t *minor_version)
{
    if (major_version)
    {
        *major_version = VA_MAJOR_VERSION;
    }

    if (minor_version)
    {
        *minor_version = VA_MINOR_VERSION;
    }

    DdiMediaUtil_LockMutex(&GlobalMutex);
    // media context is already created, return directly to support multiple entry
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (mediaCtx)
    {
        mediaCtx->uiRef++;
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_SUCCESS;
    }

    mediaCtx = DdiMedia_CreateMediaDriverContext();
    if (nullptr == mediaCtx)
    {
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    mediaCtx->uiRef++;
    ctx->pDriverData = (void *)mediaCtx;
    mediaCtx->fd = devicefd;

    mediaCtx->m_apoMosEnabled = SetupApoMosSwitch(devicefd);

#ifdef _MMC_SUPPORTED
    mediaCtx->pfnMemoryDecompress = DdiMedia_MediaMemoryDecompressInternal;
    mediaCtx->pfnMediaMemoryCopy = DdiMedia_MediaMemoryCopyInternal;
    mediaCtx->pfnMediaMemoryCopy2D = DdiMedia_MediaMemoryCopy2DInternal;
    mediaCtx->pfnMediaMemoryTileConvert = DdiMedia_MediaMemoryTileConvertInternal;
#endif
    mediaCtx->modularizedGpuCtxEnabled = true;

    if (mediaCtx->m_apoMosEnabled)
    {
        MOS_CONTEXT mosCtx = {};
        mosCtx.fd = mediaCtx->fd;
        mosCtx.m_apoMosEnabled = mediaCtx->m_apoMosEnabled;

        MosInterface::InitOsUtilities(&mosCtx);

        mediaCtx->pGtSystemInfo = (MEDIA_SYSTEM_INFO *)MOS_AllocAndZeroMemory(sizeof(MEDIA_SYSTEM_INFO));
        if (nullptr == mediaCtx->pGtSystemInfo)
        {
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        if (MosInterface::CreateOsDeviceContext(&mosCtx, &mediaCtx->m_osDeviceContext) != MOS_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("Unable to create MOS device context.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        mediaCtx->pDrmBufMgr = mosCtx.bufmgr;
        mediaCtx->iDeviceId = mosCtx.iDeviceId;
        mediaCtx->SkuTable = mosCtx.SkuTable;
        mediaCtx->WaTable = mosCtx.WaTable;
        *mediaCtx->pGtSystemInfo = mosCtx.gtSystemInfo;
        mediaCtx->platform = mosCtx.platform;
        mediaCtx->m_auxTableMgr = mosCtx.m_auxTableMgr;
        mediaCtx->pGmmClientContext = mosCtx.pGmmClientContext;
        mediaCtx->m_useSwSwizzling = mosCtx.bUseSwSwizzling;
        mediaCtx->m_tileYFlag = mosCtx.bTileYFlag;
        mediaCtx->bIsAtomSOC = mosCtx.bIsAtomSOC;
#ifdef _MMC_SUPPORTED
        if (mosCtx.ppMediaMemDecompState == nullptr)
        {
            DDI_ASSERTMESSAGE("media decomp state is null.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        mediaCtx->pMediaMemDecompState = *mosCtx.ppMediaMemDecompState;
#endif
        mediaCtx->pMediaCopyState = *mosCtx.ppMediaCopyState;
    }
    else if (mediaCtx->modularizedGpuCtxEnabled)
    {
        // prepare m_osContext
        MosUtilities::MosUtilitiesInit(nullptr);
        // Read user feature key here for Per Utility Tool Enabling

        if (!g_perfutility->bPerfUtilityKey)
        {
            MOS_USER_FEATURE_VALUE_DATA UserFeatureData;
            MOS_ZeroMemory(&UserFeatureData, sizeof(UserFeatureData));
            MOS_UserFeature_ReadValue_ID(
                NULL,
                __MEDIA_USER_FEATURE_VALUE_PERF_UTILITY_TOOL_ENABLE_ID,
                &UserFeatureData,
                nullptr);
            g_perfutility->dwPerfUtilityIsEnabled = UserFeatureData.i32Data;

            char sFilePath[MOS_MAX_PERF_FILENAME_LEN + 1] = "";
            MOS_USER_FEATURE_VALUE_DATA perfFilePath;
            MOS_STATUS eStatus_Perf = MOS_STATUS_SUCCESS;

            MOS_ZeroMemory(&perfFilePath, sizeof(perfFilePath));
            perfFilePath.StringData.pStringData = sFilePath;
            eStatus_Perf = MOS_UserFeature_ReadValue_ID(
                nullptr,
                __MEDIA_USER_FEATURE_VALUE_PERF_OUTPUT_DIRECTORY_ID,
                &perfFilePath,
                nullptr);
            if (eStatus_Perf == MOS_STATUS_SUCCESS)
            {
                g_perfutility->setupFilePath(sFilePath);
            }
            else
            {
                g_perfutility->setupFilePath();
            }

            g_perfutility->bPerfUtilityKey = true;
        }

        mediaCtx->pDrmBufMgr = mos_bufmgr_gem_init(mediaCtx->fd, DDI_CODEC_BATCH_BUFFER_SIZE);
        if (nullptr == mediaCtx->pDrmBufMgr)
        {
            DDI_ASSERTMESSAGE("DDI:No able to allocate buffer manager, fd=0x%d", mediaCtx->fd);
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        mos_bufmgr_gem_enable_reuse(mediaCtx->pDrmBufMgr);

        // Latency reducation:replace HWGetDeviceID to get device using ioctl from drm.
        mediaCtx->iDeviceId = mos_bufmgr_gem_get_devid(mediaCtx->pDrmBufMgr);

        // TO--DO, apo set it to FALSE by default, to remove the logic in apo mos controlled by it????
        mediaCtx->bIsAtomSOC = IS_ATOMSOC(mediaCtx->iDeviceId);

        MEDIA_FEATURE_TABLE *skuTable = &mediaCtx->SkuTable;
        MEDIA_WA_TABLE *waTable = &mediaCtx->WaTable;
        skuTable->reset();
        waTable->reset();
        // get Sku/Wa tables and platform information
        PLATFORM platform = {};
        // Allocate memory for Media System Info
        mediaCtx->pGtSystemInfo = (MEDIA_SYSTEM_INFO *)MOS_AllocAndZeroMemory(sizeof(MEDIA_SYSTEM_INFO));
        if (nullptr == mediaCtx->pGtSystemInfo)
        {
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        MOS_STATUS eStatus = HWInfo_GetGfxInfo(mediaCtx->fd, mediaCtx->pDrmBufMgr, &platform, skuTable, waTable, mediaCtx->pGtSystemInfo);
        if (MOS_STATUS_SUCCESS != eStatus)
        {
            DDI_ASSERTMESSAGE("Fatal error - unsuccesfull Sku/Wa/GtSystemInfo initialization");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        mediaCtx->platform = platform;

        MOS_TraceSetupInfo(
            (VA_MAJOR_VERSION << 16) | VA_MINOR_VERSION,
            platform.eProductFamily,
            platform.eRenderCoreFamily,
            (platform.usRevId << 16) | platform.usDeviceID);

        // TO--DO, gen12+ still need this wa????, not porting yet
        if (MEDIA_IS_SKU(skuTable, FtrEnableMediaKernels) == 0)
        {
            MEDIA_WR_WA(waTable, WaHucStreamoutOnlyDisable, 0);
        }
        MediaUserSettingsMgr::MediaUserSettingsInit(platform.eProductFamily);

        GMM_SKU_FEATURE_TABLE gmmSkuTable;
        memset(&gmmSkuTable, 0, sizeof(gmmSkuTable));

        GMM_WA_TABLE gmmWaTable;
        memset(&gmmWaTable, 0, sizeof(gmmWaTable));

        GMM_GT_SYSTEM_INFO gmmGtInfo;
        memset(&gmmGtInfo, 0, sizeof(gmmGtInfo));

        eStatus = HWInfo_GetGmmInfo(mediaCtx->fd, &gmmSkuTable, &gmmWaTable, &gmmGtInfo);
        if (MOS_STATUS_SUCCESS != eStatus)
        {
            DDI_ASSERTMESSAGE("Fatal error - unsuccesfull Gmm Sku/Wa/GtSystemInfo initialization");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        eStatus = Mos_Solo_DdiInitializeDeviceId(
            (void *)mediaCtx->pDrmBufMgr,
            &mediaCtx->SkuTable,
            &mediaCtx->WaTable,
            &gmmSkuTable,
            &gmmWaTable,
            &gmmGtInfo,
            &mediaCtx->iDeviceId,
            &mediaCtx->fd,
            &mediaCtx->platform);
        if (eStatus != MOS_STATUS_SUCCESS)
        {
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        GMM_STATUS gmmStatus = OpenGmm(&mediaCtx->GmmFuncs);
        if (gmmStatus != GMM_SUCCESS)
        {
            DDI_ASSERTMESSAGE("gmm init failed.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        // init GMM context
        gmmStatus = mediaCtx->GmmFuncs.pfnCreateSingletonContext(mediaCtx->platform,
                                                                 &gmmSkuTable,
                                                                 &gmmWaTable,
                                                                 &gmmGtInfo);

        if (gmmStatus != GMM_SUCCESS)
        {
            DDI_ASSERTMESSAGE("gmm init failed.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        // Create GMM Client Context
        mediaCtx->pGmmClientContext = mediaCtx->GmmFuncs.pfnCreateClientContext((GMM_CLIENT)GMM_LIBVA_LINUX);

        // Create GMM page table manager
        mediaCtx->m_auxTableMgr = AuxTableMgr::CreateAuxTableMgr(mediaCtx->pDrmBufMgr, &mediaCtx->SkuTable);

        MOS_USER_FEATURE_VALUE_DATA UserFeatureData;
        MOS_ZeroMemory(&UserFeatureData, sizeof(UserFeatureData));
#if (_DEBUG || _RELEASE_INTERNAL)
        MOS_UserFeature_ReadValue_ID(
            nullptr,
            __MEDIA_USER_FEATURE_VALUE_SIM_ENABLE_ID,
            &UserFeatureData,
            nullptr);
#endif

        mediaCtx->m_useSwSwizzling = UserFeatureData.i32Data || MEDIA_IS_SKU(&mediaCtx->SkuTable, FtrUseSwSwizzling);
        mediaCtx->m_tileYFlag = MEDIA_IS_SKU(&mediaCtx->SkuTable, FtrTileY);

        mediaCtx->m_osContext = OsContext::GetOsContextObject();
        if (mediaCtx->m_osContext == nullptr)
        {
            MOS_OS_ASSERTMESSAGE("Unable to get the active OS context.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        // fill in the mos context struct as input to initialize m_osContext
        MOS_CONTEXT mosCtx = {};
        mosCtx.bufmgr = mediaCtx->pDrmBufMgr;
        mosCtx.fd = mediaCtx->fd;
        mosCtx.iDeviceId = mediaCtx->iDeviceId;
        mosCtx.SkuTable = mediaCtx->SkuTable;
        mosCtx.WaTable = mediaCtx->WaTable;
        mosCtx.gtSystemInfo = *mediaCtx->pGtSystemInfo;
        mosCtx.platform = mediaCtx->platform;
        mosCtx.ppMediaMemDecompState = &mediaCtx->pMediaMemDecompState;
        mosCtx.pfnMemoryDecompress = mediaCtx->pfnMemoryDecompress;
        mosCtx.pfnMediaMemoryCopy = mediaCtx->pfnMediaMemoryCopy;
        mosCtx.pfnMediaMemoryCopy2D = mediaCtx->pfnMediaMemoryCopy2D;
        mosCtx.ppMediaCopyState = &mediaCtx->pMediaCopyState;
        mosCtx.m_auxTableMgr = mediaCtx->m_auxTableMgr;
        mosCtx.pGmmClientContext = mediaCtx->pGmmClientContext;

        eStatus = mediaCtx->m_osContext->Init(&mosCtx);
        if (MOS_STATUS_SUCCESS != eStatus)
        {
            MOS_OS_ASSERTMESSAGE("Unable to initialize OS context.");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        // Prepare the command buffer manager
        mediaCtx->m_cmdBufMgr = CmdBufMgr::GetObject();
        if (mediaCtx->m_cmdBufMgr == nullptr)
        {
            MOS_OS_ASSERTMESSAGE(" nullptr returned by CmdBufMgr::GetObject");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        MOS_STATUS ret = mediaCtx->m_cmdBufMgr->Initialize(mediaCtx->m_osContext, COMMAND_BUFFER_SIZE / 2);
        if (ret != MOS_STATUS_SUCCESS)
        {
            MOS_OS_ASSERTMESSAGE(" cmdBufMgr Initialization failed");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        // Prepare the gpu Context manager
        mediaCtx->m_gpuContextMgr = GpuContextMgr::GetObject(mediaCtx->pGtSystemInfo, mediaCtx->m_osContext);
        if (mediaCtx->m_gpuContextMgr == nullptr)
        {
            MOS_OS_ASSERTMESSAGE(" nullptr returned by GpuContextMgr::GetObject");
            FreeForMediaContext(mediaCtx);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }
    else
    {
        MOS_OS_ASSERTMESSAGE(" Invalid mos module state");
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    if (DdiMedia_HeapInitialize(mediaCtx) != VA_STATUS_SUCCESS)
    {
        DestroyMediaContextMutex(mediaCtx);
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
#ifdef CNM_VPUAPI_INTERFACE_CAP
    VpuApiCapInit();
#endif

    // Caps need platform and sku table, especially in MediaLibvaCapsCp::IsDecEncryptionSupported
    mediaCtx->m_caps = MediaLibvaCaps::CreateMediaLibvaCaps(mediaCtx);
    if (!mediaCtx->m_caps)
    {
        DDI_ASSERTMESSAGE("Caps create failed. Not supported GFX device.");
        DestroyMediaContextMutex(mediaCtx);
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    if (mediaCtx->m_caps->Init() != VA_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Caps init failed. Not supported GFX device.");
        MOS_Delete(mediaCtx->m_caps);
        mediaCtx->m_caps = nullptr;
        DestroyMediaContextMutex(mediaCtx);
        FreeForMediaContext(mediaCtx);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
#ifdef CNM_VPUAPI_INTERFACE_CAP
    ctx->max_image_formats = VPUAPI_MAX_IMAGE_FORMATS;
#else
    ctx->max_image_formats = mediaCtx->m_caps->GetImageFormatsMaxNum();
#endif

#if !defined(ANDROID) && defined(X11_FOUND)
    DdiMediaUtil_InitMutex(&mediaCtx->PutSurfaceRenderMutex);
    DdiMediaUtil_InitMutex(&mediaCtx->PutSurfaceSwapBufferMutex);

    // try to open X11 lib, if fail, assume no X11 environment
    if (VA_STATUS_SUCCESS != DdiMedia_ConnectX11(mediaCtx))
    {
        // assume no X11 environment. In current implementation,
        // PutSurface (Linux) needs X11 support, so just replace
        // it with a dummy version. DdiCodec_PutSurfaceDummy() will
        // return VA_STATUS_ERROR_UNIMPLEMENTED directly.
        ctx->vtable->vaPutSurface = NULL;
    }
    output_dri_init(ctx);
#endif

    DdiMediaUtil_SetMediaResetEnableFlag(mediaCtx);

    DdiMediaUtil_UnLockMutex(&GlobalMutex);

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_Terminate(
    VADriverContextP ctx)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DdiMediaUtil_LockMutex(&GlobalMutex);

#if !defined(ANDROID) && defined(X11_FOUND)
    DdiMedia_DestroyX11Connection(mediaCtx);
    DdiMediaUtil_DestroyMutex(&mediaCtx->PutSurfaceRenderMutex);
    DdiMediaUtil_DestroyMutex(&mediaCtx->PutSurfaceSwapBufferMutex);

    if (mediaCtx->m_caps)
    {
        if (mediaCtx->dri_output != nullptr)
        {
            if (mediaCtx->dri_output->handle)
                dso_close(mediaCtx->dri_output->handle);

            free(mediaCtx->dri_output);
            mediaCtx->dri_output = nullptr;
        }
    }
#endif

    if (mediaCtx->m_caps)
    {
        MOS_Delete(mediaCtx->m_caps);
        mediaCtx->m_caps = nullptr;
    }
    // destory resources
    DdiMedia_FreeSurfaceHeapElements(mediaCtx);
    DdiMedia_FreeBufferHeapElements(ctx);
    DdiMedia_FreeImageHeapElements(ctx);
    DdiMedia_FreeContextHeapElements(ctx);
    DdiMedia_FreeContextCMElements(ctx);

    DdiMedia_HeapDestroy(mediaCtx);
    DdiMediaProtected::FreeInstances();

    if (mediaCtx->m_apoMosEnabled)
    {
        MosInterface::DestroyOsDeviceContext(mediaCtx->m_osDeviceContext);
        mediaCtx->m_osDeviceContext = MOS_INVALID_HANDLE;
        MOS_FreeMemory(mediaCtx->pGtSystemInfo);
        MosInterface::CloseOsUtilities(nullptr);
    }
    else if (mediaCtx->modularizedGpuCtxEnabled)
    {
        if (mediaCtx->m_auxTableMgr != nullptr)
        {
            MOS_Delete(mediaCtx->m_auxTableMgr);
            mediaCtx->m_auxTableMgr = nullptr;
        }

        if (mediaCtx->m_gpuContextMgr)
        {
            mediaCtx->m_gpuContextMgr->CleanUp();
            MOS_Delete(mediaCtx->m_gpuContextMgr);
        }

        if (mediaCtx->m_cmdBufMgr)
        {
            mediaCtx->m_cmdBufMgr->CleanUp();
            MOS_Delete(mediaCtx->m_cmdBufMgr);
        }

        if (mediaCtx->m_osContext)
        {
            mediaCtx->m_osContext->CleanUp();
            MOS_Delete(mediaCtx->m_osContext);
        }

        // destroy libdrm buffer manager
        mos_bufmgr_destroy(mediaCtx->pDrmBufMgr);

        // Destroy memory allocated to store Media System Info
        MOS_FreeMemory(mediaCtx->pGtSystemInfo);
        // Free GMM memory.
        mediaCtx->GmmFuncs.pfnDeleteClientContext(mediaCtx->pGmmClientContext);
        mediaCtx->GmmFuncs.pfnDestroySingletonContext();
        MosUtilities::MosUtilitiesClose(nullptr);
    }

    if (mediaCtx->uiRef > 1)
    {
        mediaCtx->uiRef--;
        DdiMediaUtil_UnLockMutex(&GlobalMutex);

        return VA_STATUS_SUCCESS;
    }
    mediaCtx->SkuTable.reset();
    mediaCtx->WaTable.reset();

    // release media driver context, ctx creation is behind the mos_utilities_init
    // If free earilier than MOS_utilities_close, memnja count error.
    MOS_FreeMemory(mediaCtx);
    mediaCtx = nullptr;
    ctx->pDriverData = nullptr;

    DdiMediaUtil_UnLockMutex(&GlobalMutex);

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_QueryConfigEntrypoints(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint *entrypoint_list,
    int32_t *num_entrypoints)
{
    DDI_FUNCTION_ENTER();
    // printf("[CNM_DEBUG]+%s profile=0x%x\n", __FUNCTION__, profile);

    PERF_UTILITY_START_ONCE("First Frame Time", PERF_MOS, PERF_LEVEL_DDI);

    DDI_CHK_NULL(ctx, "nullptr Ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);
    DDI_CHK_NULL(entrypoint_list, "nullptr entrypoint_list", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_entrypoints, "nullptr num_entrypoints", VA_STATUS_ERROR_INVALID_PARAMETER);

    VAStatus status;

    {
        int idx = 0;
        int i;
        int j;
        int32_t num = 0;
        bool find_profile = false;
        status = VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
        *num_entrypoints = 0;
        for (i = 0; i < VPUAPI_MAX_PROFILE; i++)
        {
            if (s_vpuApiCaps[i].profile == VAProfileNone)
            {
                continue;
            }
            if (s_vpuApiCaps[i].profile == profile)
            {
                find_profile = true;
                break;
            }
        }
        if (find_profile == true)
        {
            for (i = 0; i < VPUAPI_MAX_PROFILE; i++)
            {
                if (s_vpuApiCaps[i].profile == profile)
                {
                    for (j = 0; j < s_vpuApiCaps[i].sizeOfEntrypoints; j++)
                    {
                        entrypoint_list[num] = s_vpuApiCaps[i].entrypoint[j];
                        num++;
                    }
                    status = VA_STATUS_SUCCESS;
                    *num_entrypoints = num;
                }
            }
        }
        else
        {
            status = VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
        }
    }
    // printf("[CNM_DEBUG]-%s profile=0x%x, status=0x%x, num_entrypoints=%d\n", __FUNCTION__, profile, status, *num_entrypoints);
    return status;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(entrypoint_list, "nullptr entrypoint_list", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_entrypoints, "nullptr num_entrypoints", VA_STATUS_ERROR_INVALID_PARAMETER);

    return mediaCtx->m_caps->QueryConfigEntrypoints(profile,
                                                    entrypoint_list, num_entrypoints);
#endif
}

VAStatus DdiMedia_QueryConfigProfiles(
    VADriverContextP ctx,
    VAProfile *profile_list,
    int32_t *num_profiles)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr Ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);
    DDI_CHK_NULL(profile_list, "nullptr profile_list", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_profiles, "nullptr num_profiles", VA_STATUS_ERROR_INVALID_PARAMETER);

    VAStatus status;

    {
        int i;
        int32_t num = 0;
        // workaround of vaapi/libva-utils/test/test_va_api_query_config.cpp bug that is not check the size of num_profiles.
        for (i = 0; i < VPUAPI_MAX_PROFILE; i++)
        {
            profile_list[i] = (VAProfile)(VAProfileProtected + 1); // assign unknown profile in default value.
        }
        for (i = 0; i < VPUAPI_MAX_PROFILE; i++)
        {
            if (s_vpuApiCaps[i].profile != VAProfileNone)
            {
                profile_list[num] = s_vpuApiCaps[i].profile;
                num++;
            }
        }
        *num_profiles = num;
        status = VA_STATUS_SUCCESS;
    }

    // printf("[CNM_DEBUG]-%s num_profiles=%d\n", __FUNCTION__, *num_profiles);
    return status;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(profile_list, "nullptr profile_list", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_profiles, "nullptr num_profiles", VA_STATUS_ERROR_INVALID_PARAMETER);

    return mediaCtx->m_caps->QueryConfigProfiles(profile_list, num_profiles);
#endif
}

VAStatus DdiMedia_QueryConfigAttributes(
    VADriverContextP ctx,
    VAConfigID config_id,
    VAProfile *profile,
    VAEntrypoint *entrypoint,
    VAConfigAttrib *attrib_list,
    int32_t *num_attribs)
{
    DDI_FUNCTION_ENTER();

    // printf("[CNM_DEBUG]+%s config_id=0x%x\n", __FUNCTION__, config_id);
    DDI_CHK_NULL(profile, "nullptr profile", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(entrypoint, "nullptr entrypoint", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(ctx, "nullptr Ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(num_attribs, "nullptr num_attribs", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);

    VAStatus status;

    {
        int i;
        int j;
        int num = 0;
        *profile = VAProfileNone;
        for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
        {
            if (s_vpuApiAttrs[i].configId == config_id)
            {
                if (s_vpuApiAttrs[i].actual_profile != VAProfileNone)
                {
                    *profile = s_vpuApiAttrs[i].actual_profile;
                    *entrypoint = s_vpuApiAttrs[i].actual_entrypoint;
                    // printf("[CNM_DEBUG]     >>> i=%d, num=%d,   profile=0x%x, entrypoint=0x%x, attrType=0x%x, attrValue=0x%x\n", i, num, *profile, *entrypoint, s_vpuApiAttrs[i].attrType, s_vpuApiAttrs[i].value);
                    attrib_list[num].type = s_vpuApiAttrs[i].attrType;
                    attrib_list[num].value = s_vpuApiAttrs[i].value;
                    num++;
                    break;
                }
            }
        }
        *num_attribs = num;
    }
    status = VA_STATUS_SUCCESS;
    // printf("[CNM_DEBUG]-%s profile=0x%x, entrypoint=0x%x, num_attribs=%d\n", __FUNCTION__, *profile, *entrypoint, *num_attribs);
    return status;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

    return mediaCtx->m_caps->QueryConfigAttributes(
        config_id, profile, entrypoint, attrib_list, num_attribs);
#endif
}
VAStatus DdiMedia_CreateConfig(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint entrypoint,
    VAConfigAttrib *attrib_list,
    int32_t num_attribs,
    VAConfigID *config_id)
{
    DDI_FUNCTION_ENTER();

#ifdef CNM_VPUAPI_INTERFACE_CAP
    VAStatus status;
#endif
    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(config_id, "nullptr config_id", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);

    {
        int i;
        int j;
        int found_config_id = 0xffffffff;
        int found_attrib_num = 0;
        bool find_profile = false;
        bool find_entrypoint = false;

        for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
        {
            if (s_vpuApiAttrs[i].actual_profile == VAProfileNone)
            {
                continue;
            }
            if (s_vpuApiAttrs[i].actual_profile == profile)
            {
                // printf("[CNM_DEBUG]  %s actual_profile[%d]=%d, profile=%d, actual_entrypoint[%d]=0x%x, entrypoint=0x%x \n", __FUNCTION__, i, s_vpuApiAttrs[i].actual_profile, profile, i, s_vpuApiAttrs[i].actual_entrypoint, entrypoint);
                find_profile = true;
                if (s_vpuApiAttrs[i].actual_entrypoint == entrypoint)
                {
                    find_entrypoint = true;
                    bool is_valid_attr = false;
                    // printf("[CNM_DEBUG]  %s idx=%d, profile actual_profile=%d, actual_entrypoint=0x%x \n", __FUNCTION__, i, s_vpuApiAttrs[i].actual_profile, s_vpuApiAttrs[i].actual_entrypoint);
                    for (j = 0; j < num_attribs; j++)
                    {
                        // printf("[CNM_DEBUG]  %s try find_valid_attr jdx=%d, type=0x%x, value=0x%x, type_list=0x%x, value_list=0x%x \n", __FUNCTION__, j, s_vpuApiAttrs[i].attrType, s_vpuApiAttrs[i].value, attrib_list[j].type, attrib_list[j].value);
                        if (attrib_list[j].type == s_vpuApiAttrs[i].attrType)
                        {
                            // printf("[CNM_DEBUG]  %s found_valid_attr jdx=%d, type=0x%x, value=0x%x, type_list=0x%x, value_list=0x%x \n", __FUNCTION__, j, s_vpuApiAttrs[i].attrType, s_vpuApiAttrs[i].value, attrib_list[j].type, attrib_list[j].value);

                            if (attrib_list[j].value == 0 /* Define for empty attrib */)
                            {
                                is_valid_attr = true;
                                break;
                            }
                            if (attrib_list[j].type == VAConfigAttribRTFormat ||
                                attrib_list[j].type == VAConfigAttribDecSliceMode ||
                                attrib_list[j].type == VAConfigAttribRateControl ||
                                attrib_list[j].type == VAConfigAttribEncPackedHeaders ||
                                attrib_list[j].type == VAConfigAttribEncIntraRefresh ||
                                attrib_list[j].type == VAConfigAttribFEIFunctionType ||
                                attrib_list[j].type == VAConfigAttribEncInterlaced)
                            {
                                if ((s_vpuApiAttrs[i].value & attrib_list[j].value) == attrib_list[j].value)
                                {
                                    is_valid_attr = true;
                                    break;
                                }
                                else
                                {
                                    if (attrib_list[j].type == VAConfigAttribRTFormat)
                                    {
                                        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
                                    }
                                    else
                                    {
                                        return VA_STATUS_ERROR_INVALID_VALUE;
                                    }
                                }
                            }
                            else if ((s_vpuApiAttrs[i].value & attrib_list[j].value) == attrib_list[j].value)
                            {
                                is_valid_attr = true;
                                break;
                            }
                            else if (attrib_list[j].type == VAConfigAttribEncSliceStructure)
                            {
                                if ((s_vpuApiAttrs[i].value & attrib_list[j].value) == attrib_list[j].value)
                                {
                                    is_valid_attr = true;
                                    break;
                                }

                                if ((s_vpuApiAttrs[i].value & VA_ENC_SLICE_STRUCTURE_ARBITRARY_MACROBLOCKS))
                                {
                                    if ((attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_EQUAL_ROWS) || (attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_EQUAL_MULTI_ROWS) || (attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_POWER_OF_TWO_ROWS) || (attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_ARBITRARY_ROWS))
                                    {
                                        if (profile == VAProfileH264Main || profile == VAProfileH264High || profile == VAProfileH264ConstrainedBaseline || profile == VAProfileH264High10)
                                        {
                                        }
                                        else
                                        {
                                            is_valid_attr = true;
                                            break;
                                        }
                                    }
                                }
                                else if ((s_vpuApiAttrs[i].value & (VA_ENC_SLICE_STRUCTURE_EQUAL_ROWS | VA_ENC_SLICE_STRUCTURE_MAX_SLICE_SIZE)))
                                {
                                    if ((attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_ARBITRARY_MACROBLOCKS) || (attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_POWER_OF_TWO_ROWS) || (attrib_list[j].value & VA_ENC_SLICE_STRUCTURE_ARBITRARY_ROWS))
                                    {
                                        if (profile == VAProfileH264Main || profile == VAProfileH264High || profile == VAProfileH264ConstrainedBaseline || profile == VAProfileH264High10)
                                        {
                                        }
                                        else
                                        {
                                            is_valid_attr = true;
                                            break;
                                        }
                                    }
                                }
                            }
                            else if ((attrib_list[j].type == VAConfigAttribMaxPictureWidth) || (attrib_list[j].type == VAConfigAttribMaxPictureHeight) || (attrib_list[j].type == VAConfigAttribEncROI) || (attrib_list[j].type == VAConfigAttribEncDirtyRect))
                            {
                                if (attrib_list[j].value <= s_vpuApiAttrs[i].value)
                                {
                                    is_valid_attr = true;
                                    break;
                                }
                                else
                                {
                                    return VA_STATUS_ERROR_INVALID_VALUE;
                                }
                            }
                            else if (attrib_list[j].type == VAConfigAttribEncMaxRefFrames)
                            {
                                if (((attrib_list[j].value & 0xffff) <= (s_vpuApiAttrs[i].value & 0xffff)) && (attrib_list[j].value <= s_vpuApiAttrs[i].value))
                                { // high16 bit  can compare with this way
                                    is_valid_attr = true;
                                    break;
                                }
                            }
                            else if (attrib_list[j].type == VAConfigAttribEncJPEG)
                            {
                                // not supported.
                            }
                        }
                    }

                    // num_attrbis is 0 means user wants to get config_id without matching flavor attribute.
                    if (num_attribs == 0)
                    {
                        is_valid_attr = true;
                        found_config_id = s_vpuApiAttrs[i].configId;
                        break;
                    }

                    if (is_valid_attr == false)
                    {
                        // printf("[CNM_DEBUG]%s can't find valid attribute\n", __FUNCTION__);
                    }
                    else
                    {
                        // printf("[CNM_DEBUG]%s can find valid attribute config_id=0x%x, profile=%d, entrypoint=0x%x\n", __FUNCTION__, s_vpuApiAttrs[i].configId, s_vpuApiAttrs[i].actual_profile, s_vpuApiAttrs[i].actual_entrypoint);
                        found_config_id = s_vpuApiAttrs[i].configId;
                        if (s_vpuApiAttrs[i].actual_entrypoint == VAEntrypointEncSlice || s_vpuApiAttrs[i].actual_entrypoint == VAEntrypointEncSliceLP)
                        {
                            if (attrib_list[j].type == VAConfigAttribRateControl)
                            {
#ifdef CNM_VPUAPI_INTERFACE
                                mediaCtx->rcMode = attrib_list[j].value;
#endif
                            }
                        }
#ifdef USE_INTEL_CONFIG_ID
                        if (attrib_list[j].type == VAConfigAttribRateControl && attrib_list[j].value == 0x10)
                        {
                            found_config_id = 0x42e;
                        }
#endif
                        found_attrib_num++;
                        if (found_attrib_num == num_attribs)
                            break;
                    }
                }
            }
        }

        if (find_profile == false)
        {
            *config_id = 0xffffffff;
            status = VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
        }
        else
        {
            if (find_entrypoint == false)
            {
                *config_id = 0xffffffff;
                status = VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
            }
            else
            {
                *config_id = found_config_id;
                if (*config_id == 0xffffffff)
                {
                    // printf("[CNM_DEBUG]-%s configid=0x%x, status=0x%x VA_STATUS_ERROR_INVALID_VALUE\n", __FUNCTION__, *config_id, VA_STATUS_ERROR_INVALID_VALUE);
                    status = VA_STATUS_ERROR_INVALID_VALUE;
                }
                else
                {
                    status = VA_STATUS_SUCCESS;
                }
            }
        }
    }
    // printf("[CNM_DEBUG]-%s profile=%d, entrypoint=0x%x, status=0x%x, config_id=0x%x\n", __FUNCTION__, profile, entrypoint, status, *config_id);

    if (*config_id >= VPUAPI_ENCODER_CONFIG_ID_START &&
        *config_id < VPUAPI_MAX_CONFIG_ID)
    {
#ifdef CNM_VPUAPI_INTERFACE
        mediaCtx->coreIdx = 1;
#endif
    }
    else
    {
#ifdef CNM_VPUAPI_INTERFACE
        mediaCtx->coreIdx = 0;
#endif
    }
#ifdef CNM_FPGA_PLATFORM
    mediaCtx->coreIdx = 0;
#endif

    return status;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

    VAStatus status;
    status = mediaCtx->m_caps->CreateConfig(
        profile, entrypoint, attrib_list, num_attribs, config_id);
    // printf("[CNM_DEBUG]-%s profile=0x%x, entrypoint=0x%x, status=0x%x, config_id=0x%x\n", __FUNCTION__, profile, entrypoint, status, *config_id);
    return status;

#endif
}

VAStatus DdiMedia_DestroyConfig(
    VADriverContextP ctx,
    VAConfigID config_id)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);

    {
        int i;
        bool found_valid_config_id = false;
        VAStatus status;
        for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
        {
            if (s_vpuApiAttrs[i].configId == config_id)
            {
                found_valid_config_id = true;
                break;
            }
        }
        if (found_valid_config_id == true)
        {
            status = VA_STATUS_SUCCESS;
        }
        else
        {
            status = VA_STATUS_ERROR_INVALID_CONFIG;
        }
        return status;
    }
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

    return mediaCtx->m_caps->DestroyConfig(config_id);
#endif
}

VAStatus DdiMedia_GetConfigAttributes(
    VADriverContextP ctx,
    VAProfile profile,
    VAEntrypoint entrypoint,
    VAConfigAttrib *attrib_list,
    int32_t num_attribs)
{
    DDI_FUNCTION_ENTER();

    printf("+%s profile=0x%x, entrypoint=0x%x, num_attribs=%d\n", __FUNCTION__, profile, entrypoint, num_attribs);
    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);

    VAStatus status;

    {
        int i;
        int j;
        bool find_profile = false;
        bool find_entrypoint = false;
        for (i = 0; i < VPUAPI_MAX_ATTRIBUTE; i++)
        {
            if (s_vpuApiAttrs[i].actual_profile == VAProfileNone)
            {
                continue;
            }
            if (s_vpuApiAttrs[i].actual_profile == profile)
            {
                find_profile = true;
                if (s_vpuApiAttrs[i].actual_entrypoint == entrypoint)
                {
                    find_entrypoint = true;
                }
            }
        }
        if (find_profile == true && find_entrypoint == true)
        {
            for (i = 0; i < num_attribs; i++)
            {
                for (j = 0; j < VPUAPI_MAX_ATTRIBUTE; j++)
                {
                    // printf("+%s j=%d, profile=0x%x, entrypoint=0x%x\n", __FUNCTION__, j, s_vpuApiAttrs[j].actual_profile, s_vpuApiAttrs[j].actual_entrypoint);
                    if (s_vpuApiAttrs[j].actual_profile == VAProfileNone)
                    {
                        continue;
                    }
                    if ((s_vpuApiAttrs[j].actual_profile == profile) && (s_vpuApiAttrs[j].actual_entrypoint == entrypoint))
                    {
                        if (attrib_list[i].type == s_vpuApiAttrs[j].attrType)
                        {
                            // printf("%s find i=%d, j=%d, profile=0x%x, entrypoint=0x%x, type1=0x%x, type2=0x%x \n", __FUNCTION__, i, j, profile, entrypoint, attrib_list[i].type, s_vpuApiAttrs[j].attrType);
                            attrib_list[i].value = s_vpuApiAttrs[j].value;
                            break;
                        }
                        else
                        {
                            attrib_list[i].value = VA_ATTRIB_NOT_SUPPORTED;
                        }
                    }
                }
            }
            status = VA_STATUS_SUCCESS;
        }
        else if (find_profile == false)
        {
            status = VA_STATUS_ERROR_UNSUPPORTED_PROFILE; // if there is no matched profile in all attributes
        }
        else
        {
            status = VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT; // if there is no matched profile in all attributes
        }
    }
    printf("-%s status=0x%x\n", __FUNCTION__, status);
    return status;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

    return mediaCtx->m_caps->GetConfigAttributes(
        profile, entrypoint, attrib_list, num_attribs);
#endif
}

VAStatus DdiMedia_CreateSurfaces(
    VADriverContextP ctx,
    int32_t width,
    int32_t height,
    int32_t format,
    int32_t num_surfaces,
    VASurfaceID *surfaces)
{
    DDI_FUNCTION_ENTER();
    int32_t event[] = {width, height, format};
    MOS_TraceEventExt(EVENT_VA_SURFACE, EVENT_TYPE_START, event, sizeof(event), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LARGER(num_surfaces, 0, "Invalid num_surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(surfaces, "nullptr surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(width, 0, "Invalid width", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(height, 0, "Invalid height", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaDrvCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaDrvCtx, "nullptr mediaDrvCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    if (format != VA_RT_FORMAT_YUV420 ||
        format != VA_RT_FORMAT_YUV422 ||
        format != VA_RT_FORMAT_YUV444 ||
        format != VA_RT_FORMAT_YUV400 ||
        format != VA_RT_FORMAT_YUV411)
    {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    DDI_MEDIA_FORMAT mediaFmt = DdiMedia_OsFormatToMediaFormat(VA_FOURCC_NV12, format);

    height = MOS_ALIGN_CEIL(height, 16);
    for (int32_t i = 0; i < num_surfaces; i++)
    {
        VASurfaceID vaSurfaceID = (VASurfaceID)DdiMedia_CreateRenderTarget(mediaDrvCtx, mediaFmt, width, height, nullptr, VA_SURFACE_ATTRIB_USAGE_HINT_GENERIC, MOS_MEMPOOL_VIDEOMEMORY);
        if (VA_INVALID_ID != vaSurfaceID)
            surfaces[i] = vaSurfaceID;
        else
        {
            // here to release the successful allocated surfaces?
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
    }

    MOS_TraceEventExt(EVENT_VA_SURFACE, EVENT_TYPE_END, &num_surfaces, sizeof(int32_t), surfaces, num_surfaces * sizeof(VAGenericID));
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_DestroySurfaces(
    VADriverContextP ctx,
    VASurfaceID *surfaces,
    int32_t num_surfaces)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LARGER(num_surfaces, 0, "Invalid num_surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(surfaces, "nullptr surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    MOS_TraceEventExt(EVENT_VA_FREE_SURFACE, EVENT_TYPE_START, &num_surfaces, sizeof(int32_t), surfaces, num_surfaces * sizeof(VAGenericID));

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_SURFACE surface = nullptr;
    for (int32_t i = 0; i < num_surfaces; i++)
    {
        DDI_CHK_LESS((uint32_t)surfaces[i], mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surfaces", VA_STATUS_ERROR_INVALID_SURFACE);
        surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surfaces[i]);
        DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);
        if (surface->pCurrentFrameSemaphore)
        {
            DdiMediaUtil_WaitSemaphore(surface->pCurrentFrameSemaphore);
            DdiMediaUtil_PostSemaphore(surface->pCurrentFrameSemaphore);
        }
        if (surface->pReferenceFrameSemaphore)
        {
            DdiMediaUtil_WaitSemaphore(surface->pReferenceFrameSemaphore);
            DdiMediaUtil_PostSemaphore(surface->pReferenceFrameSemaphore);
        }
    }

    for (int32_t i = 0; i < num_surfaces; i++)
    {
        DDI_CHK_LESS((uint32_t)surfaces[i], mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surfaces", VA_STATUS_ERROR_INVALID_SURFACE);
        surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surfaces[i]);
        DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);
        if (surface->pCurrentFrameSemaphore)
        {
            DdiMediaUtil_DestroySemaphore(surface->pCurrentFrameSemaphore);
            surface->pCurrentFrameSemaphore = nullptr;
        }

        if (surface->pReferenceFrameSemaphore)
        {
            DdiMediaUtil_DestroySemaphore(surface->pReferenceFrameSemaphore);
            surface->pReferenceFrameSemaphore = nullptr;
        }

        DdiMediaUtil_UnRegisterRTSurfaces(ctx, surface);

        DdiMediaUtil_LockMutex(&mediaCtx->SurfaceMutex);
        DdiMediaUtil_FreeSurface(surface);
        MOS_FreeMemory(surface);
        DdiMediaUtil_ReleasePMediaSurfaceFromHeap(mediaCtx->pSurfaceHeap, (uint32_t)surfaces[i]);
        mediaCtx->uiNumSurfaces--;
        DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);
#ifdef CNM_VPUAPI_INTERFACE
        VpuApiRemoveSurfaceInfo(mediaCtx, surfaces[i]);
#endif
    }

    MOS_TraceEventExt(EVENT_VA_FREE_SURFACE, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_CreateSurfaces2(
    VADriverContextP ctx,
    uint32_t format,
    uint32_t width,
    uint32_t height,
    VASurfaceID *surfaces,
    uint32_t num_surfaces,
    VASurfaceAttrib *attrib_list,
    uint32_t num_attribs)
{
    DDI_FUNCTION_ENTER();
    uint32_t event[] = {width, height, format};
    MOS_TraceEventExt(EVENT_VA_SURFACE, EVENT_TYPE_START, event, sizeof(event), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LARGER(num_surfaces, 0, "Invalid num_surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(surfaces, "nullptr surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(width, 0, "Invalid width", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(height, 0, "Invalid height", VA_STATUS_ERROR_INVALID_PARAMETER);

    if (num_attribs > 0)
    {
        DDI_CHK_NULL(attrib_list, "nullptr attrib_list", VA_STATUS_ERROR_INVALID_PARAMETER);
    }

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    int32_t expected_fourcc = VA_FOURCC_NV12;
    switch (format)
    {
    case VA_RT_FORMAT_YUV420:
        expected_fourcc = VA_FOURCC_NV12;
        break;
    case VA_RT_FORMAT_YUV420_12:
        expected_fourcc = VA_FOURCC_P012;
        break;
    case VA_RT_FORMAT_YUV422:
        expected_fourcc = VA_FOURCC_YUY2;
        break;
    case VA_RT_FORMAT_YUV422_10:
        expected_fourcc = VA_FOURCC_Y210;
        break;
    case VA_RT_FORMAT_YUV422_12:
#if VA_CHECK_VERSION(1, 9, 0)
        expected_fourcc = VA_FOURCC_Y212;
#else
        expected_fourcc = VA_FOURCC_Y216;
#endif
        break;
    case VA_RT_FORMAT_YUV444:
        expected_fourcc = VA_FOURCC_444P;
        break;
    case VA_RT_FORMAT_YUV444_10:
        expected_fourcc = VA_FOURCC_Y410;
        break;
    case VA_RT_FORMAT_YUV444_12:
#if VA_CHECK_VERSION(1, 9, 0)
        expected_fourcc = VA_FOURCC_Y412;
#else
        expected_fourcc = VA_FOURCC_Y416;
#endif
        break;
    case VA_RT_FORMAT_YUV411:
        expected_fourcc = VA_FOURCC_411P;
        break;
    case VA_RT_FORMAT_YUV400:
        expected_fourcc = VA_FOURCC('4', '0', '0', 'P');
        break;
    case VA_RT_FORMAT_YUV420_10BPP:
        expected_fourcc = VA_FOURCC_P010;
        break;
    case VA_RT_FORMAT_RGB16:
        expected_fourcc = VA_FOURCC_R5G6B5;
        break;
    case VA_RT_FORMAT_RGB32:
        expected_fourcc = VA_FOURCC_BGRA;
        break;
    case VA_RT_FORMAT_RGBP:
        expected_fourcc = VA_FOURCC_RGBP;
        break;
#ifdef VA_RT_FORMAT_RGB32_10BPP
    case VA_RT_FORMAT_RGB32_10BPP:
        expected_fourcc = VA_FOURCC_BGRA;
        break;
#endif
#if 1 // added for having MDF sanity test pass, will be removed after MDF formal patch checked in
    case VA_FOURCC_NV12:
        expected_fourcc = VA_FOURCC_NV12;
        break;
    case VA_FOURCC_NV21:
        expected_fourcc = VA_FOURCC_NV21;
        break;
    case VA_FOURCC_ABGR:
        expected_fourcc = VA_FOURCC_ABGR;
        break;
    case VA_FOURCC_ARGB:
        expected_fourcc = VA_FOURCC_ARGB;
        break;
    case VA_FOURCC_XBGR:
        expected_fourcc = VA_FOURCC_XBGR;
        break;
    case VA_FOURCC_XRGB:
        expected_fourcc = VA_FOURCC_XRGB;
        break;
    case VA_FOURCC_R5G6B5:
        expected_fourcc = VA_FOURCC_R5G6B5;
        break;
    case VA_FOURCC_R8G8B8:
        expected_fourcc = VA_FOURCC_R8G8B8;
        break;
    case VA_FOURCC_YUY2:
        expected_fourcc = VA_FOURCC_YUY2;
        break;
    case VA_FOURCC_YV12:
        expected_fourcc = VA_FOURCC_YV12;
        break;
    case VA_FOURCC_422H:
        expected_fourcc = VA_FOURCC_422H;
        break;
    case VA_FOURCC_422V:
        expected_fourcc = VA_FOURCC_422V;
        break;
    case VA_FOURCC_P208:
        expected_fourcc = VA_FOURCC_P208;
        break;
    case VA_FOURCC_P010:
        expected_fourcc = VA_FOURCC_P010;
        break;
    case VA_FOURCC_P012:
        expected_fourcc = VA_FOURCC_P012;
        break;
    case VA_FOURCC_P016:
        expected_fourcc = VA_FOURCC_P016;
        break;
    case VA_FOURCC_Y210:
        expected_fourcc = VA_FOURCC_Y210;
        break;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y212:
        expected_fourcc = VA_FOURCC_Y212;
        break;
#endif
    case VA_FOURCC_Y216:
        expected_fourcc = VA_FOURCC_Y216;
        break;
    case VA_FOURCC_AYUV:
        expected_fourcc = VA_FOURCC_AYUV;
        break;
    case VA_FOURCC_Y410:
        expected_fourcc = VA_FOURCC_Y410;
        break;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y412:
        expected_fourcc = VA_FOURCC_Y412;
        break;
#endif
    case VA_FOURCC_Y416:
        expected_fourcc = VA_FOURCC_Y416;
        break;
    case VA_FOURCC_I420:
        expected_fourcc = VA_FOURCC_I420;
        break;
#endif
    default:
        DDI_ASSERTMESSAGE("Invalid VAConfigAttribRTFormat: 0x%x. Please uses the format defined in libva/va.h", format);
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    VASurfaceAttribExternalBuffers externalBufDescripor;
    VADRMPRIMESurfaceDescriptor drmPrimeSurfaceDescriptor;
    MOS_ZeroMemory(&externalBufDescripor, sizeof(VASurfaceAttribExternalBuffers));
    MOS_ZeroMemory(&drmPrimeSurfaceDescriptor, sizeof(VADRMPRIMESurfaceDescriptor));

    int32_t memTypeFlag = VA_SURFACE_ATTRIB_MEM_TYPE_VA;
    int32_t descFlag = 0;
    uint32_t surfaceUsageHint = VA_SURFACE_ATTRIB_USAGE_HINT_GENERIC;
    bool surfDescProvided = false;
    bool surfIsUserPtr = false;

    for (uint32_t i = 0; i < num_attribs && attrib_list; i++)
    {
        if (attrib_list[i].flags & VA_SURFACE_ATTRIB_SETTABLE)
        {
            switch (attrib_list[i].type)
            {
            case VASurfaceAttribUsageHint:
                DDI_ASSERT(attrib_list[i].value.type == VAGenericValueTypeInteger);
                surfaceUsageHint = attrib_list[i].value.value.i;
                break;
            case VASurfaceAttribPixelFormat:
                DDI_ASSERT(attrib_list[i].value.type == VAGenericValueTypeInteger);
                expected_fourcc = attrib_list[i].value.value.i;
                break;
            case VASurfaceAttribMemoryType:
                DDI_ASSERT(attrib_list[i].value.type == VAGenericValueTypeInteger);
                if ((attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_VA) ||
                    (attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM) ||
                    (attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) ||
                    (attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2) ||
                    (attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_USER_PTR))
                {
                    memTypeFlag = attrib_list[i].value.value.i;
                    surfIsUserPtr = (attrib_list[i].value.value.i == VA_SURFACE_ATTRIB_MEM_TYPE_USER_PTR);
                }
                else
                {
                    DDI_ASSERTMESSAGE("Not supported external buffer type.");
                    return VA_STATUS_ERROR_INVALID_PARAMETER;
                }
                break;
            case (VASurfaceAttribType)VASurfaceAttribExternalBufferDescriptor:
                DDI_ASSERT(attrib_list[i].value.type == VAGenericValueTypePointer);
                if (nullptr == attrib_list[i].value.value.p)
                {
                    DDI_ASSERTMESSAGE("Invalid VASurfaceAttribExternalBuffers used.");
                    // remove the check for libva-utils conformance test, need libva-utils change cases
                    // after libva-utils fix the case, return VA_STATUS_ERROR_INVALID_PARAMETER;
                    break;
                }
                surfDescProvided = true;
                if (memTypeFlag == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
                {
                    MOS_SecureMemcpy(&drmPrimeSurfaceDescriptor, sizeof(VADRMPRIMESurfaceDescriptor), attrib_list[i].value.value.p, sizeof(VADRMPRIMESurfaceDescriptor));
                    expected_fourcc = drmPrimeSurfaceDescriptor.fourcc;
                    width = drmPrimeSurfaceDescriptor.width;
                    height = drmPrimeSurfaceDescriptor.height;
                }
                else
                {
                    MOS_SecureMemcpy(&externalBufDescripor, sizeof(VASurfaceAttribExternalBuffers), attrib_list[i].value.value.p, sizeof(VASurfaceAttribExternalBuffers));

                    expected_fourcc = externalBufDescripor.pixel_format;
                    width = externalBufDescripor.width;
                    height = externalBufDescripor.height;
                    // the following code is for backward compatible and it will be removed in the future
                    // new implemention should use VASurfaceAttribMemoryType attrib and set its value to VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM
                    if ((externalBufDescripor.flags & VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM) ||
                        (externalBufDescripor.flags & VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) ||
                        (externalBufDescripor.flags & VA_SURFACE_EXTBUF_DESC_PROTECTED) ||
                        (externalBufDescripor.flags & VA_SURFACE_EXTBUF_DESC_ENABLE_TILING))
                    {
                        if (externalBufDescripor.flags & VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM)
                        {
                            memTypeFlag = VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM;
                        }
                        else if (externalBufDescripor.flags & VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME)
                        {
                            memTypeFlag = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
                        }

                        descFlag = (externalBufDescripor.flags & ~(VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM | VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME));
                        surfIsUserPtr = false;
                    }
                }

                break;
            default:
                DDI_ASSERTMESSAGE("Unsupported type.");
                break;
            }
        }
    }

    DDI_MEDIA_FORMAT mediaFmt = DdiMedia_OsFormatToMediaFormat(expected_fourcc, format);
    if (mediaFmt == Media_Format_Count)
    {
        DDI_ASSERTMESSAGE("DDI: unsupported surface type in DdiMedia_CreateSurfaces2.");
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }

    for (uint32_t i = 0; i < num_surfaces; i++)
    {
        PDDI_MEDIA_SURFACE_DESCRIPTOR surfDesc = nullptr;
        MOS_STATUS eStatus = MOS_STATUS_SUCCESS;

        if (surfDescProvided == true)
        {
            surfDesc = (PDDI_MEDIA_SURFACE_DESCRIPTOR)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_SURFACE_DESCRIPTOR));
            if (surfDesc == nullptr)
            {
                return VA_STATUS_ERROR_ALLOCATION_FAILED;
            }
            surfDesc->uiFlags = descFlag;
            surfDesc->uiVaMemType = memTypeFlag;

            if (memTypeFlag == VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
            {
                surfDesc->ulBuffer = drmPrimeSurfaceDescriptor.objects[0].fd;
                surfDesc->modifier = drmPrimeSurfaceDescriptor.objects[0].drm_format_modifier;
                surfDesc->uiSize = drmPrimeSurfaceDescriptor.objects[0].size;
                surfDesc->uiBuffserSize = drmPrimeSurfaceDescriptor.objects[0].size;
                surfDesc->uiPlanes = drmPrimeSurfaceDescriptor.layers[0].num_planes;
                eStatus = MOS_SecureMemcpy(surfDesc->uiPitches, sizeof(surfDesc->uiPitches), drmPrimeSurfaceDescriptor.layers[0].pitch, sizeof(drmPrimeSurfaceDescriptor.layers[0].pitch));
                if (eStatus != MOS_STATUS_SUCCESS)
                {
                    DDI_VERBOSEMESSAGE("DDI:Failed to copy surface buffer data!");
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }
                eStatus = MOS_SecureMemcpy(surfDesc->uiOffsets, sizeof(surfDesc->uiOffsets), drmPrimeSurfaceDescriptor.layers[0].offset, sizeof(drmPrimeSurfaceDescriptor.layers[0].offset));
                if (eStatus != MOS_STATUS_SUCCESS)
                {
                    DDI_VERBOSEMESSAGE("DDI:Failed to copy surface buffer data!");
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }
            }
            else if (memTypeFlag != VA_SURFACE_ATTRIB_MEM_TYPE_VA)
            {
                surfDesc->uiPlanes = externalBufDescripor.num_planes;
                surfDesc->ulBuffer = externalBufDescripor.buffers[i];
                surfDesc->uiSize = externalBufDescripor.data_size;
                surfDesc->uiBuffserSize = externalBufDescripor.data_size;

                eStatus = MOS_SecureMemcpy(surfDesc->uiPitches, sizeof(surfDesc->uiPitches), externalBufDescripor.pitches, sizeof(externalBufDescripor.pitches));
                if (eStatus != MOS_STATUS_SUCCESS)
                {
                    DDI_VERBOSEMESSAGE("DDI:Failed to copy surface buffer data!");
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }
                eStatus = MOS_SecureMemcpy(surfDesc->uiOffsets, sizeof(surfDesc->uiOffsets), externalBufDescripor.offsets, sizeof(externalBufDescripor.offsets));
                if (eStatus != MOS_STATUS_SUCCESS)
                {
                    DDI_VERBOSEMESSAGE("DDI:Failed to copy surface buffer data!");
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }

                if (surfIsUserPtr)
                {
                    surfDesc->uiTile = I915_TILING_NONE;
                    if (surfDesc->ulBuffer % 4096 != 0)
                    {
                        MOS_FreeMemory(surfDesc);
                        DDI_VERBOSEMESSAGE("Buffer Address is invalid");
                        return VA_STATUS_ERROR_INVALID_PARAMETER;
                    }
                }
            }
        }
        VASurfaceID vaSurfaceID = (VASurfaceID)DdiMedia_CreateRenderTarget(mediaCtx, mediaFmt, width, height, surfDesc, surfaceUsageHint, MOS_MEMPOOL_VIDEOMEMORY);
        if (VA_INVALID_ID != vaSurfaceID)
        {
            surfaces[i] = vaSurfaceID;
        }
        else
        {
            // here to release the successful allocated surfaces?
            if (nullptr != surfDesc)
            {
                MOS_FreeMemory(surfDesc);
            }
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
#ifdef CNM_VPUAPI_INTERFACE
        VpuApiAddSurfaceInfo(mediaCtx, vaSurfaceID, width, height);
#endif
    }

    MOS_TraceEventExt(EVENT_VA_SURFACE, EVENT_TYPE_END, &num_surfaces, sizeof(uint32_t), surfaces, num_surfaces * sizeof(VAGenericID));
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_CreateMfeContextInternal(
    VADriverContextP ctx,
    VAMFContextID *mfe_context)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaDrvCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaDrvCtx, "nullptr pMediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mfe_context, "nullptr mfe_context", VA_STATUS_ERROR_INVALID_PARAMETER);
    *mfe_context = DDI_MEDIA_INVALID_VACONTEXTID;

    if (!mediaDrvCtx->m_caps->IsMfeSupportedOnPlatform(mediaDrvCtx->platform))
    {
        DDI_VERBOSEMESSAGE("MFE is not supported on the platform!");
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }

    PDDI_ENCODE_MFE_CONTEXT encodeMfeContext = (PDDI_ENCODE_MFE_CONTEXT)MOS_AllocAndZeroMemory(sizeof(DDI_ENCODE_MFE_CONTEXT));
    if (nullptr == encodeMfeContext)
    {
        MOS_FreeMemory(encodeMfeContext);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    DdiMediaUtil_LockMutex(&mediaDrvCtx->MfeMutex);
    PDDI_MEDIA_VACONTEXT_HEAP_ELEMENT vaContextHeapElmt = DdiMediaUtil_AllocPVAContextFromHeap(mediaDrvCtx->pMfeCtxHeap);
    if (nullptr == vaContextHeapElmt)
    {
        DdiMediaUtil_UnLockMutex(&mediaDrvCtx->MfeMutex);
        MOS_FreeMemory(encodeMfeContext);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }

    vaContextHeapElmt->pVaContext = (void *)encodeMfeContext;
    mediaDrvCtx->uiNumMfes++;
    *mfe_context = (VAMFContextID)(vaContextHeapElmt->uiVaContextID + DDI_MEDIA_VACONTEXTID_OFFSET_MFE);
    DdiMediaUtil_UnLockMutex(&mediaDrvCtx->MfeMutex);

    // Create shared state, which is used by all the sub contexts
    MfeSharedState *mfeEncodeSharedState = (MfeSharedState *)MOS_AllocAndZeroMemory(sizeof(MfeSharedState));
    if (nullptr == mfeEncodeSharedState)
    {
        MOS_FreeMemory(encodeMfeContext);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    encodeMfeContext->mfeEncodeSharedState = mfeEncodeSharedState;

    DdiMediaUtil_InitMutex(&encodeMfeContext->encodeMfeMutex);

    return VA_STATUS_SUCCESS;
}

static VAStatus DdiMedia_DestoryMfeContext(
    VADriverContextP ctx,
    VAMFContextID mfe_context)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    PDDI_ENCODE_MFE_CONTEXT encodeMfeContext = (PDDI_ENCODE_MFE_CONTEXT)DdiMedia_GetContextFromContextID(ctx, mfe_context, &ctxType);
    DDI_CHK_NULL(encodeMfeContext, "nullptr encodeMfeContext", VA_STATUS_ERROR_INVALID_CONTEXT);

    // Release std::vector memory
    encodeMfeContext->pDdiEncodeContexts.clear();
    encodeMfeContext->pDdiEncodeContexts.shrink_to_fit();

    encodeMfeContext->mfeEncodeSharedState->encoders.clear();
    encodeMfeContext->mfeEncodeSharedState->encoders.shrink_to_fit();

    DdiMediaUtil_DestroyMutex(&encodeMfeContext->encodeMfeMutex);
    MOS_FreeMemory(encodeMfeContext->mfeEncodeSharedState);
    MOS_FreeMemory(encodeMfeContext);

    DdiMediaUtil_LockMutex(&mediaCtx->MfeMutex);
    DdiMediaUtil_ReleasePVAContextFromHeap(mediaCtx->pMfeCtxHeap, (mfe_context & DDI_MEDIA_MASK_VACONTEXTID));
    mediaCtx->uiNumMfes--;
    DdiMediaUtil_UnLockMutex(&mediaCtx->MfeMutex);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_AddContextInternal(
    VADriverContextP ctx,
    VAContextID context,
    VAMFContextID mfe_context)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    PDDI_ENCODE_MFE_CONTEXT encodeMfeContext = (PDDI_ENCODE_MFE_CONTEXT)DdiMedia_GetContextFromContextID(ctx, mfe_context, &ctxType);
    DDI_CHK_NULL(encodeMfeContext, "nullptr encodeMfeContext", VA_STATUS_ERROR_INVALID_CONTEXT);

    if (ctxType != DDI_MEDIA_CONTEXT_TYPE_MFE)
    {
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    PDDI_ENCODE_CONTEXT encodeContext = DdiEncode_GetEncContextFromContextID(ctx, context);
    DDI_CHK_NULL(encodeContext, "nullptr encodeContext", VA_STATUS_ERROR_INVALID_CONTEXT);

    CodechalEncoderState *encoder = dynamic_cast<CodechalEncoderState *>(encodeContext->pCodecHal);
    DDI_CHK_NULL(encoder, "nullptr codechal encoder", VA_STATUS_ERROR_INVALID_CONTEXT);

    if (!mediaCtx->m_caps->IsMfeSupportedEntrypoint(encodeContext->vaEntrypoint))
    {
        return VA_STATUS_ERROR_UNSUPPORTED_ENTRYPOINT;
    }

    if (!mediaCtx->m_caps->IsMfeSupportedProfile(encodeContext->vaProfile))
    {
        return VA_STATUS_ERROR_UNSUPPORTED_PROFILE;
    }

    DdiMediaUtil_LockMutex(&encodeMfeContext->encodeMfeMutex);
    encodeMfeContext->pDdiEncodeContexts.push_back(encodeContext);

    if (encodeMfeContext->currentStreamId == 0)
    {
        encodeMfeContext->isFEI = (encodeContext->vaEntrypoint == VAEntrypointFEI) ? true : false;
    }

    // MFE cannot support legacy and FEI together
    if ((encodeContext->vaEntrypoint != VAEntrypointFEI && encodeMfeContext->isFEI) ||
        (encodeContext->vaEntrypoint == VAEntrypointFEI && !encodeMfeContext->isFEI))
    {
        DdiMediaUtil_UnLockMutex(&encodeMfeContext->encodeMfeMutex);
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }

    encoder->m_mfeEnabled = true;
    // Assign one unique id to this sub context/stream
    encoder->m_mfeEncodeParams.streamId = encodeMfeContext->currentStreamId;

    MOS_STATUS eStatus = encoder->SetMfeSharedState(encodeMfeContext->mfeEncodeSharedState);
    if (eStatus != MOS_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE(
            "Failed to set MFE Shared State for encoder #%d",
            encodeMfeContext->currentStreamId);

        encoder->m_mfeEnabled = false;

        DdiMediaUtil_UnLockMutex(&encodeMfeContext->encodeMfeMutex);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    encodeMfeContext->currentStreamId++;
    DdiMediaUtil_UnLockMutex(&encodeMfeContext->encodeMfeMutex);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_ReleaseContextInternal(
    VADriverContextP ctx,
    VAContextID context,
    VAMFContextID mfe_context)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    PDDI_ENCODE_MFE_CONTEXT encodeMfeContext = (PDDI_ENCODE_MFE_CONTEXT)DdiMedia_GetContextFromContextID(ctx, mfe_context, &ctxType);
    DDI_CHK_NULL(encodeMfeContext, "nullptr encodeMfeContext", VA_STATUS_ERROR_INVALID_CONTEXT);

    if (ctxType != DDI_MEDIA_CONTEXT_TYPE_MFE ||
        encodeMfeContext->pDdiEncodeContexts.size() == 0)
    {
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    PDDI_ENCODE_CONTEXT encodeContext = DdiEncode_GetEncContextFromContextID(ctx, context);
    DDI_CHK_NULL(encodeMfeContext, "nullptr encodeContext", VA_STATUS_ERROR_INVALID_CONTEXT);

    bool contextErased = false;
    DdiMediaUtil_LockMutex(&encodeMfeContext->encodeMfeMutex);
    for (uint32_t i = 0; i < encodeMfeContext->pDdiEncodeContexts.size(); i++)
    {
        if (encodeMfeContext->pDdiEncodeContexts[i] == encodeContext)
        {
            encodeMfeContext->pDdiEncodeContexts.erase(encodeMfeContext->pDdiEncodeContexts.begin() + i);
            contextErased = true;
            break;
        }
    }

    if (!contextErased)
    {
        DdiMediaUtil_UnLockMutex(&encodeMfeContext->encodeMfeMutex);
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    DdiMediaUtil_UnLockMutex(&encodeMfeContext->encodeMfeMutex);

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_CreateContext(
    VADriverContextP ctx,
    VAConfigID config_id,
    int32_t picture_width,
    int32_t picture_height,
    int32_t flag,
    VASurfaceID *render_targets,
    int32_t num_render_targets,
    VAContextID *context)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(context, "nullptr context", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaDrvCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaDrvCtx, "nullptr mediaDrvCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    if (num_render_targets > 0)
    {
        DDI_CHK_NULL(render_targets, "nullptr render_targets", VA_STATUS_ERROR_INVALID_PARAMETER);
        DDI_CHK_NULL(mediaDrvCtx->pSurfaceHeap, "nullptr mediaDrvCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
        for (int32_t i = 0; i < num_render_targets; i++)
        {
            uint32_t surfaceId = (uint32_t)render_targets[i];
            DDI_CHK_LESS(surfaceId, mediaDrvCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid Surface", VA_STATUS_ERROR_INVALID_SURFACE);
        }
    }

    VAStatus vaStatus = VA_STATUS_SUCCESS;
#ifdef CNM_VPUAPI_INTERFACE
    vaStatus = VpuApiInit(mediaDrvCtx->coreIdx);
    if (config_id < VPUAPI_ENCODER_CONFIG_ID_START)
    {
        vaStatus = VpuApiDecOpen(ctx, config_id, picture_width, picture_height, context);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            VpuApiDecClose(ctx, *context);
            VpuApiDeInit(mediaDrvCtx->coreIdx);
        }
    }
    else if (config_id < VPUAPI_MAX_CONFIG_ID)
    {
        vaStatus = VpuApiEncOpen(ctx, config_id, picture_width, picture_height, context);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            VpuApiEncClose(ctx, *context);
            VpuApiDeInit(mediaDrvCtx->coreIdx);
        }
    }
    else
    {
        DDI_ASSERTMESSAGE("DDI: Invalid config_id");
        VpuApiDeInit(mediaDrvCtx->coreIdx);
        vaStatus = VA_STATUS_ERROR_INVALID_CONFIG;
    }
#else
    if (mediaDrvCtx->m_caps->IsDecConfigId(config_id))
    {
        vaStatus = DdiDecode_CreateContext(ctx, config_id - DDI_CODEC_GEN_CONFIG_ATTRIBUTES_DEC_BASE, picture_width, picture_height, flag, render_targets, num_render_targets, context);
    }
    else if (mediaDrvCtx->m_caps->IsEncConfigId(config_id))
    {
        vaStatus = DdiEncode_CreateContext(ctx, config_id - DDI_CODEC_GEN_CONFIG_ATTRIBUTES_ENC_BASE, picture_width, picture_height, flag, render_targets, num_render_targets, context);
    }
    else if (mediaDrvCtx->m_caps->IsVpConfigId(config_id))
    {
        vaStatus = DdiVp_CreateContext(ctx, config_id - DDI_VP_GEN_CONFIG_ATTRIBUTES_BASE, picture_width, picture_height, flag, render_targets, num_render_targets, context);
    }
    else
    {
        DDI_ASSERTMESSAGE("DDI: Invalid config_id");
        vaStatus = VA_STATUS_ERROR_INVALID_CONFIG;
    }
#endif

    return vaStatus;
}

VAStatus DdiMedia_DestroyContext(
    VADriverContextP ctx,
    VAContextID context)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    void *ctxPtr = DdiMedia_GetContextFromContextID(ctx, context, &ctxType);

    switch (ctxType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
    {
        PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
        VpuApiDecClose(ctx, context);
        VpuApiDeInit(mediaCtx->coreIdx);
        return VA_STATUS_SUCCESS;
    }
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
    {
        PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
        VpuApiEncClose(ctx, context);
        VpuApiDeInit(mediaCtx->coreIdx);
        return VA_STATUS_SUCCESS;
    }
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_MFE:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_DestroyContext.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
#else
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        return DdiDecode_DestroyContext(ctx, context);
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        return DdiEncode_DestroyContext(ctx, context);
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        return DdiVp_DestroyContext(ctx, context);
    case DDI_MEDIA_CONTEXT_TYPE_MFE:
        return DdiMedia_DestoryMfeContext(ctx, context);
#endif
    default:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_DestroyContext.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
}

VAStatus DdiMedia_CreateBuffer(
    VADriverContextP ctx,
    VAContextID context,
    VABufferType type,
    uint32_t size,
    uint32_t num_elements,
    void *data,
    VABufferID *bufId)
{
    DDI_FUNCTION_ENTER();
    int32_t event[] = {size, num_elements, type};
    MOS_TraceEventExt(EVENT_VA_BUFFER, EVENT_TYPE_START, event, sizeof(event), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(bufId, "nullptr buf_id", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(size, 0, "Invalid size", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    void *ctxPtr = DdiMedia_GetContextFromContextID(ctx, context, &ctxType);
    DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

    *bufId = VA_INVALID_ID;

    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    VAStatus va = VA_STATUS_SUCCESS;
    switch (ctxType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        va = VpuApiDecCreateBuffer(ctx, ctxPtr, type, size, num_elements, data, bufId);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        va = VpuApiEncCreateBuffer(ctx, ctxPtr, type, size, num_elements, data, bufId);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        va = VA_STATUS_ERROR_INVALID_CONTEXT;
        break;
#else
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        va = DdiDecode_CreateBuffer(ctx, DdiDecode_GetDecContextFromPVOID(ctxPtr), type, size, num_elements, data, bufId);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        va = DdiEncode_CreateBuffer(ctx, context, type, size, num_elements, data, bufId);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        va = DdiVp_CreateBuffer(ctx, ctxPtr, type, size, num_elements, data, bufId);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        printf("+%s DdiDecode_CreateBuffer DDI_MEDIA_CONTEXT_TYPE_PROTECTED\n", __FUNCTION__);
        va = DdiMediaProtected::DdiMedia_ProtectedSessionCreateBuffer(ctx, context, type, size, num_elements, data, bufId);
        printf("-%s DdiDecode_CreateBuffer va=0x%x\n", __FUNCTION__, va);
        break;
#endif
    default:
        va = VA_STATUS_ERROR_INVALID_CONTEXT;
    }

    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
    MOS_TraceEventExt(EVENT_VA_BUFFER, EVENT_TYPE_END, bufId, sizeof(bufId), nullptr, 0);
    return va;
}

VAStatus DdiMedia_BufferSetNumElements(
    VADriverContextP ctx,
    VABufferID buf_id,
    uint32_t num_elements)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)buf_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid buf_id", VA_STATUS_ERROR_INVALID_BUFFER);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buf, "Invalid buffer.", VA_STATUS_ERROR_INVALID_BUFFER);

    // only for VASliceParameterBufferType of buffer, the number of elements can be greater than 1
    if (buf->uiType != VASliceParameterBufferType &&
        num_elements > 1)
    {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    if (buf->uiType == VASliceParameterBufferType &&
        buf->uiNumElements < num_elements)
    {
        MOS_FreeMemory(buf->pData);
        buf->iSize = buf->iSize / buf->uiNumElements;
        buf->pData = (uint8_t *)MOS_AllocAndZeroMemory(buf->iSize * num_elements);
        buf->iSize = buf->iSize * num_elements;
    }

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Map data store of the buffer into the client's address space
//!         vaCreateBuffer() needs to be called with "data" set to nullptr before
//!         calling vaMapBuffer()
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] buf_id
//!         VA buffer ID
//! \param  [out] pbuf
//!         Pointer to buffer
//! \param  [in] flag
//!         Flag
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_MapBufferInternal(
    VADriverContextP ctx,
    VABufferID buf_id,
    void **pbuf,
    uint32_t flag)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_START, &buf_id, sizeof(buf_id), &flag, sizeof(flag));

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(pbuf, "nullptr pbuf", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)buf_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid bufferId", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buf, "nullptr buf", VA_STATUS_ERROR_INVALID_BUFFER);

    // The context is nullptr when the buffer is created from DdiMedia_DeriveImage
    // So doesn't need to check the context for all cases
    // Only check the context in dec/enc mode
    VAStatus vaStatus = VA_STATUS_SUCCESS;
    uint32_t ctxType = DdiMedia_GetCtxTypeFromVABufferID(mediaCtx, buf_id);
    void *ctxPtr = nullptr;
#ifdef CNM_VPUAPI_INTERFACE
    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#else
    DDI_CODEC_COM_BUFFER_MGR *bufMgr = nullptr;
    PDDI_ENCODE_CONTEXT encCtx = nullptr;
    PDDI_DECODE_CONTEXT decCtx = nullptr;

    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        break;
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

        decCtx = DdiDecode_GetDecContextFromPVOID(ctxPtr);
        bufMgr = &(decCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

        encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
        DDI_CHK_NULL(encCtx, "nullptr encCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
        bufMgr = &(encCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#endif

    MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_INFO, &ctxType, sizeof(ctxType), &buf->uiType, sizeof(uint32_t));
    switch ((int32_t)buf->uiType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case VAEncCodedBufferType:
        mediaCtx->pCodedBufferSegment->buf = DdiMediaUtil_LockBuffer(buf, flag);
        mediaCtx->pCodedBufferSegment->size = buf->iSize;
        *pbuf = mediaCtx->pCodedBufferSegment;
        break;
#else
    case VASliceDataBufferType:
    case VAProtectedSliceDataBufferType:
    case VABitPlaneBufferType:
        *pbuf = (void *)(buf->pData + buf->uiOffset);
        break;

    case VASliceParameterBufferType:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

        decCtx = DdiDecode_GetDecContextFromPVOID(ctxPtr);
        bufMgr = &(decCtx->BufMgr);
        switch (decCtx->wMode)
        {
        case CODECHAL_DECODE_MODE_AVCVLD:
            if (decCtx->bShortFormatInUse)
                *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_H264.pVASliceParaBufH264Base) + buf->uiOffset);
            else
                *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_H264.pVASliceParaBufH264) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_MPEG2VLD:
            *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_MPEG2.pVASliceParaBufMPEG2) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_VC1VLD:
            *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_VC1.pVASliceParaBufVC1) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_JPEG:
            *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_JPEG.pVASliceParaBufJPEG) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_VP8VLD:
            *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_VP8.pVASliceParaBufVP8) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_HEVCVLD:
            if (decCtx->bShortFormatInUse)
                *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufBaseHEVC) + buf->uiOffset);
            else
            {
                if (!decCtx->m_ddiDecode->IsRextProfile())
                    *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufHEVC) + buf->uiOffset);
                else
                    *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_HEVC.pVASliceParaBufHEVCRext) + buf->uiOffset);
            }
            break;
        case CODECHAL_DECODE_MODE_VP9VLD:
            *pbuf = (void *)((uint8_t *)(bufMgr->Codec_Param.Codec_Param_VP9.pVASliceParaBufVP9) + buf->uiOffset);
            break;
        case CODECHAL_DECODE_MODE_AV1VLD:
            *pbuf = (void *)((uint8_t *)(bufMgr->pCodecSlcParamReserved) + buf->uiOffset);
            break;
        default:
            break;
        }
        break;

    case VAEncCodedBufferType:
        DDI_CHK_NULL(encCtx, "nullptr encCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

        if (DdiEncode_CodedBufferExistInStatusReport(encCtx, buf))
        {
            vaStatus = DdiEncode_StatusReport(encCtx, buf, pbuf);
            MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
            return vaStatus;
        }
        // so far a coded buffer that has NOT been added into status report is skipped frame in non-CP case
        // but this can change in future if new usage models come up
        encCtx->BufMgr.pCodedBufferSegment->buf = DdiMediaUtil_LockBuffer(buf, flag);
        encCtx->BufMgr.pCodedBufferSegment->size = buf->iSize;
        *pbuf = encCtx->BufMgr.pCodedBufferSegment;

        break;

    case VAStatsStatisticsBufferType:
    case VAStatsStatisticsBottomFieldBufferType:
    case VAStatsMVBufferType:
    {
        DDI_CHK_NULL(encCtx, "nullptr encCtx", VA_STATUS_ERROR_INVALID_CONTEXT)
        DDI_ENCODE_PRE_ENC_BUFFER_TYPE idx = (buf->uiType == VAStatsMVBufferType) ? PRE_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAStatsStatisticsBufferType) ? PRE_ENC_BUFFER_TYPE_STATS : PRE_ENC_BUFFER_TYPE_STATS_BOT);
        if ((encCtx->codecFunction == CODECHAL_FUNCTION_FEI_PRE_ENC) && DdiEncode_PreEncBufferExistInStatusReport(encCtx, buf, idx))
        {
            vaStatus = DdiEncode_PreEncStatusReport(encCtx, buf, pbuf);
            MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
            return vaStatus;
        }
        if (buf->bo)
        {
            *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        }
        break;
    }
    case VAStatsMVPredictorBufferType:
    case VAEncFEIMBControlBufferType:
    case VAEncFEIMVPredictorBufferType:
    case VAEncQPBufferType:
        if (buf->bo)
        {
            *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        }
        break;
    case VADecodeStreamoutBufferType:
        if (buf->bo)
        {
            uint32_t timeout_NS = 100000000;
            while (0 != mos_gem_bo_wait(buf->bo, timeout_NS))
            {
                // Just loop while gem_bo_wait times-out.
            }
            *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        }
        break;
    case VAEncFEIMVBufferType:
    case VAEncFEIMBCodeBufferType:
    case VAEncFEICURecordBufferType:
    case VAEncFEICTBCmdBufferType:
    case VAEncFEIDistortionBufferType:
    {
        DDI_CHK_NULL(encCtx, "nullptr encCtx", VA_STATUS_ERROR_INVALID_CONTEXT)
        if (encCtx->wModeType == CODECHAL_ENCODE_MODE_AVC)
        {
            CodecEncodeAvcFeiPicParams *feiPicParams = (CodecEncodeAvcFeiPicParams *)encCtx->pFeiPicParams;

            DDI_ENCODE_FEI_ENC_BUFFER_TYPE idx = (buf->uiType == VAEncFEIMVBufferType) ? FEI_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAEncFEIMBCodeBufferType) ? FEI_ENC_BUFFER_TYPE_MBCODE : FEI_ENC_BUFFER_TYPE_DISTORTION);
            if ((feiPicParams != nullptr) && (encCtx->codecFunction == CODECHAL_FUNCTION_FEI_ENC) && DdiEncode_EncBufferExistInStatusReport(encCtx, buf, idx))
            {
                vaStatus = DdiEncode_EncStatusReport(encCtx, buf, pbuf);
                MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
                return vaStatus;
            }
        }
        else if (encCtx->wModeType == CODECHAL_ENCODE_MODE_HEVC)

        {
            CodecEncodeHevcFeiPicParams *feiPicParams = (CodecEncodeHevcFeiPicParams *)encCtx->pFeiPicParams;
            DDI_ENCODE_FEI_ENC_BUFFER_TYPE idx = (buf->uiType == VAEncFEICTBCmdBufferType) ? FEI_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAEncFEICURecordBufferType) ? FEI_ENC_BUFFER_TYPE_MBCODE : FEI_ENC_BUFFER_TYPE_DISTORTION);
            if ((feiPicParams != nullptr) && (encCtx->codecFunction == CODECHAL_FUNCTION_FEI_ENC) && DdiEncode_EncBufferExistInStatusReport(encCtx, buf, idx))
            {
                vaStatus = DdiEncode_EncStatusReport(encCtx, buf, pbuf);
                MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
                return vaStatus;
            }
        }
        if (buf->bo)
        {
            *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        }
    }
    break;
    case VAStatsStatisticsParameterBufferType:
        *pbuf = (void *)(buf->pData + buf->uiOffset);
        break;
    case VAEncMacroblockMapBufferType:
        DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
        *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
        if (nullptr == (*pbuf))
        {
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
        else
        {
            return VA_STATUS_SUCCESS;
        }
        break;

    case VAProbabilityBufferType:
        *pbuf = (void *)(buf->pData + buf->uiOffset);

        break;

    case VAEncMacroblockDisableSkipMapBufferType:
        if (buf->bo)
        {
            *pbuf = DdiMediaUtil_LockBuffer(buf, flag);
        }
        break;
#endif
    case VAImageBufferType:
    default:
        if ((buf->format != Media_Format_CPU) && (DdiMedia_MediaFormatToOsFormat(buf->format) != VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT))
        {
            DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
            // A critical section starts.
            // Make sure not to bailout with a return until the section ends.

            if (nullptr != buf->pSurface && Media_Format_CPU != buf->format)
            {
                vaStatus = DdiMedia_MediaMemoryDecompress(mediaCtx, buf->pSurface);
            }

            if (VA_STATUS_SUCCESS == vaStatus)
            {
                *pbuf = DdiMediaUtil_LockBuffer(buf, flag);

                if (nullptr == *pbuf)
                {
                    vaStatus = VA_STATUS_ERROR_OPERATION_FAILED;
                }
            }

            // The critical section ends.
            DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        }
        else
        {
            *pbuf = (void *)(buf->pData + buf->uiOffset);
        }
        break;
    }

    MOS_TraceEventExt(EVENT_VA_MAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return vaStatus;
}

VAStatus DdiMedia_MapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id,
    void **pbuf)
{
    return DdiMedia_MapBufferInternal(ctx, buf_id, pbuf, MOS_LOCKFLAG_READONLY | MOS_LOCKFLAG_WRITEONLY);
}

VAStatus DdiMedia_UnmapBuffer(
    VADriverContextP ctx,
    VABufferID buf_id)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_UNMAP, EVENT_TYPE_START, &buf_id, sizeof(buf_id), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr  mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)buf_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid buf_id", VA_STATUS_ERROR_INVALID_BUFFER);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buf, "nullptr buf", VA_STATUS_ERROR_INVALID_BUFFER);

    // The context is nullptr when the buffer is created from DdiMedia_DeriveImage
    // So doesn't need to check the context for all cases
    // Only check the context in dec/enc mode
    void *ctxPtr = nullptr;
    uint32_t ctxType = DdiMedia_GetCtxTypeFromVABufferID(mediaCtx, buf_id);
#ifdef CNM_VPUAPI_INTERFACE
    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#else
    DDI_CODEC_COM_BUFFER_MGR *bufMgr = nullptr;
    PDDI_DECODE_CONTEXT decCtx = nullptr;
    PDDI_ENCODE_CONTEXT encCtx = nullptr;

    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        break;
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

        decCtx = DdiDecode_GetDecContextFromPVOID(ctxPtr);
        bufMgr = &(decCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buf_id);
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);

        encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
        bufMgr = &(encCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#endif

    switch ((int32_t)buf->uiType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case VAEncCodedBufferType:
        DdiMediaUtil_UnlockBuffer(buf);
        break;
#else
    case VASliceDataBufferType:
    case VAProtectedSliceDataBufferType:
    case VABitPlaneBufferType:
        break;
    case VAEncCodedBufferType:
    case VAStatsStatisticsBufferType:
    case VAStatsStatisticsBottomFieldBufferType:
    case VAStatsMVBufferType:
    case VAStatsMVPredictorBufferType:
    case VAEncFEIMBControlBufferType:
    case VAEncFEIMVPredictorBufferType:
    case VAEncFEIMVBufferType:
    case VAEncFEIMBCodeBufferType:
    case VAEncFEICURecordBufferType:
    case VAEncFEICTBCmdBufferType:
    case VAEncFEIDistortionBufferType:
    case VAEncQPBufferType:
    case VADecodeStreamoutBufferType:
        if (buf->bo)
        {
            DdiMediaUtil_UnlockBuffer(buf);
        }
        break;
    case VAEncMacroblockDisableSkipMapBufferType:
        if (buf->bo)
        {
            DdiMediaUtil_UnlockBuffer(buf);
        }
        break;
#endif
    case VAImageBufferType:
    default:
        if ((buf->format != Media_Format_CPU) && (DdiMedia_MediaFormatToOsFormat(buf->format) != VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT))
        {
            DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
            DdiMediaUtil_UnlockBuffer(buf);
            DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        }
        break;
    }

    MOS_TraceEventExt(EVENT_VA_UNMAP, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_DestroyBuffer(
    VADriverContextP ctx,
    VABufferID buffer_id)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_FREE_BUFFER, EVENT_TYPE_START, &buffer_id, sizeof(buffer_id), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)buffer_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid bufferId", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buffer_id);
    DDI_CHK_NULL(buf, "nullptr buf", VA_STATUS_ERROR_INVALID_BUFFER);

    void *ctxPtr = DdiMedia_GetCtxFromVABufferID(mediaCtx, buffer_id);
    uint32_t ctxType = DdiMedia_GetCtxTypeFromVABufferID(mediaCtx, buffer_id);

#ifdef CNM_VPUAPI_INTERFACE
    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        DdiMediaUtil_FreeBuffer(buf);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        DdiMediaUtil_FreeBuffer(buf);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#else
    DDI_CODEC_COM_BUFFER_MGR *bufMgr = nullptr;
    PDDI_ENCODE_CONTEXT encCtx = nullptr;
    PDDI_DECODE_CONTEXT decCtx = nullptr;

    switch (ctxType)
    {
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        decCtx = DdiDecode_GetDecContextFromPVOID(ctxPtr);
        bufMgr = &(decCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        DDI_CHK_NULL(ctxPtr, "nullptr ctxPtr", VA_STATUS_ERROR_INVALID_CONTEXT);
        encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
        bufMgr = &(encCtx->BufMgr);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        break;
    case DDI_MEDIA_CONTEXT_TYPE_MEDIA:
        break;
    case DDI_MEDIA_CONTEXT_TYPE_PROTECTED:
        break;
    default:
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
#endif

    switch ((int32_t)buf->uiType)
    {
#ifdef CNM_VPUAPI_INTERFACE
#else
    case VASliceDataBufferType:
    case VAProtectedSliceDataBufferType:
        DdiMedia_ReleaseBsBuffer(bufMgr, buf);
        break;
    case VABitPlaneBufferType:
        DdiMedia_ReleaseBpBuffer(bufMgr, buf);
        break;
    case VAProbabilityBufferType:
        DdiMedia_ReleaseBpBuffer(bufMgr, buf);
        break;

    case VASliceParameterBufferType:
        DdiMedia_ReleaseSliceControlBuffer(ctxType, ctxPtr, buf);
        break;
    case VAPictureParameterBufferType:
        break;
    case VAProcPipelineParameterBufferType:
    case VAProcFilterParameterBufferType:
        MOS_FreeMemory(buf->pData);
        break;
    case VASubsetsParameterBufferType:
    case VAIQMatrixBufferType:
    case VAHuffmanTableBufferType:
    case VAEncSliceParameterBufferType:
    case VAEncPictureParameterBufferType:
    case VAEncSequenceParameterBufferType:
    case VAEncPackedHeaderDataBufferType:
    case VAEncPackedHeaderParameterBufferType:
        MOS_FreeMemory(buf->pData);
        break;
    case VAEncMacroblockMapBufferType:
        DdiMediaUtil_FreeBuffer(buf);
        break;
#ifdef ENABLE_ENC_UNLIMITED_OUTPUT
    case VAEncCodedBufferType:
        if (nullptr == encCtx)
        {
            encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
            if (nullptr == encCtx)
                return VA_STATUS_ERROR_INVALID_CONTEXT;
        }
        DdiMediaUtil_FreeBuffer(buf);
        break;
#endif
    case VAStatsStatisticsParameterBufferType:
        MOS_FreeMemory(buf->pData);
        break;
    case VAStatsStatisticsBufferType:
    case VAStatsStatisticsBottomFieldBufferType:
    case VAStatsMVBufferType:
    {
        if (nullptr == encCtx)
        {
            encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
            if (nullptr == encCtx)
                return VA_STATUS_ERROR_INVALID_CONTEXT;
        }
        if (encCtx->codecFunction == CODECHAL_FUNCTION_FEI_PRE_ENC)
        {
            DDI_ENCODE_PRE_ENC_BUFFER_TYPE idx = (buf->uiType == VAStatsMVBufferType) ? PRE_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAStatsStatisticsBufferType) ? PRE_ENC_BUFFER_TYPE_STATS : PRE_ENC_BUFFER_TYPE_STATS_BOT);
            DdiEncode_RemoveFromPreEncStatusReportQueue(encCtx, buf, idx);
        }
    }
        DdiMediaUtil_FreeBuffer(buf);
        break;
    case VAStatsMVPredictorBufferType:
    case VAEncFEIMBControlBufferType:
    case VAEncFEIMVPredictorBufferType:
    case VAEncQPBufferType:
    case VADecodeStreamoutBufferType:
        DdiMediaUtil_FreeBuffer(buf);
        break;
    case VAEncFEIMVBufferType:
    case VAEncFEIMBCodeBufferType:
    case VAEncFEICURecordBufferType:
    case VAEncFEICTBCmdBufferType:
    case VAEncFEIDistortionBufferType:
    {
        if (nullptr == encCtx)
        {
            encCtx = DdiEncode_GetEncContextFromPVOID(ctxPtr);
            if (nullptr == encCtx)
                return VA_STATUS_ERROR_INVALID_CONTEXT;
        }
        if (encCtx->wModeType == CODECHAL_ENCODE_MODE_AVC)
        {
            CodecEncodeAvcFeiPicParams *feiPicParams;
            feiPicParams = (CodecEncodeAvcFeiPicParams *)(encCtx->pFeiPicParams);
            if ((feiPicParams != nullptr) && (encCtx->codecFunction == CODECHAL_FUNCTION_FEI_ENC))
            {
                DDI_ENCODE_FEI_ENC_BUFFER_TYPE idx = (buf->uiType == VAEncFEIMVBufferType) ? FEI_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAEncFEIMBCodeBufferType) ? FEI_ENC_BUFFER_TYPE_MBCODE : FEI_ENC_BUFFER_TYPE_DISTORTION);
                DdiEncode_RemoveFromEncStatusReportQueue(encCtx, buf, idx);
            }
        }
        else if (encCtx->wModeType == CODECHAL_ENCODE_MODE_HEVC)
        {
            CodecEncodeHevcFeiPicParams *feiPicParams;
            feiPicParams = (CodecEncodeHevcFeiPicParams *)(encCtx->pFeiPicParams);
            if ((feiPicParams != nullptr) && (encCtx->codecFunction == CODECHAL_FUNCTION_FEI_ENC))
            {
                DDI_ENCODE_FEI_ENC_BUFFER_TYPE idx = (buf->uiType == VAEncFEICTBCmdBufferType) ? FEI_ENC_BUFFER_TYPE_MVDATA : ((buf->uiType == VAEncFEICURecordBufferType) ? FEI_ENC_BUFFER_TYPE_MBCODE : FEI_ENC_BUFFER_TYPE_DISTORTION);
                DdiEncode_RemoveFromEncStatusReportQueue(encCtx, buf, idx);
            }
        }
    }
        DdiMediaUtil_FreeBuffer(buf);
        break;
#endif
    case VAImageBufferType:
        if (buf->format == Media_Format_CPU)
        {
            MOS_FreeMemory(buf->pData);
        }
        else
        {
            DdiMediaUtil_UnRefBufObjInMediaBuffer(buf);

            if (buf->uiExportcount)
            {
                buf->bPostponedBufFree = true;
                MOS_TraceEventExt(EVENT_VA_FREE_BUFFER, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
                return VA_STATUS_SUCCESS;
            }
        }
        break;
    default: // do not handle any un-listed buffer type
        MOS_FreeMemory(buf->pData);
        break;
        // return va_STATUS_SUCCESS;
    }

    MOS_FreeMemory(buf);

    DdiMedia_DestroyBufFromVABufferID(mediaCtx, buffer_id);
    MOS_TraceEventExt(EVENT_VA_FREE_BUFFER, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_BeginPicture(
    VADriverContextP ctx,
    VAContextID context,
    VASurfaceID render_target)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)render_target, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "render_target", VA_STATUS_ERROR_INVALID_SURFACE);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    void *ctxPtr = DdiMedia_GetContextFromContextID(ctx, context, &ctxType);
    uint32_t event[] = {(uint32_t)context, ctxType, (uint32_t)render_target};
    MOS_TraceEventExt(EVENT_VA_PICTURE, EVENT_TYPE_START, event, sizeof(event), nullptr, 0);

    PDDI_MEDIA_SURFACE surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, render_target);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);

    DdiMediaUtil_LockMutex(&mediaCtx->SurfaceMutex);
    surface->curCtxType = ctxType;
    surface->curStatusReportQueryState = DDI_MEDIA_STATUS_REPORT_QUERY_STATE_PENDING;
    if (ctxType == DDI_MEDIA_CONTEXT_TYPE_VP)
    {
        surface->curStatusReport.vpp.status = VPREP_NOTAVAILABLE;
    }
    DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);

    switch (ctxType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        mediaCtx->renderTarget = render_target;
        return VA_STATUS_SUCCESS;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        mediaCtx->renderTarget = render_target;
        return VA_STATUS_SUCCESS;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_BeginPicture.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
#else
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        return DdiDecode_BeginPicture(ctx, context, render_target);
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        return DdiEncode_BeginPicture(ctx, context, render_target);
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        return DdiVp_BeginPicture(ctx, context, render_target);
#endif
    default:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_BeginPicture.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
}

VAStatus DdiMedia_RenderPicture(
    VADriverContextP ctx,
    VAContextID context,
    VABufferID *buffers,
    int32_t num_buffers)
{

    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(buffers, "nullptr buffers", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(num_buffers, 0, "Invalid number buffers", VA_STATUS_ERROR_INVALID_PARAMETER);
    uint32_t event[] = {(uint32_t)context, (uint32_t)num_buffers};
    MOS_TraceEventExt(EVENT_VA_PICTURE, EVENT_TYPE_INFO, event, sizeof(event), buffers, num_buffers * sizeof(VAGenericID));

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    for (int32_t i = 0; i < num_buffers; i++)
    {
        DDI_CHK_LESS((uint32_t)buffers[i], mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid Buffer", VA_STATUS_ERROR_INVALID_BUFFER);
    }

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    void *ctxPtr = DdiMedia_GetContextFromContextID(ctx, context, &ctxType);

    switch (ctxType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        return VpuApiDecRenderPicture(ctx, buffers, num_buffers);
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        return VA_STATUS_SUCCESS;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_RenderPicture.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
#else
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        return DdiDecode_RenderPicture(ctx, context, buffers, num_buffers);
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
#ifdef USE_INTEL_CONFIG_ID
    {
        VAStatus status;
        status = DdiEncode_RenderPicture(ctx, context, buffers, num_buffers);
        if (status == 18 || status == 7)
        {
            status = VA_STATUS_SUCCESS;
        }
        return status;
    }
#else
        return DdiEncode_RenderPicture(ctx, context, buffers, num_buffers);
#endif
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        return DdiVp_RenderPicture(ctx, context, buffers, num_buffers);
#endif
    default:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_RenderPicture.");
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
}

VAStatus DdiMedia_EndPicture(
    VADriverContextP ctx,
    VAContextID context)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
    void *ctxPtr = DdiMedia_GetContextFromContextID(ctx, context, &ctxType);
    VAStatus vaStatus = VA_STATUS_SUCCESS;
    switch (ctxType)
    {
#ifdef CNM_VPUAPI_INTERFACE
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        vaStatus = VpuApiDecCheckValidity(ctx);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            break;
        }
        vaStatus = VpuApiDecSeqInit(ctx);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            break;
        }
        vaStatus = VpuApiDecPic(ctx);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        vaStatus = VpuApiEncSeqInit(ctx);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            break;
        }
        vaStatus = VpuApiEncPic(ctx);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_EndPicture.");
        vaStatus = VA_STATUS_ERROR_INVALID_CONTEXT;
        break;
#else
    case DDI_MEDIA_CONTEXT_TYPE_DECODER:
        vaStatus = DdiDecode_EndPicture(ctx, context);
        break;
    case DDI_MEDIA_CONTEXT_TYPE_ENCODER:
        vaStatus = DdiEncode_EndPicture(ctx, context);
#ifdef USE_INTEL_CONFIG_ID
        if (vaStatus == 24)
        {
            vaStatus = VA_STATUS_SUCCESS;
        }
#endif
        break;
    case DDI_MEDIA_CONTEXT_TYPE_VP:
        vaStatus = DdiVp_EndPicture(ctx, context);
        break;
#endif
    default:
        DDI_ASSERTMESSAGE("DDI: unsupported context in DdiCodec_EndPicture.");
        vaStatus = VA_STATUS_ERROR_INVALID_CONTEXT;
    }

    MOS_TraceEventExt(EVENT_VA_PICTURE, EVENT_TYPE_END, &context, sizeof(context), &vaStatus, sizeof(vaStatus));
    PERF_UTILITY_STOP_ONCE("First Frame Time", PERF_MOS, PERF_LEVEL_DDI);

    return vaStatus;
}

static VAStatus DdiMedia_StatusCheck(
    PDDI_MEDIA_CONTEXT mediaCtx,
    DDI_MEDIA_SURFACE *surface,
    VASurfaceID surface_id)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_CONTEXT);

    uint32_t i = 0;
    PDDI_DECODE_CONTEXT decCtx = (PDDI_DECODE_CONTEXT)surface->pDecCtx;
    if (decCtx && surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_DECODER)
    {
        DdiMediaUtil_LockGuard guard(&mediaCtx->SurfaceMutex);

        Codechal *codecHal = decCtx->pCodecHal;
        // return success just avoid vaDestroyContext is ahead of vaSyncSurface
        DDI_CHK_NULL(codecHal, "nullptr decCtx->pCodecHal", VA_STATUS_SUCCESS);

        // return success just avoid vaDestroyContext is ahead of vaSyncSurface
        if (codecHal->IsApogeiosEnabled())
        {
            DecodePipelineAdapter *decoder = dynamic_cast<DecodePipelineAdapter *>(codecHal);
            DDI_CHK_NULL(decoder, "nullptr (DecodePipelineAdapter *decoder) ", VA_STATUS_SUCCESS);
            return DdiDecode_StatusReport(mediaCtx, decoder, surface);
        }
        else
        {
            CodechalDecode *decoder = dynamic_cast<CodechalDecode *>(codecHal);
            DDI_CHK_NULL(decoder, "nullptr (CodechalDecode *decoder)", VA_STATUS_SUCCESS);
            return DdiDecode_StatusReport(mediaCtx, decoder, surface);
        }
    }

    if (surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_VP)
    {
        PDDI_VP_CONTEXT vpCtx = (PDDI_VP_CONTEXT)surface->pVpCtx;
        DDI_CHK_NULL(vpCtx, "nullptr vpCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
        DDI_CHK_NULL(vpCtx->pVpHal, "nullptr vpCtx->pVpHal", VA_STATUS_ERROR_INVALID_CONTEXT);

        QUERY_STATUS_REPORT_APP tempVpReport;
        MOS_ZeroMemory(&tempVpReport, sizeof(QUERY_STATUS_REPORT_APP));

        // Get reported surfaces' count
        uint32_t tableLen = 0;
        vpCtx->pVpHal->GetStatusReportEntryLength(&tableLen);

        if (tableLen > 0 && surface->curStatusReportQueryState == DDI_MEDIA_STATUS_REPORT_QUERY_STATE_PENDING)
        {
            // Query the status for all of surfaces which have finished
            for (i = 0; i < tableLen; i++)
            {
                MOS_ZeroMemory(&tempVpReport, sizeof(QUERY_STATUS_REPORT_APP));
                vpCtx->pVpHal->GetStatusReport(&tempVpReport, 1);

                // StatusFeedBackID is last time submitted Target Surface ID which is set in BeginPicture,
                // So we can know the report is for which surface here.
                DDI_MEDIA_SURFACE *tempSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, tempVpReport.StatusFeedBackID);
                if (tempSurface == nullptr)
                {
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }

                // Update the status of the surface which is reported.
                tempSurface->curStatusReport.vpp.status = (uint32_t)tempVpReport.dwStatus;
                tempSurface->curStatusReportQueryState = DDI_MEDIA_STATUS_REPORT_QUERY_STATE_COMPLETED;

                if (tempVpReport.StatusFeedBackID == surface_id)
                {
                    break;
                }
            }
        }

        if (surface->curStatusReportQueryState == DDI_MEDIA_STATUS_REPORT_QUERY_STATE_COMPLETED)
        {
            if (surface->curStatusReport.vpp.status == VPREP_OK)
            {
                return VA_STATUS_SUCCESS;
            }
            else if (surface->curStatusReport.vpp.status == VPREP_NOTREADY)
            {
                return mediaCtx->bMediaResetEnable ? VA_STATUS_SUCCESS : VA_STATUS_ERROR_HW_BUSY;
            }
            else
            {
                return VA_STATUS_ERROR_OPERATION_FAILED;
            }
        }
        else
        {
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_SyncSurface(
    VADriverContextP ctx,
    VASurfaceID render_target)
{
    PERF_UTILITY_AUTO(__FUNCTION__, "ENCODE", "DDI");

    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_START, &render_target, sizeof(VAGenericID), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_LESS((uint32_t)render_target, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid render_target", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, render_target);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_CONTEXT);

#ifdef CNM_VPUAPI_INTERFACE
    printf("DdiMedia_SyncSurface \r\n");
    if (surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_DECODER)
    {
        do
        {
            VAStatus vaStatus = VpuApiDecGetResult(ctx, GetUsedRenderTarget(mediaCtx));
            if (vaStatus != VA_STATUS_SUCCESS)
                return vaStatus;
        } while (FindUsedRenderTarget(mediaCtx, render_target));
    }
#endif
    if (surface->pCurrentFrameSemaphore)
    {
        DdiMediaUtil_WaitSemaphore(surface->pCurrentFrameSemaphore);
        DdiMediaUtil_PostSemaphore(surface->pCurrentFrameSemaphore);
    }

    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_INFO, surface->bo ? &surface->bo->handle : nullptr, sizeof(uint32_t), nullptr, 0);
    // check the bo here?
    // zero is a expected return value
    uint32_t timeout_NS = 100000000;
    while (0 != mos_gem_bo_wait(surface->bo, timeout_NS))
    {
        // Just loop while gem_bo_wait times-out.
    }

    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
// #define GREGORY_REMOVE
#ifdef GREGORY_REMOVE
    DdiMedia_StatusCheck(mediaCtx, surface, render_target);
    return VA_STATUS_SUCCESS;
#else
    return DdiMedia_StatusCheck(mediaCtx, surface, render_target);
#endif
}

#if VA_CHECK_VERSION(1, 9, 0)
VAStatus DdiMedia_SyncSurface2(
    VADriverContextP ctx,
    VASurfaceID surface_id,
    uint64_t timeout_ns)
{
    PERF_UTILITY_AUTO(__FUNCTION__, "ENCODE", "DDI");

    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_START, &surface_id, sizeof(VAGenericID), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_LESS((uint32_t)surface_id, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid render_target", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface_id);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_CONTEXT);
    if (surface->pCurrentFrameSemaphore)
    {
        DdiMediaUtil_WaitSemaphore(surface->pCurrentFrameSemaphore);
        DdiMediaUtil_PostSemaphore(surface->pCurrentFrameSemaphore);
    }
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_INFO, surface->bo ? &surface->bo->handle : nullptr, sizeof(uint32_t), nullptr, 0);

    if (timeout_ns == VA_TIMEOUT_INFINITE)
    {
        // zero is an expected return value when not hit timeout
        auto ret = mos_gem_bo_wait(surface->bo, DDI_BO_INFINITE_TIMEOUT);
        if (0 != ret)
        {
            DDI_NORMALMESSAGE("vaSyncSurface2: surface is still used by HW\n\r");
            return VA_STATUS_ERROR_TIMEDOUT;
        }
    }
    else
    {
        int64_t timeoutBoWait1 = 0;
        int64_t timeoutBoWait2 = 0;
        if (timeout_ns >= DDI_BO_MAX_TIMEOUT)
        {
            timeoutBoWait1 = DDI_BO_MAX_TIMEOUT - 1;
            timeoutBoWait2 = timeout_ns - DDI_BO_MAX_TIMEOUT + 1;
        }
        else
        {
            timeoutBoWait1 = (int64_t)timeout_ns;
        }

        // zero is an expected return value when not hit timeout
        auto ret = mos_gem_bo_wait(surface->bo, timeoutBoWait1);
        if (0 != ret)
        {
            if (timeoutBoWait2)
            {
                ret = mos_gem_bo_wait(surface->bo, timeoutBoWait2);
            }
            if (0 != ret)
            {
                DDI_NORMALMESSAGE("vaSyncSurface2: surface is still used by HW\n\r");
                return VA_STATUS_ERROR_TIMEDOUT;
            }
        }
    }
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return DdiMedia_StatusCheck(mediaCtx, surface, surface_id);
}

VAStatus DdiMedia_SyncBuffer(
    VADriverContextP ctx,
    VABufferID buf_id,
    uint64_t timeout_ns)
{
    PERF_UTILITY_AUTO(__FUNCTION__, "ENCODE", "DDI");

    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_START, &buf_id, sizeof(buf_id), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_LESS((uint32_t)buf_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid buffer", VA_STATUS_ERROR_INVALID_BUFFER);

    DDI_MEDIA_BUFFER *buffer = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buffer, "nullptr buffer", VA_STATUS_ERROR_INVALID_CONTEXT);

    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_INFO, buffer->bo ? &buffer->bo->handle : nullptr, sizeof(uint32_t), nullptr, 0);
    if (timeout_ns == VA_TIMEOUT_INFINITE)
    {
        // zero is a expected return value when not hit timeout
        auto ret = mos_gem_bo_wait(buffer->bo, DDI_BO_INFINITE_TIMEOUT);
        if (0 != ret)
        {
            DDI_NORMALMESSAGE("vaSyncBuffer: buffer is still used by HW\n\r");
            return VA_STATUS_ERROR_TIMEDOUT;
        }
    }
    else
    {
        int64_t timeoutBoWait1 = 0;
        int64_t timeoutBoWait2 = 0;
        if (timeout_ns >= DDI_BO_MAX_TIMEOUT)
        {
            timeoutBoWait1 = DDI_BO_MAX_TIMEOUT - 1;
            timeoutBoWait2 = timeout_ns - DDI_BO_MAX_TIMEOUT + 1;
        }
        else
        {
            timeoutBoWait1 = (int64_t)timeout_ns;
        }

        // zero is a expected return value when not hit timeout
        auto ret = mos_gem_bo_wait(buffer->bo, timeoutBoWait1);
        if (0 != ret)
        {
            if (timeoutBoWait2)
            {
                ret = mos_gem_bo_wait(buffer->bo, timeoutBoWait2);
            }
            if (0 != ret)
            {
                DDI_NORMALMESSAGE("vaSyncBuffer: buffer is still used by HW\n\r");
                return VA_STATUS_ERROR_TIMEDOUT;
            }
        }
    }
    MOS_TraceEventExt(EVENT_VA_SYNC, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}
#endif

VAStatus DdiMedia_QuerySurfaceStatus(
    VADriverContextP ctx,
    VASurfaceID render_target,
    VASurfaceStatus *status)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(status, "nullptr status", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_LESS((uint32_t)render_target, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid render_target", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_MEDIA_SURFACE *surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, render_target);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);

    if (surface->pCurrentFrameSemaphore)
    {
        if (DdiMediaUtil_TryWaitSemaphore(surface->pCurrentFrameSemaphore) == 0)
        {
            DdiMediaUtil_PostSemaphore(surface->pCurrentFrameSemaphore);
        }
        else
        {
            // Return busy state if the surface is not submitted
            *status = VASurfaceRendering;
            return VA_STATUS_SUCCESS;
        }
    }

    // Query the busy state of bo.
    // check the bo here?
    if (mos_bo_busy(surface->bo))
    {
        // busy
        *status = VASurfaceRendering;
    }
    else
    {
        // idle
        *status = VASurfaceReady;
    }

    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_QuerySurfaceError(
    VADriverContextP ctx,
    VASurfaceID render_target,
    VAStatus error_status,
    void **error_info /*out*/
)
{
    DDI_UNUSED(error_status);

    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_SURFACE *surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, render_target);
    DDI_CHK_NULL(surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);

    PDDI_DECODE_CONTEXT decCtx = (PDDI_DECODE_CONTEXT)surface->pDecCtx;
    DDI_CHK_NULL(decCtx, "nullptr surface->pDecCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    VASurfaceDecodeMBErrors *surfaceErrors = decCtx->vaSurfDecErrOutput;
    DDI_CHK_NULL(surfaceErrors, "nullptr surfaceErrors", VA_STATUS_ERROR_INVALID_CONTEXT);

    VAStatus vaStatus = VA_STATUS_SUCCESS;

    DdiMediaUtil_LockMutex(&mediaCtx->SurfaceMutex);
    if (surface->curStatusReportQueryState == DDI_MEDIA_STATUS_REPORT_QUERY_STATE_COMPLETED)
    {
        if (error_status != -1 && surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_DECODER &&
            surface->curStatusReport.decode.status == CODECHAL_STATUS_ERROR)
        {
            surfaceErrors[1].status = -1;
            surfaceErrors[0].status = 2;
            surfaceErrors[0].start_mb = 0;
            surfaceErrors[0].end_mb = 0;
            surfaceErrors[0].num_mb = surface->curStatusReport.decode.errMbNum;
            surfaceErrors[0].decode_error_type = VADecodeMBError;
            *error_info = surfaceErrors;
            DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);
            return VA_STATUS_SUCCESS;
        }

        if (error_status == -1 && surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_DECODER)
        //&& surface->curStatusReport.decode.status == CODECHAL_STATUS_SUCCESSFUL)  // get the crc value whatever the status is
        {
            CodechalDecode *decoder = dynamic_cast<CodechalDecode *>(decCtx->pCodecHal);

            if (nullptr == decoder)
            {
                DDI_ASSERTMESSAGE("nullptr codechal decoder");
                vaStatus = VA_STATUS_ERROR_INVALID_CONTEXT;
            }
            else
            {
                if (decoder->GetStandard() != CODECHAL_AVC)
                {
                    vaStatus = VA_STATUS_ERROR_UNIMPLEMENTED;
                }
                else
                {
                    *error_info = (void *)&surface->curStatusReport.decode.crcValue;
                }
            }

            DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);
            return vaStatus;
        }

        if (surface->curCtxType == DDI_MEDIA_CONTEXT_TYPE_VP &&
            surface->curStatusReport.vpp.status == CODECHAL_STATUS_ERROR)
        {
            DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);
            return VA_STATUS_SUCCESS;
        }
    }

    surfaceErrors[0].status = -1;
    DdiMediaUtil_UnLockMutex(&mediaCtx->SurfaceMutex);
    return VA_STATUS_SUCCESS;
}

VAStatus DdiMedia_QuerySurfaceAttributes(
    VADriverContextP ctx,
    VAConfigID config_id,
    VASurfaceAttrib *attrib_list,
    uint32_t *num_attribs)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(num_attribs, "nullptr num_attribs", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

#ifdef CNM_VPUAPI_INTERFACE_CAP
    return VpuApiCapQuerySurfaceAttributes(config_id, attrib_list, num_attribs);
#else
    return mediaCtx->m_caps->QuerySurfaceAttributes(config_id,
                                                    attrib_list, num_attribs);
#endif
}

VAStatus DdiMedia_PutSurface(
    VADriverContextP ctx,
    VASurfaceID surface,
    void *draw,
    int16_t srcx,
    int16_t srcy,
    uint16_t srcw,
    uint16_t srch,
    int16_t destx,
    int16_t desty,
    uint16_t destw,
    uint16_t desth,
    VARectangle *cliprects,    /* client supplied clip list */
    uint32_t number_cliprects, /* number of clip rects in the clip list */
    uint32_t flags             /* de-interlacing flags */
)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_PARAMETER);
    if (number_cliprects > 0)
    {
        DDI_CHK_NULL(cliprects, "nullptr cliprects", VA_STATUS_ERROR_INVALID_PARAMETER);
    }

    void *vpCtx = nullptr;
    PDDI_MEDIA_CONTEXT mediaDrvCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(mediaDrvCtx, "nullptr mediaDrvCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaDrvCtx->pSurfaceHeap, "nullptr mediaDrvCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_LESS((uint32_t)surface, mediaDrvCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface", VA_STATUS_ERROR_INVALID_SURFACE);

    if (nullptr != mediaDrvCtx->pVpCtxHeap->pHeapBase)
    {
        uint32_t ctxType = DDI_MEDIA_CONTEXT_TYPE_NONE;
        vpCtx = DdiMedia_GetContextFromContextID(ctx, (VAContextID)(0 + DDI_MEDIA_VACONTEXTID_OFFSET_VP), &ctxType);
    }

#if defined(ANDROID) || !defined(X11_FOUND)
    return VA_STATUS_ERROR_UNIMPLEMENTED;
#else
    if (nullptr == vpCtx)
    {
        VAContextID context = VA_INVALID_ID;
        VAStatus vaStatus = DdiVp_CreateContext(ctx, 0, 0, 0, 0, 0, 0, &context);
        DDI_CHK_RET(vaStatus, "Create VP Context failed");
    }
    return DdiCodec_PutSurfaceLinuxHW(ctx, surface, draw, srcx, srcy, srcw, srch, destx, desty, destw, desth, cliprects, number_cliprects, flags);
#endif
}

VAStatus DdiMedia_QueryImageFormats(
    VADriverContextP ctx,
    VAImageFormat *format_list,
    int32_t *num_formats)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr pointer.", VA_STATUS_ERROR_INVALID_PARAMETER);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    VAStatus status;
    int num;
    num = 0;
    memset(format_list, 0x00, sizeof(s_supportedImageformatsVPU));

    for (uint32_t idx = 0; idx < sizeof(s_supportedImageformatsVPU) / sizeof(s_supportedImageformatsVPU[0]); idx++)
    {
        format_list[num].fourcc = s_supportedImageformatsVPU[idx].fourcc;
        format_list[num].byte_order = s_supportedImageformatsVPU[idx].byte_order;
        format_list[num].bits_per_pixel = s_supportedImageformatsVPU[idx].bits_per_pixel;
        format_list[num].depth = s_supportedImageformatsVPU[idx].depth;
        format_list[num].red_mask = s_supportedImageformatsVPU[idx].red_mask;
        format_list[num].green_mask = s_supportedImageformatsVPU[idx].green_mask;
        format_list[num].blue_mask = s_supportedImageformatsVPU[idx].blue_mask;
        format_list[num].alpha_mask = s_supportedImageformatsVPU[idx].alpha_mask;
        num++;
    }
    *num_formats = num;
    status = VA_STATUS_SUCCESS;
    return status;
#else
    return mediaCtx->m_caps->QueryImageFormats(format_list, num_formats);
#endif
}

VAStatus DdiMedia_CreateImage(
    VADriverContextP ctx,
    VAImageFormat *format,
    int32_t width,
    int32_t height,
    VAImage *image /* out */
)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "Invalid context!", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(format, "Invalid format!", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(image, "Invalid image!", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(width, 0, "Invalid width!", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_LARGER(height, 0, "Invalid height!", VA_STATUS_ERROR_INVALID_PARAMETER);
    int32_t event[] = {width, height, format->fourcc};
    MOS_TraceEventExt(EVENT_VA_IMAGE, EVENT_TYPE_START, event, sizeof(event), nullptr, 0);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(mediaCtx->pGmmClientContext, "nullptr mediaCtx->pGmmClientContext.", VA_STATUS_ERROR_INVALID_PARAMETER);

    VAImage *vaimg = (VAImage *)MOS_AllocAndZeroMemory(sizeof(VAImage));
    DDI_CHK_NULL(vaimg, "Insufficient to allocate an VAImage.", VA_STATUS_ERROR_ALLOCATION_FAILED);

    GMM_RESCREATE_PARAMS gmmParams;
    GMM_RESOURCE_INFO *gmmResourceInfo;
    MOS_ZeroMemory(&gmmParams, sizeof(gmmParams));

    gmmParams.BaseWidth = width;
    gmmParams.BaseHeight = height;
    gmmParams.ArraySize = 1;
    gmmParams.Type = RESOURCE_2D;
    gmmParams.Flags.Gpu.Video = true;
    gmmParams.Format = mediaCtx->m_caps->ConvertFourccToGmmFmt(format->fourcc);

    if (gmmParams.Format == GMM_FORMAT_INVALID)
    {
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
    gmmResourceInfo = mediaCtx->pGmmClientContext->CreateResInfoObject(&gmmParams);
    if (nullptr == gmmResourceInfo)
    {
        DDI_ASSERTMESSAGE("Gmm Create Resource Failed.");
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    // Get offset from GMM
    GMM_REQ_OFFSET_INFO reqInfo = {0};
    reqInfo.Plane = GMM_PLANE_U;
    reqInfo.ReqRender = 1;
    gmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetU = reqInfo.Render.Offset;
    MOS_ZeroMemory(&reqInfo, sizeof(GMM_REQ_OFFSET_INFO));
    reqInfo.Plane = GMM_PLANE_V;
    reqInfo.ReqRender = 1;
    gmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetV = reqInfo.Render.Offset;
    uint32_t size = (uint32_t)gmmResourceInfo->GetSizeSurface();
    uint32_t pitch = (uint32_t)gmmResourceInfo->GetRenderPitch();
    vaimg->format = *format;
    vaimg->format.byte_order = VA_LSB_FIRST;
    vaimg->width = width;
    vaimg->height = height;
    vaimg->data_size = size;

    mediaCtx->pGmmClientContext->DestroyResInfoObject(gmmResourceInfo);

    switch (format->fourcc)
    {
    case VA_FOURCC_RGBA:
    case VA_FOURCC_BGRA:
    case VA_FOURCC_ARGB:
    case VA_FOURCC_ABGR:
    case VA_FOURCC_BGRX:
    case VA_FOURCC_RGBX:
    case VA_FOURCC_XRGB:
    case VA_FOURCC_XBGR:
    case VA_FOURCC_A2R10G10B10:
    case VA_FOURCC_A2B10G10R10:
    case VA_FOURCC_X2R10G10B10:
    case VA_FOURCC_X2B10G10R10:
    case VA_FOURCC_R8G8B8:
    case VA_FOURCC_RGB565:
    case VA_FOURCC_UYVY:
    case VA_FOURCC_YUY2:
    case VA_FOURCC_VYUY:
    case VA_FOURCC_YVYU:
    case VA_FOURCC_AYUV:
    case VA_FOURCC_Y210:
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y212:
#endif
    case VA_FOURCC_Y216:
    case VA_FOURCC_Y410:
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y412:
#endif
    case VA_FOURCC_Y416:
    case VA_FOURCC_Y800:
        vaimg->num_planes = 1;
        vaimg->pitches[0] = pitch;
        vaimg->offsets[0] = 0;
        break;
    case VA_FOURCC_NV12:
    case VA_FOURCC_NV21:
    case VA_FOURCC_P010:
    case VA_FOURCC_P012:
    case VA_FOURCC_P016:
        vaimg->num_planes = 2;
        vaimg->pitches[0] = pitch;
        vaimg->pitches[1] = pitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = offsetU;
        break;
    case VA_FOURCC_YV12:
        vaimg->num_planes = 3;
        vaimg->pitches[0] = pitch;
        vaimg->pitches[1] = pitch / 2;
        vaimg->pitches[2] = pitch / 2;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = offsetV;
        vaimg->offsets[2] = offsetU;
        break;
    case VA_FOURCC_I420:
        vaimg->num_planes = 3;
        vaimg->pitches[0] = pitch;
        vaimg->pitches[1] = pitch / 2;
        vaimg->pitches[2] = pitch / 2;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = offsetU;
        vaimg->offsets[2] = offsetV;
        break;
    case VA_FOURCC_IMC3:
    case VA_FOURCC_411P:
    case VA_FOURCC_422V:
    case VA_FOURCC_422H:
    case VA_FOURCC_444P:
    case VA_FOURCC_RGBP:
    case VA_FOURCC_BGRP:
        vaimg->num_planes = 3;
        vaimg->pitches[0] = pitch;
        vaimg->pitches[1] = pitch;
        vaimg->pitches[2] = pitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = offsetU;
        vaimg->offsets[2] = offsetV;
        break;
    default:
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }

    DDI_MEDIA_BUFFER *buf = (DDI_MEDIA_BUFFER *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_BUFFER));
    if (nullptr == buf)
    {
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    buf->uiNumElements = 1;
    buf->iSize = vaimg->data_size;
    buf->uiType = VAImageBufferType;
    buf->format = Media_Format_CPU; // DdiCodec_OsFormatToMediaFormat(vaimg->format.fourcc); //Media_Format_Buffer;
    buf->uiOffset = 0;
    buf->pMediaCtx = mediaCtx;

    // Put Image in untiled buffer for better CPU access?
    VAStatus status = DdiMediaUtil_CreateBuffer(buf, mediaCtx->pDrmBufMgr);
    if ((status != VA_STATUS_SUCCESS))
    {
        MOS_FreeMemory(vaimg);
        MOS_FreeMemory(buf);
        return status;
    }
    buf->TileType = I915_TILING_NONE;

    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufferHeapElement = DdiMediaUtil_AllocPMediaBufferFromHeap(mediaCtx->pBufferHeap);

    if (nullptr == bufferHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        MOS_FreeMemory(vaimg);
        DdiMediaUtil_FreeBuffer(buf);
        MOS_FreeMemory(buf);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }

    bufferHeapElement->pBuffer = buf;
    bufferHeapElement->pCtx = nullptr;
    bufferHeapElement->uiCtxType = DDI_MEDIA_CONTEXT_TYPE_MEDIA;

    vaimg->buf = bufferHeapElement->uiVaBufferID;
    mediaCtx->uiNumBufs++;
    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);

    DdiMediaUtil_LockMutex(&mediaCtx->ImageMutex);
    PDDI_MEDIA_IMAGE_HEAP_ELEMENT imageHeapElement = DdiMediaUtil_AllocPVAImageFromHeap(mediaCtx->pImageHeap);
    if (nullptr == imageHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    imageHeapElement->pImage = vaimg;
    mediaCtx->uiNumImages++;
    vaimg->image_id = imageHeapElement->uiVaImageID;
    DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);

    *image = *vaimg;
    MOS_TraceEventExt(EVENT_VA_IMAGE, EVENT_TYPE_END, &vaimg->image_id, sizeof(VAGenericID), nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Derive image
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] surface
//!         VA surface ID
//! \param  [in] image
//!         VA image
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_DeriveImage(
    VADriverContextP ctx,
    VASurfaceID surface,
    VAImage *image)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_DERIVE, EVENT_TYPE_START, nullptr, 0, nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(image, "nullptr image", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)surface, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface", VA_STATUS_ERROR_INVALID_SURFACE);

    VAImage *vaimg = (VAImage *)MOS_AllocAndZeroMemory(sizeof(VAImage));
    DDI_CHK_NULL(vaimg, "nullptr vaimg", VA_STATUS_ERROR_ALLOCATION_FAILED);

    if (mediaSurface->pCurrentFrameSemaphore)
    {
        DdiMediaUtil_WaitSemaphore(mediaSurface->pCurrentFrameSemaphore);
        DdiMediaUtil_PostSemaphore(mediaSurface->pCurrentFrameSemaphore);
    }
    DdiMediaUtil_LockMutex(&mediaCtx->ImageMutex);
    PDDI_MEDIA_IMAGE_HEAP_ELEMENT imageHeapElement = DdiMediaUtil_AllocPVAImageFromHeap(mediaCtx->pImageHeap);
    if (nullptr == imageHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);
        MOS_FreeMemory(vaimg);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    imageHeapElement->pImage = vaimg;
    mediaCtx->uiNumImages++;
    vaimg->image_id = imageHeapElement->uiVaImageID;
    DdiMediaUtil_UnLockMutex(&mediaCtx->ImageMutex);

    vaimg->format.fourcc = DdiMedia_MediaFormatToOsFormat(mediaSurface->format);
    vaimg->width = mediaSurface->iWidth;
    vaimg->height = mediaSurface->iRealHeight;
    vaimg->format.byte_order = VA_LSB_FIRST;

    GMM_RESOURCE_INFO *gmmResourceInfo = mediaSurface->pGmmResourceInfo;
    GMM_REQ_OFFSET_INFO reqInfo = {0};
    reqInfo.Plane = GMM_PLANE_U;
    reqInfo.ReqRender = 1;
    gmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetU = reqInfo.Render.Offset;
    MOS_ZeroMemory(&reqInfo, sizeof(GMM_REQ_OFFSET_INFO));
    reqInfo.Plane = GMM_PLANE_V;
    reqInfo.ReqRender = 1;
    gmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetV = reqInfo.Render.Offset;
    vaimg->data_size = (uint32_t)gmmResourceInfo->GetSizeSurface();

    switch (mediaSurface->format)
    {
    case Media_Format_YV12:
    case Media_Format_I420:
        vaimg->format.bits_per_pixel = 12;
        vaimg->num_planes = 3;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->pitches[1] =
            vaimg->pitches[2] = mediaSurface->iPitch / 2;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = vaimg->offsets[1] + vaimg->pitches[1] * MOS_ALIGN_CEIL(mediaSurface->iHeight, 2) / 2;
        break;
    case Media_Format_A8B8G8R8:
    case Media_Format_R8G8B8A8:
    case Media_Format_A8R8G8B8:
        vaimg->format.bits_per_pixel = 32;
        vaimg->format.alpha_mask = RGB_8BIT_ALPHAMASK;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_X8R8G8B8:
    case Media_Format_X8B8G8R8:
        vaimg->format.bits_per_pixel = 32;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_R10G10B10A2:
    case Media_Format_B10G10R10A2:
        vaimg->format.bits_per_pixel = 32;
        vaimg->format.alpha_mask = RGB_10BIT_ALPHAMASK;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_R10G10B10X2:
    case Media_Format_B10G10R10X2:
        vaimg->format.bits_per_pixel = 32;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_R5G6B5:
        vaimg->format.bits_per_pixel = 16;
        vaimg->data_size = mediaSurface->iPitch * mediaSurface->iHeight;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_R8G8B8:
        vaimg->format.bits_per_pixel = 24;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_YUY2:
    case Media_Format_UYVY:
        vaimg->format.bits_per_pixel = 16;
        vaimg->data_size = mediaSurface->iPitch * mediaSurface->iHeight;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_400P:
        vaimg->format.bits_per_pixel = 8;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    case Media_Format_444P:
    case Media_Format_RGBP:
    case Media_Format_BGRP:
        vaimg->format.bits_per_pixel = 24;
        vaimg->num_planes = 3;
        vaimg->pitches[0] =
            vaimg->pitches[1] =
                vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = mediaSurface->iHeight * mediaSurface->iPitch * 2;
        break;
    case Media_Format_IMC3:
        vaimg->format.bits_per_pixel = 12;
        vaimg->num_planes = 3;
        vaimg->pitches[0] =
            vaimg->pitches[1] =
                vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = mediaSurface->iHeight * mediaSurface->iPitch * 3 / 2;
        break;
    case Media_Format_411P:
        vaimg->format.bits_per_pixel = 12;
        vaimg->num_planes = 3;
        vaimg->pitches[0] =
            vaimg->pitches[1] =
                vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = mediaSurface->iHeight * mediaSurface->iPitch * 2;
        break;
    case Media_Format_422V:
        vaimg->format.bits_per_pixel = 16;
        vaimg->num_planes = 3;
        vaimg->pitches[0] =
            vaimg->pitches[1] =
                vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = mediaSurface->iHeight * mediaSurface->iPitch * 3 / 2;
        break;
    case Media_Format_422H:
        vaimg->format.bits_per_pixel = 16;
        vaimg->num_planes = 3;
        vaimg->pitches[0] =
            vaimg->pitches[1] =
                vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = mediaSurface->iHeight * mediaSurface->iPitch * 2;
        break;
    case Media_Format_P010:
    case Media_Format_P012:
    case Media_Format_P016:
        vaimg->format.bits_per_pixel = 24;
        vaimg->num_planes = 2;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->pitches[1] =
            vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
        vaimg->offsets[2] = vaimg->offsets[1] + 2;
        break;
    case Media_Format_Y410:
    case Media_Format_AYUV:
    case Media_Format_Y210:
#if VA_CHECK_VERSION(1, 9, 0)
    case Media_Format_Y212:
#endif
    case Media_Format_Y216:
        vaimg->format.bits_per_pixel = 32;
        vaimg->data_size = mediaSurface->iPitch * mediaSurface->iHeight;
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
#if VA_CHECK_VERSION(1, 9, 0)
    case Media_Format_Y412:
#endif
    case Media_Format_Y416:
        vaimg->format.bits_per_pixel = 64; // packed format [alpha, Y, U, V], 16 bits per channel
        vaimg->num_planes = 1;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        break;
    default:
        vaimg->format.bits_per_pixel = 12;
        vaimg->num_planes = 2;
        vaimg->pitches[0] = mediaSurface->iPitch;
        vaimg->pitches[1] =
            vaimg->pitches[2] = mediaSurface->iPitch;
        vaimg->offsets[0] = 0;
        if (MEDIA_IS_WA(&mediaCtx->WaTable, WaDisableGmmLibOffsetInDeriveImage))
        {
            vaimg->offsets[1] = mediaSurface->iHeight * mediaSurface->iPitch;
            vaimg->offsets[2] = vaimg->offsets[1] + 1;
        }
        else
        {
            vaimg->offsets[1] = offsetU;
            vaimg->offsets[2] = offsetV;
        }
        break;
    }

    mediaCtx->m_caps->PopulateColorMaskInfo(&vaimg->format);

    DDI_MEDIA_BUFFER *buf = (DDI_MEDIA_BUFFER *)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_BUFFER));
    if (buf == nullptr)
    {
        MOS_FreeMemory(vaimg);
        MOS_FreeMemory(buf);
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }
    buf->uiNumElements = 1;
    buf->iSize = vaimg->data_size;
    buf->uiType = VAImageBufferType;
    buf->format = mediaSurface->format;
    buf->uiOffset = 0;

    buf->bo = mediaSurface->bo;
    buf->format = mediaSurface->format;
    buf->TileType = mediaSurface->TileType;
    buf->pSurface = mediaSurface;
    mos_bo_reference(mediaSurface->bo);

    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    PDDI_MEDIA_BUFFER_HEAP_ELEMENT bufferHeapElement = DdiMediaUtil_AllocPMediaBufferFromHeap(mediaCtx->pBufferHeap);

    if (nullptr == bufferHeapElement)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        MOS_FreeMemory(vaimg);
        MOS_FreeMemory(buf);
        return VA_STATUS_ERROR_MAX_NUM_EXCEEDED;
    }
    bufferHeapElement->pBuffer = buf;
    bufferHeapElement->pCtx = nullptr;
    bufferHeapElement->uiCtxType = DDI_MEDIA_CONTEXT_TYPE_MEDIA;

    vaimg->buf = bufferHeapElement->uiVaBufferID;
    mediaCtx->uiNumBufs++;
    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);

    *image = *vaimg;

    MOS_TraceEventExt(EVENT_VA_DERIVE, EVENT_TYPE_END, &surface, sizeof(surface), &vaimg->image_id, sizeof(VAGenericID));
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Free allocated surfaceheap elements
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] image
//!         VA image ID
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_DestroyImage(
    VADriverContextP ctx,
    VAImageID image)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_FREE_IMAGE, EVENT_TYPE_START, &image, sizeof(image), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(mediaCtx, "nullptr Media", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pImageHeap, "nullptr mediaCtx->pImageHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)image, mediaCtx->pImageHeap->uiAllocatedHeapElements, "Invalid image", VA_STATUS_ERROR_INVALID_IMAGE);

    VAImage *vaImage = DdiMedia_GetVAImageFromVAImageID(mediaCtx, image);
    if (vaImage == nullptr)
    {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }
    DdiMedia_DestroyBuffer(ctx, vaImage->buf);
    MOS_FreeMemory(vaImage);

    DdiMedia_DestroyImageFromVAImageID(mediaCtx, image);

    MOS_TraceEventExt(EVENT_VA_FREE_IMAGE, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Set image palette
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] image
//!         VA image ID
//! \param  [in] palette
//!         Palette
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED if call success, else fail reason
//!
VAStatus DdiMedia_SetImagePalette(
    VADriverContextP ctx,
    VAImageID image,
    unsigned char *palette)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(image);
    DDI_UNUSED(palette);
    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

VAStatus SwizzleSurface(PDDI_MEDIA_CONTEXT mediaCtx, PGMM_RESOURCE_INFO pGmmResInfo, void *pLockedAddr, uint32_t TileType, uint8_t *pResourceBase, bool bUpload)
{
    uint32_t uiSize, uiPitch;
    GMM_RES_COPY_BLT gmmResCopyBlt;
    uint32_t uiPicHeight;
    uint32_t ulSwizzledSize;
    VAStatus vaStatus = VA_STATUS_SUCCESS;

    DDI_CHK_NULL(pGmmResInfo, "pGmmResInfo is NULL", VA_STATUS_ERROR_OPERATION_FAILED);
    DDI_CHK_NULL(pLockedAddr, "pLockedAddr is NULL", VA_STATUS_ERROR_OPERATION_FAILED);
    DDI_CHK_NULL(pResourceBase, "pResourceBase is NULL", VA_STATUS_ERROR_ALLOCATION_FAILED);

    memset(&gmmResCopyBlt, 0x0, sizeof(GMM_RES_COPY_BLT));
    uiPicHeight = pGmmResInfo->GetBaseHeight();
    uiSize = pGmmResInfo->GetSizeSurface();
    uiPitch = pGmmResInfo->GetRenderPitch();
    gmmResCopyBlt.Gpu.pData = pLockedAddr;
    gmmResCopyBlt.Sys.pData = pResourceBase;
    gmmResCopyBlt.Sys.RowPitch = uiPitch;
    gmmResCopyBlt.Sys.BufferSize = uiSize;
    gmmResCopyBlt.Sys.SlicePitch = uiSize;
    gmmResCopyBlt.Blt.Slices = 1;
    gmmResCopyBlt.Blt.Upload = bUpload;

    if (mediaCtx->pGmmClientContext->IsPlanar(pGmmResInfo->GetResourceFormat()) == true)
    {
        gmmResCopyBlt.Blt.Width = pGmmResInfo->GetBaseWidth();
        gmmResCopyBlt.Blt.Height = uiSize / uiPitch;
    }

    pGmmResInfo->CpuBlt(&gmmResCopyBlt);

    return vaStatus;
}

//!
//! \brief  Copy plane from src to dst row by row when src and dst strides are different
//!
//! \param  [in] dst
//!         Destination plane
//! \param  [in] dstPitch
//!         Destination plane pitch
//! \param  [in] src
//!         Source plane
//! \param  [in] srcPitch
//!         Source plane pitch
//! \param  [in] height
//!         Plane hight
//!
static void DdiMedia_CopyPlane(
    uint8_t *dst,
    uint32_t dstPitch,
    uint8_t *src,
    uint32_t srcPitch,
    uint32_t height)
{
    uint32_t rowSize = std::min(dstPitch, srcPitch);
    for (int y = 0; y < height; y += 1)
    {
        memcpy(dst, src, rowSize);
        dst += dstPitch;
        src += srcPitch;
    }
}

//!
//! \brief  Copy data from surface to image
//!
//! \param  [in] ctx
//!         Input driver context
//! \param  [in] surface
//!         Pointer to surface
//! \param  [in] image
//!         Pointer to image
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
static VAStatus DdiMedia_CopySurfaceToImage(
    VADriverContextP ctx,
    DDI_MEDIA_SURFACE *surface,
    VAImage *image)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx.", VA_STATUS_ERROR_INVALID_CONTEXT);
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(surface, "nullptr meida surface.", VA_STATUS_ERROR_INVALID_BUFFER);

    VAStatus vaStatus = VA_STATUS_SUCCESS;
    // Lock Surface
    if ((Media_Format_CPU != surface->format))
    {
        vaStatus = DdiMedia_MediaMemoryDecompress(mediaCtx, surface);

        if (vaStatus != VA_STATUS_SUCCESS)
        {
            DDI_NORMALMESSAGE("surface Decompression fail, continue next steps.");
        }
    }
    void *surfData = DdiMediaUtil_LockSurface(surface, (MOS_LOCKFLAG_READONLY | MOS_LOCKFLAG_NO_SWIZZLE));
    if (surfData == nullptr)
    {
        DDI_ASSERTMESSAGE("nullptr surfData.");
        return vaStatus;
    }

    void *imageData = nullptr;
    vaStatus = DdiMedia_MapBuffer(ctx, image->buf, &imageData);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Failed to map buffer.");
        DdiMediaUtil_UnlockSurface(surface);
        return vaStatus;
    }

    uint8_t *ySrc = nullptr;
    uint8_t *yDst = (uint8_t *)imageData;
    uint8_t *swizzleData = (uint8_t *)MOS_AllocMemory(surface->data_size);

    if (!surface->pMediaCtx->bIsAtomSOC && surface->TileType != I915_TILING_NONE)
    {
        SwizzleSurface(surface->pMediaCtx, surface->pGmmResourceInfo, surfData, (MOS_TILE_TYPE)surface->TileType, (uint8_t *)swizzleData, false);
        ySrc = swizzleData;
    }
    else
    {
        ySrc = (uint8_t *)surfData;
    }

    DdiMedia_CopyPlane(yDst, image->pitches[0], ySrc, surface->iPitch, image->height);
    if (image->num_planes > 1)
    {
        uint8_t *uSrc = ySrc + surface->iPitch * surface->iHeight;
        uint8_t *uDst = yDst + image->offsets[1];
        uint32_t chromaPitch = 0;
        uint32_t chromaHeight = 0;
        uint32_t imageChromaPitch = 0;
        uint32_t imageChromaHeight = 0;
        DdiMedia_GetChromaPitchHeight(DdiMedia_MediaFormatToOsFormat(surface->format), surface->iPitch, surface->iHeight, &chromaPitch, &chromaHeight);
        DdiMedia_GetChromaPitchHeight(image->format.fourcc, image->pitches[0], image->height, &imageChromaPitch, &imageChromaHeight);
        DdiMedia_CopyPlane(uDst, image->pitches[1], uSrc, chromaPitch, imageChromaHeight);

        if (image->num_planes > 2)
        {
            uint8_t *vSrc = uSrc + chromaPitch * chromaHeight;
            uint8_t *vDst = yDst + image->offsets[2];
            DdiMedia_CopyPlane(vDst, image->pitches[2], vSrc, chromaPitch, imageChromaHeight);
        }
    }

    MOS_FreeMemory(swizzleData);

    vaStatus = DdiMedia_UnmapBuffer(ctx, image->buf);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Failed to unmap buffer.");
        DdiMediaUtil_UnlockSurface(surface);
        return vaStatus;
    }

    DdiMediaUtil_UnlockSurface(surface);

    return vaStatus;
}

//!
//! \brief  Retrive surface data into a VAImage
//! \details    Image must be in a format supported by the implementation
//!
//! \param  [in] ctx
//!         Input driver context
//! \param  [in] surface
//!         Input surface ID of source
//! \param  [in] x
//!         X offset of the wanted region
//! \param  [in] y
//!         Y offset of the wanted region
//! \param  [in] width
//!         Width of the wanted region
//! \param  [in] height
//!         Height of the wanted region
//! \param  [in] image
//!     The image ID of the source image
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_GetImage(
    VADriverContextP ctx,
    VASurfaceID surface,
    int32_t x, /* coordinates of the upper left source pixel */
    int32_t y,
    uint32_t width, /* width and height of the region */
    uint32_t height,
    VAImageID image)
{
    DDI_FUNCTION_ENTER();
    uint32_t event[] = {surface, x, y, width, height, image};
    MOS_TraceEventExt(EVENT_VA_GET, EVENT_TYPE_START, &event, sizeof(event), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx.", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pImageHeap, "nullptr mediaCtx->pImageHeap.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)surface, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface.", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_LESS((uint32_t)image, mediaCtx->pImageHeap->uiAllocatedHeapElements, "Invalid image.", VA_STATUS_ERROR_INVALID_IMAGE);

    VAImage *vaimg = DdiMedia_GetVAImageFromVAImageID(mediaCtx, image);
    DDI_CHK_NULL(vaimg, "nullptr vaimg.", VA_STATUS_ERROR_INVALID_IMAGE);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, vaimg->buf);
    DDI_CHK_NULL(buf, "nullptr buf.", VA_STATUS_ERROR_INVALID_BUFFER);

    DDI_MEDIA_SURFACE *inputSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);
    DDI_CHK_NULL(inputSurface, "nullptr inputSurface.", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(inputSurface->bo, "nullptr inputSurface->bo.", VA_STATUS_ERROR_INVALID_SURFACE);

    VAStatus vaStatus = VA_STATUS_SUCCESS;
#ifndef _FULL_OPEN_SOURCE
    VASurfaceID target_surface = VA_INVALID_SURFACE;
    VASurfaceID output_surface = surface;

    if (inputSurface->format != DdiMedia_OsFormatToMediaFormat(vaimg->format.fourcc, vaimg->format.alpha_mask) ||
        width != vaimg->width || height != vaimg->height ||
        (MEDIA_IS_WA(&mediaCtx->WaTable, WaEnableVPPCopy) &&
         vaimg->format.fourcc != VA_FOURCC_444P &&
         vaimg->format.fourcc != VA_FOURCC_422V &&
         vaimg->format.fourcc != VA_FOURCC_422H))
    {
        VAContextID context = VA_INVALID_ID;
        // Create VP Context.
        vaStatus = DdiVp_CreateContext(ctx, 0, 0, 0, 0, 0, 0, &context);
        DDI_CHK_RET(vaStatus, "Create VP Context failed.");

        // Create target surface for VP pipeline.
        DDI_MEDIA_FORMAT mediaFmt = DdiMedia_OsFormatToMediaFormat(vaimg->format.fourcc, vaimg->format.fourcc);
        if (mediaFmt == Media_Format_Count)
        {
            DDI_ASSERTMESSAGE("Unsupported surface type.");
            DdiVp_DestroyContext(ctx, context);
            return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
        }
        PDDI_MEDIA_SURFACE_DESCRIPTOR surfDesc = (PDDI_MEDIA_SURFACE_DESCRIPTOR)MOS_AllocAndZeroMemory(sizeof(DDI_MEDIA_SURFACE_DESCRIPTOR));
        if (!surfDesc)
        {
            DDI_ASSERTMESSAGE("nullptr surfDesc.");
            DdiVp_DestroyContext(ctx, context);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }
        surfDesc->uiVaMemType = VA_SURFACE_ATTRIB_MEM_TYPE_VA;
        int memType = MOS_MEMPOOL_VIDEOMEMORY;
        if (MEDIA_IS_SKU(&mediaCtx->SkuTable, FtrLocalMemory))
        {
            memType = MOS_MEMPOOL_SYSTEMMEMORY;
        }
        target_surface = (VASurfaceID)DdiMedia_CreateRenderTarget(mediaCtx, mediaFmt, vaimg->width, vaimg->height, surfDesc, VA_SURFACE_ATTRIB_USAGE_HINT_GENERIC, memType);
        if (VA_INVALID_SURFACE == target_surface)
        {
            DDI_ASSERTMESSAGE("Create temp surface failed.");
            DdiVp_DestroyContext(ctx, context);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        VARectangle srcRect, dstRect;
        srcRect.x = x;
        srcRect.y = y;
        srcRect.width = width;
        srcRect.height = height;
        dstRect.x = 0;
        dstRect.y = 0;
        dstRect.width = vaimg->width;
        dstRect.height = vaimg->height;

        // Execute VP pipeline.
        vaStatus = DdiVp_VideoProcessPipeline(ctx, context, surface, &srcRect, target_surface, &dstRect);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("VP Pipeline failed.");
            DdiMedia_DestroySurfaces(ctx, &target_surface, 1);
            DdiVp_DestroyContext(ctx, context);
            return vaStatus;
        }
        vaStatus = DdiMedia_SyncSurface(ctx, target_surface);
        vaStatus = DdiVp_DestroyContext(ctx, context);
        output_surface = target_surface;
    }

    // Get Media Surface from output surface ID
    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, output_surface);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface.", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(mediaSurface->bo, "nullptr mediaSurface->bo.", VA_STATUS_ERROR_INVALID_SURFACE);

    vaStatus = DdiMedia_CopySurfaceToImage(ctx, mediaSurface, vaimg);

    if (vaStatus != MOS_STATUS_SUCCESS)
    {
        DDI_ASSERTMESSAGE("Failed to copy surface to image buffer data!");
        if (target_surface != VA_INVALID_SURFACE)
        {
            DdiMedia_DestroySurfaces(ctx, &target_surface, 1);
        }
        return vaStatus;
    }

    // Destroy temp surface if created
    if (target_surface != VA_INVALID_SURFACE)
    {
        DdiMedia_DestroySurfaces(ctx, &target_surface, 1);
    }
#else
    vaStatus = DdiMedia_CopySurfaceToImage(ctx, inputSurface, vaimg);
    DDI_CHK_RET(vaStatus, "Copy surface to image failed.");
#endif
    MOS_TraceEventExt(EVENT_VA_GET, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Copy data from a VAImage to a surface
//! \details    Image must be in a format supported by the implementation
//!
//! \param  [in] ctx
//!         Input driver context
//! \param  [in] surface
//!         Surface ID of destination
//! \param  [in] image
//!         The image ID of the destination image
//! \param  [in] src_x
//!         Source x offset of the image region
//! \param  [in] src_y
//!         Source y offset of the image region
//! \param  [in] src_width
//!         Source width offset of the image region
//! \param  [in] src_height
//!         Source height offset of the image region
//! \param  [in] dest_x
//!         Destination x offset of the surface region
//! \param  [in] dest_y
//!         Destination y offset of the surface region
//! \param  [in] dest_width
//!         Destination width offset of the surface region
//! \param  [in] dest_height
//!         Destination height offset of the surface region
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_PutImage(
    VADriverContextP ctx,
    VASurfaceID surface,
    VAImageID image,
    int32_t src_x,
    int32_t src_y,
    uint32_t src_width,
    uint32_t src_height,
    int32_t dest_x,
    int32_t dest_y,
    uint32_t dest_width,
    uint32_t dest_height)
{
    DDI_FUNCTION_ENTER();
    uint32_t event[] = {surface, image, src_x, src_y, src_width, src_height, dest_x, dest_y, dest_width, dest_height};
    MOS_TraceEventExt(EVENT_VA_PUT, EVENT_TYPE_START, &event, sizeof(event), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx.", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx.", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pImageHeap, "nullptr mediaCtx->pImageHeap.", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)surface, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface.", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_LESS((uint32_t)image, mediaCtx->pImageHeap->uiAllocatedHeapElements, "Invalid image.", VA_STATUS_ERROR_INVALID_IMAGE);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface.", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(mediaSurface->bo, "Invalid buffer.", VA_STATUS_ERROR_INVALID_BUFFER);

    if (mediaSurface->pCurrentFrameSemaphore)
    {
        DdiMediaUtil_WaitSemaphore(mediaSurface->pCurrentFrameSemaphore);
        DdiMediaUtil_PostSemaphore(mediaSurface->pCurrentFrameSemaphore);
    }

    VAImage *vaimg = DdiMedia_GetVAImageFromVAImageID(mediaCtx, image);
    DDI_CHK_NULL(vaimg, "Invalid image.", VA_STATUS_ERROR_INVALID_IMAGE);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, vaimg->buf);
    DDI_CHK_NULL(buf, "Invalid buffer.", VA_STATUS_ERROR_INVALID_BUFFER);

    VAStatus vaStatus = VA_STATUS_SUCCESS;
    void *imageData = nullptr;

    vaStatus = DdiMedia_MapBuffer(ctx, vaimg->buf, &imageData);
    DDI_CHK_RET(vaStatus, "MapBuffer failed.");
    DDI_CHK_NULL(imageData, "nullptr imageData.", VA_STATUS_ERROR_INVALID_IMAGE);

    // VP Pipeline will be called for CSC/Scaling if the surface format or data size is not consistent with image.
    if (mediaSurface->format != DdiMedia_OsFormatToMediaFormat(vaimg->format.fourcc, vaimg->format.alpha_mask) ||
        dest_width != src_width || dest_height != src_height ||
        src_x != 0 || dest_x != 0 || src_y != 0 || dest_y != 0)
    {
        VAContextID context = VA_INVALID_ID;

        // Create VP Context.
        vaStatus = DdiVp_CreateContext(ctx, 0, 0, 0, 0, 0, 0, &context);
        DDI_CHK_RET(vaStatus, "Create VP Context failed");

        // Create temp surface for VP pipeline.
        DDI_MEDIA_FORMAT mediaFmt = DdiMedia_OsFormatToMediaFormat(vaimg->format.fourcc, vaimg->format.fourcc);
        if (mediaFmt == Media_Format_Count)
        {
            DDI_ASSERTMESSAGE("Unsupported surface type.");
            return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
        }

        int memType = MOS_MEMPOOL_VIDEOMEMORY;
        if (MEDIA_IS_SKU(&mediaCtx->SkuTable, FtrLocalMemory))
        {
            memType = MOS_MEMPOOL_SYSTEMMEMORY;
        }
        VASurfaceID tempSurface = (VASurfaceID)DdiMedia_CreateRenderTarget(mediaCtx, mediaFmt, vaimg->width, vaimg->height, nullptr, VA_SURFACE_ATTRIB_USAGE_HINT_VPP_READ, memType);
        if (tempSurface == VA_INVALID_ID)
        {
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        DDI_MEDIA_SURFACE *tempMediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, tempSurface);
        DDI_CHK_NULL(tempMediaSurface, "nullptr tempMediaSurface.", VA_STATUS_ERROR_INVALID_SURFACE);

        // Lock Surface
        void *tempSurfData = DdiMediaUtil_LockSurface(tempMediaSurface, (MOS_LOCKFLAG_READONLY | MOS_LOCKFLAG_WRITEONLY));
        if (nullptr == tempSurfData)
        {
            DdiMedia_DestroySurfaces(ctx, &tempSurface, 1);
            return VA_STATUS_ERROR_SURFACE_BUSY;
        }

        // Copy data from image to temp surferce
        MOS_STATUS eStatus = MOS_SecureMemcpy(tempSurfData, vaimg->data_size, imageData, vaimg->data_size);
        if (eStatus != MOS_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("Failed to copy image to surface buffer.");
            DdiMediaUtil_UnlockSurface(tempMediaSurface);
            DdiMedia_DestroySurfaces(ctx, &tempSurface, 1);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        vaStatus = DdiMedia_UnmapBuffer(ctx, vaimg->buf);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("Failed to unmap buffer.");
            DdiMediaUtil_UnlockSurface(tempMediaSurface);
            DdiMedia_DestroySurfaces(ctx, &tempSurface, 1);
            return vaStatus;
        }

        DdiMediaUtil_UnlockSurface(tempMediaSurface);

        VARectangle srcRect, dstRect;
        srcRect.x = src_x;
        srcRect.y = src_y;
        srcRect.width = src_width;
        srcRect.height = src_height;
        dstRect.x = dest_x;
        dstRect.y = dest_y;
        dstRect.width = dest_width;
        dstRect.height = dest_height;

        // Execute VP pipeline.
        vaStatus = DdiVp_VideoProcessPipeline(ctx, context, tempSurface, &srcRect, surface, &dstRect);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("VP Pipeline failed.");
            DdiMedia_DestroySurfaces(ctx, &tempSurface, 1);
            return vaStatus;
        }

        vaStatus = DdiMedia_SyncSurface(ctx, tempSurface);
        DdiMedia_DestroySurfaces(ctx, &tempSurface, 1);
        vaStatus = DdiVp_DestroyContext(ctx, context);
    }
    else
    {
        // Lock Surface
        if ((nullptr != buf->pSurface) && (Media_Format_CPU != mediaSurface->format))
        {
            vaStatus = DdiMedia_MediaMemoryDecompress(mediaCtx, mediaSurface);

            if (vaStatus != VA_STATUS_SUCCESS)
            {
                DDI_NORMALMESSAGE("surface Decompression fail, continue next steps.");
            }
        }

        void *surfData = DdiMediaUtil_LockSurface(mediaSurface, (MOS_LOCKFLAG_READONLY | MOS_LOCKFLAG_WRITEONLY));
        if (nullptr == surfData)
        {
            DDI_ASSERTMESSAGE("Failed to lock surface.");
            return VA_STATUS_ERROR_SURFACE_BUSY;
        }

        if (src_width == dest_width && src_height == dest_height &&
            src_width == vaimg->width && src_height == vaimg->height &&
            src_width == mediaSurface->iWidth && src_height == mediaSurface->iHeight &&
            mediaSurface->data_size == vaimg->data_size)
        {
            // Copy data from image to surface
            MOS_STATUS eStatus = MOS_SecureMemcpy(surfData, vaimg->data_size, imageData, vaimg->data_size);
            DDI_CHK_CONDITION((eStatus != MOS_STATUS_SUCCESS), "Failed to copy image to surface buffer.", VA_STATUS_ERROR_OPERATION_FAILED);
        }
        else
        {
            uint8_t *ySrc = (uint8_t *)imageData + vaimg->offsets[0];
            uint8_t *yDst = (uint8_t *)surfData;
            DdiMedia_CopyPlane(yDst, mediaSurface->iPitch, ySrc, vaimg->pitches[0], src_height);

            if (vaimg->num_planes > 1)
            {
                DDI_MEDIA_SURFACE uPlane = *mediaSurface;

                uPlane.iWidth = src_width;
                uPlane.iRealHeight = src_height;
                uPlane.iHeight = src_height;
                uint32_t chromaHeight = 0;
                uint32_t chromaPitch = 0;
                DdiMedia_GetChromaPitchHeight(DdiMedia_MediaFormatToOsFormat(uPlane.format), uPlane.iPitch, uPlane.iHeight, &chromaPitch, &chromaHeight);

                uint8_t *uSrc = (uint8_t *)imageData + vaimg->offsets[1];
                uint8_t *uDst = yDst + mediaSurface->iPitch * mediaSurface->iHeight;
                DdiMedia_CopyPlane(uDst, chromaPitch, uSrc, vaimg->pitches[1], chromaHeight);
                if (vaimg->num_planes > 2)
                {
                    uint8_t *vSrc = (uint8_t *)imageData + vaimg->offsets[2];
                    uint8_t *vDst = uDst + chromaPitch * chromaHeight;
                    DdiMedia_CopyPlane(vDst, chromaPitch, vSrc, vaimg->pitches[2], chromaHeight);
                }
            }
        }

        vaStatus = DdiMedia_UnmapBuffer(ctx, vaimg->buf);
        if (vaStatus != VA_STATUS_SUCCESS)
        {
            DDI_ASSERTMESSAGE("Failed to unmap buffer.");
            DdiMediaUtil_UnlockSurface(mediaSurface);
            return vaStatus;
        }

        DdiMediaUtil_UnlockSurface(mediaSurface);
    }
    MOS_TraceEventExt(EVENT_VA_PUT, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Query subpicture formats
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] format_list
//!         VA image format
//! \param  [in] flags
//!         Flags
//! \param  [in] num_formats
//!         Number of formats
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_QuerySubpictureFormats(
    VADriverContextP ctx,
    VAImageFormat *format_list,
    uint32_t *flags,
    uint32_t *num_formats)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(format_list);
    DDI_UNUSED(flags);
    DDI_UNUSED(num_formats);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Create subpicture
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] image
//!         VA image ID
//! \param  [out] subpicture
//!         VA subpicture ID
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_CreateSubpicture(
    VADriverContextP ctx,
    VAImageID image,
    VASubpictureID *subpicture /* out */
)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(image);
    DDI_UNUSED(subpicture);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Destroy subpicture
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_DestroySubpicture(
    VADriverContextP ctx,
    VASubpictureID subpicture)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Set subpicture image
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//! \param  [in] image
//!         VA image ID
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_SetSubpictureImage(
    VADriverContextP ctx,
    VASubpictureID subpicture,
    VAImageID image)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);
    DDI_UNUSED(image);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Set subpicture chrome key
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//! \param  [in] chromakey_min
//!         Minimum chroma key
//! \param  [in] chromakey_max
//!         Maximum chroma key
//! \param  [in] chromakey_mask
//!         Chromakey mask
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_SetSubpictureChromakey(
    VADriverContextP ctx,
    VASubpictureID subpicture,
    uint32_t chromakey_min,
    uint32_t chromakey_max,
    uint32_t chromakey_mask)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);
    DDI_UNUSED(chromakey_min);
    DDI_UNUSED(chromakey_max);
    DDI_UNUSED(chromakey_mask);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  set subpicture global alpha
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//! \param  [in] global_alpha
//!         Global alpha
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
VAStatus DdiMedia_SetSubpictureGlobalAlpha(
    VADriverContextP ctx,
    VASubpictureID subpicture,
    float global_alpha)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);
    DDI_UNUSED(global_alpha);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Associate subpicture
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//! \param  [in] target_surfaces
//!         VA surface ID
//! \param  [in] num_surfaces
//!         Number of surfaces
//! \param  [in] src_x
//!         Source x of the region
//! \param  [in] src_y
//!         Source y of the region
//! \param  [in] src_width
//!         Source width of the region
//! \param  [in] src_height
//!         Source height of the region
//! \param  [in] dest_x
//!         Destination x
//! \param  [in] dest_y
//!         Destination y
//! \param  [in] dest_width
//!         Destination width
//! \param  [in] dest_height
//!         Destination height
//! \param  [in] flags
//!         Flags
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_AssociateSubpicture(
    VADriverContextP ctx,
    VASubpictureID subpicture,
    VASurfaceID *target_surfaces,
    int32_t num_surfaces,
    int16_t src_x, /* upper left offset in subpicture */
    int16_t src_y,
    uint16_t src_width,
    uint16_t src_height,
    int16_t dest_x, /* upper left offset in surface */
    int16_t dest_y,
    uint16_t dest_width,
    uint16_t dest_height,
    /*
     * whether to enable chroma-keying or global-alpha
     * see VA_SUBPICTURE_XXX values
     */
    uint32_t flags)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);
    DDI_UNUSED(target_surfaces);
    DDI_UNUSED(num_surfaces);
    DDI_UNUSED(src_x);
    DDI_UNUSED(src_y);
    DDI_UNUSED(src_width);
    DDI_UNUSED(src_height);
    DDI_UNUSED(dest_x);
    DDI_UNUSED(dest_y);
    DDI_UNUSED(dest_width);
    DDI_UNUSED(dest_height);
    DDI_UNUSED(flags);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Deassociate subpicture
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] subpicture
//!         VA subpicture ID
//! \param  [in] target_surfaces
//!         VA surface ID
//! \param  [in] num_surfaces
//!         Number of surfaces
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_DeassociateSubpicture(
    VADriverContextP ctx,
    VASubpictureID subpicture,
    VASurfaceID *target_surfaces,
    int32_t num_surfaces)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(subpicture);
    DDI_UNUSED(target_surfaces);
    DDI_UNUSED(num_surfaces);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Query display attributes
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] attr_list
//!         VA display attribute
//! \param  [in] num_attributes
//!         Number of attributes
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_QueryDisplayAttributes(
    VADriverContextP ctx,
    VADisplayAttribute *attr_list,
    int32_t *num_attributes)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(attr_list);

    DDI_FUNCTION_ENTER();

    if (num_attributes)
        *num_attributes = 0;

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Get display attributes
//! \details    This function returns the current attribute values in "attr_list".
//!         Only attributes returned with VA_DISPLAY_ATTRIB_GETTABLE set in the "flags" field
//!         from vaQueryDisplayAttributes() can have their values retrieved.
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] attr_list
//!         VA display attribute
//! \param  [in] num_attributes
//!         Number of attributes
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_GetDisplayAttributes(
    VADriverContextP ctx,
    VADisplayAttribute *attr_list,
    int32_t num_attributes)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(attr_list);
    DDI_UNUSED(num_attributes);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Set display attributes
//! \details    Only attributes returned with VA_DISPLAY_ATTRIB_SETTABLE set in the "flags" field
//!         from vaQueryDisplayAttributes() can be set.  If the attribute is not settable or
//!         the value is out of range, the function returns VA_STATUS_ERROR_ATTR_NOT_SUPPORTED
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] attr_list
//!         VA display attribute
//! \param  [in] num_attributes
//!         Number of attributes
//!
//! \return VAStatus
//!     VA_STATUS_ERROR_UNIMPLEMENTED
//!
VAStatus DdiMedia_SetDisplayAttributes(
    VADriverContextP ctx,
    VADisplayAttribute *attr_list,
    int32_t num_attributes)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(attr_list);
    DDI_UNUSED(num_attributes);

    DDI_FUNCTION_ENTER();

    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

//!
//! \brief  Query processing rate
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] config_id
//!         VA configuration ID
//! \param  [in] proc_buf
//!         VA processing rate parameter
//! \param  [out] processing_rate
//!         Processing rate
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_QueryProcessingRate(
    VADriverContextP ctx,
    VAConfigID config_id,
    VAProcessingRateParameter *proc_buf,
    uint32_t *processing_rate /* output parameter */)
{
    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(proc_buf, "nullptr proc_buf", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(processing_rate, "nullptr processing_rate", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
#ifdef CNM_VPUAPI_INTERFACE_CAP
    DDI_CHK_CONDITION((s_sizeOfVpuApiCapMap == 0), "VpuApiCapInit is not allced", VA_STATUS_ERROR_INVALID_CONFIG);

    return VA_STATUS_ERROR_UNIMPLEMENTED;
#else
    DDI_CHK_NULL(mediaCtx->m_caps, "nullptr m_caps", VA_STATUS_ERROR_INVALID_CONTEXT);

    return mediaCtx->m_caps->QueryProcessingRate(config_id,
                                                 proc_buf, processing_rate);
#endif
}

//!
//! \brief  media copy internal
//!
//! \param  [in] mosCtx
//!         Pointer to mos context
//! \param  [in] src
//!         VA copy mos resource src.
//! \param  [in] dst
//!         VA copy mos resrouce dst.
//! \param  [in] option
//!         VA copy option, copy mode.
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_CopyInternal(
    PMOS_CONTEXT mosCtx,
    PMOS_RESOURCE src,
    PMOS_RESOURCE dst,
    uint32_t copy_mode)
{
    VAStatus vaStatus = VA_STATUS_SUCCESS;
    MOS_STATUS mosStatus = MOS_STATUS_UNINITIALIZED;
    DDI_CHK_NULL(mosCtx, "nullptr mosCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(src, "nullptr input osResource", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(dst, "nullptr output osResource", VA_STATUS_ERROR_INVALID_SURFACE);

    MediaCopyBaseState *mediaCopyState = static_cast<MediaCopyBaseState *>(*mosCtx->ppMediaCopyState);

    if (!mediaCopyState)
    {
        mediaCopyState = static_cast<MediaCopyBaseState *>(McpyDevice::CreateFactory(mosCtx));
        *mosCtx->ppMediaCopyState = mediaCopyState;
    }

    DDI_CHK_NULL(mediaCopyState, "Invalid mediaCopy State", VA_STATUS_ERROR_INVALID_PARAMETER);

    mosStatus = mediaCopyState->SurfaceCopy(src, dst, (MCPY_METHOD)copy_mode);
    if (mosStatus != MOS_STATUS_SUCCESS)
    {
        vaStatus = VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    return vaStatus;
}

#if VA_CHECK_VERSION(1, 10, 0)
//!
//! \brief  media copy
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] dst_obj
//!         VA copy object dst.
//! \param  [in] src_obj
//!         VA copy object src.
//! \param  [in] option
//!         VA copy option, copy mode.
//! \param  [in] sync_handle
//!         VA copy sync handle
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_Copy(
    VADriverContextP ctx,
    VACopyObject *dst_obj,
    VACopyObject *src_obj,
    VACopyOption option)
{
    VAStatus vaStatus = VA_STATUS_SUCCESS;
    MOS_CONTEXT mosCtx;
    MOS_RESOURCE src, dst;
    DdiCpInterface *pCpDdiInterface = nullptr;
    PDDI_MEDIA_SURFACE src_surface = nullptr;
    PDDI_MEDIA_SURFACE dst_surface = nullptr;
    PDDI_MEDIA_BUFFER src_buffer = nullptr;
    PDDI_MEDIA_BUFFER dst_buffer = nullptr;

    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(dst_obj, "nullptr copy dst", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(src_obj, "nullptr copy src", VA_STATUS_ERROR_INVALID_SURFACE);

    if (dst_obj->obj_type == VACopyObjectSurface)
    {
        DDI_CHK_LESS((uint32_t)dst_obj->object.surface_id, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "copy_dst", VA_STATUS_ERROR_INVALID_SURFACE);
        dst_surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, dst_obj->object.surface_id);
        DDI_CHK_NULL(dst_surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);
        DDI_CHK_NULL(dst_surface->pGmmResourceInfo, "nullptr dst_surface->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_PARAMETER);

        MOS_ZeroMemory(&dst, sizeof(dst));
        DdiMedia_MediaSurfaceToMosResource(dst_surface, &dst);
    }
    else if (dst_obj->obj_type == VACopyObjectBuffer)
    {
        DDI_CHK_LESS((uint32_t)dst_obj->object.buffer_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid copy dst buf_id", VA_STATUS_ERROR_INVALID_BUFFER);
        dst_buffer = DdiMedia_GetBufferFromVABufferID(mediaCtx, dst_obj->object.buffer_id);
        DDI_CHK_NULL(dst_buffer, "nullptr buffer", VA_STATUS_ERROR_INVALID_BUFFER);
        DDI_CHK_NULL(dst_buffer->pGmmResourceInfo, "nullptr dst_buffer->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_PARAMETER);

        MOS_ZeroMemory(&dst, sizeof(dst));
        DdiMedia_MediaBufferToMosResource(dst_buffer, &dst);
    }
    else
    {
        DDI_ASSERTMESSAGE("DDI: unsupported src copy object in DdiMedia_copy.");
    }

    if (src_obj->obj_type == VACopyObjectSurface)
    {
        DDI_CHK_LESS((uint32_t)src_obj->object.surface_id, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "copy_src", VA_STATUS_ERROR_INVALID_SURFACE);
        src_surface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, src_obj->object.surface_id);
        DDI_CHK_NULL(src_surface, "nullptr surface", VA_STATUS_ERROR_INVALID_SURFACE);
        DDI_CHK_NULL(src_surface->pGmmResourceInfo, "nullptr src_surface->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_PARAMETER);

        MOS_ZeroMemory(&src, sizeof(src));
        DdiMedia_MediaSurfaceToMosResource(src_surface, &src);
    }
    else if (src_obj->obj_type == VACopyObjectBuffer)
    {
        DDI_CHK_LESS((uint32_t)src_obj->object.buffer_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid copy dst buf_id", VA_STATUS_ERROR_INVALID_BUFFER);
        src_buffer = DdiMedia_GetBufferFromVABufferID(mediaCtx, src_obj->object.buffer_id);
        DDI_CHK_NULL(src_buffer, "nullptr buffer", VA_STATUS_ERROR_INVALID_BUFFER);
        DDI_CHK_NULL(src_buffer->pGmmResourceInfo, "nullptr src_buffer->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_PARAMETER);

        MOS_ZeroMemory(&src, sizeof(src));
        DdiMedia_MediaBufferToMosResource(src_buffer, &src);
    }
    else
    {
        DDI_ASSERTMESSAGE("DDI: unsupported dst copy object in DdiMedia_copy.");
    }

    MOS_ZeroMemory(&mosCtx, sizeof(mosCtx));

    mosCtx.bufmgr = mediaCtx->pDrmBufMgr;
    mosCtx.m_gpuContextMgr = mediaCtx->m_gpuContextMgr;
    mosCtx.m_cmdBufMgr = mediaCtx->m_cmdBufMgr;
    mosCtx.fd = mediaCtx->fd;
    mosCtx.iDeviceId = mediaCtx->iDeviceId;
    mosCtx.SkuTable = mediaCtx->SkuTable;
    mosCtx.WaTable = mediaCtx->WaTable;
    mosCtx.gtSystemInfo = *mediaCtx->pGtSystemInfo;
    mosCtx.platform = mediaCtx->platform;

    mosCtx.ppMediaCopyState = &mediaCtx->pMediaCopyState;
    mosCtx.gtSystemInfo = *mediaCtx->pGtSystemInfo;
    mosCtx.m_auxTableMgr = mediaCtx->m_auxTableMgr;
    mosCtx.pGmmClientContext = mediaCtx->pGmmClientContext;

    mosCtx.m_osDeviceContext = mediaCtx->m_osDeviceContext;
    mosCtx.m_apoMosEnabled = mediaCtx->m_apoMosEnabled;

    pCpDdiInterface = Create_DdiCpInterface(mosCtx);

    if (nullptr == pCpDdiInterface)
    {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    vaStatus = DdiMedia_CopyInternal(&mosCtx, &src, &dst, option.bits.va_copy_mode);

    if ((option.bits.va_copy_sync == VA_EXEC_SYNC) && dst_surface)
    {
        uint32_t timeout_NS = 100000000;
        while (0 != mos_gem_bo_wait(dst_surface->bo, timeout_NS))
        {
            // Just loop while gem_bo_wait times-out.
        }
    }

    if (pCpDdiInterface)
    {
        Delete_DdiCpInterface(pCpDdiInterface);
        pCpDdiInterface = NULL;
    }

    return vaStatus;
}
#endif

//!
//! \brief  Check for buffer info
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] buf_id
//!         VA buffer ID
//! \param  [out] type
//!         VA buffer type
//! \param  [out] size
//!         Size
//! \param  [out] num_elements
//!         Number of elements
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_BufferInfo(
    VADriverContextP ctx,
    VABufferID buf_id,
    VABufferType *type,
    uint32_t *size,
    uint32_t *num_elements)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(type, "nullptr type", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(size, "nullptr size", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_elements, "nullptr num_elements", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    if (nullptr == mediaCtx)
        return VA_STATUS_ERROR_INVALID_CONTEXT;

    DDI_CHK_NULL(mediaCtx->pBufferHeap, "nullptr mediaCtx->pBufferHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)buf_id, mediaCtx->pBufferHeap->uiAllocatedHeapElements, "Invalid buf_id", VA_STATUS_ERROR_INVALID_BUFFER);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    if (nullptr == buf)
    {
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }

    *type = (VABufferType)buf->uiType;
    *size = buf->iSize / buf->uiNumElements;
    *num_elements = buf->uiNumElements;

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Lock surface
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] surface
//!         VA surface ID
//! \param  [out] fourcc
//!         FourCC
//! \param  [out] luma_stride
//!         Luma stride
//! \param  [out] chroma_u_stride
//!         Chroma U stride
//! \param  [out] chroma_v_stride
//!         Chroma V stride
//! \param  [out] luma_offset
//!         Luma offset
//! \param  [out] chroma_u_offset
//!         Chroma U offset
//! \param  [out] chroma_v_offset
//!         Chroma V offset
//! \param  [out] buffer_name
//!         Buffer name
//! \param  [out] buffer
//!         Buffer
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_LockSurface(
    VADriverContextP ctx,
    VASurfaceID surface,
    uint32_t *fourcc,
    uint32_t *luma_stride,
    uint32_t *chroma_u_stride,
    uint32_t *chroma_v_stride,
    uint32_t *luma_offset,
    uint32_t *chroma_u_offset,
    uint32_t *chroma_v_offset,
    uint32_t *buffer_name,
    void **buffer)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_LOCK, EVENT_TYPE_START, &surface, sizeof(surface), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr context", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(fourcc, "nullptr fourcc", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(luma_stride, "nullptr luma_stride", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(chroma_u_stride, "nullptr chroma_u_stride", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(chroma_v_stride, "nullptr chroma_v_stride", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(luma_offset, "nullptr luma_offset", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(chroma_u_offset, "nullptr chroma_u_offset", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(chroma_v_offset, "nullptr chroma_v_offset", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(buffer_name, "nullptr buffer_name", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(buffer, "nullptr buffer", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr Media", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)surface, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);

#ifdef _MMC_SUPPORTED
    // Decompress surface is needed
    DdiMedia_MediaMemoryDecompress(mediaCtx, mediaSurface);
#endif

    if (nullptr == mediaSurface)
    {
        // Surface is absent.
        buffer = nullptr;
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }

    if (mediaSurface->uiLockedImageID != VA_INVALID_ID)
    {
        // Surface is locked already.
        buffer = nullptr;
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    VAImage tmpImage;
    tmpImage.image_id = VA_INVALID_ID;
    VAStatus vaStatus = DdiMedia_DeriveImage(ctx, surface, &tmpImage);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        buffer = nullptr;
        return vaStatus;
    }

    mediaSurface->uiLockedImageID = tmpImage.image_id;

    vaStatus = DdiMedia_MapBuffer(ctx, tmpImage.buf, buffer);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        buffer = nullptr;
        return vaStatus;
    }

    mediaSurface->uiLockedBufID = tmpImage.buf;

    *fourcc = tmpImage.format.fourcc;
    *luma_offset = tmpImage.offsets[0];
    *luma_stride = tmpImage.pitches[0];
    *chroma_u_offset = tmpImage.offsets[1];
    *chroma_u_stride = tmpImage.pitches[1];
    *chroma_v_offset = tmpImage.offsets[2];
    *chroma_v_stride = tmpImage.pitches[2];
    *buffer_name = tmpImage.buf;

    MOS_TraceEventExt(EVENT_VA_LOCK, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Unlock surface
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] surface
//!         VA surface ID
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_UnlockSurface(
    VADriverContextP ctx,
    VASurfaceID surface)
{
    DDI_FUNCTION_ENTER();
    MOS_TraceEventExt(EVENT_VA_UNLOCK, EVENT_TYPE_START, &surface, sizeof(surface), nullptr, 0);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)surface, mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surface", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface", VA_STATUS_ERROR_INVALID_SURFACE);

    if (mediaSurface->uiLockedImageID == VA_INVALID_ID)
    {
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    VABufferID bufID = (VABufferID)(mediaSurface->uiLockedBufID);
    VAStatus vaStatus = DdiMedia_UnmapBuffer(ctx, bufID);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        return vaStatus;
    }
    mediaSurface->uiLockedBufID = VA_INVALID_ID;

    VAImageID imageID = (VAImageID)(mediaSurface->uiLockedImageID);
    vaStatus = DdiMedia_DestroyImage(ctx, imageID);
    if (vaStatus != VA_STATUS_SUCCESS)
    {
        return vaStatus;
    }
    mediaSurface->uiLockedImageID = VA_INVALID_ID;

    MOS_TraceEventExt(EVENT_VA_UNLOCK, EVENT_TYPE_END, nullptr, 0, nullptr, 0);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Query video proc filters
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] context
//!         VA context ID
//! \param  [in] filters
//!         VA proc filter type
//! \param  [in] num_filters
//!         Number of filters
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_QueryVideoProcFilters(
    VADriverContextP ctx,
    VAContextID context,
    VAProcFilterType *filters,
    uint32_t *num_filters)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(context);

    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(filters, "nullptr filters", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(num_filters, "nullptr num_filters", VA_STATUS_ERROR_INVALID_PARAMETER);

    uint32_t max_num_filters = DDI_VP_MAX_NUM_FILTERS;
    // check if array size is less than VP_MAX_NUM_FILTERS
    if (*num_filters < max_num_filters)
    {
        DDI_NORMALMESSAGE("num_filters %d < max_num_filters %d. Probably caused by Libva version upgrade!", *num_filters, max_num_filters);
    }

    // Set the filters
    uint32_t i = 0;
    while (i < *num_filters && i < DDI_VP_MAX_NUM_FILTERS)
    {
        filters[i] = vp_supported_filters[i];
        i++;
    }

    // Tell the app how many valid filters are filled in the array
    *num_filters = DDI_VP_MAX_NUM_FILTERS;

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Query video processing filter capabilities.
//!         The real implementation is in media_libva_vp.c, since it needs to use some definitions in vphal.h.
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] context
//!         VA context ID
//! \param  [in] type
//!         VA proc filter type
//! \param  [inout] filter_caps
//!         FIlter caps
//! \param  [inout] num_filter_caps
//!         Number of filter caps
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_QueryVideoProcFilterCaps(
    VADriverContextP ctx,
    VAContextID context,
    VAProcFilterType type,
    void *filter_caps,
    uint32_t *num_filter_caps)
{
    DDI_FUNCTION_ENTER();

    return DdiVp_QueryVideoProcFilterCaps(ctx, context, type, filter_caps, num_filter_caps);
}

//!
//! \brief  Query video proc pipeline caps
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] context
//!         VA context ID
//! \param  [in] filters
//!         VA buffer ID
//! \param  [in] num_filters
//!         Number of filters
//! \param  [in] pipeline_caps
//!         VA proc pipeline caps
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus
DdiMedia_QueryVideoProcPipelineCaps(
    VADriverContextP ctx,
    VAContextID context,
    VABufferID *filters,
    uint32_t num_filters,
    VAProcPipelineCaps *pipeline_caps)
{
    DDI_FUNCTION_ENTER();

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(pipeline_caps, "nullptr pipeline_caps", VA_STATUS_ERROR_INVALID_PARAMETER);
    if (num_filters > 0)
        DDI_CHK_NULL(filters, "nullptr filters", VA_STATUS_ERROR_INVALID_PARAMETER);

    pipeline_caps->pipeline_flags = VA_PROC_PIPELINE_FAST;
    pipeline_caps->filter_flags = 0;
    pipeline_caps->rotation_flags = (1 << VA_ROTATION_NONE) | (1 << VA_ROTATION_90) | (1 << VA_ROTATION_180) | (1 << VA_ROTATION_270);
    pipeline_caps->mirror_flags = VA_MIRROR_HORIZONTAL | VA_MIRROR_VERTICAL;
    pipeline_caps->blend_flags = VA_BLEND_GLOBAL_ALPHA | VA_BLEND_PREMULTIPLIED_ALPHA | VA_BLEND_LUMA_KEY;
    pipeline_caps->num_forward_references = DDI_CODEC_NUM_FWD_REF;
    pipeline_caps->num_backward_references = DDI_CODEC_NUM_BK_REF;
    pipeline_caps->input_color_standards = vp_input_color_std;
    pipeline_caps->num_input_color_standards = DDI_VP_NUM_INPUT_COLOR_STD;
    pipeline_caps->output_color_standards = vp_output_color_std;
    pipeline_caps->num_output_color_standards = DDI_VP_NUM_OUT_COLOR_STD;

    if ((context & DDI_MEDIA_MASK_VACONTEXT_TYPE) == DDI_MEDIA_VACONTEXTID_OFFSET_DECODER)
    {
        // Decode+SFC, go SFC path, the restriction here is the capability of SFC
        pipeline_caps->num_input_pixel_formats = 1;
        pipeline_caps->input_pixel_format[0] = VA_FOURCC_NV12;
        pipeline_caps->num_output_pixel_formats = 1;
        pipeline_caps->output_pixel_format[0] = VA_FOURCC_NV12;
        if ((MEDIA_IS_SKU(&(mediaCtx->SkuTable), FtrHCP2SFCPipe)))
        {
            pipeline_caps->max_input_width = DDI_DECODE_HCP_SFC_MAX_WIDTH;
            pipeline_caps->max_input_height = DDI_DECODE_HCP_SFC_MAX_HEIGHT;
        }
        else
        {
            pipeline_caps->max_input_width = DDI_DECODE_SFC_MAX_WIDTH;
            pipeline_caps->max_input_height = DDI_DECODE_SFC_MAX_HEIGHT;
        }
        pipeline_caps->min_input_width = DDI_DECODE_SFC_MIN_WIDTH;
        pipeline_caps->min_input_height = DDI_DECODE_SFC_MIN_HEIGHT;
        pipeline_caps->max_output_width = DDI_DECODE_SFC_MAX_WIDTH;
        pipeline_caps->max_output_height = DDI_DECODE_SFC_MAX_HEIGHT;
        pipeline_caps->min_output_width = DDI_DECODE_SFC_MIN_WIDTH;
        pipeline_caps->min_output_height = DDI_DECODE_SFC_MIN_HEIGHT;
    }
    else if ((context & DDI_MEDIA_MASK_VACONTEXT_TYPE) == DDI_MEDIA_VACONTEXTID_OFFSET_VP)
    {
        if (mediaCtx->platform.eRenderCoreFamily <= IGFX_GEN8_CORE)
        {
            // Capability of Gen8- platform
            pipeline_caps->max_input_width = VP_MAX_PIC_WIDTH_Gen8;
            pipeline_caps->max_input_height = VP_MAX_PIC_HEIGHT_Gen8;
            pipeline_caps->max_output_width = VP_MAX_PIC_WIDTH_Gen8;
            pipeline_caps->max_output_height = VP_MAX_PIC_HEIGHT_Gen8;
        }
        else
        {
            // Capability of Gen9+ platform
            pipeline_caps->max_input_width = VP_MAX_PIC_WIDTH;
            pipeline_caps->max_input_height = VP_MAX_PIC_HEIGHT;
            pipeline_caps->max_output_width = VP_MAX_PIC_WIDTH;
            pipeline_caps->max_output_height = VP_MAX_PIC_HEIGHT;
        }
        pipeline_caps->min_input_width = VP_MIN_PIC_WIDTH;
        pipeline_caps->min_input_height = VP_MIN_PIC_HEIGHT;
        pipeline_caps->min_output_width = VP_MIN_PIC_WIDTH;
        pipeline_caps->min_output_height = VP_MIN_PIC_WIDTH;
    }
    return VA_STATUS_SUCCESS;
}

/**
 * \brief Get surface attributes for the supplied config.
 *
 * This function retrieves the surface attributes matching the supplied
 * config. The caller shall provide an \c attrib_list with all attributes
 * to be retrieved. Upon successful return, the attributes in \c attrib_list
 * are updated with the requested value. Unknown attributes or attributes
 * that are not supported for the given config will have their \c flags
 * field set to \c VA_SURFACE_ATTRIB_NOT_SUPPORTED.
 *
 * param[in] ctx               the VA display
 * param[in] config            the config identifying a codec or a video
 *     processing pipeline
 * param[out] attrib_list        the list of attributes on output, with at
 *     least \c type fields filled in, and possibly \c value fields whenever
 *     necessary.The updated list of attributes and flags on output
 * param[in] num_attribs       the number of attributes supplied in the
 *     \c attrib_list array
 */
VAStatus DdiMedia_GetSurfaceAttributes(
    VADriverContextP ctx,
    VAConfigID config,
    VASurfaceAttrib *attrib_list,
    uint32_t num_attribs)
{
    DDI_UNUSED(ctx);
    DDI_UNUSED(config);
    DDI_UNUSED(attrib_list);
    DDI_UNUSED(num_attribs);

    DDI_FUNCTION_ENTER();

    VAStatus vaStatus = VA_STATUS_ERROR_UNIMPLEMENTED;

    return vaStatus;
}

//!
//! \brief  Aquire buffer handle
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] buf_id
//!         VA buffer ID
//! \param  [in] buf_info
//!         VA buffer Info
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_AcquireBufferHandle(
    VADriverContextP ctx,
    VABufferID buf_id,
    VABufferInfo *buf_info)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(buf_info, "nullptr buf_info", VA_STATUS_ERROR_INVALID_PARAMETER);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "Invalid Media ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buf, "Invalid Media Buffer", VA_STATUS_ERROR_INVALID_BUFFER);
    DDI_CHK_NULL(buf->bo, "Invalid Media Buffer", VA_STATUS_ERROR_INVALID_BUFFER);

    // If user did not specify memtype he want's we use something we prefer, we prefer PRIME
    if (!buf_info->mem_type)
    {
        buf_info->mem_type = VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME;
    }
    // now chekcing memtype whether we support it
    if ((buf_info->mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME) &&
        (buf_info->mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM))
    {
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }

    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    // already acquired?
    if (buf->uiExportcount)
    { // yes, already acquired
        // can't provide access thru another memtype
        if (buf->uiMemtype != buf_info->mem_type)
        {
            DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
    }
    else
    { // no, not acquired - doing this now
        switch (buf_info->mem_type)
        {
        case VA_SURFACE_ATTRIB_MEM_TYPE_KERNEL_DRM:
        {
            uint32_t flink = 0;
            if (mos_bo_flink(buf->bo, &flink) != 0)
            {
                DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
                return VA_STATUS_ERROR_INVALID_BUFFER;
            }
            buf->handle = (intptr_t)flink;
            break;
        }
        case VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME:
        {
            int32_t prime_fd = 0;
            if (mos_bo_gem_export_to_prime(buf->bo, &prime_fd) != 0)
            {
                DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
                return VA_STATUS_ERROR_INVALID_BUFFER;
            }

            buf->handle = (intptr_t)prime_fd;
            break;
        }
        }
        // saving memtepy which was provided to the user
        buf->uiMemtype = buf_info->mem_type;
    }

    ++buf->uiExportcount;
    mos_bo_reference(buf->bo);

    buf_info->type = buf->uiType;
    buf_info->handle = buf->handle;
    buf_info->mem_size = buf->uiNumElements * buf->iSize;

    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Release buffer handle
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//! \param  [in] buf_id
//!         VA bufferID
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_ReleaseBufferHandle(
    VADriverContextP ctx,
    VABufferID buf_id)
{
    DDI_FUNCTION_ENTER();

    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "Invalid Media ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    DDI_MEDIA_BUFFER *buf = DdiMedia_GetBufferFromVABufferID(mediaCtx, buf_id);
    DDI_CHK_NULL(buf, "Invalid Media Buffer", VA_STATUS_ERROR_INVALID_BUFFER);
    DDI_CHK_NULL(buf->bo, "Invalid Media Buffer", VA_STATUS_ERROR_INVALID_BUFFER);

    DdiMediaUtil_LockMutex(&mediaCtx->BufferMutex);
    if (!buf->uiMemtype || !buf->uiExportcount)
    {
        DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);
        return VA_STATUS_SUCCESS;
    }
    mos_bo_unreference(buf->bo);
    --buf->uiExportcount;

    if (!buf->uiExportcount)
    {
        switch (buf->uiMemtype)
        {
        case VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME:
        {
            close((intptr_t)buf->handle);
            break;
        }
        }
        buf->uiMemtype = 0;
    }
    DdiMediaUtil_UnLockMutex(&mediaCtx->BufferMutex);

    if (!buf->uiExportcount && buf->bPostponedBufFree)
    {
        MOS_FreeMemory(buf);
        DdiMedia_DestroyBufFromVABufferID(mediaCtx, buf_id);
    }

    return VA_STATUS_SUCCESS;
}

static uint32_t DdiMedia_GetPlaneNum(PDDI_MEDIA_SURFACE mediaSurface, bool hasAuxPlane)
{
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface", VA_STATUS_ERROR_INVALID_PARAMETER);

    uint32_t fourcc = DdiMedia_MediaFormatToOsFormat(mediaSurface->format);
    uint32_t plane_num = 0;
    switch (fourcc)
    {
    case VA_FOURCC_NV12:
    case VA_FOURCC_NV21:
    case VA_FOURCC_P010:
    case VA_FOURCC_P012:
    case VA_FOURCC_P016:
        plane_num = hasAuxPlane ? 4 : 2;
        break;
        plane_num = hasAuxPlane ? 4 : 2;
        break;
    case VA_FOURCC_I420:
    case VA_FOURCC_YV12:
    case VA_FOURCC_411P:
    case VA_FOURCC_422H:
    case VA_FOURCC_422V:
    case VA_FOURCC_444P:
    case VA_FOURCC_IMC3:
    case VA_FOURCC_RGBP:
    case VA_FOURCC_BGRP:
        plane_num = 3;
        break;
    case VA_FOURCC_YUY2:
    case VA_FOURCC_UYVY:
    case VA_FOURCC_YVYU:
    case VA_FOURCC_VYUY:
    case VA_FOURCC_Y800:
    case VA_FOURCC_Y210:
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y212:
#endif
    case VA_FOURCC_Y216:
    case VA_FOURCC_Y410:
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y412:
#endif
    case VA_FOURCC_Y416:
    case VA_FOURCC_AYUV:
    case VA_FOURCC_RGBA:
    case VA_FOURCC_RGBX:
    case VA_FOURCC_BGRA:
    case VA_FOURCC_BGRX:
    case VA_FOURCC_ARGB:
    case VA_FOURCC_ABGR:
    case VA_FOURCC_XRGB:
    case VA_FOURCC_XBGR:
    case VA_FOURCC_RGB565:
    case VA_FOURCC_R8G8B8:
    case VA_FOURCC_A2R10G10B10:
    case VA_FOURCC_A2B10G10R10:
    case VA_FOURCC_X2R10G10B10:
    case VA_FOURCC_X2B10G10R10:
        plane_num = hasAuxPlane ? 2 : 1;
        break;
    default:
        DDI_ASSERTMESSAGE("Unsupported format.\n");
    }
    return plane_num;
}

static uint32_t DdiMedia_GetDrmFormatOfSeparatePlane(uint32_t fourcc, int plane)
{
    if (plane == 0)
    {
        switch (fourcc)
        {
        case VA_FOURCC_NV12:
        case VA_FOURCC_I420:
        case VA_FOURCC_YV12:
        case VA_FOURCC_YV16:
        case VA_FOURCC_Y800:
        case VA_FOURCC_RGBP:
        case VA_FOURCC_BGRP:
            return DRM_FORMAT_R8;
        case VA_FOURCC_P010:
        case VA_FOURCC_P012:
        case VA_FOURCC_P016:
        case VA_FOURCC_I010:
            return DRM_FORMAT_R16;

        case VA_FOURCC_YUY2:
            return DRM_FORMAT_YUYV;
        case VA_FOURCC_YVYU:
            return DRM_FORMAT_YVYU;
        case VA_FOURCC_VYUY:
            return DRM_FORMAT_VYUY;
        case VA_FOURCC_UYVY:
            return DRM_FORMAT_UYVY;
        case VA_FOURCC_AYUV:
            return DRM_FORMAT_AYUV;
        case VA_FOURCC_Y210:
            return DRM_FORMAT_Y210;
        case VA_FOURCC_Y216:
            return DRM_FORMAT_Y216;
        case VA_FOURCC_Y410:
            return DRM_FORMAT_Y410;
        case VA_FOURCC_Y416:
            return DRM_FORMAT_Y416;
#if VA_CHECK_VERSION(1, 9, 0)
        case VA_FOURCC_Y212:
            return DRM_FORMAT_Y216;
        case VA_FOURCC_Y412:
            return DRM_FORMAT_Y416;
#endif

        case VA_FOURCC_ARGB:
            return DRM_FORMAT_ARGB8888;
        case VA_FOURCC_ABGR:
            return DRM_FORMAT_ABGR8888;
        case VA_FOURCC_RGBA:
            return DRM_FORMAT_RGBA8888;
        case VA_FOURCC_BGRA:
            return DRM_FORMAT_BGRA8888;
        case VA_FOURCC_XRGB:
            return DRM_FORMAT_XRGB8888;
        case VA_FOURCC_XBGR:
            return DRM_FORMAT_XBGR8888;
        case VA_FOURCC_RGBX:
            return DRM_FORMAT_RGBX8888;
        case VA_FOURCC_BGRX:
            return DRM_FORMAT_BGRX8888;
        case VA_FOURCC_A2R10G10B10:
            return DRM_FORMAT_ARGB2101010;
        case VA_FOURCC_A2B10G10R10:
            return DRM_FORMAT_ABGR2101010;
        case VA_FOURCC_X2R10G10B10:
            return DRM_FORMAT_XRGB2101010;
        case VA_FOURCC_X2B10G10R10:
            return DRM_FORMAT_XBGR2101010;
        }
    }
    else
    {
        switch (fourcc)
        {
        case VA_FOURCC_NV12:
            return DRM_FORMAT_GR88;
        case VA_FOURCC_I420:
        case VA_FOURCC_YV12:
        case VA_FOURCC_YV16:
        case VA_FOURCC_RGBP:
        case VA_FOURCC_BGRP:
            return DRM_FORMAT_R8;
        case VA_FOURCC_P010:
        case VA_FOURCC_P012:
        case VA_FOURCC_P016:
            return DRM_FORMAT_GR1616;
        case VA_FOURCC_I010:
            return DRM_FORMAT_R16;
        }
    }
    return 0;
}

static uint32_t DdiMedia_GetDrmFormatOfCompositeObject(uint32_t fourcc)
{
    switch (fourcc)
    {
    case VA_FOURCC_NV12:
        return DRM_FORMAT_NV12;
    case VA_FOURCC_I420:
        return DRM_FORMAT_YUV420;
    case VA_FOURCC_YV12:
        return DRM_FORMAT_YVU420;
    case VA_FOURCC_YV16:
        return DRM_FORMAT_YVU422;
    case VA_FOURCC_YUY2:
        return DRM_FORMAT_YUYV;
    case VA_FOURCC_YVYU:
        return DRM_FORMAT_YVYU;
    case VA_FOURCC_VYUY:
        return DRM_FORMAT_VYUY;
    case VA_FOURCC_UYVY:
        return DRM_FORMAT_UYVY;
    case VA_FOURCC_Y210:
        return DRM_FORMAT_Y210;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y212:
        return DRM_FORMAT_Y216;
#endif
    case VA_FOURCC_Y216:
        return DRM_FORMAT_Y216;
    case VA_FOURCC_Y410:
        return DRM_FORMAT_Y410;
#if VA_CHECK_VERSION(1, 9, 0)
    case VA_FOURCC_Y412:
        return DRM_FORMAT_Y416;
#endif
    case VA_FOURCC_Y416:
        return DRM_FORMAT_Y416;
    case VA_FOURCC_Y800:
        return DRM_FORMAT_R8;
    case VA_FOURCC_P010:
        return DRM_FORMAT_P010;
    case VA_FOURCC_P012:
        return DRM_FORMAT_P016;
    case VA_FOURCC_P016:
        return DRM_FORMAT_P016;
    case VA_FOURCC_ARGB:
        return DRM_FORMAT_ARGB8888;
    case VA_FOURCC_ABGR:
        return DRM_FORMAT_ABGR8888;
    case VA_FOURCC_RGBA:
        return DRM_FORMAT_RGBA8888;
    case VA_FOURCC_BGRA:
        return DRM_FORMAT_BGRA8888;
    case VA_FOURCC_XRGB:
        return DRM_FORMAT_XRGB8888;
    case VA_FOURCC_XBGR:
        return DRM_FORMAT_XBGR8888;
    case VA_FOURCC_RGBX:
        return DRM_FORMAT_RGBX8888;
    case VA_FOURCC_BGRX:
        return DRM_FORMAT_BGRX8888;
    case VA_FOURCC_A2R10G10B10:
        return DRM_FORMAT_ARGB2101010;
    case VA_FOURCC_A2B10G10R10:
        return DRM_FORMAT_ABGR2101010;
    case VA_FOURCC_X2R10G10B10:
        return DRM_FORMAT_XRGB2101010;
    case VA_FOURCC_X2B10G10R10:
        return DRM_FORMAT_XBGR2101010;
    }
    return 0;
}

//!
//! \brief   API for export surface handle to other component
//!
//! \param [in] dpy
//!          VA display.
//! \param [in] surface_id
//!          Surface to export.
//! \param [in] mem_type
//!          Memory type to export to.
//! \param [in] flags
//!          Combination of flags to apply
//!\param [out] descriptor
//! Pointer to the descriptor structure to fill
//! with the handle details.  The type of this structure depends on
//! the value of mem_type.
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus DdiMedia_ExportSurfaceHandle(
    VADriverContextP ctx,
    VASurfaceID surface_id,
    uint32_t mem_type,
    uint32_t flags,
    void *descriptor)
{
    DDI_CHK_NULL(descriptor, "nullptr descriptor", VA_STATUS_ERROR_INVALID_PARAMETER);
    DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

    PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
    DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
    DDI_CHK_LESS((uint32_t)(surface_id), mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surfaces", VA_STATUS_ERROR_INVALID_SURFACE);

    DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, surface_id);
    DDI_CHK_NULL(mediaSurface, "nullptr mediaSurface", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(mediaSurface->bo, "nullptr mediaSurface->bo", VA_STATUS_ERROR_INVALID_SURFACE);
    DDI_CHK_NULL(mediaSurface->pGmmResourceInfo, "nullptr mediaSurface->pGmmResourceInfo", VA_STATUS_ERROR_INVALID_SURFACE);

    if (mem_type != VA_SURFACE_ATTRIB_MEM_TYPE_DRM_PRIME_2)
    {
        DDI_ASSERTMESSAGE("vaExportSurfaceHandle: memory type %08x is not supported.\n", mem_type);
        return VA_STATUS_ERROR_UNSUPPORTED_MEMORY_TYPE;
    }

    if (mos_bo_gem_export_to_prime(mediaSurface->bo, (int32_t *)&mediaSurface->name))
    {
        DDI_ASSERTMESSAGE("Failed drm_intel_gem_export_to_prime operation!!!\n");
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    VADRMPRIMESurfaceDescriptor *desc = (VADRMPRIMESurfaceDescriptor *)descriptor;
    desc->fourcc = DdiMedia_MediaFormatToOsFormat(mediaSurface->format);
    if (desc->fourcc == VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT)
    {
        return VA_STATUS_ERROR_UNSUPPORTED_RT_FORMAT;
    }
    desc->width = mediaSurface->iWidth;
    desc->height = mediaSurface->iRealHeight;
    desc->num_objects = 1;
    desc->objects[0].fd = mediaSurface->name;
    desc->objects[0].size = mediaSurface->pGmmResourceInfo->GetSizeSurface();

    // Prepare Compression info for export surface handle
    GMM_RESOURCE_FLAG GmmFlags = {0};
    bool bMmcEnabled = false;
    GmmFlags = mediaSurface->pGmmResourceInfo->GetResFlags();

    if ((GmmFlags.Gpu.MMC ||
         GmmFlags.Gpu.CCS) &&
        (GmmFlags.Info.MediaCompressed ||
         GmmFlags.Info.RenderCompressed))
    {
        bMmcEnabled = true;
    }
    else
    {
        bMmcEnabled = false;
    }

    switch (mediaSurface->TileType)
    {
    case I915_TILING_X:
        desc->objects[0].drm_format_modifier = I915_FORMAT_MOD_X_TILED;
        break;
    case I915_TILING_Y:
        if (mediaCtx->m_auxTableMgr && bMmcEnabled)
        {
            desc->objects[0].drm_format_modifier = GmmFlags.Info.MediaCompressed ? I915_FORMAT_MOD_Y_TILED_GEN12_MC_CCS : (GmmFlags.Info.RenderCompressed ? I915_FORMAT_MOD_Y_TILED_GEN12_RC_CCS : I915_FORMAT_MOD_Y_TILED);
        }
        else
        {
            desc->objects[0].drm_format_modifier = I915_FORMAT_MOD_Y_TILED;
        }
        break;
    case I915_TILING_NONE:
    default:
        desc->objects[0].drm_format_modifier = DRM_FORMAT_MOD_NONE;
    }

    int composite_object = flags & VA_EXPORT_SURFACE_COMPOSED_LAYERS;

    uint32_t formats[4];
    bool hasAuxPlane = (mediaCtx->m_auxTableMgr) ? true : false;
    uint32_t num_planes = DdiMedia_GetPlaneNum(mediaSurface, hasAuxPlane);
    if (composite_object)
    {
        formats[0] = DdiMedia_GetDrmFormatOfCompositeObject(desc->fourcc);
        if (!formats[0])
        {
            DDI_ASSERTMESSAGE("vaExportSurfaceHandle: fourcc %08x is not supported for export as a composite object.\n", desc->fourcc);
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
    }
    else
    {
        for (int i = 0; i < num_planes; i++)
        {
            formats[i] = DdiMedia_GetDrmFormatOfSeparatePlane(desc->fourcc, i);
            if (!formats[i])
            {
                DDI_ASSERTMESSAGE("vaExportSurfaceHandle: fourcc %08x is not supported for export as separate planes.\n", desc->fourcc);
                return VA_STATUS_ERROR_INVALID_SURFACE;
            }
        }
    }

    uint32_t pitch, height, chromaPitch, chromaHeight = 0;
    pitch = mediaSurface->iPitch;
    height = mediaSurface->iRealHeight;
    DdiMedia_GetChromaPitchHeight(desc->fourcc, pitch, height, &chromaPitch, &chromaHeight);

    // Get offset from GMM
    GMM_REQ_OFFSET_INFO reqInfo = {0};
    reqInfo.Plane = GMM_PLANE_Y;
    reqInfo.ReqRender = 1;
    mediaSurface->pGmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetY = reqInfo.Render.Offset;
    MOS_ZeroMemory(&reqInfo, sizeof(GMM_REQ_OFFSET_INFO));
    reqInfo.Plane = GMM_PLANE_U;
    reqInfo.ReqRender = 1;
    mediaSurface->pGmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetU = reqInfo.Render.Offset;
    MOS_ZeroMemory(&reqInfo, sizeof(GMM_REQ_OFFSET_INFO));
    reqInfo.Plane = GMM_PLANE_V;
    reqInfo.ReqRender = 1;
    mediaSurface->pGmmResourceInfo->GetOffset(reqInfo);
    uint32_t offsetV = reqInfo.Render.Offset;
    uint32_t auxOffsetY = (uint32_t)mediaSurface->pGmmResourceInfo->GetPlanarAuxOffset(0, GMM_AUX_Y_CCS);
    uint32_t auxOffsetUV = (uint32_t)mediaSurface->pGmmResourceInfo->GetPlanarAuxOffset(0, GMM_AUX_UV_CCS);

    if (composite_object)
    {
        desc->num_layers = 1;
        desc->layers[0].drm_format = formats[0];
        desc->layers[0].num_planes = num_planes;
        if (mediaCtx->m_auxTableMgr)
        {
            // For semi-planar formats like NV12, CCS planes follow the Y and UV planes,
            // i.e. planes 0 and 1 are used for Y and UV surfaces, planes 2 and 3 for the respective CCS.
            for (int i = 0; i < num_planes / 2; i++)
            {
                desc->layers[0].object_index[2 * i] = 0;
                desc->layers[0].object_index[2 * i + 1] = 0;
                if (i == 0)
                {
                    // Y plane
                    desc->layers[0].offset[i] = offsetY;
                    desc->layers[0].pitch[i] = mediaSurface->iPitch;
                    // Y aux plane
                    desc->layers[0].offset[i + num_planes / 2] = auxOffsetY;
                    desc->layers[0].pitch[i + num_planes / 2] = mediaSurface->iPitch / 8;
                }
                else
                {
                    // UV plane
                    desc->layers[0].offset[i] = offsetU;
                    desc->layers[0].pitch[i] = mediaSurface->iPitch;
                    // UV aux plane
                    desc->layers[0].offset[i + num_planes / 2] = auxOffsetUV;
                    desc->layers[0].pitch[i + num_planes / 2] = mediaSurface->iPitch / 8;
                }
            }
        }
        else
        {
            for (int i = 0; i < num_planes; i++)
            {
                desc->layers[0].object_index[i] = 0;
                switch (i)
                {
                case 0:
                    desc->layers[0].offset[i] = offsetY;
                    desc->layers[0].pitch[i] = pitch;
                    break;
                case 1:
                    if (desc->fourcc == VA_FOURCC_YV12)
                    {
                        desc->layers[0].offset[i] = offsetV;
                    }
                    else
                    {
                        desc->layers[0].offset[i] = offsetU;
                    }
                    desc->layers[0].pitch[i] = chromaPitch;
                    break;
                case 2:
                    if (desc->fourcc == VA_FOURCC_YV12)
                    {
                        desc->layers[0].offset[i] = offsetU;
                    }
                    else
                    {
                        desc->layers[0].offset[i] = offsetV;
                    }
                    desc->layers[0].pitch[i] = chromaPitch;
                    break;
                default:
                    DDI_ASSERTMESSAGE("vaExportSurfaceHandle: invalid plan numbers");
                }
            }
        }
    }
    else
    {
        if (mediaCtx->m_auxTableMgr)
        {
            desc->num_layers = num_planes / 2;

            for (int i = 0; i < desc->num_layers; i++)
            {
                desc->layers[i].drm_format = formats[i];
                desc->layers[i].num_planes = 2;

                desc->layers[i].object_index[0] = 0;

                if (i == 0)
                {
                    desc->layers[i].offset[0] = offsetY;
                    desc->layers[i].offset[1] = auxOffsetY;
                    desc->layers[i].pitch[0] = mediaSurface->iPitch;
                    desc->layers[i].pitch[1] = mediaSurface->iPitch / 8;
                }
                else
                {
                    desc->layers[i].offset[0] = offsetU;
                    desc->layers[i].offset[1] = auxOffsetUV;
                    desc->layers[i].pitch[0] = mediaSurface->iPitch;
                    desc->layers[i].pitch[1] = mediaSurface->iPitch / 8;
                }
            }
        }
        else
        {
            desc->num_layers = num_planes;

            for (int i = 0; i < num_planes; i++)
            {
                desc->layers[i].drm_format = formats[i];
                desc->layers[i].num_planes = 1;

                desc->layers[i].object_index[0] = 0;

                switch (i)
                {
                case 0:
                    desc->layers[i].offset[0] = offsetY;
                    desc->layers[i].pitch[0] = pitch;
                    break;
                case 1:
                    if (desc->fourcc == VA_FOURCC_YV12)
                    {
                        desc->layers[i].offset[0] = offsetV;
                    }
                    else
                    {
                        desc->layers[i].offset[0] = offsetU;
                    }
                    desc->layers[i].pitch[0] = chromaPitch;
                    break;
                case 2:
                    if (desc->fourcc == VA_FOURCC_YV12)
                    {
                        desc->layers[i].offset[0] = offsetU;
                    }
                    else
                    {
                        desc->layers[i].offset[0] = offsetV;
                    }
                    desc->layers[i].pitch[0] = chromaPitch;
                    break;
                default:
                    DDI_ASSERTMESSAGE("vaExportSurfaceHandle: invalid plan numbers");
                }
            }
        }
    }

    return VA_STATUS_SUCCESS;
}

//!
//! \brief  Init VA driver 0.31
//!
//! \param  [in] ctx
//!         Pointer to VA driver context
//!
//! \return VAStatus
//!     VA_STATUS_SUCCESS if success, else fail reason
//!
VAStatus __vaDriverInit(VADriverContextP ctx)
{
    return DdiMedia__Initialize(ctx, nullptr, nullptr);
}

#ifdef __cplusplus
extern "C"
{
#endif

//!
//! \brief Get VA_MAJOR_VERSION and VA_MINOR_VERSION from libva
//!         To form the function name in the format of _vaDriverInit_[VA_MAJOR_VERSION]_[VA_MINOR_VERSION]
//!
#define VA_DRV_INIT_DEF(_major, _minor) __vaDriverInit_##_major##_##_minor
#define VA_DRV_INIT_FUNC(va_major_version, va_minor_version) VA_DRV_INIT_DEF(va_major_version, va_minor_version)
#define VA_DRV_INIT_FUC_NAME VA_DRV_INIT_FUNC(VA_MAJOR_VERSION, VA_MINOR_VERSION)

    //!
    //! \brief  VA driver init function name
    //!
    //! \param  [in] ctx
    //!         Pointer to VA driver context
    //!
    //! \return VAStatus
    //!     VA_STATUS_SUCCESS if success, else fail reason
    //!
    MEDIAAPI_EXPORT VAStatus VA_DRV_INIT_FUC_NAME(VADriverContextP ctx)
    {
        return __vaDriverInit(ctx);
    }

    //!
    //! \brief  Private API for openCL
    //!
    //! \param  [in] dpy
    //!         VA display
    //! \param  [in] surface
    //!         VA surface ID
    //! \param  [in] prime_fd
    //!         Prime fd
    //!
    //! \return VAStatus
    //!     VA_STATUS_SUCCESS if success, else fail reason
    //!
    MEDIAAPI_EXPORT VAStatus DdiMedia_ExtGetSurfaceHandle(
        VADisplay dpy,
        VASurfaceID *surface,
        int32_t *prime_fd)
    {
        DDI_CHK_NULL(dpy, "nullptr dpy", VA_STATUS_ERROR_INVALID_DISPLAY);
        DDI_CHK_NULL(surface, "nullptr surfaces", VA_STATUS_ERROR_INVALID_PARAMETER);
        DDI_CHK_NULL(prime_fd, "nullptr id", VA_STATUS_ERROR_INVALID_PARAMETER);

        VADriverContextP ctx = ((VADisplayContextP)dpy)->pDriverContext;
        DDI_CHK_NULL(ctx, "nullptr ctx", VA_STATUS_ERROR_INVALID_CONTEXT);

        PDDI_MEDIA_CONTEXT mediaCtx = DdiMedia_GetMediaContext(ctx);
        DDI_CHK_NULL(mediaCtx, "nullptr mediaCtx", VA_STATUS_ERROR_INVALID_CONTEXT);
        DDI_CHK_NULL(mediaCtx->pSurfaceHeap, "nullptr mediaCtx->pSurfaceHeap", VA_STATUS_ERROR_INVALID_CONTEXT);
        DDI_CHK_LESS((uint32_t)(*surface), mediaCtx->pSurfaceHeap->uiAllocatedHeapElements, "Invalid surfaces", VA_STATUS_ERROR_INVALID_SURFACE);

        DDI_MEDIA_SURFACE *mediaSurface = DdiMedia_GetSurfaceFromVASurfaceID(mediaCtx, *surface);
        if (mediaSurface)
        {
            if (mediaSurface->bo)
            {
                int32_t ret = mos_bo_gem_export_to_prime(mediaSurface->bo, (int32_t *)&mediaSurface->name);
                if (ret)
                {
                    // LOGE("Failed drm_intel_gem_export_to_prime operation!!!\n");
                    return VA_STATUS_ERROR_OPERATION_FAILED;
                }
            }
        }
        else
        {
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }

        *prime_fd = mediaSurface->name;

        return VA_STATUS_SUCCESS;
    }

    //!
    //! \brief  Map buffer 2
    //!
    //! \param  [in] dpy
    //!         VA display
    //! \param  [in] buf_id
    //!         VA buffer ID
    //! \param  [out] pbuf
    //!         Pointer to buffer
    //! \param  [in] flag
    //!         Flag
    //!
    //! \return VAStatus
    //!     VA_STATUS_SUCCESS if success, else fail reason
    //!
    MEDIAAPI_EXPORT VAStatus DdiMedia_MapBuffer2(
        VADisplay dpy,
        VABufferID buf_id,
        void **pbuf,
        int32_t flag)
    {
        DDI_CHK_NULL(dpy, "nullptr dpy", VA_STATUS_ERROR_INVALID_DISPLAY);

        VADriverContextP ctx = ((VADisplayContextP)dpy)->pDriverContext;

        if ((flag == 0) || (flag & ~(MOS_LOCKFLAG_READONLY | MOS_LOCKFLAG_WRITEONLY)))
            return VA_STATUS_ERROR_INVALID_PARAMETER;

        return DdiMedia_MapBufferInternal(ctx, buf_id, pbuf, flag);
    }

#ifdef __cplusplus
}
#endif
