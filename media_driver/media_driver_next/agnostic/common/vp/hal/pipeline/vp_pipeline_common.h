/*
* Copyright (c) 2018-2021, Intel Corporation
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
//! \file     vp_pipeline_common.h
//! \brief    Defines the common interface for vp pipeline
//!           this file is for the base interface which is shared by all features/vp Pipelines.
//!
#ifndef __VP_PIPELINE_COMMON_H__
#define __VP_PIPELINE_COMMON_H__

#include "mos_utilities.h"
#include "vphal_common.h"
#include "renderhal.h"
#include "vphal.h"

namespace vp
{
class VpPlatformInterface;
}

using VP_PIPELINE_PARAMS   = VPHAL_RENDER_PARAMS;
using PVP_PIPELINE_PARAMS  = VPHAL_RENDER_PARAMS*;
using PCVP_PIPELINE_PARAMS = const VPHAL_RENDER_PARAMS*;

//!
//! \brief Flags for update/copy/FMD kernels
//!
#define VP_VEBOX_FLAG_ENABLE_KERNEL_COPY                   0x00000001
#define VP_VEBOX_FLAG_ENABLE_KERNEL_COPY_DEBUG             0x00000002
#define VP_VEBOX_FLAG_ENABLE_KERNEL_DN_UPDATE              0x00000004
#define VP_VEBOX_FLAG_ENABLE_KERNEL_DN_UPDATE_DEBUG        0x00000008
#define VP_VEBOX_FLAG_ENABLE_KERNEL_FMD_SUMMATION          0x00000010

#define RESOURCE_ASSIGNMENT_HINT_BITS_DI        \
    uint32_t    bDi         : 1;                \
    uint32_t    b60fpsDi    : 1;                \

#define RESOURCE_ASSIGNMENT_HINT_BITS           \
        RESOURCE_ASSIGNMENT_HINT_BITS_DI
#define RESOURCE_ASSIGNMENT_HINT_SIZE   2

struct VP_SURFACE
{
    MOS_SURFACE                 *osSurface;         //!< mos surface
    bool                        isResourceOwner;    //!< true if the resource is owned by current instance.
    VPHAL_CSPACE                ColorSpace;         //!< Color Space
    uint32_t                    ChromaSiting;       //!< ChromaSiting

    bool                        bQueryVariance;     //!< enable variance query. Not in use for internal surface
    int32_t                     FrameID;            //!< Not in use for internal surface
    bool                        ExtendedGamut;      //!< Extended Gamut Flag. Not in use for internal surface
    VPHAL_PALETTE               Palette;            //!< Palette data. Not in use for internal surface
    VPHAL_SURFACE_TYPE          SurfType;           //!< Surface type (context). Not in use for internal surface
    uint32_t                    uFwdRefCount;       //!< Not in use for internal surface
    uint32_t                    uBwdRefCount;       //!< Not in use for internal surface
    VPHAL_SURFACE               *pFwdRef;           //!< Use VP_SURFACE instead of VPHAL_SURFACE later. Not in use for internal surface.
    VPHAL_SURFACE               *pBwdRef;           //!< Use VP_SURFACE instead of VPHAL_SURFACE later. Not in use for internal surface.
    VPHAL_SAMPLE_TYPE           SampleType;         //!<  Interlaced/Progressive sample type.
    // Use index of m_InputSurfaces for layerID. No need iLayerID here anymore.
    RECT                        rcSrc;              //!< Source rectangle
    RECT                        rcDst;              //!< Destination rectangle
    RECT                        rcMaxSrc;           //!< Max source rectangle
    bool                        bVEBOXCroppingUsed; //!<Vebox crop case need use rcSrc as vebox input.
    uint32_t                    bufferWidth;        //!< 1D buffer Width, n/a if 2D surface
    uint32_t                    bufferHeight;       //!< 1D buffer Height, n/a if 2D surface
    // Return true if no resource assigned to current vp surface.
    bool        IsEmpty();
    // Clean the vp surface to empty state. Only valid for false == isResourceOwner case.
    MOS_STATUS  Clean();

    // Get Allocation Handle of resource
    uint64_t    GetAllocationHandle();
};

struct _VP_SETTINGS
{
    // For validation purpose settings
    uint32_t               disableDnDi;                              //!< Disable DNDI(Vebox)
    uint32_t               kernelUpdate;                             //!< For VEBox Copy and Update kernels
    uint32_t               disableHdr;                               //!< Disable Hdr
    uint32_t               veboxParallelExecution;                   //!< Control VEBox parallel execution with render engine
};

using VP_SETTINGS = _VP_SETTINGS;

struct _VP_MHWINTERFACE
{
    // Internals
    PLATFORM                    m_platform;
    MEDIA_FEATURE_TABLE         *m_skuTable;
    MEDIA_WA_TABLE              *m_waTable;

    // States
    PMOS_INTERFACE              m_osInterface;
    PRENDERHAL_INTERFACE        m_renderHal;
    PMHW_VEBOX_INTERFACE        m_veboxInterface;
    MhwCpInterface             *m_cpInterface;
    PMHW_SFC_INTERFACE          m_sfcInterface;
    VphalRenderer              *m_renderer;
    PMHW_MI_INTERFACE           m_mhwMiInterface;
    vp::VpPlatformInterface    *m_vpPlatformInterface;
    void                       *m_settings;
    VphalFeatureReport         *m_reporting;

    // Render GPU context/node
    MOS_GPU_NODE                m_renderGpuNode;
    MOS_GPU_CONTEXT             m_renderGpuContext;

    // vp Pipeline workload status report
    PVPHAL_STATUS_TABLE        m_statusTable;

    void                      *m_debugInterface;
};

// To define the features enabling on different engines
struct _VP_EXECUTE_CAPS
{
    union {
        uint64_t value;
        struct {
            uint64_t bVebox         : 1;   // Vebox needed
            uint64_t bSFC           : 1;   // SFC needed
            uint64_t bRender        : 1;   // Render Only needed
            uint64_t bSecureVebox   : 1;   // Vebox in Secure Mode
            // Vebox Features
            uint64_t bDN            : 1;   // Vebox DN needed
            uint64_t bDI            : 1;   // Vebox DI enabled
            uint64_t bDiProcess2ndField : 1;   // Vebox DI enabled
            uint64_t bDIFmdKernel   : 1;   // Vebox FMD Kernel enabled
            uint64_t bIECP          : 1;   // Vebox IECP needed
            uint64_t bSTE           : 1;   // Vebox STE needed
            uint64_t bACE           : 1;   // Vebox ACE needed
            uint64_t bTCC           : 1;   // Vebox TCC needed
            uint64_t bCGC           : 1;   // Vebox CGC needed
            uint64_t bBt2020ToRGB   : 1;   // Vebox Bt2020 gamut compression to RGB format
            uint64_t bProcamp       : 1;   // Vebox Procamp needed
            uint64_t bBeCSC         : 1;   // Vebox back end CSC needed
            uint64_t bLACE          : 1;   // Vebox LACE Needed
            uint64_t bQueryVariance : 1;
            uint64_t bRefValid      : 1;   // Vebox Ref is Valid
            uint64_t bSTD           : 1;   // Vebox LACE STD Needed
            uint64_t bDnKernelUpdate: 1;
            uint64_t bVeboxSecureCopy : 1;
            uint64_t bHDR3DLUT      : 1;  // Vebox 3DLUT needed
            uint64_t bDV            : 1;
            uint64_t b3DlutOutput   : 1;

            // SFC features
            uint64_t bSfcCsc        : 1;   // Sfc Csc enabled
            uint64_t bSfcRotMir     : 1;       // Sfc Rotation/Mirror needed
            uint64_t bSfcScaling    : 1;   // Sfc Scaling Needed
            uint64_t bSfcIef        : 1;   // Sfc Details Needed
            uint64_t b1stPassOfSfc2PassScaling : 1; // 1st pass of sfc 2pass scaling

            // Render Features
            uint64_t bComposite     : 1;
            uint64_t bSR            : 1;
            uint64_t reserved       : 33;   // Reserved
        };
    };
};

typedef struct _VP_EngineEntry
{
    union
    {
        struct
        {
            uint32_t bEnabled : 1;
            uint32_t SfcNeeded : 2;
            uint32_t VeboxNeeded : 2;
            uint32_t RenderNeeded : 2;
            uint32_t VeboxARGBOut : 1;
            uint32_t VeboxARGB10bitOutput : 1;
            uint32_t VeboxIECPNeeded : 1;
            uint32_t DisableVeboxSFCMode : 1;
            uint32_t FurtherProcessNeeded : 1;
            uint32_t CompositionNeeded : 1;
            uint32_t bypassVeboxFeatures : 1;
            uint32_t sfc2PassScalingNeededX : 1;
            uint32_t sfc2PassScalingNeededY : 1;
            uint32_t reserve : 16;
        };
        uint32_t value;
    };
}VP_EngineEntry;

enum _VP_PACKET_ENGINE
{
    VP_PACKET_COMP = 0,
    VP_PACKET_VEBOX,
};

enum VP_RGB_OUTPUT_OVERRIDE_ID
{
    VP_RGB_OUTPUT_OVERRIDE_ID_INVALID = 0,
    VP_RGB_OUTPUT_OVERRIDE_ID_RGBP_LINEAR,
    VP_RGB_OUTPUT_OVERRIDE_ID_RGBP_TILE,
    VP_RGB_OUTPUT_OVERRIDE_ID_RGB24LINEAR,
    VP_RGB_OUTPUT_OVERRIDE_ID_BGRP_LINEAR,
    VP_RGB_OUTPUT_OVERRIDE_ID_BGRP_TILE,
    VP_RGB_OUTPUT_OVERRIDE_ID_MAX,
};

typedef enum _VP_COMP_BYPASS_MODE
{
    VP_COMP_BYPASS_NOT_SET  = 0xffffffff,
    VP_COMP_BYPASS_DISABLED = 0x0,
    VP_COMP_BYPASS_ENABLED  = 0x1
} VP_COMP_BYPASS_MODE, * PVP_COMP_BYPASS_MODE;

// RESOURCE_ASSIGNMENT_HINT are feature parameters which will affect resource assignment.
// For caps existing in VP_EXECUTE_CAPS, they should not be added to RESOURCE_ASSIGNMENT_HINT.
union RESOURCE_ASSIGNMENT_HINT
{
    struct
    {
        RESOURCE_ASSIGNMENT_HINT_BITS;
    };
    uint32_t value[RESOURCE_ASSIGNMENT_HINT_SIZE];
};

using VP_MHWINTERFACE  = _VP_MHWINTERFACE;
using PVP_MHWINTERFACE = VP_MHWINTERFACE * ;
using VP_EXECUTE_CAPS  = _VP_EXECUTE_CAPS;
using VP_PACKET_ENGINE = _VP_PACKET_ENGINE;
using PVP_SURFACE      = VP_SURFACE*;

inline bool IsVeboxFeatureInuse(VP_EXECUTE_CAPS &caps)
{
    return (caps.bVebox && (!caps.bSFC || caps.bDN || caps.bDI || caps.bIECP || caps.bSTE ||
            caps.bACE || caps.bTCC || caps.bBeCSC || caps.bQueryVariance || caps.bLACE || caps.bSTD ||
            caps.bHDR3DLUT || caps.bDV));
}

#endif
