/*
 * Copyright (c) 2014, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of Freescale Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#if !defined(__FSL_DMA_FEATURES_H__)
#define __FSL_DMA_FEATURES_H__

#if defined(CPU_MK20DX128VMP5) || defined(CPU_MK20DN128VMP5) || defined(CPU_MK20DX64VMP5) || defined(CPU_MK20DN64VMP5) || \
    defined(CPU_MK20DX32VMP5) || defined(CPU_MK20DN32VMP5) || defined(CPU_MK20DX128VLH5) || defined(CPU_MK20DN128VLH5) || \
    defined(CPU_MK20DX64VLH5) || defined(CPU_MK20DN64VLH5) || defined(CPU_MK20DX32VLH5) || defined(CPU_MK20DN32VLH5) || \
    defined(CPU_MK20DX128VFM5) || defined(CPU_MK20DN128VFM5) || defined(CPU_MK20DX64VFM5) || defined(CPU_MK20DN64VFM5) || \
    defined(CPU_MK20DX32VFM5) || defined(CPU_MK20DN32VFM5) || defined(CPU_MK20DX128VFT5) || defined(CPU_MK20DN128VFT5) || \
    defined(CPU_MK20DX64VFT5) || defined(CPU_MK20DN64VFT5) || defined(CPU_MK20DX32VFT5) || defined(CPU_MK20DN32VFT5) || \
    defined(CPU_MK20DX128VLF5) || defined(CPU_MK20DN128VLF5) || defined(CPU_MK20DX64VLF5) || defined(CPU_MK20DN64VLF5) || \
    defined(CPU_MK20DX32VLF5) || defined(CPU_MK20DN32VLF5)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (4)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 4)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (1)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (1)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (0)
#elif defined(CPU_MK22FN256VDC12) || defined(CPU_MK22FN256VLH12) || defined(CPU_MK22FN256VLL12) || defined(CPU_MK22FN256VMP12) || \
    defined(CPU_MK22FN512VDC12) || defined(CPU_MK22FN512VLH12) || defined(CPU_MK22FN512VLL12) || defined(CPU_MKV31F256VLH12) || \
    defined(CPU_MKV31F256VLL12) || defined(CPU_MKV31FN512VLH12) || defined(CPU_MKV31F512VLL12)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (16)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 16)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (1)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (1)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (4)
#elif defined(CPU_MK24FN1M0VDC12) || defined(CPU_MK24FN1M0VLQ12) || defined(CPU_MK63FN1M0VLQ12) || defined(CPU_MK63FN1M0VMD12) || \
    defined(CPU_MK64FN1M0VDC12) || defined(CPU_MK64FN1M0VLL12) || defined(CPU_MK64FN1M0VLQ12) || defined(CPU_MK64FX512VLQ12) || \
    defined(CPU_MK64FN1M0VMD12) || defined(CPU_MK64FX512VMD12)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (16)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 16)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (1)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (1)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (0)
#elif defined(CPU_MK65FN2M0VMF18) || defined(CPU_MK65FX1M0VMF18) || defined(CPU_MK66FN2M0VLQ18) || defined(CPU_MK66FX1M0VLQ18)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (32)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 32)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (1)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (2)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (32)
#elif defined(CPU_MK70FN1M0VMF12) || defined(CPU_MK70FX512VMF12) || defined(CPU_MK70FN1M0VMF15) || defined(CPU_MK70FX512VMF15) || \
    defined(CPU_MK70FN1M0VMJ12) || defined(CPU_MK70FX512VMJ12) || defined(CPU_MK70FN1M0VMJ15) || defined(CPU_MK70FX512VMJ15)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (32)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 32)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (1)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (2)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (0)
#elif defined(CPU_MKL05Z8VFK4) || defined(CPU_MKL05Z16VFK4) || defined(CPU_MKL05Z32VFK4) || defined(CPU_MKL05Z8VLC4) || \
    defined(CPU_MKL05Z16VLC4) || defined(CPU_MKL05Z32VLC4) || defined(CPU_MKL05Z8VFM4) || defined(CPU_MKL05Z16VFM4) || \
    defined(CPU_MKL05Z32VFM4) || defined(CPU_MKL05Z16VLF4) || defined(CPU_MKL05Z32VLF4) || defined(CPU_MKL25Z32VFM4) || \
    defined(CPU_MKL25Z64VFM4) || defined(CPU_MKL25Z128VFM4) || defined(CPU_MKL25Z32VFT4) || defined(CPU_MKL25Z64VFT4) || \
    defined(CPU_MKL25Z128VFT4) || defined(CPU_MKL25Z32VLH4) || defined(CPU_MKL25Z64VLH4) || defined(CPU_MKL25Z128VLH4) || \
    defined(CPU_MKL25Z32VLK4) || defined(CPU_MKL25Z64VLK4) || defined(CPU_MKL25Z128VLK4) || defined(CPU_MKL46Z128VLH4) || \
    defined(CPU_MKL46Z256VLH4) || defined(CPU_MKL46Z128VLL4) || defined(CPU_MKL46Z256VLL4) || defined(CPU_MKL46Z128VMC4) || \
    defined(CPU_MKL46Z256VMC4)
    /* @brief Number of DMA channels (related to number of registers TCD, DCHPRI, bit fields ERQ[ERQn], EEI[EEIn], INT[INTn], ERR[ERRn], HRS[HRSn] and bit field widths ES[ERRCHN], CEEI[CEEI], SEEI[SEEI], CERQ[CERQ], SERQ[SERQ], CDNE[CDNE], SSRT[SSRT], CERR[CERR], CINT[CINT], TCDn_CITER_ELINKYES[LINKCH], TCDn_CSR[MAJORLINKCH], TCDn_BITER_ELINKYES[LINKCH]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_MODULE_CHANNEL (4)
    /* @brief Total number of DMA channels on all modules.*/
    #define FSL_FEATURE_DMA_DMAMUX_CHANNELS (HW_DMA_INSTANCE_COUNT * 4)
    /* @brief Enhanced Direct Memory Access Controller (registers CR, ES, ERQ, EEI, CEEI, SEEI, CERQ, SERQ, CDNE, SSRT, CERR, CINT, INT, ERR, HRS, DCHPRIn, TCDn if non-zero, otherwise SARn, DARn, DSR_BCRn, DCRn).*/
    #define FSL_FEATURE_DMA_IS_EDMA (0)
    /* @brief Number of DMA channel groups (register bit fields CR[ERGA], CR[GRPnPRI], ES[GPE], DCHPRIn[GRPPRI]). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_CHANNEL_GROUP_COUNT (0)
    /* @brief Number of DMA channels with asynchronous request capability (register EARS). (Valid only for eDMA modules.)*/
    #define FSL_FEATURE_DMA_ASYNCHRO_REQUEST_CHANNEL_COUNT (4)
#else
    #error "No valid CPU defined!"
#endif

#endif /* __FSL_DMA_FEATURES_H__*/
/*******************************************************************************
 * EOF
 ******************************************************************************/

