/*****************************************************************************
* Copyright (C) 2013 x265 project
*
* Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02111, USA.
*
* This program is also available under a commercial proprietary license.
* For more information, contact us at license @ x265.com.
*****************************************************************************/

#include "common.h"
#include "slice.h"
#include "framedata.h"
#include "picyuv.h"
#include "predict.h"
#include "primitives.h"

using namespace X265_NS;

#if _MSC_VER
#pragma warning(disable: 4127) // conditional expression is constant
#endif

PredictionUnit::PredictionUnit(const CUData& cu, const CUGeom& cuGeom, int puIdx)
{
    /* address of CTU */
    ctuAddr = cu.m_cuAddr;

    /* offset of CU */
    cuAbsPartIdx = cuGeom.absPartIdx;

    /* offset and dimensions of PU */
    cu.getPartIndexAndSize(puIdx, puAbsPartIdx, width, height);
}

namespace
{
inline pixel weightBidir(int w0, int16_t P0, int w1, int16_t P1, int round, int shift, int offset)
{
    return x265_clip((w0 * (P0 + IF_INTERNAL_OFFS) + w1 * (P1 + IF_INTERNAL_OFFS) + round + (offset * (1 << (shift - 1)))) >> shift);
}
}

Predict::Predict()
{
    m_immedVals = NULL;
}

Predict::~Predict()
{
    X265_FREE(m_immedVals);
    m_predShortYuv[0].destroy();
    m_predShortYuv[1].destroy();
}

bool Predict::allocBuffers(int csp)
{
    m_csp = csp;
    m_hChromaShift = CHROMA_H_SHIFT(csp);
    m_vChromaShift = CHROMA_V_SHIFT(csp);
    CHECKED_MALLOC(m_immedVals, int16_t, 64 * (64 + NTAPS_LUMA - 1));

    return m_predShortYuv[0].create(MAX_CU_SIZE, csp) && m_predShortYuv[1].create(MAX_CU_SIZE, csp);

fail:
    return false;
}
void Predict::predIntraLumaAng(uint32_t dirMode, pixel* dst, intptr_t stride, uint32_t log2TrSize)
{
    int tuSize = 1 << log2TrSize;
    int sizeIdx = log2TrSize - 2;
    X265_CHECK(sizeIdx >= 0 && sizeIdx < 4, "intra block size is out of range\n");

    int filter = !!(g_intraFilterFlags[dirMode] & tuSize);
    bool bFilter = log2TrSize <= 4;
    primitives.cu[sizeIdx].intra_pred[dirMode](dst, stride, intraNeighbourBuf[filter], dirMode, bFilter);
}

void Predict::predIntraChromaAng(uint32_t dirMode, pixel* dst, intptr_t stride, uint32_t log2TrSizeC)
{
    int tuSize = 1 << log2TrSizeC;
    int sizeIdx = log2TrSizeC - 2;
    X265_CHECK(sizeIdx >= 0 && sizeIdx < 4, "intra block size is out of range\n");

    int filter = !!(m_csp == X265_CSP_I444 && (g_intraFilterFlags[dirMode] & tuSize));
    primitives.cu[sizeIdx].intra_pred[dirMode](dst, stride, intraNeighbourBuf[filter], dirMode, 0);
}

void Predict::initAdiPattern(const CUData& cu, const CUGeom& cuGeom, uint32_t puAbsPartIdx, const IntraNeighbors& intraNeighbors, int dirMode)
{
    int tuSize = 1 << intraNeighbors.log2TrSize;
    int tuSize2 = tuSize << 1;

    pixel* adiOrigin = cu.m_encData->m_reconPic->getLumaAddr(cu.m_cuAddr, cuGeom.absPartIdx + puAbsPartIdx);
    intptr_t picStride = cu.m_encData->m_reconPic->m_stride;

    fillReferenceSamples(adiOrigin, picStride, intraNeighbors, intraNeighbourBuf[0]);

    pixel* refBuf = intraNeighbourBuf[0];
    pixel* fltBuf = intraNeighbourBuf[1];

    pixel topLeft = refBuf[0], topLast = refBuf[tuSize2], leftLast = refBuf[tuSize2 + tuSize2];

    if (dirMode == ALL_IDX ? (8 | 16 | 32) & tuSize : g_intraFilterFlags[dirMode] & tuSize)
    {
        // generate filtered intra prediction samples

        if (cu.m_slice->m_sps->bUseStrongIntraSmoothing && tuSize == 32)
        {
            const int threshold = 1 << (X265_DEPTH - 5);

            pixel topMiddle = refBuf[32], leftMiddle = refBuf[tuSize2 + 32];

            if (abs(topLeft + topLast  - (topMiddle  << 1)) < threshold &&
                abs(topLeft + leftLast - (leftMiddle << 1)) < threshold)
            {
                // "strong" bilinear interpolation
                const int shift = 5 + 1;
                int init = (topLeft << shift) + tuSize;
                int deltaL, deltaR;

                deltaL = leftLast - topLeft; deltaR = topLast - topLeft;

                fltBuf[0] = topLeft;
                for (int i = 1; i < tuSize2; i++)
                {
                    fltBuf[i + tuSize2] = (pixel)((init + deltaL * i) >> shift); // Left Filtering
                    fltBuf[i] = (pixel)((init + deltaR * i) >> shift);           // Above Filtering
                }
                fltBuf[tuSize2] = topLast;
                fltBuf[tuSize2 + tuSize2] = leftLast;
                return;
            }
        }

        primitives.cu[intraNeighbors.log2TrSize - 2].intra_filter(refBuf, fltBuf);
    }
}

void Predict::initAdiPatternChroma(const CUData& cu, const CUGeom& cuGeom, uint32_t puAbsPartIdx, const IntraNeighbors& intraNeighbors, uint32_t chromaId)
{
    const pixel* adiOrigin = cu.m_encData->m_reconPic->getChromaAddr(chromaId, cu.m_cuAddr, cuGeom.absPartIdx + puAbsPartIdx);
    intptr_t picStride = cu.m_encData->m_reconPic->m_strideC;

    fillReferenceSamples(adiOrigin, picStride, intraNeighbors, intraNeighbourBuf[0]);

    if (m_csp == X265_CSP_I444)
        primitives.cu[intraNeighbors.log2TrSize - 2].intra_filter(intraNeighbourBuf[0], intraNeighbourBuf[1]);
}

void Predict::initIntraNeighbors(const CUData& cu, uint32_t absPartIdx, uint32_t tuDepth, bool isLuma, IntraNeighbors *intraNeighbors)
{
    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;
    int log2UnitWidth = LOG2_UNIT_SIZE;
    int log2UnitHeight = LOG2_UNIT_SIZE;

    if (!isLuma)
    {
        log2TrSize -= cu.m_hChromaShift;
        log2UnitWidth -= cu.m_hChromaShift;
        log2UnitHeight -= cu.m_vChromaShift;
    }

    int numIntraNeighbor;
    bool* bNeighborFlags = intraNeighbors->bNeighborFlags;

    uint32_t numPartInWidth = 1 << (cu.m_log2CUSize[0] - LOG2_UNIT_SIZE - tuDepth);
    uint32_t partIdxLT = cu.m_absIdxInCTU + absPartIdx;
    uint32_t partIdxRT = g_rasterToZscan[g_zscanToRaster[partIdxLT] + numPartInWidth - 1];

    uint32_t tuSize = 1 << log2TrSize;
    int  tuWidthInUnits = tuSize >> log2UnitWidth;
    int  tuHeightInUnits = tuSize >> log2UnitHeight;
    int  aboveUnits = tuWidthInUnits << 1;
    int  leftUnits = tuHeightInUnits << 1;
    int  partIdxStride = cu.m_slice->m_sps->numPartInCUSize;
    uint32_t partIdxLB = g_rasterToZscan[g_zscanToRaster[partIdxLT] + ((tuHeightInUnits - 1) * partIdxStride)];

    if (cu.m_slice->isIntra() || !cu.m_slice->m_pps->bConstrainedIntraPred)
    {
        bNeighborFlags[leftUnits] = isAboveLeftAvailable<false>(cu, partIdxLT);
        numIntraNeighbor  = (int)(bNeighborFlags[leftUnits]);
        numIntraNeighbor += isAboveAvailable<false>(cu, partIdxLT, partIdxRT, bNeighborFlags + leftUnits + 1);
        numIntraNeighbor += isAboveRightAvailable<false>(cu, partIdxRT, bNeighborFlags + leftUnits + 1 + tuWidthInUnits, tuWidthInUnits);
        numIntraNeighbor += isLeftAvailable<false>(cu, partIdxLT, partIdxLB, bNeighborFlags + leftUnits - 1);
        numIntraNeighbor += isBelowLeftAvailable<false>(cu, partIdxLB, bNeighborFlags + tuHeightInUnits - 1, tuHeightInUnits);
    }
    else
    {
        bNeighborFlags[leftUnits] = isAboveLeftAvailable<true>(cu, partIdxLT);
        numIntraNeighbor  = (int)(bNeighborFlags[leftUnits]);
        numIntraNeighbor += isAboveAvailable<true>(cu, partIdxLT, partIdxRT, bNeighborFlags + leftUnits + 1);
        numIntraNeighbor += isAboveRightAvailable<true>(cu, partIdxRT, bNeighborFlags + leftUnits + 1 + tuWidthInUnits, tuWidthInUnits);
        numIntraNeighbor += isLeftAvailable<true>(cu, partIdxLT, partIdxLB, bNeighborFlags + leftUnits - 1);
        numIntraNeighbor += isBelowLeftAvailable<true>(cu, partIdxLB, bNeighborFlags + tuHeightInUnits - 1, tuHeightInUnits);
    }

    intraNeighbors->numIntraNeighbor = numIntraNeighbor;
    intraNeighbors->totalUnits = aboveUnits + leftUnits + 1;
    intraNeighbors->aboveUnits = aboveUnits;
    intraNeighbors->leftUnits = leftUnits;
    intraNeighbors->unitWidth = 1 << log2UnitWidth;
    intraNeighbors->unitHeight = 1 << log2UnitHeight;
    intraNeighbors->log2TrSize = log2TrSize;
}

void Predict::fillReferenceSamples(const pixel* adiOrigin, intptr_t picStride, const IntraNeighbors& intraNeighbors, pixel dst[258])
{
    const pixel dcValue = (pixel)(1 << (X265_DEPTH - 1));
    int numIntraNeighbor = intraNeighbors.numIntraNeighbor;
    int totalUnits = intraNeighbors.totalUnits;
    uint32_t tuSize = 1 << intraNeighbors.log2TrSize;
    uint32_t refSize = tuSize * 2 + 1;

    // Nothing is available, perform DC prediction.
    if (numIntraNeighbor == 0)
    {
        // Fill top border with DC value
        for (uint32_t i = 0; i < refSize; i++)
            dst[i] = dcValue;

        // Fill left border with DC value
        for (uint32_t i = 0; i < refSize - 1; i++)
            dst[i + refSize] = dcValue;
    }
    else if (numIntraNeighbor == totalUnits)
    {
        // Fill top border with rec. samples
        const pixel* adiTemp = adiOrigin - picStride - 1;
        memcpy(dst, adiTemp, refSize * sizeof(pixel));

        // Fill left border with rec. samples
        adiTemp = adiOrigin - 1;
        for (uint32_t i = 0; i < refSize - 1; i++)
        {
            dst[i + refSize] = adiTemp[0];
            adiTemp += picStride;
        }
    }
    else // reference samples are partially available
    {
        const bool *bNeighborFlags = intraNeighbors.bNeighborFlags;
        const bool *pNeighborFlags;
        int aboveUnits = intraNeighbors.aboveUnits;
        int leftUnits = intraNeighbors.leftUnits;
        int unitWidth = intraNeighbors.unitWidth;
        int unitHeight = intraNeighbors.unitHeight;
        int totalSamples = (leftUnits * unitHeight) + ((aboveUnits + 1) * unitWidth);
        pixel adiLineBuffer[5 * MAX_CU_SIZE];
        pixel *adi;

        // Initialize
        for (int i = 0; i < totalSamples; i++)
            adiLineBuffer[i] = dcValue;

        // Fill top-left sample
        const pixel* adiTemp = adiOrigin - picStride - 1;
        adi = adiLineBuffer + (leftUnits * unitHeight);
        pNeighborFlags = bNeighborFlags + leftUnits;
        if (*pNeighborFlags)
        {
            pixel topLeftVal = adiTemp[0];
            for (int i = 0; i < unitWidth; i++)
                adi[i] = topLeftVal;
        }

        // Fill left & below-left samples
        adiTemp += picStride;
        adi--;
        // NOTE: over copy here, but reduce condition operators
        for (int j = 0; j < leftUnits * unitHeight; j++)
        {
            adi[-j] = adiTemp[j * picStride];
        }

        // Fill above & above-right samples
        adiTemp = adiOrigin - picStride;
        adi = adiLineBuffer + (leftUnits * unitHeight) + unitWidth;
        // NOTE: over copy here, but reduce condition operators
        memcpy(adi, adiTemp, aboveUnits * unitWidth * sizeof(*adiTemp));

        // Pad reference samples when necessary
        int curr = 0;
        int next = 1;
        adi = adiLineBuffer;
        int pAdiLineTopRowOffset = leftUnits * (unitHeight - unitWidth);
        if (!bNeighborFlags[0])
        {
            // very bottom unit of bottom-left; at least one unit will be valid.
            while (next < totalUnits && !bNeighborFlags[next])
                next++;

            pixel* pAdiLineNext = adiLineBuffer + ((next < leftUnits) ? (next * unitHeight) : (pAdiLineTopRowOffset + (next * unitWidth)));
            const pixel refSample = *pAdiLineNext;
            // Pad unavailable samples with new value
            int nextOrTop = X265_MIN(next, leftUnits);

            // fill left column
#if HIGH_BIT_DEPTH
            while (curr < nextOrTop)
            {
                for (int i = 0; i < unitHeight; i++)
                    adi[i] = refSample;

                adi += unitHeight;
                curr++;
            }

            // fill top row
            while (curr < next)
            {
                for (int i = 0; i < unitWidth; i++)
                    adi[i] = refSample;

                adi += unitWidth;
                curr++;
            }
#else
            X265_CHECK(curr <= nextOrTop, "curr must be less than or equal to nextOrTop\n");
            if (curr < nextOrTop)
            {
                const int fillSize = unitHeight * (nextOrTop - curr);
                memset(adi, refSample, fillSize * sizeof(pixel));
                curr = nextOrTop;
                adi += fillSize;
            }

            if (curr < next)
            {
                const int fillSize = unitWidth * (next - curr);
                memset(adi, refSample, fillSize * sizeof(pixel));
                curr = next;
                adi += fillSize;
            }
#endif
        }

        // pad all other reference samples.
        while (curr < totalUnits)
        {
            if (!bNeighborFlags[curr]) // samples not available
            {
                int numSamplesInCurrUnit = (curr >= leftUnits) ? unitWidth : unitHeight;
                const pixel refSample = *(adi - 1);
                for (int i = 0; i < numSamplesInCurrUnit; i++)
                    adi[i] = refSample;

                adi += numSamplesInCurrUnit;
                curr++;
            }
            else
            {
                adi += (curr >= leftUnits) ? unitWidth : unitHeight;
                curr++;
            }
        }

        // Copy processed samples
        adi = adiLineBuffer + refSize + unitWidth - 2;
        memcpy(dst, adi, refSize * sizeof(pixel));

        adi = adiLineBuffer + refSize - 1;
        for (int i = 0; i < (int)refSize - 1; i++)
            dst[i + refSize] = adi[-(i + 1)];
    }
}

template<bool cip>
bool Predict::isAboveLeftAvailable(const CUData& cu, uint32_t partIdxLT)
{
    uint32_t partAboveLeft;
    const CUData* cuAboveLeft = cu.getPUAboveLeft(partAboveLeft, partIdxLT);

    return cuAboveLeft && (!cip || cuAboveLeft->isIntra(partAboveLeft));
}

template<bool cip>
int Predict::isAboveAvailable(const CUData& cu, uint32_t partIdxLT, uint32_t partIdxRT, bool* bValidFlags)
{
    const uint32_t rasterPartBegin = g_zscanToRaster[partIdxLT];
    const uint32_t rasterPartEnd = g_zscanToRaster[partIdxRT];
    const uint32_t idxStep = 1;
    int numIntra = 0;

    for (uint32_t rasterPart = rasterPartBegin; rasterPart <= rasterPartEnd; rasterPart += idxStep, bValidFlags++)
    {
        uint32_t partAbove;
        const CUData* cuAbove = cu.getPUAbove(partAbove, g_rasterToZscan[rasterPart]);
        if (cuAbove && (!cip || cuAbove->isIntra(partAbove)))
        {
            numIntra++;
            *bValidFlags = true;
        }
        else
            *bValidFlags = false;
    }

    return numIntra;
}

template<bool cip>
int Predict::isLeftAvailable(const CUData& cu, uint32_t partIdxLT, uint32_t partIdxLB, bool* bValidFlags)
{
    const uint32_t rasterPartBegin = g_zscanToRaster[partIdxLT];
    const uint32_t rasterPartEnd = g_zscanToRaster[partIdxLB];
    const uint32_t idxStep = cu.m_slice->m_sps->numPartInCUSize;
    int numIntra = 0;

    for (uint32_t rasterPart = rasterPartBegin; rasterPart <= rasterPartEnd; rasterPart += idxStep, bValidFlags--) // opposite direction
    {
        uint32_t partLeft;
        const CUData* cuLeft = cu.getPULeft(partLeft, g_rasterToZscan[rasterPart]);
        if (cuLeft && (!cip || cuLeft->isIntra(partLeft)))
        {
            numIntra++;
            *bValidFlags = true;
        }
        else
            *bValidFlags = false;
    }

    return numIntra;
}

template<bool cip>
int Predict::isAboveRightAvailable(const CUData& cu, uint32_t partIdxRT, bool* bValidFlags, uint32_t numUnits)
{
    int numIntra = 0;

    for (uint32_t offset = 1; offset <= numUnits; offset++, bValidFlags++)
    {
        uint32_t partAboveRight;
        const CUData* cuAboveRight = cu.getPUAboveRightAdi(partAboveRight, partIdxRT, offset);
        if (cuAboveRight && (!cip || cuAboveRight->isIntra(partAboveRight)))
        {
            numIntra++;
            *bValidFlags = true;
        }
        else
            *bValidFlags = false;
    }

    return numIntra;
}

template<bool cip>
int Predict::isBelowLeftAvailable(const CUData& cu, uint32_t partIdxLB, bool* bValidFlags, uint32_t numUnits)
{
    int numIntra = 0;

    for (uint32_t offset = 1; offset <= numUnits; offset++, bValidFlags--) // opposite direction
    {
        uint32_t partBelowLeft;
        const CUData* cuBelowLeft = cu.getPUBelowLeftAdi(partBelowLeft, partIdxLB, offset);
        if (cuBelowLeft && (!cip || cuBelowLeft->isIntra(partBelowLeft)))
        {
            numIntra++;
            *bValidFlags = true;
        }
        else
            *bValidFlags = false;
    }

    return numIntra;
}
