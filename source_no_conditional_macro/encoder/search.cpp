/*****************************************************************************
* Copyright (C) 2013 x265 project
*
* Authors: Steve Borho <steve@borho.org>
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
#include "primitives.h"
#include "picyuv.h"
#include "cudata.h"

#include "search.h"
#include "entropy.h"
#include "rdcost.h"

#include "analysis.h"  // TLD
#include "framedata.h"

using namespace X265_NS;

#if _MSC_VER
#pragma warning(disable: 4800) // 'uint8_t' : forcing value to bool 'true' or 'false' (performance warning)
#pragma warning(disable: 4244) // '=' : conversion from 'int' to 'uint8_t', possible loss of data)
#pragma warning(disable: 4127) // conditional expression is constant
#endif

#define MVP_IDX_BITS 1

ALIGN_VAR_32(const int16_t, Search::zeroShort[MAX_CU_SIZE]) = { 0 };

Search::Search()
{
    memset(m_rqt, 0, sizeof(m_rqt));

    for (int i = 0; i < 3; i++)
    {
        m_qtTempTransformSkipFlag[i] = NULL;
        m_qtTempCbf[i] = NULL;
    }

    m_numLayers = 0;
    m_intraPred = NULL;
    m_intraPredAngs = NULL;
    m_fencScaled = NULL;
    m_fencTransposed = NULL;
    m_tsCoeff = NULL;
    m_tsResidual = NULL;
    m_tsRecon = NULL;
    m_param = NULL;
    m_slice = NULL;
    m_frame = NULL;
}

bool Search::initSearch(const x265_param& param, ScalingList& scalingList)
{
    uint32_t maxLog2CUSize = g_log2Size[param.maxCUSize];
    m_param = &param;
    m_bEnableRDOQ = !!param.rdoqLevel;
    m_bFrameParallel = param.frameNumThreads > 1;
    m_numLayers = g_log2Size[param.maxCUSize] - 2;

    m_rdCost.setPsyRdScale(param.psyRd);
    m_me.init(param.searchMethod, param.subpelRefine, param.internalCsp);

    bool ok = m_quant.init(param.rdoqLevel, param.psyRdoq, scalingList, m_entropyCoder);
    if (m_param->noiseReductionIntra || m_param->noiseReductionInter)
        ok &= m_quant.allocNoiseReduction(param);

    ok &= Predict::allocBuffers(param.internalCsp); /* sets m_hChromaShift & m_vChromaShift */

    /* When frame parallelism is active, only 'refLagPixels' of reference frames will be guaranteed
     * available for motion reference.  See refLagRows in FrameEncoder::compressCTURows() */
    m_refLagPixels = m_bFrameParallel ? param.searchRange : param.sourceHeight;

    uint32_t sizeL = 1 << (maxLog2CUSize * 2);
    uint32_t sizeC = sizeL >> (m_hChromaShift + m_vChromaShift);
    uint32_t numPartitions = 1 << (maxLog2CUSize - LOG2_UNIT_SIZE) * 2;

    /* these are indexed by qtLayer (log2size - 2) so nominally 0=4x4, 1=8x8, 2=16x16, 3=32x32
     * the coeffRQT and reconQtYuv are allocated to the max CU size at every depth. The parts
     * which are reconstructed at each depth are valid. At the end, the transform depth table
     * is walked and the coeff and recon at the correct depths are collected */
    for (uint32_t i = 0; i <= m_numLayers; i++)
    {
        CHECKED_MALLOC(m_rqt[i].coeffRQT[0], coeff_t, sizeL + sizeC * 2);
        m_rqt[i].coeffRQT[1] = m_rqt[i].coeffRQT[0] + sizeL;
        m_rqt[i].coeffRQT[2] = m_rqt[i].coeffRQT[0] + sizeL + sizeC;
        ok &= m_rqt[i].reconQtYuv.create(g_maxCUSize, param.internalCsp);
        ok &= m_rqt[i].resiQtYuv.create(g_maxCUSize, param.internalCsp);
    }

    /* the rest of these buffers are indexed per-depth */
    for (uint32_t i = 0; i <= g_maxCUDepth; i++)
    {
        int cuSize = g_maxCUSize >> i;
        ok &= m_rqt[i].tmpResiYuv.create(cuSize, param.internalCsp);
        ok &= m_rqt[i].tmpPredYuv.create(cuSize, param.internalCsp);
        ok &= m_rqt[i].bidirPredYuv[0].create(cuSize, param.internalCsp);
        ok &= m_rqt[i].bidirPredYuv[1].create(cuSize, param.internalCsp);
    }

    CHECKED_MALLOC(m_qtTempCbf[0], uint8_t, numPartitions * 3);
    m_qtTempCbf[1] = m_qtTempCbf[0] + numPartitions;
    m_qtTempCbf[2] = m_qtTempCbf[0] + numPartitions * 2;
    CHECKED_MALLOC(m_qtTempTransformSkipFlag[0], uint8_t, numPartitions * 3);
    m_qtTempTransformSkipFlag[1] = m_qtTempTransformSkipFlag[0] + numPartitions;
    m_qtTempTransformSkipFlag[2] = m_qtTempTransformSkipFlag[0] + numPartitions * 2;

    CHECKED_MALLOC(m_intraPred, pixel, (32 * 32) * (33 + 3));
    m_fencScaled = m_intraPred + 32 * 32;
    m_fencTransposed = m_fencScaled + 32 * 32;
    m_intraPredAngs = m_fencTransposed + 32 * 32;

    CHECKED_MALLOC(m_tsCoeff,    coeff_t, MAX_TS_SIZE * MAX_TS_SIZE);
    CHECKED_MALLOC(m_tsResidual, int16_t, MAX_TS_SIZE * MAX_TS_SIZE);
    CHECKED_MALLOC(m_tsRecon,    pixel,   MAX_TS_SIZE * MAX_TS_SIZE);

    return ok;

fail:
    return false;
}

Search::~Search()
{
    for (uint32_t i = 0; i <= m_numLayers; i++)
    {
        X265_FREE(m_rqt[i].coeffRQT[0]);
        m_rqt[i].reconQtYuv.destroy();
        m_rqt[i].resiQtYuv.destroy();
    }

    for (uint32_t i = 0; i <= g_maxCUDepth; i++)
    {
        m_rqt[i].tmpResiYuv.destroy();
        m_rqt[i].tmpPredYuv.destroy();
        m_rqt[i].bidirPredYuv[0].destroy();
        m_rqt[i].bidirPredYuv[1].destroy();
    }

    X265_FREE(m_qtTempCbf[0]);
    X265_FREE(m_qtTempTransformSkipFlag[0]);
    X265_FREE(m_intraPred);
    X265_FREE(m_tsCoeff);
    X265_FREE(m_tsResidual);
    X265_FREE(m_tsRecon);
}

int Search::setLambdaFromQP(const CUData& ctu, int qp)
{
    X265_CHECK(qp >= QP_MIN && qp <= QP_MAX_MAX, "QP used for lambda is out of range\n");

    m_me.setQP(qp);
    m_rdCost.setQP(*m_slice, qp);

    int quantQP = x265_clip3(QP_MIN, QP_MAX_SPEC, qp);
    m_quant.setQPforQuant(ctu, quantQP);
    return quantQP;
}

#if CHECKED_BUILD || _DEBUG
void Search::invalidateContexts(int fromDepth)
{
    /* catch reads without previous writes */
    for (int d = fromDepth; d < NUM_FULL_DEPTH; d++)
    {
        m_rqt[d].cur.markInvalid();
        m_rqt[d].rqtTemp.markInvalid();
        m_rqt[d].rqtRoot.markInvalid();
        m_rqt[d].rqtTest.markInvalid();
    }
}
#else
void Search::invalidateContexts(int) {}
#endif

void Search::codeSubdivCbfQTChroma(const CUData& cu, uint32_t tuDepth, uint32_t absPartIdx)
{
    uint32_t subdiv     = tuDepth < cu.m_tuDepth[absPartIdx];
    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;

    if (!(log2TrSize - m_hChromaShift < 2))
    {
        if (!tuDepth || cu.getCbf(absPartIdx, TEXT_CHROMA_U, tuDepth - 1))
            m_entropyCoder.codeQtCbfChroma(cu, absPartIdx, TEXT_CHROMA_U, tuDepth, !subdiv);
        if (!tuDepth || cu.getCbf(absPartIdx, TEXT_CHROMA_V, tuDepth - 1))
            m_entropyCoder.codeQtCbfChroma(cu, absPartIdx, TEXT_CHROMA_V, tuDepth, !subdiv);
    }

    if (subdiv)
    {
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;
        for (uint32_t qIdx = 0; qIdx < 4; ++qIdx, absPartIdx += qNumParts)
            codeSubdivCbfQTChroma(cu, tuDepth + 1, absPartIdx);
    }
}

void Search::codeCoeffQTChroma(const CUData& cu, uint32_t tuDepth, uint32_t absPartIdx, TextType ttype)
{
    if (!cu.getCbf(absPartIdx, ttype, tuDepth))
        return;

    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;

    if (tuDepth < cu.m_tuDepth[absPartIdx])
    {
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;
        for (uint32_t qIdx = 0; qIdx < 4; ++qIdx, absPartIdx += qNumParts)
            codeCoeffQTChroma(cu, tuDepth + 1, absPartIdx, ttype);

        return;
    }

    uint32_t tuDepthC = tuDepth;
    uint32_t log2TrSizeC = log2TrSize - m_hChromaShift;

    if (log2TrSizeC < 2)
    {
        X265_CHECK(log2TrSize == 2 && m_csp != X265_CSP_I444 && tuDepth, "invalid tuDepth\n");
        if (absPartIdx & 3)
            return;
        log2TrSizeC = 2;
        tuDepthC--;
    }

    uint32_t qtLayer = log2TrSize - 2;

    if (m_csp != X265_CSP_I422)
    {
        uint32_t shift = (m_csp == X265_CSP_I420) ? 2 : 0;
        uint32_t coeffOffset = absPartIdx << (LOG2_UNIT_SIZE * 2 - shift);
        coeff_t* coeff = m_rqt[qtLayer].coeffRQT[ttype] + coeffOffset;
        m_entropyCoder.codeCoeffNxN(cu, coeff, absPartIdx, log2TrSizeC, ttype);
    }
    else
    {
        uint32_t coeffOffset = absPartIdx << (LOG2_UNIT_SIZE * 2 - 1);
        coeff_t* coeff = m_rqt[qtLayer].coeffRQT[ttype] + coeffOffset;
        uint32_t subTUSize = 1 << (log2TrSizeC * 2);
        uint32_t tuNumParts = 2 << ((log2TrSizeC - LOG2_UNIT_SIZE) * 2);
        if (cu.getCbf(absPartIdx, ttype, tuDepth + 1))
            m_entropyCoder.codeCoeffNxN(cu, coeff, absPartIdx, log2TrSizeC, ttype);
        if (cu.getCbf(absPartIdx + tuNumParts, ttype, tuDepth + 1))
            m_entropyCoder.codeCoeffNxN(cu, coeff + subTUSize, absPartIdx + tuNumParts, log2TrSizeC, ttype);
    }
}

void Search::codeIntraLumaQT(Mode& mode, const CUGeom& cuGeom, uint32_t tuDepth, uint32_t absPartIdx, bool bAllowSplit, Cost& outCost, const uint32_t depthRange[2])
{
    CUData& cu = mode.cu;
    uint32_t fullDepth  = cuGeom.depth + tuDepth;
    uint32_t log2TrSize = cuGeom.log2CUSize - tuDepth;
    uint32_t qtLayer    = log2TrSize - 2;
    uint32_t sizeIdx    = log2TrSize - 2;
    bool mightNotSplit  = log2TrSize <= depthRange[1];
    bool mightSplit     = (log2TrSize > depthRange[0]) && (bAllowSplit || !mightNotSplit);

    /* If maximum RD penalty, force spits at TU size 32x32 if SPS allows TUs of 16x16 */
    if (m_param->rdPenalty == 2 && m_slice->m_sliceType != I_SLICE && log2TrSize == 5 && depthRange[0] <= 4)
    {
        mightNotSplit = false;
        mightSplit = true;
    }

    Cost fullCost;
    uint32_t bCBF = 0;

    pixel*   reconQt = m_rqt[qtLayer].reconQtYuv.getLumaAddr(absPartIdx);
    uint32_t reconQtStride = m_rqt[qtLayer].reconQtYuv.m_size;

    if (mightNotSplit)
    {
        if (mightSplit)
            m_entropyCoder.store(m_rqt[fullDepth].rqtRoot);

        const pixel* fenc = mode.fencYuv->getLumaAddr(absPartIdx);
        pixel*   pred     = mode.predYuv.getLumaAddr(absPartIdx);
        int16_t* residual = m_rqt[cuGeom.depth].tmpResiYuv.getLumaAddr(absPartIdx);
        uint32_t stride   = mode.fencYuv->m_size;

        // init availability pattern
        uint32_t lumaPredMode = cu.m_lumaIntraDir[absPartIdx];
        IntraNeighbors intraNeighbors;
        initIntraNeighbors(cu, absPartIdx, tuDepth, true, &intraNeighbors);
        initAdiPattern(cu, cuGeom, absPartIdx, intraNeighbors, lumaPredMode);

        // get prediction signal
        predIntraLumaAng(lumaPredMode, pred, stride, log2TrSize);

        cu.setTransformSkipSubParts(0, TEXT_LUMA, absPartIdx, fullDepth);
        cu.setTUDepthSubParts(tuDepth, absPartIdx, fullDepth);

        uint32_t coeffOffsetY = absPartIdx << (LOG2_UNIT_SIZE * 2);
        coeff_t* coeffY       = m_rqt[qtLayer].coeffRQT[0] + coeffOffsetY;

        // store original entropy coding status
        if (m_bEnableRDOQ)
            m_entropyCoder.estBit(m_entropyCoder.m_estBitsSbac, log2TrSize, true);

        primitives.cu[sizeIdx].calcresidual(fenc, pred, residual, stride);

        uint32_t numSig = m_quant.transformNxN(cu, fenc, stride, residual, stride, coeffY, log2TrSize, TEXT_LUMA, absPartIdx, false);
        if (numSig)
        {
            m_quant.invtransformNxN(cu, residual, stride, coeffY, log2TrSize, TEXT_LUMA, true, false, numSig);
            primitives.cu[sizeIdx].add_ps(reconQt, reconQtStride, pred, residual, stride, stride);
        }
        else
            // no coded residual, recon = pred
            primitives.cu[sizeIdx].copy_pp(reconQt, reconQtStride, pred, stride);

        bCBF = !!numSig << tuDepth;
        cu.setCbfSubParts(bCBF, TEXT_LUMA, absPartIdx, fullDepth);
        fullCost.distortion = primitives.cu[sizeIdx].sse_pp(reconQt, reconQtStride, fenc, stride);

        m_entropyCoder.resetBits();
        if (!absPartIdx)
        {
            if (!cu.m_slice->isIntra())
            {
                if (cu.m_slice->m_pps->bTransquantBypassEnabled)
                    m_entropyCoder.codeCUTransquantBypassFlag(cu.m_tqBypass[0]);
                m_entropyCoder.codeSkipFlag(cu, 0);
                m_entropyCoder.codePredMode(cu.m_predMode[0]);
            }

            m_entropyCoder.codePartSize(cu, 0, cuGeom.depth);
        }
        if (cu.m_partSize[0] == SIZE_2Nx2N)
        {
            if (!absPartIdx)
                m_entropyCoder.codeIntraDirLumaAng(cu, 0, false);
        }
        else
        {
            uint32_t qNumParts = cuGeom.numPartitions >> 2;
            if (!tuDepth)
            {
                for (uint32_t qIdx = 0; qIdx < 4; ++qIdx)
                    m_entropyCoder.codeIntraDirLumaAng(cu, qIdx * qNumParts, false);
            }
            else if (!(absPartIdx & (qNumParts - 1)))
                m_entropyCoder.codeIntraDirLumaAng(cu, absPartIdx, false);
        }
        if (log2TrSize != depthRange[0])
            m_entropyCoder.codeTransformSubdivFlag(0, 5 - log2TrSize);

        m_entropyCoder.codeQtCbfLuma(!!numSig, tuDepth);

        if (cu.getCbf(absPartIdx, TEXT_LUMA, tuDepth))
            m_entropyCoder.codeCoeffNxN(cu, coeffY, absPartIdx, log2TrSize, TEXT_LUMA);

        fullCost.bits = m_entropyCoder.getNumberOfWrittenBits();

        if (m_param->rdPenalty && log2TrSize == 5 && m_slice->m_sliceType != I_SLICE)
            fullCost.bits *= 4;

        if (m_rdCost.m_psyRd)
        {
            fullCost.energy = m_rdCost.psyCost(sizeIdx, fenc, mode.fencYuv->m_size, reconQt, reconQtStride);
            fullCost.rdcost = m_rdCost.calcPsyRdCost(fullCost.distortion, fullCost.bits, fullCost.energy);
        }
        else
            fullCost.rdcost = m_rdCost.calcRdCost(fullCost.distortion, fullCost.bits);
    }
    else
        fullCost.rdcost = MAX_INT64;

    if (mightSplit)
    {
        if (mightNotSplit)
        {
            m_entropyCoder.store(m_rqt[fullDepth].rqtTest);  // save state after full TU encode
            m_entropyCoder.load(m_rqt[fullDepth].rqtRoot);   // prep state of split encode
        }

        /* code split block */
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;

        int checkTransformSkip = m_slice->m_pps->bTransformSkipEnabled && (log2TrSize - 1) <= MAX_LOG2_TS_SIZE && !cu.m_tqBypass[0];
        if (m_param->bEnableTSkipFast)
            checkTransformSkip &= cu.m_partSize[0] != SIZE_2Nx2N;

        Cost splitCost;
        uint32_t cbf = 0;
        for (uint32_t qIdx = 0, qPartIdx = absPartIdx; qIdx < 4; ++qIdx, qPartIdx += qNumParts)
        {
            if (checkTransformSkip)
                codeIntraLumaTSkip(mode, cuGeom, tuDepth + 1, qPartIdx, splitCost);
            else
                codeIntraLumaQT(mode, cuGeom, tuDepth + 1, qPartIdx, bAllowSplit, splitCost, depthRange);

            cbf |= cu.getCbf(qPartIdx, TEXT_LUMA, tuDepth + 1);
        }
        for (uint32_t offs = 0; offs < 4 * qNumParts; offs++)
            cu.m_cbf[0][absPartIdx + offs] |= (cbf << tuDepth);

        if (mightNotSplit && log2TrSize != depthRange[0])
        {
            /* If we could have coded this TU depth, include cost of subdiv flag */
            m_entropyCoder.resetBits();
            m_entropyCoder.codeTransformSubdivFlag(1, 5 - log2TrSize);
            splitCost.bits += m_entropyCoder.getNumberOfWrittenBits();

            if (m_rdCost.m_psyRd)
                splitCost.rdcost = m_rdCost.calcPsyRdCost(splitCost.distortion, splitCost.bits, splitCost.energy);
            else
                splitCost.rdcost = m_rdCost.calcRdCost(splitCost.distortion, splitCost.bits);
        }

        if (splitCost.rdcost < fullCost.rdcost)
        {
            outCost.rdcost     += splitCost.rdcost;
            outCost.distortion += splitCost.distortion;
            outCost.bits       += splitCost.bits;
            outCost.energy     += splitCost.energy;
            return;
        }
        else
        {
            // recover entropy state of full-size TU encode
            m_entropyCoder.load(m_rqt[fullDepth].rqtTest);

            // recover transform index and Cbf values
            cu.setTUDepthSubParts(tuDepth, absPartIdx, fullDepth);
            cu.setCbfSubParts(bCBF, TEXT_LUMA, absPartIdx, fullDepth);
            cu.setTransformSkipSubParts(0, TEXT_LUMA, absPartIdx, fullDepth);
        }
    }

    // set reconstruction for next intra prediction blocks if full TU prediction won
    pixel*   picReconY = m_frame->m_reconPic->getLumaAddr(cu.m_cuAddr, cuGeom.absPartIdx + absPartIdx);
    intptr_t picStride = m_frame->m_reconPic->m_stride;
    primitives.cu[sizeIdx].copy_pp(picReconY, picStride, reconQt, reconQtStride);

    outCost.rdcost     += fullCost.rdcost;
    outCost.distortion += fullCost.distortion;
    outCost.bits       += fullCost.bits;
    outCost.energy     += fullCost.energy;
}

void Search::codeIntraLumaTSkip(Mode& mode, const CUGeom& cuGeom, uint32_t tuDepth, uint32_t absPartIdx, Cost& outCost)
{
    uint32_t fullDepth = cuGeom.depth + tuDepth;
    uint32_t log2TrSize = cuGeom.log2CUSize - tuDepth;
    uint32_t tuSize = 1 << log2TrSize;

    X265_CHECK(tuSize <= MAX_TS_SIZE, "transform skip is only possible at 4x4 TUs\n");

    CUData& cu = mode.cu;
    Yuv* predYuv = &mode.predYuv;
    const Yuv* fencYuv = mode.fencYuv;

    Cost fullCost;
    fullCost.rdcost = MAX_INT64;
    int      bTSkip = 0;
    uint32_t bCBF = 0;

    const pixel* fenc = fencYuv->getLumaAddr(absPartIdx);
    pixel*   pred = predYuv->getLumaAddr(absPartIdx);
    int16_t* residual = m_rqt[cuGeom.depth].tmpResiYuv.getLumaAddr(absPartIdx);
    uint32_t stride = fencYuv->m_size;
    uint32_t sizeIdx = log2TrSize - 2;

    // init availability pattern
    uint32_t lumaPredMode = cu.m_lumaIntraDir[absPartIdx];
    IntraNeighbors intraNeighbors;
    initIntraNeighbors(cu, absPartIdx, tuDepth, true, &intraNeighbors);
    initAdiPattern(cu, cuGeom, absPartIdx, intraNeighbors, lumaPredMode);

    // get prediction signal
    predIntraLumaAng(lumaPredMode, pred, stride, log2TrSize);

    cu.setTUDepthSubParts(tuDepth, absPartIdx, fullDepth);

    uint32_t qtLayer = log2TrSize - 2;
    uint32_t coeffOffsetY = absPartIdx << (LOG2_UNIT_SIZE * 2);
    coeff_t* coeffY = m_rqt[qtLayer].coeffRQT[0] + coeffOffsetY;
    pixel*   reconQt = m_rqt[qtLayer].reconQtYuv.getLumaAddr(absPartIdx);
    uint32_t reconQtStride = m_rqt[qtLayer].reconQtYuv.m_size;

    // store original entropy coding status
    m_entropyCoder.store(m_rqt[fullDepth].rqtRoot);

    if (m_bEnableRDOQ)
        m_entropyCoder.estBit(m_entropyCoder.m_estBitsSbac, log2TrSize, true);

    int checkTransformSkip = 1;
    for (int useTSkip = 0; useTSkip <= checkTransformSkip; useTSkip++)
    {
        uint64_t tmpCost;
        uint32_t tmpEnergy = 0;

        coeff_t* coeff = (useTSkip ? m_tsCoeff : coeffY);
        pixel*   tmpRecon = (useTSkip ? m_tsRecon : reconQt);
        uint32_t tmpReconStride = (useTSkip ? MAX_TS_SIZE : reconQtStride);

        primitives.cu[sizeIdx].calcresidual(fenc, pred, residual, stride);

        uint32_t numSig = m_quant.transformNxN(cu, fenc, stride, residual, stride, coeff, log2TrSize, TEXT_LUMA, absPartIdx, useTSkip);
        if (numSig)
        {
            m_quant.invtransformNxN(cu, residual, stride, coeff, log2TrSize, TEXT_LUMA, true, useTSkip, numSig);
            primitives.cu[sizeIdx].add_ps(tmpRecon, tmpReconStride, pred, residual, stride, stride);
        }
        else if (useTSkip)
        {
            /* do not allow tskip if CBF=0, pretend we did not try tskip */
            checkTransformSkip = 0;
            break;
        }
        else
            // no residual coded, recon = pred
            primitives.cu[sizeIdx].copy_pp(tmpRecon, tmpReconStride, pred, stride);

        sse_ret_t tmpDist = primitives.cu[sizeIdx].sse_pp(tmpRecon, tmpReconStride, fenc, stride);

        cu.setTransformSkipSubParts(useTSkip, TEXT_LUMA, absPartIdx, fullDepth);
        cu.setCbfSubParts((!!numSig) << tuDepth, TEXT_LUMA, absPartIdx, fullDepth);

        if (useTSkip)
            m_entropyCoder.load(m_rqt[fullDepth].rqtRoot);

        m_entropyCoder.resetBits();
        if (!absPartIdx)
        {
            if (!cu.m_slice->isIntra())
            {
                if (cu.m_slice->m_pps->bTransquantBypassEnabled)
                    m_entropyCoder.codeCUTransquantBypassFlag(cu.m_tqBypass[0]);
                m_entropyCoder.codeSkipFlag(cu, 0);
                m_entropyCoder.codePredMode(cu.m_predMode[0]);
            }

            m_entropyCoder.codePartSize(cu, 0, cuGeom.depth);
        }
        if (cu.m_partSize[0] == SIZE_2Nx2N)
        {
            if (!absPartIdx)
                m_entropyCoder.codeIntraDirLumaAng(cu, 0, false);
        }
        else
        {
            uint32_t qNumParts = cuGeom.numPartitions >> 2;
            if (!tuDepth)
            {
                for (uint32_t qIdx = 0; qIdx < 4; ++qIdx)
                    m_entropyCoder.codeIntraDirLumaAng(cu, qIdx * qNumParts, false);
            }
            else if (!(absPartIdx & (qNumParts - 1)))
                m_entropyCoder.codeIntraDirLumaAng(cu, absPartIdx, false);
        }
        m_entropyCoder.codeTransformSubdivFlag(0, 5 - log2TrSize);

        m_entropyCoder.codeQtCbfLuma(!!numSig, tuDepth);

        if (cu.getCbf(absPartIdx, TEXT_LUMA, tuDepth))
            m_entropyCoder.codeCoeffNxN(cu, coeff, absPartIdx, log2TrSize, TEXT_LUMA);

        uint32_t tmpBits = m_entropyCoder.getNumberOfWrittenBits();

        if (!useTSkip)
            m_entropyCoder.store(m_rqt[fullDepth].rqtTemp);

        if (m_rdCost.m_psyRd)
        {
            tmpEnergy = m_rdCost.psyCost(sizeIdx, fenc, fencYuv->m_size, tmpRecon, tmpReconStride);
            tmpCost = m_rdCost.calcPsyRdCost(tmpDist, tmpBits, tmpEnergy);
        }
        else
            tmpCost = m_rdCost.calcRdCost(tmpDist, tmpBits);

        if (tmpCost < fullCost.rdcost)
        {
            bTSkip = useTSkip;
            bCBF = !!numSig;
            fullCost.rdcost = tmpCost;
            fullCost.distortion = tmpDist;
            fullCost.bits = tmpBits;
            fullCost.energy = tmpEnergy;
        }
    }

    if (bTSkip)
    {
        memcpy(coeffY, m_tsCoeff, sizeof(coeff_t) << (log2TrSize * 2));
        primitives.cu[sizeIdx].copy_pp(reconQt, reconQtStride, m_tsRecon, tuSize);
    }
    else if (checkTransformSkip)
    {
        cu.setTransformSkipSubParts(0, TEXT_LUMA, absPartIdx, fullDepth);
        cu.setCbfSubParts(bCBF << tuDepth, TEXT_LUMA, absPartIdx, fullDepth);
        m_entropyCoder.load(m_rqt[fullDepth].rqtTemp);
    }

    // set reconstruction for next intra prediction blocks
    pixel*   picReconY = m_frame->m_reconPic->getLumaAddr(cu.m_cuAddr, cuGeom.absPartIdx + absPartIdx);
    intptr_t picStride = m_frame->m_reconPic->m_stride;
    primitives.cu[sizeIdx].copy_pp(picReconY, picStride, reconQt, reconQtStride);

    outCost.rdcost += fullCost.rdcost;
    outCost.distortion += fullCost.distortion;
    outCost.bits += fullCost.bits;
    outCost.energy += fullCost.energy;
}
void Search::extractIntraResultQT(CUData& cu, Yuv& reconYuv, uint32_t tuDepth, uint32_t absPartIdx)
{
    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;

    if (tuDepth == cu.m_tuDepth[absPartIdx])
    {
        uint32_t qtLayer    = log2TrSize - 2;

        // copy transform coefficients
        uint32_t coeffOffsetY = absPartIdx << (LOG2_UNIT_SIZE * 2);
        coeff_t* coeffSrcY    = m_rqt[qtLayer].coeffRQT[0] + coeffOffsetY;
        coeff_t* coeffDestY   = cu.m_trCoeff[0]            + coeffOffsetY;
        memcpy(coeffDestY, coeffSrcY, sizeof(coeff_t) << (log2TrSize * 2));

        // copy reconstruction
        m_rqt[qtLayer].reconQtYuv.copyPartToPartLuma(reconYuv, absPartIdx, log2TrSize);
    }
    else
    {
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;
        for (uint32_t qIdx = 0; qIdx < 4; ++qIdx, absPartIdx += qNumParts)
            extractIntraResultQT(cu, reconYuv, tuDepth + 1, absPartIdx);
    }
}

inline void offsetCBFs(uint8_t subTUCBF[2])
{
    uint8_t combinedCBF = subTUCBF[0] | subTUCBF[1];
    subTUCBF[0] = subTUCBF[0] << 1 | combinedCBF;
    subTUCBF[1] = subTUCBF[1] << 1 | combinedCBF;
}

/* 4:2:2 post-TU split processing */
void Search::offsetSubTUCBFs(CUData& cu, TextType ttype, uint32_t tuDepth, uint32_t absPartIdx)
{
    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;

    if (log2TrSize == 2)
    {
        X265_CHECK(m_csp != X265_CSP_I444 && tuDepth, "invalid tuDepth\n");
        ++log2TrSize;
    }

    uint32_t tuNumParts = 1 << ((log2TrSize - LOG2_UNIT_SIZE) * 2 - 1);

    // move the CBFs down a level and set the parent CBF
    uint8_t subTUCBF[2];
    subTUCBF[0] = cu.getCbf(absPartIdx            , ttype, tuDepth);
    subTUCBF[1] = cu.getCbf(absPartIdx+ tuNumParts, ttype, tuDepth);
    offsetCBFs(subTUCBF);

    cu.setCbfPartRange(subTUCBF[0] << tuDepth, ttype, absPartIdx             , tuNumParts);
    cu.setCbfPartRange(subTUCBF[1] << tuDepth, ttype, absPartIdx + tuNumParts, tuNumParts);
}

/* returns distortion */
uint32_t Search::codeIntraChromaQt(Mode& mode, const CUGeom& cuGeom, uint32_t tuDepth, uint32_t absPartIdx, uint32_t& psyEnergy)
{
    CUData& cu = mode.cu;
    uint32_t log2TrSize = cuGeom.log2CUSize - tuDepth;

    if (tuDepth < cu.m_tuDepth[absPartIdx])
    {
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;
        uint32_t outDist = 0, splitCbfU = 0, splitCbfV = 0;
        for (uint32_t qIdx = 0, qPartIdx = absPartIdx; qIdx < 4; ++qIdx, qPartIdx += qNumParts)
        {
            outDist += codeIntraChromaQt(mode, cuGeom, tuDepth + 1, qPartIdx, psyEnergy);
            splitCbfU |= cu.getCbf(qPartIdx, TEXT_CHROMA_U, tuDepth + 1);
            splitCbfV |= cu.getCbf(qPartIdx, TEXT_CHROMA_V, tuDepth + 1);
        }
        for (uint32_t offs = 0; offs < 4 * qNumParts; offs++)
        {
            cu.m_cbf[1][absPartIdx + offs] |= (splitCbfU << tuDepth);
            cu.m_cbf[2][absPartIdx + offs] |= (splitCbfV << tuDepth);
        }

        return outDist;
    }

    uint32_t log2TrSizeC = log2TrSize - m_hChromaShift;
    uint32_t tuDepthC = tuDepth;
    if (log2TrSizeC < 2)
    {
        X265_CHECK(log2TrSize == 2 && m_csp != X265_CSP_I444 && tuDepth, "invalid tuDepth\n");
        if (absPartIdx & 3)
            return 0;
        log2TrSizeC = 2;
        tuDepthC--;
    }

    if (m_bEnableRDOQ)
        m_entropyCoder.estBit(m_entropyCoder.m_estBitsSbac, log2TrSizeC, false);

    bool checkTransformSkip = m_slice->m_pps->bTransformSkipEnabled && log2TrSizeC <= MAX_LOG2_TS_SIZE && !cu.m_tqBypass[0];
    checkTransformSkip &= !m_param->bEnableTSkipFast || (log2TrSize <= MAX_LOG2_TS_SIZE && cu.m_transformSkip[TEXT_LUMA][absPartIdx]);
    if (checkTransformSkip)
        return codeIntraChromaTSkip(mode, cuGeom, tuDepth, tuDepthC, absPartIdx, psyEnergy);

    ShortYuv& resiYuv = m_rqt[cuGeom.depth].tmpResiYuv;
    uint32_t qtLayer = log2TrSize - 2;
    uint32_t stride = mode.fencYuv->m_csize;
    const uint32_t sizeIdxC = log2TrSizeC - 2;
    sse_ret_t outDist = 0;

    uint32_t curPartNum = cuGeom.numPartitions >> tuDepthC * 2;
    const SplitType splitType = (m_csp == X265_CSP_I422) ? VERTICAL_SPLIT : DONT_SPLIT;

    TURecurse tuIterator(splitType, curPartNum, absPartIdx);
    do
    {
        uint32_t absPartIdxC = tuIterator.absPartIdxTURelCU;

        IntraNeighbors intraNeighbors;
        initIntraNeighbors(cu, absPartIdxC, tuDepthC, false, &intraNeighbors);

        for (uint32_t chromaId = TEXT_CHROMA_U; chromaId <= TEXT_CHROMA_V; chromaId++)
        {
            TextType ttype = (TextType)chromaId;

            const pixel* fenc = mode.fencYuv->getChromaAddr(chromaId, absPartIdxC);
            pixel*   pred     = mode.predYuv.getChromaAddr(chromaId, absPartIdxC);
            int16_t* residual = resiYuv.getChromaAddr(chromaId, absPartIdxC);
            uint32_t coeffOffsetC  = absPartIdxC << (LOG2_UNIT_SIZE * 2 - (m_hChromaShift + m_vChromaShift));
            coeff_t* coeffC        = m_rqt[qtLayer].coeffRQT[chromaId] + coeffOffsetC;
            pixel*   reconQt       = m_rqt[qtLayer].reconQtYuv.getChromaAddr(chromaId, absPartIdxC);
            uint32_t reconQtStride = m_rqt[qtLayer].reconQtYuv.m_csize;
            pixel*   picReconC = m_frame->m_reconPic->getChromaAddr(chromaId, cu.m_cuAddr, cuGeom.absPartIdx + absPartIdxC);
            intptr_t picStride = m_frame->m_reconPic->m_strideC;

            uint32_t chromaPredMode = cu.m_chromaIntraDir[absPartIdxC];
            if (chromaPredMode == DM_CHROMA_IDX)
                chromaPredMode = cu.m_lumaIntraDir[(m_csp == X265_CSP_I444) ? absPartIdxC : 0];
            if (m_csp == X265_CSP_I422)
                chromaPredMode = g_chroma422IntraAngleMappingTable[chromaPredMode];

            // init availability pattern
            initAdiPatternChroma(cu, cuGeom, absPartIdxC, intraNeighbors, chromaId);

            // get prediction signal
            predIntraChromaAng(chromaPredMode, pred, stride, log2TrSizeC);
            cu.setTransformSkipPartRange(0, ttype, absPartIdxC, tuIterator.absPartIdxStep);

            primitives.cu[sizeIdxC].calcresidual(fenc, pred, residual, stride);
            uint32_t numSig = m_quant.transformNxN(cu, fenc, stride, residual, stride, coeffC, log2TrSizeC, ttype, absPartIdxC, false);
            if (numSig)
            {
                m_quant.invtransformNxN(cu, residual, stride, coeffC, log2TrSizeC, ttype, true, false, numSig);
                primitives.cu[sizeIdxC].add_ps(reconQt, reconQtStride, pred, residual, stride, stride);
                cu.setCbfPartRange(1 << tuDepth, ttype, absPartIdxC, tuIterator.absPartIdxStep);
            }
            else
            {
                // no coded residual, recon = pred
                primitives.cu[sizeIdxC].copy_pp(reconQt, reconQtStride, pred, stride);
                cu.setCbfPartRange(0, ttype, absPartIdxC, tuIterator.absPartIdxStep);
            }

            outDist += m_rdCost.scaleChromaDist(chromaId, primitives.cu[sizeIdxC].sse_pp(reconQt, reconQtStride, fenc, stride));

            if (m_rdCost.m_psyRd)
                psyEnergy += m_rdCost.psyCost(sizeIdxC, fenc, stride, reconQt, reconQtStride);

            primitives.cu[sizeIdxC].copy_pp(picReconC, picStride, reconQt, reconQtStride);
        }
    }
    while (tuIterator.isNextSection());

    if (splitType == VERTICAL_SPLIT)
    {
        offsetSubTUCBFs(cu, TEXT_CHROMA_U, tuDepth, absPartIdx);
        offsetSubTUCBFs(cu, TEXT_CHROMA_V, tuDepth, absPartIdx);
    }

    return outDist;
}

/* returns distortion */
uint32_t Search::codeIntraChromaTSkip(Mode& mode, const CUGeom& cuGeom, uint32_t tuDepth, uint32_t tuDepthC, uint32_t absPartIdx, uint32_t& psyEnergy)
{
    CUData& cu = mode.cu;
    uint32_t fullDepth  = cuGeom.depth + tuDepth;
    uint32_t log2TrSize = cuGeom.log2CUSize - tuDepth;
    const uint32_t log2TrSizeC = 2;
    uint32_t qtLayer = log2TrSize - 2;
    uint32_t outDist = 0;

    /* At the TU layers above this one, no RDO is performed, only distortion is being measured,
     * so the entropy coder is not very accurate. The best we can do is return it in the same
     * condition as it arrived, and to do all bit estimates from the same state. */
    m_entropyCoder.store(m_rqt[fullDepth].rqtRoot);

    uint32_t curPartNum = cuGeom.numPartitions >> tuDepthC * 2;
    const SplitType splitType = (m_csp == X265_CSP_I422) ? VERTICAL_SPLIT : DONT_SPLIT;

    TURecurse tuIterator(splitType, curPartNum, absPartIdx);
    do
    {
        uint32_t absPartIdxC = tuIterator.absPartIdxTURelCU;

        IntraNeighbors intraNeighbors;
        initIntraNeighbors(cu, absPartIdxC, tuDepthC, false, &intraNeighbors);

        for (uint32_t chromaId = TEXT_CHROMA_U; chromaId <= TEXT_CHROMA_V; chromaId++)
        {
            TextType ttype = (TextType)chromaId;

            const pixel* fenc = mode.fencYuv->getChromaAddr(chromaId, absPartIdxC);
            pixel*   pred = mode.predYuv.getChromaAddr(chromaId, absPartIdxC);
            int16_t* residual = m_rqt[cuGeom.depth].tmpResiYuv.getChromaAddr(chromaId, absPartIdxC);
            uint32_t stride = mode.fencYuv->m_csize;
            const uint32_t sizeIdxC = log2TrSizeC - 2;

            uint32_t coeffOffsetC = absPartIdxC << (LOG2_UNIT_SIZE * 2 - (m_hChromaShift + m_vChromaShift));
            coeff_t* coeffC = m_rqt[qtLayer].coeffRQT[chromaId] + coeffOffsetC;
            pixel*   reconQt = m_rqt[qtLayer].reconQtYuv.getChromaAddr(chromaId, absPartIdxC);
            uint32_t reconQtStride = m_rqt[qtLayer].reconQtYuv.m_csize;

            // init availability pattern
            initAdiPatternChroma(cu, cuGeom, absPartIdxC, intraNeighbors, chromaId);

            uint32_t chromaPredMode = cu.m_chromaIntraDir[absPartIdxC];
            if (chromaPredMode == DM_CHROMA_IDX)
                chromaPredMode = cu.m_lumaIntraDir[(m_csp == X265_CSP_I444) ? absPartIdxC : 0];
            if (m_csp == X265_CSP_I422)
                chromaPredMode = g_chroma422IntraAngleMappingTable[chromaPredMode];

            // get prediction signal
            predIntraChromaAng(chromaPredMode, pred, stride, log2TrSizeC);

            uint64_t bCost = MAX_INT64;
            uint32_t bDist = 0;
            uint32_t bCbf = 0;
            uint32_t bEnergy = 0;
            int      bTSkip = 0;

            int checkTransformSkip = 1;
            for (int useTSkip = 0; useTSkip <= checkTransformSkip; useTSkip++)
            {
                coeff_t* coeff = (useTSkip ? m_tsCoeff : coeffC);
                pixel*   recon = (useTSkip ? m_tsRecon : reconQt);
                uint32_t reconStride = (useTSkip ? MAX_TS_SIZE : reconQtStride);

                primitives.cu[sizeIdxC].calcresidual(fenc, pred, residual, stride);

                uint32_t numSig = m_quant.transformNxN(cu, fenc, stride, residual, stride, coeff, log2TrSizeC, ttype, absPartIdxC, useTSkip);
                if (numSig)
                {
                    m_quant.invtransformNxN(cu, residual, stride, coeff, log2TrSizeC, ttype, true, useTSkip, numSig);
                    primitives.cu[sizeIdxC].add_ps(recon, reconStride, pred, residual, stride, stride);
                    cu.setCbfPartRange(1 << tuDepth, ttype, absPartIdxC, tuIterator.absPartIdxStep);
                }
                else if (useTSkip)
                {
                    checkTransformSkip = 0;
                    break;
                }
                else
                {
                    primitives.cu[sizeIdxC].copy_pp(recon, reconStride, pred, stride);
                    cu.setCbfPartRange(0, ttype, absPartIdxC, tuIterator.absPartIdxStep);
                }
                sse_ret_t tmpDist = primitives.cu[sizeIdxC].sse_pp(recon, reconStride, fenc, stride);
                tmpDist = m_rdCost.scaleChromaDist(chromaId, tmpDist);

                cu.setTransformSkipPartRange(useTSkip, ttype, absPartIdxC, tuIterator.absPartIdxStep);

                uint32_t tmpBits = 0, tmpEnergy = 0;
                if (numSig)
                {
                    m_entropyCoder.load(m_rqt[fullDepth].rqtRoot);
                    m_entropyCoder.resetBits();
                    m_entropyCoder.codeCoeffNxN(cu, coeff, absPartIdxC, log2TrSizeC, (TextType)chromaId);
                    tmpBits = m_entropyCoder.getNumberOfWrittenBits();
                }

                uint64_t tmpCost;
                if (m_rdCost.m_psyRd)
                {
                    tmpEnergy = m_rdCost.psyCost(sizeIdxC, fenc, stride, reconQt, reconQtStride);
                    tmpCost = m_rdCost.calcPsyRdCost(tmpDist, tmpBits, tmpEnergy);
                }
                else
                    tmpCost = m_rdCost.calcRdCost(tmpDist, tmpBits);

                if (tmpCost < bCost)
                {
                    bCost = tmpCost;
                    bDist = tmpDist;
                    bTSkip = useTSkip;
                    bCbf = !!numSig;
                    bEnergy = tmpEnergy;
                }
            }

            if (bTSkip)
            {
                memcpy(coeffC, m_tsCoeff, sizeof(coeff_t) << (log2TrSizeC * 2));
                primitives.cu[sizeIdxC].copy_pp(reconQt, reconQtStride, m_tsRecon, MAX_TS_SIZE);
            }

            cu.setCbfPartRange(bCbf << tuDepth, ttype, absPartIdxC, tuIterator.absPartIdxStep);
            cu.setTransformSkipPartRange(bTSkip, ttype, absPartIdxC, tuIterator.absPartIdxStep);

            pixel*   reconPicC = m_frame->m_reconPic->getChromaAddr(chromaId, cu.m_cuAddr, cuGeom.absPartIdx + absPartIdxC);
            intptr_t picStride = m_frame->m_reconPic->m_strideC;
            primitives.cu[sizeIdxC].copy_pp(reconPicC, picStride, reconQt, reconQtStride);

            outDist += bDist;
            psyEnergy += bEnergy;
        }
    }
    while (tuIterator.isNextSection());

    if (splitType == VERTICAL_SPLIT)
    {
        offsetSubTUCBFs(cu, TEXT_CHROMA_U, tuDepth, absPartIdx);
        offsetSubTUCBFs(cu, TEXT_CHROMA_V, tuDepth, absPartIdx);
    }

    m_entropyCoder.load(m_rqt[fullDepth].rqtRoot);
    return outDist;
}

void Search::extractIntraResultChromaQT(CUData& cu, Yuv& reconYuv, uint32_t absPartIdx, uint32_t tuDepth)
{
    uint32_t tuDepthL  = cu.m_tuDepth[absPartIdx];
    uint32_t log2TrSize = cu.m_log2CUSize[0] - tuDepth;
    uint32_t log2TrSizeC = log2TrSize - m_hChromaShift;

    if (tuDepthL == tuDepth || log2TrSizeC == 2)
    {
        // copy transform coefficients
        uint32_t numCoeffC = 1 << (log2TrSizeC * 2 + (m_csp == X265_CSP_I422));
        uint32_t coeffOffsetC = absPartIdx << (LOG2_UNIT_SIZE * 2 - (m_hChromaShift + m_vChromaShift));

        uint32_t qtLayer   = log2TrSize - 2 - (tuDepthL - tuDepth);
        coeff_t* coeffSrcU = m_rqt[qtLayer].coeffRQT[1] + coeffOffsetC;
        coeff_t* coeffSrcV = m_rqt[qtLayer].coeffRQT[2] + coeffOffsetC;
        coeff_t* coeffDstU = cu.m_trCoeff[1]           + coeffOffsetC;
        coeff_t* coeffDstV = cu.m_trCoeff[2]           + coeffOffsetC;
        memcpy(coeffDstU, coeffSrcU, sizeof(coeff_t) * numCoeffC);
        memcpy(coeffDstV, coeffSrcV, sizeof(coeff_t) * numCoeffC);

        // copy reconstruction
        m_rqt[qtLayer].reconQtYuv.copyPartToPartChroma(reconYuv, absPartIdx, log2TrSizeC + m_hChromaShift);
    }
    else
    {
        uint32_t qNumParts = 1 << (log2TrSize - 1 - LOG2_UNIT_SIZE) * 2;
        for (uint32_t qIdx = 0; qIdx < 4; ++qIdx, absPartIdx += qNumParts)
            extractIntraResultChromaQT(cu, reconYuv, absPartIdx, tuDepth + 1);
    }
}
void Search::checkIntra(Mode& intraMode, const CUGeom& cuGeom, PartSize partSize, uint8_t* sharedModes, uint8_t* sharedChromaModes)
{
    CUData& cu = intraMode.cu;

    cu.setPartSizeSubParts(partSize);
    cu.setPredModeSubParts(MODE_INTRA);

    uint32_t tuDepthRange[2];
    cu.getIntraTUQtDepthRange(tuDepthRange, 0);

    intraMode.initCosts();
    intraMode.lumaDistortion += estIntraPredQT(intraMode, cuGeom, tuDepthRange, sharedModes);
    intraMode.chromaDistortion += estIntraPredChromaQT(intraMode, cuGeom, sharedChromaModes);
    intraMode.distortion += intraMode.lumaDistortion + intraMode.chromaDistortion;

    m_entropyCoder.resetBits();
    if (m_slice->m_pps->bTransquantBypassEnabled)
        m_entropyCoder.codeCUTransquantBypassFlag(cu.m_tqBypass[0]);

    if (!m_slice->isIntra())
    {
        m_entropyCoder.codeSkipFlag(cu, 0);
        m_entropyCoder.codePredMode(cu.m_predMode[0]);
    }

    m_entropyCoder.codePartSize(cu, 0, cuGeom.depth);
    m_entropyCoder.codePredInfo(cu, 0);
    intraMode.mvBits = m_entropyCoder.getNumberOfWrittenBits();

    bool bCodeDQP = m_slice->m_pps->bUseDQP;
    m_entropyCoder.codeCoeff(cu, 0, bCodeDQP, tuDepthRange);
    m_entropyCoder.store(intraMode.contexts);
    intraMode.totalBits = m_entropyCoder.getNumberOfWrittenBits();
    intraMode.coeffBits = intraMode.totalBits - intraMode.mvBits;
    if (m_rdCost.m_psyRd)
    {
        const Yuv* fencYuv = intraMode.fencYuv;
        intraMode.psyEnergy = m_rdCost.psyCost(cuGeom.log2CUSize - 2, fencYuv->m_buf[0], fencYuv->m_size, intraMode.reconYuv.m_buf[0], intraMode.reconYuv.m_size);
    }
    updateModeCost(intraMode);
    checkDQP(intraMode, cuGeom);
}
uint32_t Search::estIntraPredQT(Mode &intraMode, const CUGeom& cuGeom, const uint32_t depthRange[2], uint8_t* sharedModes)
{
    CUData& cu = intraMode.cu;
    Yuv* reconYuv = &intraMode.reconYuv;
    Yuv* predYuv = &intraMode.predYuv;
    const Yuv* fencYuv = intraMode.fencYuv;

    uint32_t depth        = cuGeom.depth;
    uint32_t initTuDepth  = cu.m_partSize[0] != SIZE_2Nx2N;
    uint32_t numPU        = 1 << (2 * initTuDepth);
    uint32_t log2TrSize   = cuGeom.log2CUSize - initTuDepth;
    uint32_t tuSize       = 1 << log2TrSize;
    uint32_t qNumParts    = cuGeom.numPartitions >> 2;
    uint32_t sizeIdx      = log2TrSize - 2;
    uint32_t absPartIdx   = 0;
    uint32_t totalDistortion = 0;

    int checkTransformSkip = m_slice->m_pps->bTransformSkipEnabled && !cu.m_tqBypass[0] && cu.m_partSize[0] != SIZE_2Nx2N;

    // loop over partitions
    for (uint32_t puIdx = 0; puIdx < numPU; puIdx++, absPartIdx += qNumParts)
    {
        uint32_t bmode = 0;

        if (sharedModes)
            bmode = sharedModes[puIdx];
        else
        {
            uint64_t candCostList[MAX_RD_INTRA_MODES];
            uint32_t rdModeList[MAX_RD_INTRA_MODES];
            uint64_t bcost;
            int maxCandCount = 2 + m_param->rdLevel + ((depth + initTuDepth) >> 1);

            {
                ProfileCUScope(intraMode.cu, intraAnalysisElapsedTime, countIntraAnalysis);

                // Reference sample smoothing
                IntraNeighbors intraNeighbors;
                initIntraNeighbors(cu, absPartIdx, initTuDepth, true, &intraNeighbors);
                initAdiPattern(cu, cuGeom, absPartIdx, intraNeighbors, ALL_IDX);

                // determine set of modes to be tested (using prediction signal only)
                const pixel* fenc = fencYuv->getLumaAddr(absPartIdx);
                uint32_t stride = predYuv->m_size;

                int scaleTuSize = tuSize;
                int scaleStride = stride;
                int costShift = 0;

                if (tuSize > 32)
                {
                    // origin is 64x64, we scale to 32x32 and setup required parameters
                    primitives.scale2D_64to32(m_fencScaled, fenc, stride);
                    fenc = m_fencScaled;

                    pixel nScale[129];
                    intraNeighbourBuf[1][0] = intraNeighbourBuf[0][0];
                    primitives.scale1D_128to64(nScale + 1, intraNeighbourBuf[0] + 1);

                    memcpy(&intraNeighbourBuf[0][1], &nScale[1], 2 * 64 * sizeof(pixel));
                    memcpy(&intraNeighbourBuf[1][1], &nScale[1], 2 * 64 * sizeof(pixel));

                    scaleTuSize = 32;
                    scaleStride = 32;
                    costShift = 2;
                    sizeIdx = 5 - 2; // log2(scaleTuSize) - 2
                }

                m_entropyCoder.loadIntraDirModeLuma(m_rqt[depth].cur);

                /* there are three cost tiers for intra modes:
                *  pred[0]          - mode probable, least cost
                *  pred[1], pred[2] - less probable, slightly more cost
                *  non-mpm modes    - all cost the same (rbits) */
                uint64_t mpms;
                uint32_t mpmModes[3];
                uint32_t rbits = getIntraRemModeBits(cu, absPartIdx, mpmModes, mpms);

                pixelcmp_t sa8d = primitives.cu[sizeIdx].sa8d;
                uint64_t modeCosts[35];

                // DC
                primitives.cu[sizeIdx].intra_pred[DC_IDX](m_intraPred, scaleStride, intraNeighbourBuf[0], 0, (scaleTuSize <= 16));
                uint32_t bits = (mpms & ((uint64_t)1 << DC_IDX)) ? m_entropyCoder.bitsIntraModeMPM(mpmModes, DC_IDX) : rbits;
                uint32_t sad = sa8d(fenc, scaleStride, m_intraPred, scaleStride) << costShift;
                modeCosts[DC_IDX] = bcost = m_rdCost.calcRdSADCost(sad, bits);

                // PLANAR
                pixel* planar = intraNeighbourBuf[0];
                if (tuSize >= 8 && tuSize <= 32)
                    planar = intraNeighbourBuf[1];

                primitives.cu[sizeIdx].intra_pred[PLANAR_IDX](m_intraPred, scaleStride, planar, 0, 0);
                bits = (mpms & ((uint64_t)1 << PLANAR_IDX)) ? m_entropyCoder.bitsIntraModeMPM(mpmModes, PLANAR_IDX) : rbits;
                sad = sa8d(fenc, scaleStride, m_intraPred, scaleStride) << costShift;
                modeCosts[PLANAR_IDX] = m_rdCost.calcRdSADCost(sad, bits);
                COPY1_IF_LT(bcost, modeCosts[PLANAR_IDX]);

                // angular predictions
                if (primitives.cu[sizeIdx].intra_pred_allangs)
                {
                    primitives.cu[sizeIdx].transpose(m_fencTransposed, fenc, scaleStride);
                    primitives.cu[sizeIdx].intra_pred_allangs(m_intraPredAngs, intraNeighbourBuf[0], intraNeighbourBuf[1], (scaleTuSize <= 16));
                    for (int mode = 2; mode < 35; mode++)
                    {
                        bits = (mpms & ((uint64_t)1 << mode)) ? m_entropyCoder.bitsIntraModeMPM(mpmModes, mode) : rbits;
                        if (mode < 18)
                            sad = sa8d(m_fencTransposed, scaleTuSize, &m_intraPredAngs[(mode - 2) * (scaleTuSize * scaleTuSize)], scaleTuSize) << costShift;
                        else
                            sad = sa8d(fenc, scaleStride, &m_intraPredAngs[(mode - 2) * (scaleTuSize * scaleTuSize)], scaleTuSize) << costShift;
                        modeCosts[mode] = m_rdCost.calcRdSADCost(sad, bits);
                        COPY1_IF_LT(bcost, modeCosts[mode]);
                    }
                }
                else
                {
                    for (int mode = 2; mode < 35; mode++)
                    {
                        bits = (mpms & ((uint64_t)1 << mode)) ? m_entropyCoder.bitsIntraModeMPM(mpmModes, mode) : rbits;
                        int filter = !!(g_intraFilterFlags[mode] & scaleTuSize);
                        primitives.cu[sizeIdx].intra_pred[mode](m_intraPred, scaleTuSize, intraNeighbourBuf[filter], mode, scaleTuSize <= 16);
                        sad = sa8d(fenc, scaleStride, m_intraPred, scaleTuSize) << costShift;
                        modeCosts[mode] = m_rdCost.calcRdSADCost(sad, bits);
                        COPY1_IF_LT(bcost, modeCosts[mode]);
                    }
                }

                /* Find the top maxCandCount candidate modes with cost within 25% of best
                * or among the most probable modes. maxCandCount is derived from the
                * rdLevel and depth. In general we want to try more modes at slower RD
                * levels and at higher depths */
                for (int i = 0; i < maxCandCount; i++)
                    candCostList[i] = MAX_INT64;

                uint64_t paddedBcost = bcost + (bcost >> 3); // 1.12%
                for (int mode = 0; mode < 35; mode++)
                    if (modeCosts[mode] < paddedBcost || (mpms & ((uint64_t)1 << mode)))
                        updateCandList(mode, modeCosts[mode], maxCandCount, rdModeList, candCostList);
            }

            /* measure best candidates using simple RDO (no TU splits) */
            bcost = MAX_INT64;
            for (int i = 0; i < maxCandCount; i++)
            {
                if (candCostList[i] == MAX_INT64)
                    break;

                ProfileCUScope(intraMode.cu, intraRDOElapsedTime[cuGeom.depth], countIntraRDO[cuGeom.depth]);

                m_entropyCoder.load(m_rqt[depth].cur);
                cu.setLumaIntraDirSubParts(rdModeList[i], absPartIdx, depth + initTuDepth);

                Cost icosts;
                if (checkTransformSkip)
                    codeIntraLumaTSkip(intraMode, cuGeom, initTuDepth, absPartIdx, icosts);
                else
                    codeIntraLumaQT(intraMode, cuGeom, initTuDepth, absPartIdx, false, icosts, depthRange);
                COPY2_IF_LT(bcost, icosts.rdcost, bmode, rdModeList[i]);
            }
        }

        ProfileCUScope(intraMode.cu, intraRDOElapsedTime[cuGeom.depth], countIntraRDO[cuGeom.depth]);

        /* remeasure best mode, allowing TU splits */
        cu.setLumaIntraDirSubParts(bmode, absPartIdx, depth + initTuDepth);
        m_entropyCoder.load(m_rqt[depth].cur);

        Cost icosts;
        if (checkTransformSkip)
            codeIntraLumaTSkip(intraMode, cuGeom, initTuDepth, absPartIdx, icosts);
        else
            codeIntraLumaQT(intraMode, cuGeom, initTuDepth, absPartIdx, true, icosts, depthRange);
        totalDistortion += icosts.distortion;

        extractIntraResultQT(cu, *reconYuv, initTuDepth, absPartIdx);

        // set reconstruction for next intra prediction blocks
        if (puIdx != numPU - 1)
        {
            /* This has important implications for parallelism and RDO.  It is writing intermediate results into the
             * output recon picture, so it cannot proceed in parallel with anything else when doing INTRA_NXN. Also
             * it is not updating m_rdContexts[depth].cur for the later PUs which I suspect is slightly wrong. I think
             * that the contexts should be tracked through each PU */
            pixel*   dst         = m_frame->m_reconPic->getLumaAddr(cu.m_cuAddr, cuGeom.absPartIdx + absPartIdx);
            uint32_t dststride   = m_frame->m_reconPic->m_stride;
            const pixel*   src   = reconYuv->getLumaAddr(absPartIdx);
            uint32_t srcstride   = reconYuv->m_size;
            primitives.cu[log2TrSize - 2].copy_pp(dst, dststride, src, srcstride);
        }
    }

    if (numPU > 1)
    {
        uint32_t combCbfY = 0;
        for (uint32_t qIdx = 0, qPartIdx = 0; qIdx < 4; ++qIdx, qPartIdx += qNumParts)
            combCbfY |= cu.getCbf(qPartIdx, TEXT_LUMA, 1);

        for (uint32_t offs = 0; offs < 4 * qNumParts; offs++)
            cu.m_cbf[0][offs] |= combCbfY;
    }

    // TODO: remove this
    m_entropyCoder.load(m_rqt[depth].cur);

    return totalDistortion;
}
uint32_t Search::estIntraPredChromaQT(Mode &intraMode, const CUGeom& cuGeom, uint8_t* sharedChromaModes)
{
    CUData& cu = intraMode.cu;
    Yuv& reconYuv = intraMode.reconYuv;

    uint32_t depth       = cuGeom.depth;
    uint32_t initTuDepth = cu.m_partSize[0] != SIZE_2Nx2N && m_csp == X265_CSP_I444;
    uint32_t log2TrSize  = cuGeom.log2CUSize - initTuDepth;
    uint32_t absPartStep = cuGeom.numPartitions;
    uint32_t totalDistortion = 0;

    int size = partitionFromLog2Size(log2TrSize);

    TURecurse tuIterator((initTuDepth == 0) ? DONT_SPLIT : QUAD_SPLIT, absPartStep, 0);

    do
    {
        uint32_t absPartIdxC = tuIterator.absPartIdxTURelCU;

        uint32_t bestMode = 0;
        uint32_t bestDist = 0;
        uint64_t bestCost = MAX_INT64;

        // init mode list
        uint32_t minMode = 0;
        uint32_t maxMode = NUM_CHROMA_MODE;
        uint32_t modeList[NUM_CHROMA_MODE];

        if (sharedChromaModes && !initTuDepth)
        {
            for (uint32_t l = 0; l < NUM_CHROMA_MODE; l++)
                modeList[l] = sharedChromaModes[0];
            maxMode = 1;
        }
        else
            cu.getAllowedChromaDir(absPartIdxC, modeList);

        // check chroma modes
        for (uint32_t mode = minMode; mode < maxMode; mode++)
        {
            // restore context models
            m_entropyCoder.load(m_rqt[depth].cur);

            cu.setChromIntraDirSubParts(modeList[mode], absPartIdxC, depth + initTuDepth);
            uint32_t psyEnergy = 0;
            uint32_t dist = codeIntraChromaQt(intraMode, cuGeom, initTuDepth, absPartIdxC, psyEnergy);

            if (m_slice->m_pps->bTransformSkipEnabled)
                m_entropyCoder.load(m_rqt[depth].cur);

            m_entropyCoder.resetBits();
            // chroma prediction mode
            if (cu.m_partSize[0] == SIZE_2Nx2N || m_csp != X265_CSP_I444)
            {
                if (!absPartIdxC)
                    m_entropyCoder.codeIntraDirChroma(cu, absPartIdxC, modeList);
            }
            else
            {
                uint32_t qNumParts = cuGeom.numPartitions >> 2;
                if (!(absPartIdxC & (qNumParts - 1)))
                    m_entropyCoder.codeIntraDirChroma(cu, absPartIdxC, modeList);
            }

            codeSubdivCbfQTChroma(cu, initTuDepth, absPartIdxC);
            codeCoeffQTChroma(cu, initTuDepth, absPartIdxC, TEXT_CHROMA_U);
            codeCoeffQTChroma(cu, initTuDepth, absPartIdxC, TEXT_CHROMA_V);
            uint32_t bits = m_entropyCoder.getNumberOfWrittenBits();
            uint64_t cost = m_rdCost.m_psyRd ? m_rdCost.calcPsyRdCost(dist, bits, psyEnergy) : m_rdCost.calcRdCost(dist, bits);

            if (cost < bestCost)
            {
                bestCost = cost;
                bestDist = dist;
                bestMode = modeList[mode];
                extractIntraResultChromaQT(cu, reconYuv, absPartIdxC, initTuDepth);
                memcpy(m_qtTempCbf[1], cu.m_cbf[1] + absPartIdxC, tuIterator.absPartIdxStep * sizeof(uint8_t));
                memcpy(m_qtTempCbf[2], cu.m_cbf[2] + absPartIdxC, tuIterator.absPartIdxStep * sizeof(uint8_t));
                memcpy(m_qtTempTransformSkipFlag[1], cu.m_transformSkip[1] + absPartIdxC, tuIterator.absPartIdxStep * sizeof(uint8_t));
                memcpy(m_qtTempTransformSkipFlag[2], cu.m_transformSkip[2] + absPartIdxC, tuIterator.absPartIdxStep * sizeof(uint8_t));
            }
        }

        if (!tuIterator.isLastSection())
        {
            uint32_t zorder    = cuGeom.absPartIdx + absPartIdxC;
            uint32_t dststride = m_frame->m_reconPic->m_strideC;
            const pixel* src;
            pixel* dst;

            dst = m_frame->m_reconPic->getCbAddr(cu.m_cuAddr, zorder);
            src = reconYuv.getCbAddr(absPartIdxC);
            primitives.chroma[m_csp].cu[size].copy_pp(dst, dststride, src, reconYuv.m_csize);

            dst = m_frame->m_reconPic->getCrAddr(cu.m_cuAddr, zorder);
            src = reconYuv.getCrAddr(absPartIdxC);
            primitives.chroma[m_csp].cu[size].copy_pp(dst, dststride, src, reconYuv.m_csize);
        }

        memcpy(cu.m_cbf[1] + absPartIdxC, m_qtTempCbf[1], tuIterator.absPartIdxStep * sizeof(uint8_t));
        memcpy(cu.m_cbf[2] + absPartIdxC, m_qtTempCbf[2], tuIterator.absPartIdxStep * sizeof(uint8_t));
        memcpy(cu.m_transformSkip[1] + absPartIdxC, m_qtTempTransformSkipFlag[1], tuIterator.absPartIdxStep * sizeof(uint8_t));
        memcpy(cu.m_transformSkip[2] + absPartIdxC, m_qtTempTransformSkipFlag[2], tuIterator.absPartIdxStep * sizeof(uint8_t));
        cu.setChromIntraDirSubParts(bestMode, absPartIdxC, depth + initTuDepth);
        totalDistortion += bestDist;
    }
    while (tuIterator.isNextSection());

    if (initTuDepth != 0)
    {
        uint32_t combCbfU = 0;
        uint32_t combCbfV = 0;
        uint32_t qNumParts = tuIterator.absPartIdxStep;
        for (uint32_t qIdx = 0, qPartIdx = 0; qIdx < 4; ++qIdx, qPartIdx += qNumParts)
        {
            combCbfU |= cu.getCbf(qPartIdx, TEXT_CHROMA_U, 1);
            combCbfV |= cu.getCbf(qPartIdx, TEXT_CHROMA_V, 1);
        }

        for (uint32_t offs = 0; offs < 4 * qNumParts; offs++)
        {
            cu.m_cbf[1][offs] |= combCbfU;
            cu.m_cbf[2][offs] |= combCbfV;
        }
    }

    /* TODO: remove this */
    m_entropyCoder.load(m_rqt[depth].cur);
    return totalDistortion;
}
/* returns the number of bits required to signal a non-most-probable mode.
 * on return mpms contains bitmap of most probable modes */
uint32_t Search::getIntraRemModeBits(CUData& cu, uint32_t absPartIdx, uint32_t mpmModes[3], uint64_t& mpms) const
{
    cu.getIntraDirLumaPredictor(absPartIdx, mpmModes);

    mpms = 0;
    for (int i = 0; i < 3; ++i)
        mpms |= ((uint64_t)1 << mpmModes[i]);

    return m_entropyCoder.bitsIntraModeNonMPM();
}

/* swap the current mode/cost with the mode with the highest cost in the
 * current candidate list, if its cost is better (maintain a top N list) */
void Search::updateCandList(uint32_t mode, uint64_t cost, int maxCandCount, uint32_t* candModeList, uint64_t* candCostList)
{
    uint32_t maxIndex = 0;
    uint64_t maxValue = 0;

    for (int i = 0; i < maxCandCount; i++)
    {
        if (maxValue < candCostList[i])
        {
            maxValue = candCostList[i];
            maxIndex = i;
        }
    }

    if (cost < maxValue)
    {
        candCostList[maxIndex] = cost;
        candModeList[maxIndex] = mode;
    }
}

void Search::checkDQP(Mode& mode, const CUGeom& cuGeom)
{
    CUData& cu = mode.cu;
    if (cu.m_slice->m_pps->bUseDQP && cuGeom.depth <= cu.m_slice->m_pps->maxCuDQPDepth)
    {
        if (cu.getQtRootCbf(0))
        {
            if (m_param->rdLevel >= 3)
            {
                mode.contexts.resetBits();
                mode.contexts.codeDeltaQP(cu, 0);
                uint32_t bits = mode.contexts.getNumberOfWrittenBits();
                mode.mvBits += bits;
                mode.totalBits += bits;
                updateModeCost(mode);
            }
            else if (m_param->rdLevel <= 1)
            {
                mode.sa8dBits++;
                mode.sa8dCost = m_rdCost.calcRdSADCost((uint32_t)mode.distortion, mode.sa8dBits);
            }
            else
            {
                mode.mvBits++;
                mode.totalBits++;
                updateModeCost(mode);
            }
        }
        else
            cu.setQPSubParts(cu.getRefQP(0), 0, cuGeom.depth);
    }
}

void Search::checkDQPForSplitPred(Mode& mode, const CUGeom& cuGeom)
{
    CUData& cu = mode.cu;

    if ((cuGeom.depth == cu.m_slice->m_pps->maxCuDQPDepth) && cu.m_slice->m_pps->bUseDQP)
    {
        bool hasResidual = false;

        /* Check if any sub-CU has a non-zero QP */
        for (uint32_t blkIdx = 0; blkIdx < cuGeom.numPartitions; blkIdx++)
        {
            if (cu.getQtRootCbf(blkIdx))
            {
                hasResidual = true;
                break;
            }
        }
        if (hasResidual)
        {
            if (m_param->rdLevel >= 3)
            {
                mode.contexts.resetBits();
                mode.contexts.codeDeltaQP(cu, 0);
                uint32_t bits = mode.contexts.getNumberOfWrittenBits();
                mode.mvBits += bits;
                mode.totalBits += bits;
                updateModeCost(mode);
            }
            else if (m_param->rdLevel <= 1)
            {
                mode.sa8dBits++;
                mode.sa8dCost = m_rdCost.calcRdSADCost((uint32_t)mode.distortion, mode.sa8dBits);
            }
            else
            {
                mode.mvBits++;
                mode.totalBits++;
                updateModeCost(mode);
            }
            /* For all zero CBF sub-CUs, reset QP to RefQP (so that deltaQP is not signalled).
            When the non-zero CBF sub-CU is found, stop */
            cu.setQPSubCUs(cu.getRefQP(0), 0, cuGeom.depth);
        }
        else
            /* No residual within this CU or subCU, so reset QP to RefQP */
            cu.setQPSubParts(cu.getRefQP(0), 0, cuGeom.depth);
    }
}
