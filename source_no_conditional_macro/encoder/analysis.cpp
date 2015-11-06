/*****************************************************************************
* Copyright (C) 2013 x265 project
*
* Authors: Deepthi Nandakumar <deepthi@multicorewareinc.com>
*          Steve Borho <steve@borho.org>
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
#include "frame.h"
#include "framedata.h"
#include "picyuv.h"
#include "primitives.h"
#include "threading.h"

#include "analysis.h"
#include "rdcost.h"
#include "encoder.h"

using namespace X265_NS;

/* An explanation of rate distortion levels (--rd-level)
 * 
 * rd-level 0 generates no recon per CU (NO RDO or Quant)
 *
 *   sa8d selection between merge / skip / inter / intra and split
 *   no recon pixels generated until CTU analysis is complete, requiring
 *   intra predictions to use source pixels
 *
 * rd-level 1 uses RDO for merge and skip, sa8d for all else
 *
 *   RDO selection between merge and skip
 *   sa8d selection between (merge/skip) / inter modes / intra and split
 *   intra prediction uses reconstructed pixels
 *
 * rd-level 2 uses RDO for merge/skip and split
 *
 *   RDO selection between merge and skip
 *   sa8d selection between (merge/skip) / inter modes / intra
 *   RDO split decisions
 *
 * rd-level 3 uses RDO for merge/skip/best inter/intra
 *
 *   RDO selection between merge and skip
 *   sa8d selection of best inter mode
 *   sa8d decisions include chroma residual cost
 *   RDO selection between (merge/skip) / best inter mode / intra / split
 *
 * rd-level 4 enables RDOQuant
 *   chroma residual cost included in satd decisions, including subpel refine
 *    (as a result of --subme 3 being used by preset slow)
 *
 * rd-level 5,6 does RDO for each inter mode
 */

Analysis::Analysis()
{
}

bool Analysis::create(ThreadLocalData *tld)
{
    m_tld = tld;
    m_bTryLossless = m_param->bCULossless && !m_param->bLossless && m_param->rdLevel >= 2;
    m_bChromaSa8d = m_param->rdLevel >= 3;

    int csp = m_param->internalCsp;
    uint32_t cuSize = g_maxCUSize;

    bool ok = true;
    for (uint32_t depth = 0; depth <= g_maxCUDepth; depth++, cuSize >>= 1)
    {
        ModeDepth &md = m_modeDepth[depth];

        md.cuMemPool.create(depth, csp, MAX_PRED_TYPES);
        ok &= md.fencYuv.create(cuSize, csp);

        for (int j = 0; j < MAX_PRED_TYPES; j++)
        {
            md.pred[j].cu.initialize(md.cuMemPool, depth, csp, j);
            ok &= md.pred[j].predYuv.create(cuSize, csp);
            ok &= md.pred[j].reconYuv.create(cuSize, csp);
            md.pred[j].fencYuv = &md.fencYuv;
        }
    }

    return ok;
}

void Analysis::destroy()
{
    for (uint32_t i = 0; i <= g_maxCUDepth; i++)
    {
        m_modeDepth[i].cuMemPool.destroy();
        m_modeDepth[i].fencYuv.destroy();

        for (int j = 0; j < MAX_PRED_TYPES; j++)
        {
            m_modeDepth[i].pred[j].predYuv.destroy();
            m_modeDepth[i].pred[j].reconYuv.destroy();
        }
    }
}

Mode& Analysis::compressCTU(CUData& ctu, Frame& frame, const CUGeom& cuGeom, const Entropy& initialContext)
{
    m_slice = ctu.m_slice;
    m_frame = &frame;

#if _DEBUG || CHECKED_BUILD
    for (uint32_t i = 0; i <= g_maxCUDepth; i++)
        for (uint32_t j = 0; j < MAX_PRED_TYPES; j++)
            m_modeDepth[i].pred[j].invalidate();
    invalidateContexts(0);
#endif

    int qp = setLambdaFromQP(ctu, m_slice->m_pps->bUseDQP ? calculateQpforCuSize(ctu, cuGeom) : m_slice->m_sliceQp);
    ctu.setQPSubParts((int8_t)qp, 0, 0);

    m_rqt[0].cur.load(initialContext);
    m_modeDepth[0].fencYuv.copyFromPicYuv(*m_frame->m_fencPic, ctu.m_cuAddr, 0);


    ProfileCUScope(ctu, totalCTUTime, totalCTUs);

    uint32_t zOrder = 0;
    if (m_slice->m_sliceType == I_SLICE)
    {
        compressIntraCU(ctu, cuGeom, zOrder, qp);
    }

    return *m_modeDepth[0].bestMode;
}

void Analysis::tryLossless(const CUGeom& cuGeom)
{
    ModeDepth& md = m_modeDepth[cuGeom.depth];

    if (!md.bestMode->distortion)
        /* already lossless */
        return;
    else if (md.bestMode->cu.isIntra(0))
    {
        md.pred[PRED_LOSSLESS].initCosts();
        md.pred[PRED_LOSSLESS].cu.initLosslessCU(md.bestMode->cu, cuGeom);
        PartSize size = (PartSize)md.pred[PRED_LOSSLESS].cu.m_partSize[0];
        uint8_t* modes = md.pred[PRED_LOSSLESS].cu.m_lumaIntraDir;
        checkIntra(md.pred[PRED_LOSSLESS], cuGeom, size, modes, NULL);
        checkBestMode(md.pred[PRED_LOSSLESS], cuGeom.depth);
    }
}

void Analysis::compressIntraCU(const CUData& parentCTU, const CUGeom& cuGeom, uint32_t& zOrder, int32_t qp)
{
    uint32_t depth = cuGeom.depth;
    ModeDepth& md = m_modeDepth[depth];
    md.bestMode = NULL;

    bool mightSplit = !(cuGeom.flags & CUGeom::LEAF);
    bool mightNotSplit = !(cuGeom.flags & CUGeom::SPLIT_MANDATORY);

	if (mightNotSplit)
    {
        md.pred[PRED_INTRA].cu.initSubCU(parentCTU, cuGeom, qp);
        checkIntra(md.pred[PRED_INTRA], cuGeom, SIZE_2Nx2N, NULL, NULL);
        checkBestMode(md.pred[PRED_INTRA], depth);

        if (cuGeom.log2CUSize == 3 && m_slice->m_sps->quadtreeTULog2MinSize < 3)
        {
            md.pred[PRED_INTRA_NxN].cu.initSubCU(parentCTU, cuGeom, qp);
            checkIntra(md.pred[PRED_INTRA_NxN], cuGeom, SIZE_NxN, NULL, NULL);
            checkBestMode(md.pred[PRED_INTRA_NxN], depth);
        }

        if (m_bTryLossless)
            tryLossless(cuGeom);

        if (mightSplit)
            addSplitFlagCost(*md.bestMode, cuGeom.depth);
    }

    if (mightSplit)
    {
        Mode* splitPred = &md.pred[PRED_SPLIT];
        splitPred->initCosts();
        CUData* splitCU = &splitPred->cu;
        splitCU->initSubCU(parentCTU, cuGeom, qp);

        uint32_t nextDepth = depth + 1;
        ModeDepth& nd = m_modeDepth[nextDepth];
        invalidateContexts(nextDepth);
        Entropy* nextContext = &m_rqt[depth].cur;
        int32_t nextQP = qp;

        for (uint32_t subPartIdx = 0; subPartIdx < 4; subPartIdx++)
        {
            const CUGeom& childGeom = *(&cuGeom + cuGeom.childOffset + subPartIdx);
            if (childGeom.flags & CUGeom::PRESENT)
            {
                m_modeDepth[0].fencYuv.copyPartToYuv(nd.fencYuv, childGeom.absPartIdx);
                m_rqt[nextDepth].cur.load(*nextContext);

                if (m_slice->m_pps->bUseDQP && nextDepth <= m_slice->m_pps->maxCuDQPDepth)
                    nextQP = setLambdaFromQP(parentCTU, calculateQpforCuSize(parentCTU, childGeom));

                compressIntraCU(parentCTU, childGeom, zOrder, nextQP);

                // Save best CU and pred data for this sub CU
                splitCU->copyPartFrom(nd.bestMode->cu, childGeom, subPartIdx);
                splitPred->addSubCosts(*nd.bestMode);
                nd.bestMode->reconYuv.copyToPartYuv(splitPred->reconYuv, childGeom.numPartitions * subPartIdx);
                nextContext = &nd.bestMode->contexts;
            }
            else
            {
                /* record the depth of this non-present sub-CU */
                splitCU->setEmptyPart(childGeom, subPartIdx);
                zOrder += g_depthInc[g_maxCUDepth - 1][nextDepth];
            }
        }
        nextContext->store(splitPred->contexts);
        if (mightNotSplit)
            addSplitFlagCost(*splitPred, cuGeom.depth);
        else
            updateModeCost(*splitPred);

        checkDQPForSplitPred(*splitPred, cuGeom);
        checkBestMode(*splitPred, depth);
    }

    /* Copy best data to encData CTU and recon */
    md.bestMode->cu.copyToPic(depth);
    if (md.bestMode != &md.pred[PRED_SPLIT])
        md.bestMode->reconYuv.copyToPicYuv(*m_frame->m_reconPic, parentCTU.m_cuAddr, cuGeom.absPartIdx);
}
void Analysis::addSplitFlagCost(Mode& mode, uint32_t depth)
{
    if (m_param->rdLevel >= 3)
    {
        /* code the split flag (0 or 1) and update bit costs */
        mode.contexts.resetBits();
        mode.contexts.codeSplitFlag(mode.cu, 0, depth);
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
int Analysis::calculateQpforCuSize(const CUData& ctu, const CUGeom& cuGeom)
{
    FrameData& curEncData = *m_frame->m_encData;
    double qp = curEncData.m_cuStat[ctu.m_cuAddr].baseQp;

    /* Use cuTree offsets if cuTree enabled and frame is referenced, else use AQ offsets */
    bool isReferenced = IS_REFERENCED(m_frame);
    double *qpoffs = (isReferenced && m_param->rc.cuTree) ? m_frame->m_lowres.qpCuTreeOffset : m_frame->m_lowres.qpAqOffset;
    if (qpoffs)
    {
        uint32_t width = m_frame->m_fencPic->m_picWidth;
        uint32_t height = m_frame->m_fencPic->m_picHeight;
        uint32_t block_x = ctu.m_cuPelX + g_zscanToPelX[cuGeom.absPartIdx];
        uint32_t block_y = ctu.m_cuPelY + g_zscanToPelY[cuGeom.absPartIdx];
        uint32_t maxCols = (m_frame->m_fencPic->m_picWidth + (16 - 1)) / 16;
        uint32_t blockSize = g_maxCUSize >> cuGeom.depth;
        double qp_offset = 0;
        uint32_t cnt = 0;
        uint32_t idx;

        for (uint32_t block_yy = block_y; block_yy < block_y + blockSize && block_yy < height; block_yy += 16)
        {
            for (uint32_t block_xx = block_x; block_xx < block_x + blockSize && block_xx < width; block_xx += 16)
            {
                idx = ((block_yy / 16) * (maxCols)) + (block_xx / 16);
                qp_offset += qpoffs[idx];
                cnt++;
            }
        }

        qp_offset /= cnt;
        qp += qp_offset;
    }

    return x265_clip3(QP_MIN, QP_MAX_MAX, (int)(qp + 0.5));
}
