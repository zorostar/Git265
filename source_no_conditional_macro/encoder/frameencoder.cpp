/*****************************************************************************
 * Copyright (C) 2013 x265 project
 *
 * Authors: Chung Shin Yee <shinyee@multicorewareinc.com>
 *          Min Chen <chenm003@163.com>
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
#include "wavefront.h"
#include "param.h"

#include "encoder.h"
#include "frameencoder.h"
#include "common.h"
#include "slicetype.h"
#include "nal.h"

namespace X265_NS {
void weightAnalyse(Slice& slice, Frame& frame, x265_param& param);

FrameEncoder::FrameEncoder()
{
    m_prevOutputTime = x265_mdate();
    m_outStreams = NULL;
    m_substreamSizes = NULL;
    m_nr = NULL;
    m_tld = NULL;
    m_rows = NULL;
    m_top = NULL;
    m_param = NULL;
    m_frame = NULL;
    m_cuGeoms = NULL;
    m_ctuGeomMap = NULL;
    m_localTldIdx = 0;
#ifndef Rc_Del
    memset(&m_rce, 0, sizeof(RateControlEntry));
#endif
}

void FrameEncoder::destroy()
{
    {
        m_tld->destroy();
        delete m_tld;
    }

    delete[] m_rows;
    delete[] m_outStreams;
    X265_FREE(m_cuGeoms);
    X265_FREE(m_ctuGeomMap);
    X265_FREE(m_substreamSizes);
    X265_FREE(m_nr);

    m_frameFilter.destroy();

#ifndef Rc_Del
    if (m_param->bEmitHRDSEI || !!m_param->interlaceMode)
    {
        delete m_rce.picTimingSEI;
        delete m_rce.hrdTiming;
    }
#endif

}

bool FrameEncoder::init(Encoder *top, int numRows, int numCols)
{
    m_top = top;
    m_param = top->m_param;
    m_numRows = numRows;
    m_numCols = numCols;
    m_filterRowDelay = (m_param->bEnableSAO && m_param->bSaoNonDeblocked) ?
                        2 : (m_param->bEnableSAO || m_param->bEnableLoopFilter ? 1 : 0);
    m_filterRowDelayCus = m_filterRowDelay * numCols;
    m_rows = new CTURow[m_numRows];
    bool ok = !!m_numRows;

    /* determine full motion search range */
    int range  = m_param->searchRange;       /* fpel search */
    range += !!(m_param->searchMethod < 2);  /* diamond/hex range check lag */
    range += NTAPS_LUMA / 2;                 /* subpel filter half-length */
    range += 2 + MotionEstimate::hpelIterationCount(m_param->subpelRefine) / 2; /* subpel refine steps */
    m_refLagRows = 1 + ((range + g_maxCUSize - 1) / g_maxCUSize);


    m_frameFilter.init(top, this, numRows);

    // initialize HRD parameters of SPS
#ifndef Rc_Del
    if (m_param->bEmitHRDSEI || !!m_param->interlaceMode)
    {
        m_rce.picTimingSEI = new SEIPictureTiming;
        m_rce.hrdTiming = new HRDTiming;

        ok &= m_rce.picTimingSEI && m_rce.hrdTiming;
    }
#endif

    if (m_param->noiseReductionIntra || m_param->noiseReductionInter)
        m_nr = X265_MALLOC(NoiseReduction, 1);
    if (m_nr)
        memset(m_nr, 0, sizeof(NoiseReduction));
    else
        m_param->noiseReductionIntra = m_param->noiseReductionInter = 0;

    return ok;
}

/* Generate a complete list of unique geom sets for the current picture dimensions */
bool FrameEncoder::initializeGeoms()
{
    /* Geoms only vary between CTUs in the presence of picture edges */
    int maxCUSize = m_param->maxCUSize;
    int minCUSize = m_param->minCUSize;
    int heightRem = m_param->sourceHeight & (maxCUSize - 1);
    int widthRem = m_param->sourceWidth & (maxCUSize - 1);
    int allocGeoms = 1; // body
    if (heightRem && widthRem)
        allocGeoms = 4; // body, right, bottom, corner
    else if (heightRem || widthRem)
        allocGeoms = 2; // body, right or bottom

    m_ctuGeomMap = X265_MALLOC(uint32_t, m_numRows * m_numCols);
    m_cuGeoms = X265_MALLOC(CUGeom, allocGeoms * CUGeom::MAX_GEOMS);
    if (!m_cuGeoms || !m_ctuGeomMap)
        return false;

    // body
    CUData::calcCTUGeoms(maxCUSize, maxCUSize, maxCUSize, minCUSize, m_cuGeoms);
    memset(m_ctuGeomMap, 0, sizeof(uint32_t) * m_numRows * m_numCols);
    if (allocGeoms == 1)
        return true;

    int countGeoms = 1;
    if (widthRem)
    {
        // right
        CUData::calcCTUGeoms(widthRem, maxCUSize, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numRows; i++)
        {
            uint32_t ctuAddr = m_numCols * (i + 1) - 1;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
        }
        countGeoms++;
    }
    if (heightRem)
    {
        // bottom
        CUData::calcCTUGeoms(maxCUSize, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);
        for (uint32_t i = 0; i < m_numCols; i++)
        {
            uint32_t ctuAddr = m_numCols * (m_numRows - 1) + i;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
        }
        countGeoms++;

        if (widthRem)
        {
            // corner
            CUData::calcCTUGeoms(widthRem, heightRem, maxCUSize, minCUSize, m_cuGeoms + countGeoms * CUGeom::MAX_GEOMS);

            uint32_t ctuAddr = m_numCols * m_numRows - 1;
            m_ctuGeomMap[ctuAddr] = countGeoms * CUGeom::MAX_GEOMS;
            countGeoms++;
        }
        X265_CHECK(countGeoms == allocGeoms, "geometry match check failure\n");
    }

    return true;
}

bool FrameEncoder::lzx_statcompress(Frame* curFrame)
{
	m_slicetypeWaitTime = x265_mdate() - m_prevOutputTime;
	m_frame = curFrame;
	m_param = curFrame->m_param;
	//m_sliceType = curFrame->m_lowres.sliceType;
	//curFrame->m_encData->m_frameEncoderID = m_jpId;
	//curFrame->m_encData->m_jobProvider = this;
	curFrame->m_encData->m_slice->m_mref = m_mref;

	if (!m_cuGeoms)
	{
		if (!initializeGeoms())
			return false;
	}

	compressFrame();

	return true;
}


void FrameEncoder::compressFrame()
{

    m_startCompressTime = x265_mdate();
    m_allRowsAvailableTime = 0;

    m_SSDY = m_SSDU = m_SSDV = 0;
    m_ssim = 0;
    m_ssimCnt = 0;
    memset(&(m_frame->m_encData->m_frameStats), 0, sizeof(m_frame->m_encData->m_frameStats));

    /* Emit access unit delimiter unless this is the first frame and the user is
     * not repeating headers (since AUD is supposed to be the first NAL in the access
     * unit) */
    Slice* slice = m_frame->m_encData->m_slice;
    if (m_param->bEnableAccessUnitDelimiters && (m_frame->m_poc || m_param->bRepeatHeaders))
    {
        m_bs.resetBits();
        m_entropyCoder.setBitstream(&m_bs);
        m_entropyCoder.codeAUD(*slice);
        m_bs.writeByteAlignment();
        m_nalList.serialize(NAL_UNIT_ACCESS_UNIT_DELIMITER, m_bs);
    }

#ifndef Lookahead_Del
    if (m_frame->m_lowres.bKeyframe && m_param->bRepeatHeaders)
#else
	if (m_frame->keyframe && m_param->bRepeatHeaders)
#endif
        m_top->getStreamHeaders(m_nalList, m_entropyCoder, m_bs);

        slice->disableWeights();


    /* Get the QP for this frame from rate control. This call may block until
     * frames ahead of it in encode order have called rateControlEnd() */
#ifndef Rc_Del
    int qp = m_top->m_rateControl->rateControlStart(m_frame, &m_rce, m_top);
    m_rce.newQp = qp;
#else
	int qp = m_param->rc.qp;
    m_frame->m_encData->m_avgQpAq = qp;

#endif // !Rc_Del
    /* Clip slice QP to 0-51 spec range before encoding */
    slice->m_sliceQp = x265_clip3(-QP_BD_OFFSET, QP_MAX_SPEC, qp);


    m_initSliceContext.resetEntropy(*slice);

    m_frameFilter.start(m_frame, m_initSliceContext, qp);


    /* reset entropy coders */
    m_entropyCoder.load(m_initSliceContext);
    for (uint32_t i = 0; i < m_numRows; i++)
        m_rows[i].init(m_initSliceContext);

	uint32_t numSubstreams = 1;
    if (!m_outStreams)
    {
        m_outStreams = new Bitstream[numSubstreams];
        m_substreamSizes = X265_MALLOC(uint32_t, numSubstreams);
        if (!m_param->bEnableSAO)
            for (uint32_t i = 0; i < numSubstreams; i++)
                m_rows[i].rowGoOnCoder.setBitstream(&m_outStreams[i]);
    }
    else
        for (uint32_t i = 0; i < numSubstreams; i++)
            m_outStreams[i].resetBits();
#ifndef Rc_Del
    int prevBPSEI = m_rce.encodeOrder ? m_top->m_lastBPSEI : 0;

	int prevBPSEI = ((m_top->m_encodedFrameNum)-1) ? m_top->m_lastBPSEI : 0;
#endif

#ifndef Lookahead_Del
    if (m_frame->m_lowres.bKeyframe)

	if (m_frame->keyframe)

    {

#ifndef Rc_Del
        if (m_param->bEmitHRDSEI)
        {
            SEIBufferingPeriod* bpSei = &m_top->m_rateControl->m_bufPeriodSEI;

            // since the temporal layer HRD is not ready, we assumed it is fixed
            bpSei->m_auCpbRemovalDelayDelta = 1;
            bpSei->m_cpbDelayOffset = 0;
            bpSei->m_dpbDelayOffset = 0;

            // hrdFullness() calculates the initial CPB removal delay and offset
            m_top->m_rateControl->hrdFullness(bpSei);

            m_bs.resetBits();
            bpSei->write(m_bs, *slice->m_sps);
            m_bs.writeByteAlignment();

            m_nalList.serialize(NAL_UNIT_PREFIX_SEI, m_bs);

            m_top->m_lastBPSEI = m_rce.encodeOrder;
        }
#endif
    }
#endif

#ifndef Rc_Del
    if (m_param->bEmitHRDSEI || !!m_param->interlaceMode)
    {
        SEIPictureTiming *sei = m_rce.picTimingSEI;
        const VUI *vui = &slice->m_sps->vuiParameters;
        const HRDInfo *hrd = &vui->hrdParameters;
        int poc = slice->m_poc;

        if (vui->frameFieldInfoPresentFlag)
        {
            if (m_param->interlaceMode == 2)
                sei->m_picStruct = (poc & 1) ? 1 /* top */ : 2 /* bottom */;
            else if (m_param->interlaceMode == 1)
                sei->m_picStruct = (poc & 1) ? 2 /* bottom */ : 1 /* top */;
            else
                sei->m_picStruct = 0;
            sei->m_sourceScanType = 0;
            sei->m_duplicateFlag = false;
        }

        if (vui->hrdParametersPresentFlag)
        {
            // The m_aucpbremoval delay specifies how many clock ticks the
            // access unit associated with the picture timing SEI message has to
            // wait after removal of the access unit with the most recent
            // buffering period SEI message
            sei->m_auCpbRemovalDelay = X265_MIN(X265_MAX(1, m_rce.encodeOrder - prevBPSEI), (1 << hrd->cpbRemovalDelayLength));
            sei->m_picDpbOutputDelay = slice->m_sps->numReorderPics + poc - m_rce.encodeOrder;
        }

        m_bs.resetBits();
        sei->write(m_bs, *slice->m_sps);
        m_bs.writeByteAlignment();
        m_nalList.serialize(NAL_UNIT_PREFIX_SEI, m_bs);
    }
#endif
    /* CQP and CRF (without capped VBV) doesn't use mid-frame statistics to 
     * tune RateControl parameters for other frames.
     * Hence, for these modes, update m_startEndOrder and unlock RC for previous threads waiting in
     * RateControlEnd here, after the slicecontexts are initialized. For the rest - ABR
     * and VBV, unlock only after rateControlUpdateStats of this frame is called */

#ifndef Rc_Del
    if (m_param->rc.rateControlMode != X265_RC_ABR && !m_top->m_rateControl->m_isVbv)
    {
        m_top->m_rateControl->m_startEndOrder.incr();

        if (m_rce.encodeOrder < m_param->frameNumThreads - 1)
            m_top->m_rateControl->m_startEndOrder.incr(); // faked rateControlEnd calls for negative frames
    }
#endif
    /* Analyze CTU rows, most of the hard work is done here.  Frame is
     * compressed in a wave-front pattern if WPP is enabled. Row based loop
     * filters runs behind the CTU compression and reconstruction */

    m_rows[0].active = true;
    {
        for (uint32_t i = 0; i < m_numRows + m_filterRowDelay; i++)
        {
            // compress
            if (i < m_numRows)
            {

                if (!i)
                    m_row0WaitTime = x265_mdate();
                else if (i == m_numRows - 1)
                    m_allRowsAvailableTime = x265_mdate();
                processRowEncoder(i, m_tld[m_localTldIdx]);
            }

            // filter
            if (i >= m_filterRowDelay)
                m_frameFilter.processRow(i - m_filterRowDelay);
        }
    }

    for (uint32_t i = 0; i < m_numRows; i++)
    {
        m_frame->m_encData->m_frameStats.cntIntraNxN      += m_rows[i].rowStats.cntIntraNxN;
        m_frame->m_encData->m_frameStats.totalCu          += m_rows[i].rowStats.totalCu;
        m_frame->m_encData->m_frameStats.totalCtu         += m_rows[i].rowStats.totalCtu;
        m_frame->m_encData->m_frameStats.lumaDistortion   += m_rows[i].rowStats.lumaDistortion;
        m_frame->m_encData->m_frameStats.chromaDistortion += m_rows[i].rowStats.chromaDistortion;
        m_frame->m_encData->m_frameStats.psyEnergy        += m_rows[i].rowStats.psyEnergy;
        m_frame->m_encData->m_frameStats.lumaLevel        += m_rows[i].rowStats.lumaLevel;

        if (m_rows[i].rowStats.maxLumaLevel > m_frame->m_encData->m_frameStats.maxLumaLevel)
            m_frame->m_encData->m_frameStats.maxLumaLevel = m_rows[i].rowStats.maxLumaLevel;
        for (uint32_t depth = 0; depth <= g_maxCUDepth; depth++)
        {
            m_frame->m_encData->m_frameStats.cntSkipCu[depth] += m_rows[i].rowStats.cntSkipCu[depth];
            m_frame->m_encData->m_frameStats.cntMergeCu[depth] += m_rows[i].rowStats.cntMergeCu[depth];
            for (int m = 0; m < INTER_MODES; m++)
                m_frame->m_encData->m_frameStats.cuInterDistribution[depth][m] += m_rows[i].rowStats.cuInterDistribution[depth][m];
            for (int n = 0; n < INTRA_MODES; n++)
                m_frame->m_encData->m_frameStats.cuIntraDistribution[depth][n] += m_rows[i].rowStats.cuIntraDistribution[depth][n];
        }
    }
    m_frame->m_encData->m_frameStats.avgLumaDistortion   = (double)(m_frame->m_encData->m_frameStats.lumaDistortion) / m_frame->m_encData->m_frameStats.totalCtu;
    m_frame->m_encData->m_frameStats.avgChromaDistortion = (double)(m_frame->m_encData->m_frameStats.chromaDistortion) / m_frame->m_encData->m_frameStats.totalCtu;
    m_frame->m_encData->m_frameStats.avgPsyEnergy        = (double)(m_frame->m_encData->m_frameStats.psyEnergy) / m_frame->m_encData->m_frameStats.totalCtu;
    m_frame->m_encData->m_frameStats.avgLumaLevel        = m_frame->m_encData->m_frameStats.lumaLevel / m_frame->m_encData->m_frameStats.totalCtu;
    m_frame->m_encData->m_frameStats.percentIntraNxN     = (double)(m_frame->m_encData->m_frameStats.cntIntraNxN * 100) / m_frame->m_encData->m_frameStats.totalCu;
    for (uint32_t depth = 0; depth <= g_maxCUDepth; depth++)
    {
        m_frame->m_encData->m_frameStats.percentSkipCu[depth]  = (double)(m_frame->m_encData->m_frameStats.cntSkipCu[depth] * 100) / m_frame->m_encData->m_frameStats.totalCu;
        m_frame->m_encData->m_frameStats.percentMergeCu[depth] = (double)(m_frame->m_encData->m_frameStats.cntMergeCu[depth] * 100) / m_frame->m_encData->m_frameStats.totalCu;
        for (int n = 0; n < INTRA_MODES; n++)
            m_frame->m_encData->m_frameStats.percentIntraDistribution[depth][n] = (double)(m_frame->m_encData->m_frameStats.cuIntraDistribution[depth][n] * 100) / m_frame->m_encData->m_frameStats.totalCu;
        uint64_t cuInterRectCnt = 0; // sum of Nx2N, 2NxN counts
        cuInterRectCnt += m_frame->m_encData->m_frameStats.cuInterDistribution[depth][1] + m_frame->m_encData->m_frameStats.cuInterDistribution[depth][2];
        m_frame->m_encData->m_frameStats.percentInterDistribution[depth][0] = (double)(m_frame->m_encData->m_frameStats.cuInterDistribution[depth][0] * 100) / m_frame->m_encData->m_frameStats.totalCu;
        m_frame->m_encData->m_frameStats.percentInterDistribution[depth][1] = (double)(cuInterRectCnt * 100) / m_frame->m_encData->m_frameStats.totalCu;
        m_frame->m_encData->m_frameStats.percentInterDistribution[depth][2] = (double)(m_frame->m_encData->m_frameStats.cuInterDistribution[depth][3] * 100) / m_frame->m_encData->m_frameStats.totalCu;
    }

    m_bs.resetBits();
    m_entropyCoder.load(m_initSliceContext);
    m_entropyCoder.setBitstream(&m_bs);
    m_entropyCoder.codeSliceHeader(*slice, *m_frame->m_encData);

    // finish encode of each CTU row, only required when SAO is enabled
    if (m_param->bEnableSAO)
        encodeSlice();

    // serialize each row, record final lengths in slice header
    uint32_t maxStreamSize = m_nalList.serializeSubstreams(m_substreamSizes, numSubstreams, m_outStreams);

    // complete the slice header by writing WPP row-starts
    m_entropyCoder.setBitstream(&m_bs);
    if (slice->m_pps->bEntropyCodingSyncEnabled)
        m_entropyCoder.codeSliceHeaderWPPEntryPoints(*slice, m_substreamSizes, maxStreamSize);
    m_bs.writeByteAlignment();

    m_nalList.serialize(slice->m_nalUnitType, m_bs);


    if (m_param->decodedPictureHashSEI)
    {
        if (m_param->decodedPictureHashSEI == 1)
        {
            m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::MD5;
            for (int i = 0; i < 3; i++)
                MD5Final(&m_state[i], m_seiReconPictureDigest.m_digest[i]);
        }
        else if (m_param->decodedPictureHashSEI == 2)
        {
            m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::CRC;
            for (int i = 0; i < 3; i++)
                crcFinish(m_crc[i], m_seiReconPictureDigest.m_digest[i]);
        }
        else if (m_param->decodedPictureHashSEI == 3)
        {
            m_seiReconPictureDigest.m_method = SEIDecodedPictureHash::CHECKSUM;
            for (int i = 0; i < 3; i++)
                checksumFinish(m_checksum[i], m_seiReconPictureDigest.m_digest[i]);
        }

        m_bs.resetBits();
        m_seiReconPictureDigest.write(m_bs, *slice->m_sps);
        m_bs.writeByteAlignment();

        m_nalList.serialize(NAL_UNIT_SUFFIX_SEI, m_bs);
    }


    uint64_t bytes = 0;
    for (uint32_t i = 0; i < m_nalList.m_numNal; i++)
    {
        int type = m_nalList.m_nal[i].type;

        // exclude SEI
        if (type != NAL_UNIT_PREFIX_SEI && type != NAL_UNIT_SUFFIX_SEI)
        {
            bytes += m_nalList.m_nal[i].sizeBytes;
            // and exclude start code prefix
            bytes -= (!i || type == NAL_UNIT_SPS || type == NAL_UNIT_PPS) ? 4 : 3;
        }
    }
    m_accessUnitBits = bytes << 3;

    m_endCompressTime = x265_mdate();

    /* rateControlEnd may also block for earlier frames to call rateControlUpdateStats */
#ifndef Rc_Del
    if (m_top->m_rateControl->rateControlEnd(m_frame, m_accessUnitBits, &m_rce) < 0)
        m_top->m_aborted = true;
#endif


    int numTLD;;
        numTLD = 1;

    if (m_nr)
    {
        /* Accumulate NR statistics from all worker threads */
        for (int i = 0; i < numTLD; i++)
        {
			NoiseReduction* nr = &m_tld[i].analysis.m_quant.m_frameNr[0];
            for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
            {
                for (int coeff = 0; coeff < MAX_NUM_TR_COEFFS; coeff++)
                    m_nr->residualSum[cat][coeff] += nr->residualSum[cat][coeff];
            
                m_nr->count[cat] += nr->count[cat];
            }
        }

        noiseReductionUpdate();

        /* Copy updated NR coefficients back to all worker threads */
        for (int i = 0; i < numTLD; i++)
        {
			NoiseReduction* nr = &m_tld[i].analysis.m_quant.m_frameNr[0];
            memcpy(nr->offsetDenoise, m_nr->offsetDenoise, sizeof(uint16_t) * MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS);
            memset(nr->count, 0, sizeof(uint32_t) * MAX_NUM_TR_CATEGORIES);
            memset(nr->residualSum, 0, sizeof(uint32_t) * MAX_NUM_TR_CATEGORIES * MAX_NUM_TR_COEFFS);
        }
    }

#if DETAILED_CU_STATS
    /* Accumulate CU statistics from each worker thread, we could report
     * per-frame stats here, but currently we do not. */
    for (int i = 0; i < numTLD; i++)
        m_cuStats.accumulate(m_tld[i].analysis.m_stats[m_jpId]);
#endif

    m_endFrameTime = x265_mdate();
}

void FrameEncoder::encodeSlice()
{
    Slice* slice = m_frame->m_encData->m_slice;
    const uint32_t widthInLCUs = slice->m_sps->numCuInWidth;
    const uint32_t lastCUAddr = (slice->m_endCUAddr + NUM_4x4_PARTITIONS - 1) / NUM_4x4_PARTITIONS;
	const uint32_t numSubstreams = 1;

    SAOParam* saoParam = slice->m_sps->bUseSAO ? m_frame->m_encData->m_saoParam : NULL;
    for (uint32_t cuAddr = 0; cuAddr < lastCUAddr; cuAddr++)
    {
        uint32_t col = cuAddr % widthInLCUs;
        uint32_t lin = cuAddr / widthInLCUs;
        uint32_t subStrm = lin % numSubstreams;
        CUData* ctu = m_frame->m_encData->getPicCTU(cuAddr);

        m_entropyCoder.setBitstream(&m_outStreams[subStrm]);

        // Synchronize cabac probabilities with upper-right CTU if it's available and we're at the start of a line.

        if (saoParam)
        {
            if (saoParam->bSaoFlag[0] || saoParam->bSaoFlag[1])
            {
                int mergeLeft = col && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_LEFT;
                int mergeUp = lin && saoParam->ctuParam[0][cuAddr].mergeMode == SAO_MERGE_UP;
                if (col)
                    m_entropyCoder.codeSaoMerge(mergeLeft);
                if (lin && !mergeLeft)
                    m_entropyCoder.codeSaoMerge(mergeUp);
                if (!mergeLeft && !mergeUp)
                {
                    if (saoParam->bSaoFlag[0])
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[0][cuAddr], 0);
                    if (saoParam->bSaoFlag[1])
                    {
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[1][cuAddr], 1);
                        m_entropyCoder.codeSaoOffset(saoParam->ctuParam[2][cuAddr], 2);
                    }
                }
            }
            else
            {
                for (int i = 0; i < 3; i++)
                    saoParam->ctuParam[i][cuAddr].reset();
            }
        }

        // final coding (bitstream generation) for this CU
        m_entropyCoder.encodeCTU(*ctu, m_cuGeoms[m_ctuGeomMap[cuAddr]]);

    }

        m_entropyCoder.finishSlice();
}
// Called by worker threads
void FrameEncoder::processRowEncoder(int intRow, ThreadLocalData& tld)
{
    uint32_t row = (uint32_t)intRow;
    CTURow& curRow = m_rows[row];

    tld.analysis.m_param = m_param;

	Entropy& rowCoder = m_rows[0].rowGoOnCoder;
    FrameData& curEncData = *m_frame->m_encData;
    Slice *slice = curEncData.m_slice;

    const uint32_t numCols = m_numCols;
    const uint32_t lineStartCUAddr = row * numCols;


    while (curRow.completed < numCols)
    {
        ProfileScopeEvent(encodeCTU);

        uint32_t col = curRow.completed;
        const uint32_t cuAddr = lineStartCUAddr + col;
        CUData* ctu = curEncData.getPicCTU(cuAddr);
        ctu->initCTU(*m_frame, cuAddr, slice->m_sliceQp);

#ifndef Rc_Del
        if (bIsVbv)
        {
            if (!row)
            {
                curEncData.m_rowStat[row].diagQp = curEncData.m_avgQpRc;
                curEncData.m_rowStat[row].diagQpScale = x265_qp2qScale(curEncData.m_avgQpRc);
            }

            FrameData::RCStatCU& cuStat = curEncData.m_cuStat[cuAddr];
            if (row >= col && row && m_vbvResetTriggerRow != intRow)
                cuStat.baseQp = curEncData.m_cuStat[cuAddr - numCols + 1].baseQp;
            else
                cuStat.baseQp = curEncData.m_rowStat[row].diagQp;

            /* TODO: use defines from slicetype.h for lowres block size */
            uint32_t maxBlockCols = (m_frame->m_fencPic->m_picWidth + (16 - 1)) / 16;
            uint32_t maxBlockRows = (m_frame->m_fencPic->m_picHeight + (16 - 1)) / 16;
            uint32_t noOfBlocks = g_maxCUSize / 16;
            uint32_t block_y = (cuAddr / curEncData.m_slice->m_sps->numCuInWidth) * noOfBlocks;
            uint32_t block_x = (cuAddr * noOfBlocks) - block_y * curEncData.m_slice->m_sps->numCuInWidth;
            
            cuStat.vbvCost = 0;
            cuStat.intraVbvCost = 0;
            for (uint32_t h = 0; h < noOfBlocks && block_y < maxBlockRows; h++, block_y++)
            {
                uint32_t idx = block_x + (block_y * maxBlockCols);

                for (uint32_t w = 0; w < noOfBlocks && (block_x + w) < maxBlockCols; w++, idx++)
                {
                    cuStat.vbvCost += m_frame->m_lowres.lowresCostForRc[idx] & LOWRES_COST_MASK;
                    cuStat.intraVbvCost += m_frame->m_lowres.intraCost[idx];
                }
            }
        }
        else
#endif
            curEncData.m_cuStat[cuAddr].baseQp = curEncData.m_avgQpRc;


        // Does all the CU analysis, returns best top level mode decision
        Mode& best = tld.analysis.compressCTU(*ctu, *m_frame, m_cuGeoms[m_ctuGeomMap[cuAddr]], rowCoder);

        // take a sample of the current active worker count

        /* advance top-level row coder to include the context of this CTU.
         * if SAO is disabled, rowCoder writes the final CTU bitstream */
        rowCoder.encodeCTU(*ctu, m_cuGeoms[m_ctuGeomMap[cuAddr]]);


        // Completed CU processing
        curRow.completed++;

        FrameStats frameLog;
        curEncData.m_rowStat[row].sumQpAq += collectCTUStatistics(*ctu, &frameLog);

        // copy no. of intra, inter Cu cnt per row into frame stats for 2 pass
        curRow.rowStats.totalCtu++;
        curRow.rowStats.lumaDistortion   += best.lumaDistortion;
        curRow.rowStats.chromaDistortion += best.chromaDistortion;
        curRow.rowStats.psyEnergy        += best.psyEnergy;
        curRow.rowStats.cntIntraNxN      += frameLog.cntIntraNxN;
        curRow.rowStats.totalCu          += frameLog.totalCu;
        for (uint32_t depth = 0; depth <= g_maxCUDepth; depth++)
        {
            curRow.rowStats.cntSkipCu[depth] += frameLog.cntSkipCu[depth];
            curRow.rowStats.cntMergeCu[depth] += frameLog.cntMergeCu[depth];
            for (int m = 0; m < INTER_MODES; m++)
                curRow.rowStats.cuInterDistribution[depth][m] += frameLog.cuInterDistribution[depth][m];
            for (int n = 0; n < INTRA_MODES; n++)
                curRow.rowStats.cuIntraDistribution[depth][n] += frameLog.cuIntraDistribution[depth][n];
        }

        /* calculate maximum and average luma levels */
        uint32_t ctuLumaLevel = 0;
        uint32_t ctuNoOfPixels = best.fencYuv->m_size * best.fencYuv->m_size;
        for (uint32_t i = 0; i < ctuNoOfPixels; i++)
        {
            pixel p = best.fencYuv->m_buf[0][i];
            ctuLumaLevel += p;
            curRow.rowStats.maxLumaLevel = X265_MAX(p, curRow.rowStats.maxLumaLevel);
        }
        curRow.rowStats.lumaLevel += (double)(ctuLumaLevel) / ctuNoOfPixels;

        curEncData.m_cuStat[cuAddr].totalBits = best.totalBits;
        x265_emms();

#ifndef Rc_Del
        if (bIsVbv)
        {
            // Update encoded bits, satdCost, baseQP for each CU
            curEncData.m_rowStat[row].diagSatd      += curEncData.m_cuStat[cuAddr].vbvCost;
            curEncData.m_rowStat[row].diagIntraSatd += curEncData.m_cuStat[cuAddr].intraVbvCost;
            curEncData.m_rowStat[row].encodedBits   += curEncData.m_cuStat[cuAddr].totalBits;
            curEncData.m_rowStat[row].sumQpRc       += curEncData.m_cuStat[cuAddr].baseQp;
            curEncData.m_rowStat[row].numEncodedCUs = cuAddr;

            // If current block is at row diagonal checkpoint, call vbv ratecontrol.

            if (row == col && row)
            {
                double qpBase = curEncData.m_cuStat[cuAddr].baseQp;
                int reEncode = m_top->m_rateControl->rowDiagonalVbvRateControl(m_frame, row, &m_rce, qpBase);
                qpBase = x265_clip3((double)QP_MIN, (double)QP_MAX_MAX, qpBase);
                curEncData.m_rowStat[row].diagQp = qpBase;
                curEncData.m_rowStat[row].diagQpScale =  x265_qp2qScale(qpBase);

                if (reEncode < 0)
                {
                    x265_log(m_param, X265_LOG_DEBUG, "POC %d row %d - encode restart required for VBV, to %.2f from %.2f\n",
                             m_frame->m_poc, row, qpBase, curEncData.m_cuStat[cuAddr].baseQp);

                    // prevent the WaveFront::findJob() method from providing new jobs
                    m_vbvResetTriggerRow = row;
                    m_bAllRowsStop = true;

                    for (uint32_t r = m_numRows - 1; r >= row; r--)
                    {
                        CTURow& stopRow = m_rows[r];

                        if (r != row)
                        {
                            /* if row was active (ready to be run) clear active bit and bitmap bit for this row */
                            stopRow.lock.acquire();
                            while (stopRow.active)
                            {
                                if (dequeueRow(r * 2))
                                    stopRow.active = false;
                                else
                                {
                                    /* we must release the row lock to allow the thread to exit */
                                    stopRow.lock.release();
                                    GIVE_UP_TIME();
                                    stopRow.lock.acquire();
                                }
                            }
                            stopRow.lock.release();

                            bool bRowBusy = true;
                            do
                            {
                                stopRow.lock.acquire();
                                bRowBusy = stopRow.busy;
                                stopRow.lock.release();

                                if (bRowBusy)
                                {
                                    GIVE_UP_TIME();
                                }
                            }
                            while (bRowBusy);
                        }

                        m_outStreams[r].resetBits();
                        stopRow.completed = 0;
                        memset(&stopRow.rowStats, 0, sizeof(stopRow.rowStats));
                        curEncData.m_rowStat[r].numEncodedCUs = 0;
                        curEncData.m_rowStat[r].encodedBits = 0;
                        curEncData.m_rowStat[r].diagSatd = 0;
                        curEncData.m_rowStat[r].diagIntraSatd = 0;
                        curEncData.m_rowStat[r].sumQpRc = 0;
                        curEncData.m_rowStat[r].sumQpAq = 0;
                    }

                    m_bAllRowsStop = false;
                }
            }
        }
#endif

        /* SAO parameter estimation using non-deblocked pixels for CTU bottom and right boundary areas */
        if (m_param->bEnableSAO && m_param->bSaoNonDeblocked)
            m_frameFilter.m_sao.calcSaoStatsCu_BeforeDblk(m_frame, col, row);


    }

    /** this row of CTUs has been compressed **/

    /* If encoding with ABR, update update bits and complexity in rate control
     * after a number of rows so the next frame's rateControlStart has more
     * accurate data for estimation. At the start of the encode we update stats
     * after half the frame is encoded, but after this initial period we update
     * after refLagRows (the number of rows reference frames must have completed
     * before referencees may begin encoding) */
    uint32_t rowCount = 0;

    /* flush row bitstream (if WPP and no SAO) or flush frame if no WPP and no SAO */
	if (!m_param->bEnableSAO && (row == m_numRows - 1))
        rowCoder.finishSlice();


    tld.analysis.m_param = NULL;
    curRow.busy = false;

}

/* collect statistics about CU coding decisions, return total QP */
int FrameEncoder::collectCTUStatistics(const CUData& ctu, FrameStats* log)
{
    int totQP = 0;
    if (ctu.m_slice->m_sliceType == I_SLICE)
    {
        uint32_t depth = 0;
        for (uint32_t absPartIdx = 0; absPartIdx < ctu.m_numPartitions; absPartIdx += ctu.m_numPartitions >> (depth * 2))
        {
            depth = ctu.m_cuDepth[absPartIdx];

            log->totalCu++;
            log->cntIntra[depth]++;
            totQP += ctu.m_qp[absPartIdx] * (ctu.m_numPartitions >> (depth * 2));

            if (ctu.m_predMode[absPartIdx] == MODE_NONE)
            {
                log->totalCu--;
                log->cntIntra[depth]--;
            }
            else if (ctu.m_partSize[absPartIdx] != SIZE_2Nx2N)
            {
                /* TODO: log intra modes at absPartIdx +0 to +3 */
                X265_CHECK(ctu.m_log2CUSize[absPartIdx] == 3 && ctu.m_slice->m_sps->quadtreeTULog2MinSize < 3, "Intra NxN found at improbable depth\n");
                log->cntIntraNxN++;
                log->cntIntra[depth]--;
            }
            else if (ctu.m_lumaIntraDir[absPartIdx] > 1)
                log->cuIntraDistribution[depth][ANGULAR_MODE_ID]++;
            else
                log->cuIntraDistribution[depth][ctu.m_lumaIntraDir[absPartIdx]]++;
        }
    }

    return totQP;
}

/* DCT-domain noise reduction / adaptive deadzone from libavcodec */
void FrameEncoder::noiseReductionUpdate()
{
    static const uint32_t maxBlocksPerTrSize[4] = {1 << 18, 1 << 16, 1 << 14, 1 << 12};

    for (int cat = 0; cat < MAX_NUM_TR_CATEGORIES; cat++)
    {
        int trSize = cat & 3;
        int coefCount = 1 << ((trSize + 2) * 2);

        if (m_nr->count[cat] > maxBlocksPerTrSize[trSize])
        {
            for (int i = 0; i < coefCount; i++)
                m_nr->residualSum[cat][i] >>= 1;
            m_nr->count[cat] >>= 1;
        }

        int nrStrength = cat < 8 ? m_param->noiseReductionIntra : m_param->noiseReductionInter;
        uint64_t scaledCount = (uint64_t)nrStrength * m_nr->count[cat];

        for (int i = 0; i < coefCount; i++)
        {
            uint64_t value = scaledCount + m_nr->residualSum[cat][i] / 2;
            uint64_t denom = m_nr->residualSum[cat][i] + 1;
            m_nr->offsetDenoise[cat][i] = (uint16_t)(value / denom);
        }

        // Don't denoise DC coefficients
        m_nr->offsetDenoise[cat][0] = 0;
    }
}

Frame *FrameEncoder::getEncodedPicture(NALList& output)
{
    if (m_frame)
    {
        /* block here until worker thread completes */

        Frame *ret = m_frame;
        m_frame = NULL;
        output.takeContents(m_nalList);
        m_prevOutputTime = x265_mdate();
        return ret;
    }

    return NULL;
}
}
