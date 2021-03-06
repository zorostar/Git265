/*****************************************************************************
 * Copyright (C) 2013 x265 project
 *
 * Authors: Shin Yee <shinyee@multicorewareinc.com>
 *          Min Chen <chenm003@163.com>
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

#ifndef X265_FRAMEENCODER_H
#define X265_FRAMEENCODER_H

#include "common.h"
#include "wavefront.h"
#include "bitstream.h"
#include "frame.h"
#include "picyuv.h"
#include "md5.h"

#include "analysis.h"
#include "sao.h"

#include "entropy.h"
#include "framefilter.h"
#include "ratecontrol.h"
#include "reference.h"
#include "nal.h"

namespace X265_NS {
// private x265 namespace

class ThreadPool;
class Encoder;

#define ANGULAR_MODE_ID 2
#define AMP_ID 3

struct StatisticLog
{
    uint64_t cntInter[4];
    uint64_t cntIntra[4];
    uint64_t cuInterDistribution[4][INTER_MODES];
    uint64_t cuIntraDistribution[4][INTRA_MODES];
    uint64_t cntIntraNxN;
    uint64_t cntSkipCu[4];
    uint64_t cntTotalCu[4];
    uint64_t totalCu;

    StatisticLog()
    {
        memset(this, 0, sizeof(StatisticLog));
    }
};

/* manages the state of encoding one row of CTU blocks.  When
 * WPP is active, several rows will be simultaneously encoded. */
struct CTURow
{
    Entropy           bufferedEntropy;  /* store CTU2 context for next row CTU0 */
    Entropy           rowGoOnCoder;     /* store context between CTUs, code bitstream if !SAO */

    FrameStats        rowStats;

    /* Threading variables */

    /* This lock must be acquired when reading or writing m_active or m_busy */

    /* row is ready to run, has no neighbor dependencies. The row may have
     * external dependencies (reference frame pixels) that prevent it from being
     * processed, so it may stay with m_active=true for some time before it is
     * encoded by a worker thread. */
    volatile bool     active;

    /* row is being processed by a worker thread.  This flag is only true when a
     * worker thread is within the context of FrameEncoder::processRow(). This
     * flag is used to detect multiple possible wavefront problems. */
    volatile bool     busy;

    /* count of completed CUs in this row */
    volatile uint32_t completed;

    /* called at the start of each frame to initialize state */
    void init(Entropy& initContext)
    {
        active = false;
        busy = false;
        completed = 0;
        memset(&rowStats, 0, sizeof(rowStats));
        rowGoOnCoder.load(initContext);
    }
};

// Manages the wave-front processing of a single encoding frame
#if ((!THREAD_DEL) && WPP_ON && (!THREAD_POOL_DEL))
class FrameEncoder : public WaveFront, public Thread
#else
class FrameEncoder
#endif
{
public:

    FrameEncoder();

    virtual ~FrameEncoder() {}

    virtual bool init(Encoder *top, int numRows, int numCols);

    void destroy();

    /* triggers encode of a new frame by the worker thread */
	bool lzx_statcompress(Frame* curFrame);

    /* blocks until worker thread is done, returns access unit */
    Frame *getEncodedPicture(NALList& list);

	int                      m_localTldIdx;

    uint32_t                 m_numRows;
    uint32_t                 m_numCols;
    uint32_t                 m_filterRowDelay;
    uint32_t                 m_filterRowDelayCus;
    uint32_t                 m_refLagRows;

    CTURow*                  m_rows;
#ifndef Rc_Del
    RateControlEntry         m_rce;
#endif
    SEIDecodedPictureHash    m_seiReconPictureDigest;

    uint64_t                 m_SSDY;
    uint64_t                 m_SSDU;
    uint64_t                 m_SSDV;
    double                   m_ssim;
    uint64_t                 m_accessUnitBits;
    uint32_t                 m_ssimCnt;
    MD5Context               m_state[3];
    uint32_t                 m_crc[3];
    uint32_t                 m_checksum[3];

    int64_t                  m_startCompressTime;        // timestamp when frame encoder is given a frame
    int64_t                  m_row0WaitTime;             // timestamp when row 0 is allowed to start
    int64_t                  m_allRowsAvailableTime;     // timestamp when all reference dependencies are resolved
    int64_t                  m_endCompressTime;          // timestamp after all CTUs are compressed
    int64_t                  m_endFrameTime;             // timestamp after RCEnd, NR updates, etc
#if (!REDANDENCY_DEL)    
	int64_t                  m_stallStartTime;           // timestamp when worker count becomes 0
#endif    
	int64_t                  m_prevOutputTime;           // timestamp when prev frame was retrieved by API thread
    int64_t                  m_slicetypeWaitTime;        // total elapsed time waiting for decided frame

#if DETAILED_CU_STATS
    CUStats                  m_cuStats;
#endif

    Encoder*                 m_top;
    x265_param*              m_param;
    Frame*                   m_frame;
    NoiseReduction*          m_nr;
    ThreadLocalData*         m_tld; /* for --no-wpp */
    Bitstream*               m_outStreams;
    uint32_t*                m_substreamSizes;

    CUGeom*                  m_cuGeoms;
    uint32_t*                m_ctuGeomMap;

    Bitstream                m_bs;
    MotionReference          m_mref[2][MAX_NUM_REF + 1];
    Entropy                  m_entropyCoder;
    Entropy                  m_initSliceContext;
    FrameFilter              m_frameFilter;
    NALList                  m_nalList;


protected:

    bool initializeGeoms();

    /* analyze / compress frame, can be run in parallel within reference constraints */
    void compressFrame();

    /* called by compressFrame to generate final per-row bitstreams */
    void encodeSlice();
    int  collectCTUStatistics(const CUData& ctu, FrameStats* frameLog);
    void noiseReductionUpdate();

    /* Called by WaveFront::findJob() */
    virtual void processRowEncoder(int row, ThreadLocalData& tld);



};
}

#endif // ifndef X265_FRAMEENCODER_H
