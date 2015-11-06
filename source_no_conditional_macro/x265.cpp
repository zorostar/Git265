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

#if _MSC_VER
#pragma warning(disable: 4127) // conditional expression is constant, yes I know
#endif

#include "x265.h"
#include "x265-extras.h"
#include "x265cli.h"

#include "common.h"
#include "input/input.h"
#include "output/output.h"
#include "output/reconplay.h"

#include "param.h"
#include "cpu.h"

#if HAVE_VLD
/* Visual Leak Detector */
#include <vld.h>
#endif

#include <signal.h>
#include <errno.h>
#include <fcntl.h>

#include <string>
#include <ostream>
#include <fstream>
#include <queue>


using namespace X265_NS;

/* Ctrl-C handler */
static volatile sig_atomic_t b_ctrl_c /* = 0 */;
static void sigint_handler(int)
{
    b_ctrl_c = 1;
}

struct CLIOptions
{
    InputFile* input;
    ReconFile* recon;
    OutputFile* output;
    FILE*       qpfile;
    const x265_api* api;
    x265_param* param;
    bool bProgress; //LZX< periodic progress reports from the CLI 控制输出每帧信息
    uint32_t seek;              // number of frames to skip from the beginning
    uint32_t framesToBeEncoded; // number of frames to encode
    uint64_t totalbytes;
    int64_t startTime;
    int64_t prevUpdateTime;

    /* in microseconds */
    static const int UPDATE_INTERVAL = 250000;

    CLIOptions()
    {
        input = NULL;
        recon = NULL;
        output = NULL;
        qpfile = NULL;
        api = NULL;
        param = NULL;
        framesToBeEncoded = seek = 0;
        totalbytes = 0;
        bProgress = true;
        startTime = x265_mdate();
        prevUpdateTime = 0;
    }

    void destroy();
    void printStatus(uint32_t frameNum);
    bool parse(int argc, char **argv);
	bool lh_parse(int argc, char **argv);
};

void CLIOptions::destroy()
{
    if (input)
        input->release();
    input = NULL;
    if (recon)
        recon->release();
    recon = NULL;
    if (output)
        output->release();
    output = NULL;
}

void CLIOptions::printStatus(uint32_t frameNum)
{
    char buf[200];
    int64_t time = x265_mdate();

    if (!bProgress || !frameNum || (prevUpdateTime && time - prevUpdateTime < UPDATE_INTERVAL))
        return;

    int64_t elapsed = time - startTime;
    double fps = elapsed > 0 ? frameNum * 1000000. / elapsed : 0;
    float bitrate = 0.008f * totalbytes * (param->fpsNum / param->fpsDenom) / ((float)frameNum);
    if (framesToBeEncoded)
    {
        int eta = (int)(elapsed * (framesToBeEncoded - frameNum) / ((int64_t)frameNum * 1000000));
        sprintf(buf, "x265 [%.1f%%] %d/%d frames, %.2f fps, %.2f kb/s, eta %d:%02d:%02d",
                100. * frameNum / framesToBeEncoded, frameNum, framesToBeEncoded, fps, bitrate,
                eta / 3600, (eta / 60) % 60, eta % 60);
    }
    else
        sprintf(buf, "x265 %d frames: %.2f fps, %.2f kb/s", frameNum, fps, bitrate);

    fprintf(stderr, "%s  \r", buf + 5);
    fflush(stderr); // needed in windows
    prevUpdateTime = time;
}

bool CLIOptions::parse(int argc, char **argv)
{
    bool bError = false;
    int bShowHelp = false;
    int inputBitDepth = 8;
    int outputBitDepth = 0;
    int reconFileBitDepth = 0;
    const char *inputfn = NULL;
    const char *reconfn = NULL;
    const char *outputfn = NULL;
    const char *preset = NULL;
    const char *tune = NULL;
    const char *profile = NULL;

    if (argc <= 1)
    {
        x265_log(NULL, X265_LOG_ERROR, "No input file. Run x265 --help for a list of options.\n");
        return true;
    }

    /* Presets are applied before all other options. */
    for (optind = 0;; )
    {
        int c = getopt_long(argc, argv, short_options, long_options, NULL);
        if (c == -1)
            break;
        else if (c == 'p')
            preset = optarg;
        else if (c == 't')
            tune = optarg;
        else if (c == 'D')
            outputBitDepth = atoi(optarg);
        else if (c == 'P')
            profile = optarg;
        else if (c == '?')
            bShowHelp = true;
    }

    if (!outputBitDepth && profile)
    {
        /* try to derive the output bit depth from the requested profile */
        if (strstr(profile, "10"))
            outputBitDepth = 10;
        else if (strstr(profile, "12"))
            outputBitDepth = 12;
        else
            outputBitDepth = 8;
    }

    api = x265_api_get(outputBitDepth);
    if (!api)
    {
        x265_log(NULL, X265_LOG_WARNING, "falling back to default bit-depth\n");
        api = x265_api_get(0);
    }

    param = api->param_alloc();
    if (!param)
    {
        x265_log(NULL, X265_LOG_ERROR, "param alloc failed\n");
        return true;
    }

    if (api->param_default_preset(param, preset, tune) < 0)
    {
        x265_log(NULL, X265_LOG_ERROR, "preset or tune unrecognized\n");
        return true;
    }

    if (bShowHelp)
    {
        printVersion(param, api);
        showHelp(param);
    }

    for (optind = 0;; )
    {
        int long_options_index = -1;
        int c = getopt_long(argc, argv, short_options, long_options, &long_options_index);
        if (c == -1)
            break;

        switch (c)
        {
        case 'h':
            printVersion(param, api);
            showHelp(param);
            break;

        case 'V':
            printVersion(param, api);
            x265_report_simd(param);
            exit(0);

        default:
            if (long_options_index < 0 && c > 0)
            {
                for (size_t i = 0; i < sizeof(long_options) / sizeof(long_options[0]); i++)
                {
                    if (long_options[i].val == c)
                    {
                        long_options_index = (int)i;
                        break;
                    }
                }

                if (long_options_index < 0)
                {
                    /* getopt_long might have already printed an error message */
                    if (c != 63)
                        x265_log(NULL, X265_LOG_WARNING, "internal error: short option '%c' has no long option\n", c);
                    return true;
                }
            }
            if (long_options_index < 0)
            {
                x265_log(NULL, X265_LOG_WARNING, "short option '%c' unrecognized\n", c);
                return true;
            }
#define OPT(longname) \
    else if (!strcmp(long_options[long_options_index].name, longname))
#define OPT2(name1, name2) \
    else if (!strcmp(long_options[long_options_index].name, name1) || \
             !strcmp(long_options[long_options_index].name, name2))

            if (0) ;
            OPT2("frame-skip", "seek") this->seek = (uint32_t)x265_atoi(optarg, bError);
            OPT("frames") this->framesToBeEncoded = (uint32_t)x265_atoi(optarg, bError);
            OPT("no-progress") this->bProgress = false;
            OPT("output") outputfn = optarg;
            OPT("input") inputfn = optarg;
            OPT("recon") reconfn = optarg;
            OPT("input-depth") inputBitDepth = (uint32_t)x265_atoi(optarg, bError);
            OPT("recon-depth") reconFileBitDepth = (uint32_t)x265_atoi(optarg, bError);
            OPT("profile") /* handled above */;
            OPT("preset")  /* handled above */;
            OPT("tune")    /* handled above */;
            OPT("output-depth")   /* handled above */;
            else
                bError |= !!api->param_parse(param, long_options[long_options_index].name, optarg);

            if (bError)
            {
                const char *name = long_options_index > 0 ? long_options[long_options_index].name : argv[optind - 2];
                x265_log(NULL, X265_LOG_ERROR, "invalid argument: %s = %s\n", name, optarg);
                return true;
            }
#undef OPT
        }
    }

    if (optind < argc && !inputfn)
        inputfn = argv[optind++];
    if (optind < argc && !outputfn)
        outputfn = argv[optind++];
    if (optind < argc)
    {
        x265_log(param, X265_LOG_WARNING, "extra unused command arguments given <%s>\n", argv[optind]);
        return true;
    }

    if (argc <= 1)
    {
        api->param_default(param);
        printVersion(param, api);
        showHelp(param);
    }

    if (!inputfn || !outputfn)
    {
        x265_log(param, X265_LOG_ERROR, "input or output file not specified, try --help for help\n");
        return true;
    }

    if (param->internalBitDepth != api->bit_depth)
    {
        x265_log(param, X265_LOG_ERROR, "Only bit depths of %d are supported in this build\n", api->bit_depth);
        return true;
    }

    InputFileInfo info;
    info.filename = inputfn;
    info.depth = inputBitDepth;
    info.csp = param->internalCsp;
    info.width = param->sourceWidth;
    info.height = param->sourceHeight;
    info.fpsNum = param->fpsNum;
    info.fpsDenom = param->fpsDenom;
    info.sarWidth = param->vui.sarWidth;
    info.sarHeight = param->vui.sarHeight;
    info.skipFrames = seek;
    info.frameCount = 0;
    getParamAspectRatio(param, info.sarWidth, info.sarHeight);

	this->input = InputFile::open(info);
    if (!this->input || this->input->isFail())
    {
        x265_log(param, X265_LOG_ERROR, "unable to open input file <%s>\n", inputfn);
        return true;
    }

#if (YUV_THREAD_DEL)
	this->input->setRead(0);
	this->input->setWrite(0);
#endif

    if (info.depth < 8 || info.depth > 16)
    {
        x265_log(param, X265_LOG_ERROR, "Input bit depth (%d) must be between 8 and 16\n", inputBitDepth);
        return true;
    }

    /* Unconditionally accept height/width/csp from file info */
    param->sourceWidth = info.width;
    param->sourceHeight = info.height;
    param->internalCsp = info.csp;

    /* Accept fps and sar from file info if not specified by user */
    if (param->fpsDenom == 0 || param->fpsNum == 0)
    {
        param->fpsDenom = info.fpsDenom;
        param->fpsNum = info.fpsNum;
    }
    if (!param->vui.aspectRatioIdc && info.sarWidth && info.sarHeight)
        setParamAspectRatio(param, info.sarWidth, info.sarHeight);
    if (this->framesToBeEncoded == 0 && info.frameCount > (int)seek)
        this->framesToBeEncoded = info.frameCount - seek;
    param->totalFrames = this->framesToBeEncoded;

    /* Force CFR until we have support for VFR */
    info.timebaseNum = param->fpsDenom;
    info.timebaseDenom = param->fpsNum;

    if (api->param_apply_profile(param, profile))
        return true;

    if (param->logLevel >= X265_LOG_INFO)
    {
        char buf[128];
        int p = sprintf(buf, "%dx%d fps %d/%d %sp%d", param->sourceWidth, param->sourceHeight,
                        param->fpsNum, param->fpsDenom, x265_source_csp_names[param->internalCsp], info.depth);

        int width, height;
        getParamAspectRatio(param, width, height);
        if (width && height)
            p += sprintf(buf + p, " sar %d:%d", width, height);

        if (framesToBeEncoded <= 0 || info.frameCount <= 0)
            strcpy(buf + p, " unknown frame count");
        else
            sprintf(buf + p, " frames %u - %d of %d", this->seek, this->seek + this->framesToBeEncoded - 1, info.frameCount);

        general_log(param, input->getName(), X265_LOG_INFO, "%s\n", buf);
    }


//#if (!REDANDENCY_DEL)
    if (reconfn)
    {
        if (reconFileBitDepth == 0)
            reconFileBitDepth = param->internalBitDepth;
        this->recon = ReconFile::open(reconfn, param->sourceWidth, param->sourceHeight, reconFileBitDepth,
                                      param->fpsNum, param->fpsDenom, param->internalCsp);
        if (this->recon->isFail())
        {
            x265_log(param, X265_LOG_WARNING, "unable to write reconstructed outputs file\n");
            this->recon->release();
            this->recon = 0;
        }
        else
            general_log(param, this->recon->getName(), X265_LOG_INFO,
                    "reconstructed images %dx%d fps %d/%d %s\n",
                    param->sourceWidth, param->sourceHeight, param->fpsNum, param->fpsDenom,
                    x265_source_csp_names[param->internalCsp]);
    }
//#endif

    this->output = OutputFile::open(outputfn, info);
    if (this->output->isFail())
    {
        x265_log(param, X265_LOG_ERROR, "failed to open output file <%s> for writing\n", outputfn);
        return true;
    }
    general_log(param, this->output->getName(), X265_LOG_INFO, "output file: %s\n", outputfn);
    return false;
}

void x265_param_set(x265_param* param)
{
	memset(param, 0, sizeof(x265_param));

	/* Applying default values to all elements in the param structure */
	param->cpuid = X265_NS::cpu_detect();
	param->bEnableWavefront = 0;//0 for no-wpp
	param->frameNumThreads = 1;//1 for μ￥??3ì

	param->logLevel = X265_LOG_INFO;
	param->csvfn = NULL;
	param->rc.lambdaFileName = NULL;
	param->bLogCuStats = 0;
	param->decodedPictureHashSEI = 0;

	/* Quality Measurement Metrics */
	param->bEnablePsnr = 0;
	param->bEnableSsim = 0;

	/* Source specifications */
	param->internalBitDepth = X265_DEPTH;
	param->internalCsp = X265_CSP_I420;

	param->levelIdc = 0;
	param->bHighTier = 0;
	param->interlaceMode = 0;
	param->bAnnexB = 1;
	param->bRepeatHeaders = 0;
	param->bEnableAccessUnitDelimiters = 0;
	param->bEmitHRDSEI = 0;
	param->bEmitInfoSEI = 1;//0 for no-info

	/* CU definitions */
	param->maxCUSize = 64;
	param->minCUSize = 8;
	param->tuQTMaxInterDepth = 3;//veryslow
	param->tuQTMaxIntraDepth = 3;//veryslow
	param->maxTUSize = 32;

	/* Coding Structure */
	param->keyframeMin = 0;
	param->keyframeMax = 1;//1 for è?I??
	param->bOpenGOP = 1;
	param->bframes = 0;//8 for veryslow, 0 for 2?′??úB??
	param->lookaheadDepth = 0;//40 for veryslow, 0 for no-rc-lookahead
	param->bFrameAdaptive = X265_B_ADAPT_TRELLIS;
	param->bBPyramid = 1;
	param->scenecutThreshold = 0; /* Magic number pulled in from x264 */ //0 for no-scenecut
	param->lookaheadSlices = 0;

	/* Intra Coding Tools */
	param->bEnableConstrainedIntra = 0;
	param->bEnableStrongIntraSmoothing = 1;
	param->bEnableFastIntra = 0;

	/* Inter Coding tools */
	param->searchMethod = X265_STAR_SEARCH;//veryslow
	param->subpelRefine = 4;//veryslow
	param->searchRange = 57;
	param->maxNumMergeCand = 4;//veryslow
	param->limitReferences = 0;
	param->bEnableWeightedPred = 1;
	param->bEnableWeightedBiPred = 1;//veryslow
	param->bEnableEarlySkip = 0;
	param->bEnableAMP = 1;//veryslow
	param->bEnableRectInter = 1;//veryslow
	param->rdLevel = 6;//veryslow
	param->rdoqLevel = 2;//veryslow
	param->bEnableSignHiding = 1;
	param->bEnableTransformSkip = 0;
	param->bEnableTSkipFast = 0;
	param->maxNumReferences = 5;//veryslow
	param->bEnableTemporalMvp = 1;

	/* Loop Filter */
	param->bEnableLoopFilter = 1;

	/* SAO Loop Filter */
	param->bEnableSAO = 1;
	param->bSaoNonDeblocked = 0;

	/* Coding Quality */
	param->cbQpOffset = 0;
	param->crQpOffset = 0;
	param->rdPenalty = 0;
	param->psyRd = 0.3;
	param->psyRdoq = 1.0;//veryslow
	param->bIntraInBFrames = 1;//veryslow
	param->bLossless = 0;
	param->bCULossless = 0;
	param->bEnableTemporalSubLayers = 0;

	/* Rate control options */
	param->rc.vbvMaxBitrate = 0;
	param->rc.vbvBufferSize = 0;
	param->rc.vbvBufferInit = 0.9;
	param->rc.rfConstant = 28;
	param->rc.bitrate = 0;
	param->rc.qCompress = 0.6;
	param->rc.ipFactor = 1;//IPBo??¨qp
	param->rc.pbFactor = 1;//IPBo??¨qp
	param->rc.qpStep = 4;
	param->rc.rateControlMode = X265_RC_CQP;//IPBo??¨qp
	param->rc.qp = 32;
	param->rc.aqMode = X265_AQ_VARIANCE;
	param->rc.qgSize = 32;
	param->rc.aqStrength = 1.0;
	param->rc.cuTree = 1;
	param->rc.rfConstantMax = 0;
	param->rc.rfConstantMin = 0;
	param->rc.bStatRead = 0;
	param->rc.bStatWrite = 0;
	param->rc.statFileName = NULL;
	param->rc.complexityBlur = 20;
	param->rc.qblur = 0.5;
	param->rc.zoneCount = 0;
	param->rc.zones = NULL;
	param->rc.bEnableSlowFirstPass = 0;
	param->rc.bStrictCbr = 0;

	/* Video Usability Information (VUI) */
	param->vui.aspectRatioIdc = 0;
	param->vui.sarWidth = 0;
	param->vui.sarHeight = 0;
	param->vui.bEnableOverscanAppropriateFlag = 0;
	param->vui.bEnableVideoSignalTypePresentFlag = 0;
	param->vui.videoFormat = 5;
	param->vui.bEnableVideoFullRangeFlag = 0;
	param->vui.bEnableColorDescriptionPresentFlag = 0;
	param->vui.colorPrimaries = 2;
	param->vui.transferCharacteristics = 2;
	param->vui.matrixCoeffs = 2;
	param->vui.bEnableChromaLocInfoPresentFlag = 0;
	param->vui.chromaSampleLocTypeTopField = 0;
	param->vui.chromaSampleLocTypeBottomField = 0;
	param->vui.bEnableDefaultDisplayWindowFlag = 0;
	param->vui.defDispWinLeftOffset = 0;
	param->vui.defDispWinRightOffset = 0;
	param->vui.defDispWinTopOffset = 0;
	param->vui.defDispWinBottomOffset = 0;
}

bool CLIOptions::lh_parse(int argc, char **argv)
{
	bool bError = false;
	int bShowHelp = false;
	int inputBitDepth = 8;
	int outputBitDepth = 0;
	int reconFileBitDepth = 0;
	const char *inputfn = "E:\\Test_Save\\11_18DSP\\input_yuv\\CIF\\News_352x288_30.yuv";
	const char *reconfn = "recon.yuv";
	const char *outputfn = "test.bin";
	const char *preset = "veryslow";
	const char *tune = NULL;
	const char *profile = NULL;

	if (!outputBitDepth && profile)
	{
		/* try to derive the output bit depth from the requested profile */
		if (strstr(profile, "10"))
			outputBitDepth = 10;
		else if (strstr(profile, "12"))
			outputBitDepth = 12;
		else
			outputBitDepth = 8;
	}

	api = x265_api_get(outputBitDepth);// api = &libapi;
	if (!api)
	{
		x265_log(NULL, X265_LOG_WARNING, "falling back to default bit-depth\n");
		api = x265_api_get(0);
	}

	param = api->param_alloc();
	if (!param)
	{
		x265_log(NULL, X265_LOG_ERROR, "param alloc failed\n");
		return true;
	}

	x265_param_set(param);

	//if (api->param_default_preset(param, preset, tune) < 0)
	//{
	//	x265_log(NULL, X265_LOG_ERROR, "preset or tune unrecognized\n");
	//	return true;
	//}

	if (bShowHelp)
	{
		printVersion(param, api);
		showHelp(param);
	}

	this->framesToBeEncoded = 50;//frames
	param->sourceWidth = 352;//input-res
	param->sourceHeight = 288;//input-res
	const char* fps = "30";//fps
	if (sscanf(fps, "%u/%u", &param->fpsNum, &param->fpsDenom) == 2)
		;
	else
	{
		float value = (float)atof(fps);
		if (value > 0 && value <= INT_MAX / 1000)
		{
			param->fpsNum = (int)(value * 1000 + .5);
			param->fpsDenom = 1000;
		}
		else
		{
			param->fpsNum = atoi(fps);
			param->fpsDenom = 1;
		}
	}
	//param->frameNumThreads = 1;//frame-threads
	//param->bEnableWavefront = 0;//wpp
	//param->scenecutThreshold = 0;//scenecut
	//param->keyframeMax = 1;//keyint
	//param->lookaheadDepth = 0;//rc-lookahead
	//param->bframes = 0;//bframes
	//param->bEmitInfoSEI = 0;//info
	//param->rc.ipFactor = 1;//ipratio
	//param->rc.pbFactor = 1;//pbratio
	//param->rc.qp = 32;//qp
	//param->rc.rateControlMode = X265_RC_CQP;

	if (!inputfn || !outputfn)
	{
		x265_log(param, X265_LOG_ERROR, "input or output file not specified, try --help for help\n");
		return true;
	}

	if (param->internalBitDepth != api->bit_depth)
	{
		x265_log(param, X265_LOG_ERROR, "Only bit depths of %d are supported in this build\n", api->bit_depth);
		return true;
	}

	InputFileInfo info;//ó?óúInputFileoíOutputFileμ?′′?¨
	info.filename = inputfn;
	info.depth = inputBitDepth;
	info.csp = param->internalCsp;
	info.width = param->sourceWidth;
	info.height = param->sourceHeight;
	info.fpsNum = param->fpsNum;
	info.fpsDenom = param->fpsDenom;
	info.sarWidth = param->vui.sarWidth;
	info.sarHeight = param->vui.sarHeight;
	info.skipFrames = seek;
	info.frameCount = 0;
	getParamAspectRatio(param, info.sarWidth, info.sarHeight);

	this->input = InputFile::open(info);
	if (!this->input || this->input->isFail())
	{
		x265_log(param, X265_LOG_ERROR, "unable to open input file <%s>\n", inputfn);
		return true;
	}

#if (YUV_THREAD_DEL)
	this->input->setRead(0);
	this->input->setWrite(0);
#endif

	if (info.depth < 8 || info.depth > 16)
	{
		x265_log(param, X265_LOG_ERROR, "Input bit depth (%d) must be between 8 and 16\n", inputBitDepth);
		return true;
	}

	/* Unconditionally accept height/width/csp from file info */
	param->sourceWidth = info.width;
	param->sourceHeight = info.height;
	param->internalCsp = info.csp;

	/* Accept fps and sar from file info if not specified by user */
	if (param->fpsDenom == 0 || param->fpsNum == 0)
	{
		param->fpsDenom = info.fpsDenom;
		param->fpsNum = info.fpsNum;
	}
	if (!param->vui.aspectRatioIdc && info.sarWidth && info.sarHeight)
		setParamAspectRatio(param, info.sarWidth, info.sarHeight);
	if (this->framesToBeEncoded == 0 && info.frameCount > (int)seek)
		this->framesToBeEncoded = info.frameCount - seek;
	param->totalFrames = this->framesToBeEncoded;

	/* Force CFR until we have support for VFR */
	info.timebaseNum = param->fpsDenom;
	info.timebaseDenom = param->fpsNum;

	if (api->param_apply_profile(param, profile))
		return true;

	if (param->logLevel >= X265_LOG_INFO)
	{
		char buf[128];
		int p = sprintf(buf, "%dx%d fps %d/%d %sp%d", param->sourceWidth, param->sourceHeight,
			param->fpsNum, param->fpsDenom, x265_source_csp_names[param->internalCsp], info.depth);

		int width, height;
		getParamAspectRatio(param, width, height);
		if (width && height)
			p += sprintf(buf + p, " sar %d:%d", width, height);

		if (framesToBeEncoded <= 0 || info.frameCount <= 0)
			strcpy(buf + p, " unknown frame count");
		else
			sprintf(buf + p, " frames %u - %d of %d", this->seek, this->seek + this->framesToBeEncoded - 1, info.frameCount);

		general_log(param, input->getName(), X265_LOG_INFO, "%s\n", buf);
	}


	if (reconfn)
	{
		if (reconFileBitDepth == 0)
			reconFileBitDepth = param->internalBitDepth;
		this->recon = ReconFile::open(reconfn, param->sourceWidth, param->sourceHeight, reconFileBitDepth,
			param->fpsNum, param->fpsDenom, param->internalCsp);
		if (this->recon->isFail())
		{
			x265_log(param, X265_LOG_WARNING, "unable to write reconstructed outputs file\n");
			this->recon->release();
			this->recon = 0;
		}
		else
			general_log(param, this->recon->getName(), X265_LOG_INFO,
			"reconstructed images %dx%d fps %d/%d %s\n",
			param->sourceWidth, param->sourceHeight, param->fpsNum, param->fpsDenom,
			x265_source_csp_names[param->internalCsp]);
	}

	this->output = OutputFile::open(outputfn, info);
	if (this->output->isFail())
	{
		x265_log(param, X265_LOG_ERROR, "failed to open output file <%s> for writing\n", outputfn);
		return true;
	}
	general_log(param, this->output->getName(), X265_LOG_INFO, "output file: %s\n", outputfn);
	return false;
}

/* CLI return codes:
 *
 * 0 - encode successful
 * 1 - unable to parse command line
 * 2 - unable to open encoder
 * 3 - unable to generate stream headers
 * 4 - encoder abort
 * 5 - unable to open csv file */

int main(int argc, char **argv)
{
#if HAVE_VLD
    // This uses Microsoft's proprietary WCHAR type, but this only builds on Windows to start with
    VLDSetReportOptions(VLD_OPT_REPORT_TO_DEBUGGER | VLD_OPT_REPORT_TO_FILE, L"x265_leaks.txt");
#endif


#if (!RECONPLAY_DEL)
    ReconPlay* reconPlay = NULL;
#endif
    CLIOptions cliopt;

	if (cliopt.lh_parse(argc, argv))
    {
        cliopt.destroy();
        if (cliopt.api)
            cliopt.api->param_free(cliopt.param);
        exit(1);
    }

    x265_param* param = cliopt.param;
    const x265_api* api = cliopt.api;

    /* This allows muxers to modify bitstream format */
    cliopt.output->setParam(param);

#if (!RECONPLAY_DEL)
    if (cliopt.reconPlayCmd)
        reconPlay = new ReconPlay(cliopt.reconPlayCmd, *param);
#endif

    /* note: we could try to acquire a different libx265 API here based on
     * the profile found during option parsing, but it must be done before
     * opening an encoder */

    x265_encoder *encoder = api->encoder_open(param);
    if (!encoder)
    {
        x265_log(param, X265_LOG_ERROR, "failed to open encoder\n");
        cliopt.destroy();
        api->param_free(param);
        api->cleanup();
        exit(2);
    }

    /* get the encoder parameters post-initialization */
    api->encoder_parameters(encoder, param);


    /* Control-C handler */
    if (signal(SIGINT, sigint_handler) == SIG_ERR)  //LZX< 改变程序对信号的处理方式，此处是Interactive attention signal信号一般是由ctrl C和Delete产生
        x265_log(param, X265_LOG_ERROR, "Unable to register CTRL+C handler: %s\n", strerror(errno));

    x265_picture pic_orig, pic_out;
    x265_picture *pic_in = &pic_orig;
    /* Allocate recon picture if analysisMode is enabled */
    std::priority_queue<int64_t>* pts_queue = cliopt.output->needPTS() ? new std::priority_queue<int64_t>() : NULL;
#if X265_ANALYSIS_ON && (!RECONPLAY_DEL) && (!REDANDENCY_DEL)
    x265_picture *pic_recon = (cliopt.recon || !!param->analysisMode || pts_queue || reconPlay || cliopt.csvLogLevel) ? &pic_out : NULL;
#else
	x265_picture *pic_recon = (cliopt.recon) ? &pic_out : NULL;
#endif
    uint32_t inFrameCount = 0;
    uint32_t outFrameCount = 0;
    x265_nal *p_nal;
    x265_stats stats;
    uint32_t nal;
    int16_t *errorBuf = NULL;
    int ret = 0;

    if (!param->bRepeatHeaders)
    {
        if (api->encoder_headers(encoder, &p_nal, &nal) < 0)
        {
            x265_log(param, X265_LOG_ERROR, "Failure generating stream headers\n");
            ret = 3;
            goto fail;
        }
        else
            cliopt.totalbytes += cliopt.output->writeHeaders(p_nal, nal);
    }

    api->picture_init(param, pic_in);


    // main encoder loop
    while (pic_in && !b_ctrl_c)
    {
        pic_orig.poc = inFrameCount;
        if (cliopt.framesToBeEncoded && inFrameCount >= cliopt.framesToBeEncoded)
            pic_in = NULL;
		else if (cliopt.input->lzx_readPicture(pic_orig))
            inFrameCount++;
        else
            pic_in = NULL;

        if (pic_in)
        {
            /* Overwrite PTS */
            pic_in->pts = pic_in->poc;
        }

        int numEncoded = api->encoder_encode(encoder, &p_nal, &nal, pic_in, pic_recon);//<0:error,=0:没有编码完成的帧,=1:存在完成了的帧
        if (numEncoded < 0)
        {
            b_ctrl_c = 1;
            ret = 4;
            break;
        }
#if (!RECONPLAY_DEL)
        if (reconPlay && numEncoded)
            reconPlay->writePicture(*pic_recon);
#endif

        outFrameCount += numEncoded;

        if (numEncoded && pic_recon && cliopt.recon)
            cliopt.recon->writePicture(pic_out);
        if (nal)
        {
            cliopt.totalbytes += cliopt.output->writeFrame(p_nal, nal, pic_out);
        }

        cliopt.printStatus(outFrameCount);
    }

    /* Flush the encoder */
    while (!b_ctrl_c)
    {
        int numEncoded = api->encoder_encode(encoder, &p_nal, &nal, NULL, pic_recon);
        if (numEncoded < 0)
        {
            ret = 4;
            break;
        }

#if (!RECONPLAY_DEL)
        if (reconPlay && numEncoded)
            reconPlay->writePicture(*pic_recon);
#endif

        outFrameCount += numEncoded;
        if (numEncoded && pic_recon && cliopt.recon)
            cliopt.recon->writePicture(pic_out);
        if (nal)
        {
            cliopt.totalbytes += cliopt.output->writeFrame(p_nal, nal, pic_out);
        }


        cliopt.printStatus(outFrameCount);

        if (!numEncoded)
            break;
    }

    /* clear progress report */
    if (cliopt.bProgress)
        fprintf(stderr, "%*s\r", 80, " ");

fail:
#if (!RECONPLAY_DEL)
    delete reconPlay;
#endif

    api->encoder_get_stats(encoder, &stats, sizeof(stats));
    api->encoder_close(encoder);

    int64_t second_largest_pts = 0;
    int64_t largest_pts = 0;
    cliopt.output->closeFile(largest_pts, second_largest_pts);

    if (b_ctrl_c)
        general_log(param, NULL, X265_LOG_INFO, "aborted at input frame %d, output frame %d\n",
                    cliopt.seek + inFrameCount, stats.encodedPictureCount);

    api->cleanup(); /* Free library singletons */

    cliopt.destroy();

    api->param_free(param);

    X265_FREE(errorBuf);


#if HAVE_VLD
    assert(VLDReportLeaks() == 0);
#endif

    return ret;
}
