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

#ifndef X265_INPUT_H
#define X265_INPUT_H

#define MIN_FRAME_WIDTH 64
#define MAX_FRAME_WIDTH 8192
#define MIN_FRAME_HEIGHT 64
#define MAX_FRAME_HEIGHT 4320
#define MIN_FRAME_RATE 1
#define MAX_FRAME_RATE 300

#include "common.h"

namespace X265_NS {
// private x265 namespace

struct InputFileInfo
{
    /* possibly user-supplied, possibly read from file header */
    int width;
    int height;
    int csp;
    int depth;
    int fpsNum;
    int fpsDenom;
    int sarWidth;
    int sarHeight;
    int frameCount;
    int timebaseNum;
    int timebaseDenom;

    /* user supplied */
    int skipFrames;
    const char *filename;
};

class InputFile
{
protected:

    virtual ~InputFile()  {}



#if (YUV_THREAD_DEL)
	int read_count;
	int write_count;
#endif

public:
    InputFile()           {}

#if (!REDANDENCY_DEL)
    static InputFile* open(InputFileInfo& info, bool bForceY4m);
#else
	static InputFile* open(InputFileInfo& info);
#endif

#if (!YUV_THREAD_DEL)
    virtual void startReader() = 0;
#endif
    virtual void release() = 0;

#if (!YUV_THREAD_DEL)
    virtual bool readPicture(x265_picture& pic) = 0;
#endif

	virtual bool lzx_readPicture(x265_picture&) = 0;

    virtual bool isEof() const = 0;

    virtual bool isFail() = 0;

    virtual const char *getName() const = 0;

    virtual int getWidth() const = 0;

    virtual int getHeight() const = 0;

	int setRead(int val) { read_count = val; }

	int setWrite(int val) { write_count = val; }

	int getRead() { return read_count; }

	int getWrite() { return write_count; }

	void incrRead() { read_count++; }

	void incrWrite() { write_count++; }
};
}

#endif // ifndef X265_INPUT_H
