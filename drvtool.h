/*
 * drvtool.h
 *
 * Copyright (c) 2019-2021, Peter Eriksson <pen@lysator.liu.se>
 *
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef DRVTOOL_H
#define DRVTOOL_H 1

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "argv.h"
#include "commands.h"
#include "basic.h"
#include "drives.h"
#include "strings.h"
#include "misc.h"
#include "opts.h"
#include "error.h"
#include "strval.h"

#include "rate.h"
#include "blocks.h"
#include "digest.h"
#include "transform.h"

struct command;

typedef struct config {
  int f_debug;
  int f_verbose;
  int f_force;
  int f_noupdate;
  int f_rwmode;
  int f_passes;
  int f_reverse;
  time_t f_timeout;
} CONFIG;

extern char *argv0;

extern OPTION global_options[];

/* Default configuration loaded from config file and global command line */
extern CONFIG default_config;

/* Per-command active configuration */
extern CONFIG config;

extern int
error(int rc, int ec, const char *msg, ...);


typedef unsigned char PATTERN[4];





#define TEST_READ     0x0001
#define TEST_WRITE    0x0002
#define TEST_VERIFY   0x0004
#define TEST_RESTORE  0x0008
#define TEST_PATTERN  0x0010
#define TEST_PURGE    0x0020
#define TEST_FLUSH    0x0040
#define TEST_TRIM     0x0080




typedef struct test {
  DRIVE *drive;
  
  RATE rate;           /* Transfer rate buffer        */

  unsigned int flags;  /* Test flags                  */
  
  int    passes;       /* Number of passes            */
  time_t timeout;      /* Max length of test run      */
  
  off_t b_size;        /* Size of each block          */
  off_t b_total;       /* Device size in # blocks     */
  
  off_t b_start;       /* First block #               */
  off_t b_length;      /* Number of blocks            */
  off_t b_end;         /* Last+1 block #              */

  TRANSFORM transform;

  struct {
    DIGEST data;
    struct {;
      struct {
	unsigned char data[DIGEST_BUFSIZE_MAX];
	size_t len;
      } first;
      struct {
	unsigned char data[DIGEST_BUFSIZE_MAX];
	size_t len;
      } last;
    } buffer;
  } digest;
  
  BLOCKS *blocks;
  
  struct timespec t0;  /* Start time                  */
  struct timespec t1;  /* Last update                 */
  
  unsigned char *obuf; /* Block buffer, original data */
  unsigned char *wbuf; /* Block buffer, data to write */
  unsigned char *rbuf; /* Block buffer, data read     */

  off_t last_b_pos;
  off_t last_pos;
} TEST;


#endif
