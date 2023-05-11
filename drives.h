/*
 * basic.c - Drive selection commands
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

#ifndef DRIVES_H
#define DRIVES_H 1

#include "commands.h"


typedef struct drive {
  char *name;
  char *path;
  
  int fd;
  
  struct stat sbuf;

  struct {
    unsigned int is_file : 1;
    unsigned int is_ssd  : 1;
    unsigned int is_ro   : 1;
  } flags;
  
  off_t media_size;
  off_t sector_size;
  off_t sectors;

  off_t stripe_size;
  off_t stripe_offset;
  off_t stripes;
  
  unsigned int fw_sectors;
  unsigned int fw_heads;
  unsigned int tracks;
  
  off_t front_reserved;
  
  char *physical_path;
  char *provider_name;
  char *ident;
  char *vendor;
  char *product;
  char *revision;
  
  struct {
    char *path;
  } cam;

  struct drive *next;
} DRIVE;

extern DRIVE *selected_drive;

extern int
drives_load(int rwmode);

extern DRIVE *
drive_open(const char *name, int rwmode);

extern int
drive_close(DRIVE *dp);

extern int
drive_flush(DRIVE *dp);

extern int
drive_delete(DRIVE *dp,
	     off_t off,
	     off_t len);

extern int
drive_blocks2chs(off_t b,
		 off_t *cv,
		 off_t *hv,
		 off_t *sv,
		 DRIVE *dp);

extern void
drive_print(FILE *fp,
	    int idx,
	    DRIVE *dp,
	    int verbose);

extern DRIVE *
drive_select(const char *name);

extern void
drives_print(int f_idx);


extern COMMAND *drive_commands[];

#endif
