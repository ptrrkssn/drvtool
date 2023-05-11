/*
 * misc.h
 *
 * Copyright (c) 2020, Peter Eriksson <pen@lysator.liu.se>
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

#ifndef MISC_H
#define MISC_H 1

#include <sys/types.h>
#include <sys/time.h>
#include <sys/stat.h>

#include "drvtool.h"


/*
 * Calculate the difference between two struct timespec, returns elapsed time i microseconds.
 * Also returns the elapsed time and a unit as a string.
 */
extern long
ts_delta(struct timespec *x,
	 const struct timespec *y,
	 long *res,
	 char **unit);



extern int
prompt_user(char *buf,
	    size_t bufsize,
	    int echo,
	    const char *prompt,
	    ...);

extern char *
strxdup(const char *s1,
	...);

extern int
str2time(const char *str,
	 time_t *vp);

extern char *
int2str(off_t b,
	char *buf,
	size_t bufsize,
	int b2f);

extern char *
time2str(time_t t,
	 char *buf,
	 size_t bufsize);


extern void                                                                          
spin(FILE *fp);


extern void
data_print(FILE *fp,
	   unsigned char *buf,
	   size_t size,
	   int mode);

extern int
prompt_yes(const char *msg,
	   ...);

#endif
