/*
** drvtool.c - A tool to analyze/test/purge hard drives
**
** Copyright (c) 2020, Peter Eriksson <pen@lysator.liu.se>
** All rights reserved.
** 
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
** 
** 1. Redistributions of source code must retain the above copyright notice, this
**    list of conditions and the following disclaimer.
** 
** 2. Redistributions in binary form must reproduce the above copyright notice,
**    this list of conditions and the following disclaimer in the documentation
**    and/or other materials provided with the distribution.
** 
** 3. Neither the name of the copyright holder nor the names of its
**    contributors may be used to endorse or promote products derived from
**    this software without specific prior written permission.
** 
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
** FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
** DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
** SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
** CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
** OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
** 
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <sys/types.h>
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <camlib.h>
#include <zlib.h>

#include "drvtool.h"


char *argv0 = "drvtool";
char *version = "1.4";

long f_update_freq = 500;

int f_yes        = 0;
int f_ibase      = 0;
int f_flush      = 0;
int f_trim     = 0;
int f_update     = 0;
int f_verbose    = 0;
int f_reverse    = 0;
int f_debug      = 0;

int f_passes     = 2; /* loops, 0 = loop continously (forever, or until timeout*/
time_t f_timeout = 0; /* seconds, 0 = no time limit */

BLOCKS *d_blocks = NULL;
off_t max_blocks = 0;


PATTERN sel_pattern =
  { 0x00, 0x00, 0x00, 0x00 };


#define NUM_TEST_PATTERNS 6

PATTERN test_patterns[NUM_TEST_PATTERNS] =
  {
   { 0xc6, 0xde, 0xc6, 0xde }, /* 1100 0110  1101 1110  ... */
   { 0x6d, 0xb6, 0xdb, 0x6d }, /* 0110 1101  1011 0110  1101  */
   { 0x00, 0x00, 0x00, 0x00 }, /* 0000 0000  0000 0000  ... */
   { 0xff, 0xff, 0xff, 0xff }, /* 1111 1111  1111 1111  ... */
   { 0xaa, 0xaa, 0xaa, 0xaa }, /* 1010 1010  1010 1010  ... */
   { 0x55, 0x55, 0x55, 0x55 }  /* 0101 0101  0101 0101  ... */
  };


#define NUM_PURGE_PATTERNS 2

PATTERN purge_patterns[NUM_PURGE_PATTERNS] =
  {
   { 0xaa, 0xaa, 0xaa, 0xaa }, /* 10101010... */
   { 0x55, 0x55, 0x55, 0x55 }  /* 01010101... */
  };


DRIVE *drives = NULL;

void
buf_xor(unsigned char *buf,
	size_t bufsize,
	int b) {
  while (bufsize-- > 0)
    *buf++ ^= b;
}

void
buf_ror(unsigned char *buf,
	size_t bufsize,
	int b) {
  unsigned char i, m, t;

  m = 0;
  for (i = 0; i < b; i++) {
    m <<= 1;
    m |= 0x01;
  }
  
  while (bufsize-- > 0) {
    t = (*buf & m) << (8-b);
    *buf >>= b;
    *buf++ |= t;
  }
}

void
buf_rol(unsigned char *buf,
	size_t bufsize,
	int b) {
  unsigned char i, m, t;

  m = 0;
  for (i = 0; i < b; i++) {
    m >>= 1;
    m |= 0x80;
  }
  
  while (bufsize-- > 0) {
    t = (*buf & m) >> (8-b);
    *buf <<= b;
    *buf++ |= t;
  }
}


char *
strxdup(const char *s1,
	...) {
  va_list ap;
  char *buf;
  const char *cp;
  size_t buflen;


  if (!s1)
    return NULL;
  
  buflen = strlen(s1)+1;
  va_start(ap, s1);
  while ((cp = va_arg(ap, const char *)) != NULL)
    buflen += strlen(cp);
  va_end(ap);

  buf = malloc(buflen);
  if (!buf)
    return NULL;
  
  strcpy(buf, s1);
  
  va_start(ap, s1);
  while ((cp = va_arg(ap, char *)) != NULL)
    strcat(buf, cp);

  va_end(ap);
  return buf;
}



void
pattern_fill(unsigned char *buf,
	     size_t bufsize,
	     unsigned char *pat,
	     size_t patsize) {
  size_t pos, len;


  pos = 0;
  while ((len = bufsize - pos) > 0) {
    if (len > patsize)
      len = patsize;
    
    memcpy(buf+pos, pat, len);
    pos += len;
  }
}



BLOCKS *
blocks_create(off_t s) {
  off_t i;
  BLOCKS *bp;
  

  bp = malloc(sizeof(*bp));
  if (!bp)
    return NULL;

  bp->s = s;
  if (s) {
    bp->v = calloc(bp->s, sizeof(bp->v[0]));
    if (bp->v == NULL) {
      free(bp);
      return NULL;
    }
    
    for (i = 0; i < bp->s; i++)
      bp->v[i] = i;
  } else
    bp->v = NULL;
  
  return bp;
}


off_t
orand(off_t size) {
  off_t r;

  r = (((off_t) lrand48() << 31) | ((off_t) lrand48() << 31)) | lrand48();
  return r % size;
}


static inline void
off_swap(off_t *a,
	 off_t *b) {
#if 0
  off_t t = *a;
  
  *a = *b;
  *b = t;
#else
  *a ^= *b;
  *b ^= *a;
  *a ^= *b;
#endif
}


/* 
 * Shuffle blocks using Fisher-Yates shuffe algorithm.
 * Ensures each block is visited atleast once.
 */
void
blocks_shuffle(BLOCKS *bp) {
  off_t i;

  for (i = bp->s-1; i > 0; i--) {
    off_t j = orand(i+1);
    
    off_swap(&bp->v[i], &bp->v[j]);
  }
}


int
drive_cam_sysctl(DRIVE *dp,
	       const char *vname,
	       void *vp,
	       size_t *vs) {
  char nbuf[128], *pname, *cp;
  int unit;

  
  if (!dp || !dp->name)
    return -1;
  
  pname = dp->name;
  cp = pname+strlen(pname)-1;
  while (cp >= pname && isdigit(*cp))
    --cp;
  ++cp;
  
  if (sscanf(cp, "%d", &unit) != 1)
    return -1;

  snprintf(nbuf, sizeof(nbuf), "kern.cam.%.*s.%d.%s", (int) (cp-pname), pname, unit, vname);
  return sysctlbyname(nbuf, vp, vs, NULL, 0);
}

int
drive_geom_sysctl(DRIVE *dp,
		const char *vname,
		void *vp,
		size_t *vs) {
  char nbuf[128], *pname, *cp;
  int unit;

  
  if (!dp || !dp->name)
    return -1;
  
  pname = dp->name;
  cp = pname+strlen(pname)-1;
  while (cp >= pname && isdigit(*cp))
    --cp;
  ++cp;
  
  if (sscanf(cp, "%d", &unit) != 1)
    return -1;

  snprintf(nbuf, sizeof(nbuf), "kern.geom.disk.%s.%s", pname, vname);
  return sysctlbyname(nbuf, vp, vs, NULL, 0);
}


/*
 * Calculate the difference between two struct timespec, returns 
 * elapsed time. Also returns the elapsed time (as a long) and 
 * with a unit as a string.
 */
double
ts_delta(const struct timespec *x,
	 const struct timespec *y,
	 long *res,
	 char **unit) {
  struct timespec xb, r;

  
  if (!x)
    clock_gettime(CLOCK_REALTIME, &xb);
  else
    xb = *x;
  
  /* Avoid overflow of r.tv_nsec */
  if (xb.tv_nsec < y->tv_nsec) {
    xb.tv_nsec += 1000000000L;
    xb.tv_sec  -= 1;
  }
  
  r.tv_sec  = xb.tv_sec - y->tv_sec;
  r.tv_nsec = xb.tv_nsec - y->tv_nsec;
  
  if (unit && res) {
    if (r.tv_sec >= 600) {
      /* More than 10 minutes -> return minutes */
      *unit = "m";
      *res = r.tv_sec / 60;
    } else if (r.tv_sec >= 10) {
      /* More than 10 seconds - return seconds */
      *unit = "s";
      *res = r.tv_sec;
    } else if (r.tv_sec == 0) {
      if (r.tv_nsec <= 10000) {
	/* Less than 10us - return nanoseconds */
	*unit = "ns";
	*res = r.tv_nsec;
      } else if (r.tv_nsec <= 10000000) {
	/* Less than 10ms - return microseconds */
	*unit = "µs";
	*res = r.tv_nsec / 1000;
      } else {
	*unit = "ms";
	*res = r.tv_nsec / 1000000;
      }
    } else {
      *unit = "ms";
      *res = r.tv_sec * 1000 + r.tv_nsec / 1000000;
    }
  }
  
  /* Microseconds */
  return (double) r.tv_sec + r.tv_nsec / 1000000000.0;
}

void
rate_init(RATE *rp) {
  rp->f = 0;
  rp->p = 0;
}


off_t
rate_get(RATE *rp) {
  unsigned int i;
  off_t sum;

  
  if (!rp->f)
    return 0;
  
  sum = 0;
  for (i = 0; i < rp->f; i++)
    sum += rp->dv[i];

  return sum / rp->f;
}


void
rate_update(RATE *rp,
	    off_t delta) {
  rp->dv[rp->p++] = delta;
  rp->p %= RATE_BUF_SIZE;
  
  if (rp->f < RATE_BUF_SIZE)
    rp->f++;
}



int
test_set_bsize(TEST *tp,
	       off_t bsize) {
  DRIVE *dev;
  off_t ns, tb, o_start, o_length;

  
  if (!tp || !tp->drive)
    return -1;

  dev = tp->drive;

  o_start = tp->b_start * tp->b_size;
  o_length = tp->b_length * tp->b_size;
  
  tb = dev->media_size / bsize;
  
  if (!dev->flags.is_file) {
    /* Make sure block size is >= sector size */
    if (bsize < dev->sector_size)
      return -1;
    
    /* Make sure block size is a multiple of the sector size */
    ns = bsize / dev->sector_size;
    if (ns * dev->sector_size != bsize)
      return -1;
    
    /* Make sure block size and total media size is compatible */
    if (tb * bsize != dev->media_size)
      return -1;
  }
  
  tp->b_size  = bsize;
  tp->b_total = tb;
  
  tp->b_start  = o_start / bsize;
  tp->b_length = o_length ? o_length / bsize : tb;
  tp->b_end    = tp->b_start + tp->b_length;
  
  return 0;
  
}

int
test_set_start(TEST *tp,
	       off_t start) {
  if (!tp || !tp->drive)
    return -1;

  if (start < 0 || start >= tp->b_total)
    return -1;
  
  tp->b_start = start;
  
  if (tp->b_length + tp->b_start > tp->b_total)
    tp->b_length = tp->b_total - tp->b_start;
  
  tp->b_end = tp->b_start + tp->b_length;

  return 0;
  
}

int
test_set_length(TEST *tp,
		off_t length) {
  if (!tp || !tp->drive)
    return -1;

  /* Negative length -> Count from end of disk */
  if (length < 0)
    length += tp->b_total;
  
  if (length < 0 || length+tp->b_start > tp->b_total)
    return -1;
  
  tp->b_length = length;
  tp->b_end    = tp->b_start + tp->b_length;

  return 0;
}


int
digest_init(DIGEST *dp,
	    int type) {
  dp->type = type;

  
  memset(&dp->ctx, 0, sizeof(dp->ctx));
  
  switch (dp->type) {
  case DIGEST_NONE:
    break;
    
  case DIGEST_ADLER32:
    dp->ctx.crc32 = adler32_z(0L, NULL, 0);
    break;
    
  case DIGEST_CRC32:
    dp->ctx.adler32 = crc32_z(0L, NULL, 0);
    break;
    
  case DIGEST_MD5:
    MD5Init(&dp->ctx.md5);
    break;
    
  case DIGEST_SHA256:
    SHA256_Init(&dp->ctx.sha256);
    break;
    
  case DIGEST_SHA384:
    SHA384_Init(&dp->ctx.sha384);
    break;
    
  case DIGEST_SHA512:
    SHA512_Init(&dp->ctx.sha512);
    break;

  default:
    return -1;
  }

  return 0;
}


int
digest_update(DIGEST *dp,
	      unsigned char *buf,
	      size_t bufsize) {

  if (!dp)
    return -1;
  
  switch (dp->type) {
  case DIGEST_ADLER32:
    dp->ctx.adler32 = adler32_z(dp->ctx.adler32, buf, bufsize);
    break;
    
  case DIGEST_CRC32:
    dp->ctx.adler32 = crc32_z(dp->ctx.crc32, buf, bufsize);
    break;
    
  case DIGEST_MD5:
    MD5Update(&dp->ctx.md5, buf, bufsize);
    break;
    
  case DIGEST_SHA256:
    SHA256_Update(&dp->ctx.sha256, buf, bufsize);
    break;
    
  case DIGEST_SHA384:
    SHA384_Update(&dp->ctx.sha384, buf, bufsize);
    break;
    
  case DIGEST_SHA512:
    SHA512_Update(&dp->ctx.sha512, buf, bufsize);
    break;

  default:
    return -1;
  }

  return 0;
}

int
test_init(TEST *tp,
	  DRIVE *dp) {
  tp->drive = dp;
  
  rate_init(&tp->rate);

  tp->flags   = 0;
  tp->passes  = f_passes;
  tp->timeout = f_timeout;

  tp->digest_type = DIGEST_NONE;
  digest_init(&tp->digest, tp->digest_type);
  
  tp->transform = 0;
  
  tp->b_size  = dp->stripe_size;
  tp->b_total = dp->media_size / tp->b_size;
  
  tp->b_start = 0;
  
  tp->b_length = tp->b_total - tp->b_start;
  tp->b_end    = tp->b_start + tp->b_length;

  tp->obuf = malloc(tp->b_size);
  if (!tp->obuf)
    return -1;
  
  tp->wbuf = malloc(tp->b_size);
  if (!tp->wbuf)
    return -1;
  
  tp->rbuf = malloc(tp->b_size);
  if (!tp->rbuf)
    return -1;

  tp->blocks = d_blocks;

  tp->last_b_pos = -1;
  tp->last_pos = -1;
  
  return 0;
}




int
str2off(const char *str,
	off_t *vp,
	int b1024,
	const char **rest) {
  off_t base = 1000;
  char c, i;
  int rc;
  
  
  if (f_ibase || b1024)
    base = 1024;
  
  c = i = 0;

  if (str[0] == '0' && str[1] == 'x') {
    rc = sscanf(str+2, "%lx%c%c", vp, &c, &i);
    if (rc < 1)
      return rc;
    if (rest) {
      *rest = str+2;
      while (**rest && isxdigit(**rest))
	++*rest;
    }
  } else {
    rc = sscanf(str, "%ld%c%c", vp, &c, &i);
    if (rc < 1)
      return rc;
    if (rest) {
      *rest = str;
      if (**rest == '-')
	++*rest;
      
      while (**rest && isdigit(**rest))
	++*rest;
    }
  }
  
  if (c && i == 'i')
    base = 1024;
  
  switch (toupper(c)) {
  case 'K':
    *vp *= base;
    break;
    
  case 'M':
    *vp *= base*base;
    break;
    
  case 'G':
    *vp *= base*base*base;
    break;
    
  case 'T':
    *vp *= base*base*base*base;
    break;
    
  case 'P':
    *vp *= base*base*base*base;
    break;

  default:
    return 1;
  }
  
  if (rest) {
    if (i == 'i')
      ++*rest;
    ++*rest;
  }
  
  return 1;
}


int
str2int(const char *str,
	int *vp) {
  off_t ov;
  int rc;

  rc = str2off(str, &ov, 0, NULL);
  if (rc < 1)
    return rc;

  if (ov < INT_MIN || ov > INT_MAX) {
    errno = ERANGE;
    return -1;
  }
  
  *vp = ov;
  return rc;
}


int
str2bytes(const char *str,
	  off_t *vp,
	  int b1024,
	  TEST *tp) {
  const char *rest = NULL;
  DRIVE *dp = tp->drive;
  int rc;
  off_t v;
  
  
  rc = str2off(str, vp, b1024, &rest);
  if (rc < 1)
    return rc;

  if (!rest)
    return rc;
  
  switch (toupper(*rest)) {
  case 0:
    break;

  case '%':
    *vp = (tp->b_total * tp->b_size * 100) / *vp;
    break;
    
  case 'B':
    *vp *= tp->b_size;
    break;
    
  case 'N':
    *vp *= dp->stripe_size;
    break;
    
  case 'S':
    *vp *= dp->sector_size;
    break;
    
  default:
    return -1;
  }

  if (!dp->flags.is_file) {
    /* Make sure it's a multiple of the sector size */
    v = *vp / dp->sector_size;
    if (v * dp->sector_size != *vp)
      return -1;
  }

  return 1;
}

int
str2blocks(const char *str,
	   off_t *vp,
	   int b1024,
	   TEST *tp) {
  const char *rest = NULL;
  int rc;
  
  
  rc = str2off(str, vp, b1024, &rest);
  if (rc < 1)
    return rc;

  if (!rest)
    return rc;

  switch (toupper(*rest)) {
  case 0:
    break;

  case '%':
    *vp = (tp->b_total * *vp) / 100;
    break;
    
  default:
    return -1;
  }
  
  return 1;
}


int
str2time(const char *str,
	time_t *vp) {
  char c;
  int rc;
  
  c = 0;
  rc = sscanf(str, "%ld%c", vp, &c);
  if (rc < 1)
    return rc;

  switch (c) {
  case 0:
  case 's':
    break;
    
  case 'm':
    *vp *= 60;
    break;

  case 'h':
    *vp *= 60*60;
    break;

  case 'd':
    *vp *= 60*60*24;
    break;

  case 'w':
    *vp *= 60*60*24*7;
    break;

  case 'M':
    *vp *= 60*60*24*30;
    break;

  case 'Y':
    *vp *= 60*60*24*365;
    break;

  default:
    return -1;
  }
  
  return 1;
}


char *
int2str(off_t b,
	char *buf,
	size_t bufsize,
	int b2f) {
  int base = 1000;
  off_t t;
  
  
  if (f_ibase)
    b2f = 1;
  if (b2f)
    base = 1024;

  if (b == 0) {
    strcpy(buf, "0");
    return buf;
  }
  
  t = b/base;
  if (llabs(b) < 10000 && (t*base != b)) {
    snprintf(buf, bufsize, "%ld", b);
    return buf;
  }
  
  b /= base;
  t = b/base;
  if (llabs(b) < 10000 && (t*base != b)) {
    snprintf(buf, bufsize, "%ldK%s", b, b2f == 1 ? "i" : "");
    return buf;
  }
  
  b /= base;
  t = b/base;
  if (llabs(b) < 10000 && (t*base != b)) {
    snprintf(buf, bufsize, "%ldM%s", b, b2f == 1 ? "i" : "");
    return buf;
  }
  
  b /= base;
  t = b/base;
  if (llabs(b) < 10000 && (t*base != b)) {
    snprintf(buf, bufsize, "%ldG%s", b, b2f == 1 ? "i" : "");
    return buf;
  }
  
  b /= base;
  t = b/base;
  if (llabs(b) < 10000 && (t*base != b)) {
    snprintf(buf, bufsize, "%ldT%s", b, b2f == 1 ? "i" : "");
    return buf;
  }
  
  snprintf(buf, bufsize, "%ldP%s", b, b2f == 1 ? "i" : "");
  return buf;
}


char *
time2str(time_t t,
	 char *buf,
	 size_t bufsize) {
  if (labs(t) < 120) {
    snprintf(buf, bufsize, "%lds", t);
    return buf;
  }

  t /= 60;
  if (labs(t) < 120) {
    snprintf(buf, bufsize, "%ldm", t);
    return buf;
  }

  t /= 60;
  if (labs(t) < 48) {
    snprintf(buf, bufsize, "%ldh", t);
    return buf;
  }
  
  t /= 24;
  if (labs(t) < 14) {
    snprintf(buf, bufsize, "%ldD", t);
    return buf;
  }
  
  if (labs(t) < 60) {
    t /= 7;
    snprintf(buf, bufsize, "%ldW", t);
    return buf;
  }

  if (labs(t) < 365*2) {
    t /= 30;
    snprintf(buf, bufsize, "%ldM", t);
    return buf;
  }

  t /= 365;
  snprintf(buf, bufsize, "%ldY", t);
  return buf;
}




int
drive_flush(DRIVE *dp) {
  int rc;


  rc = ioctl(dp->fd, DIOCGFLUSH);
  return rc;
}


int
drive_delete(DRIVE *dp,
	   off_t off,
	   off_t len) {
  int rc;
  off_t iov[2];

  iov[0] = off;
  iov[1] = len;

  rc = ioctl(dp->fd, DIOCGDELETE, iov);
  return rc;
}

int
drive_close(DRIVE *dp) {
  int rc;

  
  if (dp->path) {
    free(dp->path);
    dp->path = NULL;
  }

  if (dp->provider_name) {
    free(dp->provider_name);
    dp->provider_name = NULL;
  }

  if (dp->ident) {
    free(dp->ident);
    dp->ident = NULL;
  }

  if (dp->physical_path) {
    free(dp->physical_path);
    dp->physical_path = NULL;
  }

  rc = 0;
  if (dp->fd != -1) {
    rc = close(dp->fd);
    dp->fd = -1;
  }
  
  free(dp);
  return rc;
}

void
drive_print(FILE *fp,
	  int idx,
	  DRIVE *dp) {
  char b1[80], b2[80], b3[80];
  

  if (idx)
    fprintf(fp, "  %4d.\t", idx);
  else
    fprintf(fp, "  ");
  
  fprintf(fp, "%-6s", dp->name);
  fprintf(fp, " : %-20s", dp->ident ? dp->ident : "");

  int2str(dp->stripe_size, b1, sizeof(b1), 2);
  strcat(b1, "n");
  if (dp->sector_size != dp->stripe_size) {
    strcat(b1, "/");
    int2str(dp->sector_size, b2, sizeof(b2), 2);
    strcat(b1, b2);
    strcat(b1, "e");
  }
  
  fprintf(fp, " : %6sB : %6s sectors @ %-12s :",
	  int2str(dp->media_size, b2, sizeof(b2), 0),
	  int2str(dp->sectors, b3, sizeof(b3), 0),
	  b1);
  
  if (dp->flags.is_file)
    fprintf(fp, " FILE");
  
  if (dp->flags.is_ssd)
    fprintf(fp, " SSD");
  
  putc('\n', fp);

  if (f_verbose) {
    if (idx)
      putc('\t', fp);
    else {
      putc(' ', fp);
      putc(' ', fp);
    }
    
    if (dp->cam.path)
      fprintf(fp, "%s ", dp->cam.path);
  
    if (dp->physical_path)
      fprintf(fp, "{%s} ", dp->physical_path);

    if (f_verbose > 1)
      if (dp->fw_heads && dp->tracks && dp->fw_sectors)
	fprintf(fp, "{%u heads, %u tracks/head, %u sectors/track} ",
		dp->fw_heads,
		dp->tracks,
		dp->fw_sectors);
    
    putc('\n', fp);
  }
}




DRIVE *
drive_open(const char *name) {
  char pnbuf[MAXPATHLEN];
  char idbuf[DISK_IDENT_SIZE];
  int s_errno;
  DRIVE *dp;
  int is_rotating = -1;
  size_t isrs = sizeof(is_rotating);
  struct cam_device *cam;
  
  dp = malloc(sizeof(*dp));
  if (!dp)
    return NULL;

  dp->name = strdup(name);
  if (!dp->name)
    goto Fail;
  
  dp->fd = -1;
  if (*name == '/' || *name == '.') {
    dp->path = strdup(name);

    if (*name == '/') {
      dp->name = strrchr(name, '/');
      dp->name++;
    }
  }
  else
    dp->path = strxdup("/dev/", name, NULL);

  drive_cam_sysctl(dp, "rotating", &is_rotating, &isrs);
  dp->flags.is_ssd = (is_rotating ? 0 : 1);

  dp->fd = open(dp->path, (f_update ? O_RDWR : O_RDONLY), 0);
  if (dp->fd < 0)
    goto Fail;

  if (fstat(dp->fd, &dp->sbuf) < 0)
    goto Fail;

  dp->flags.is_file = S_ISREG(dp->sbuf.st_mode) ? 1 : 0;
  
  dp->media_size = 0;
  (void) ioctl(dp->fd, DIOCGMEDIASIZE, &dp->media_size);
  if (dp->media_size == 0)
    dp->media_size = dp->sbuf.st_size;
  
  dp->sector_size = 0;
  (void) ioctl(dp->fd, DIOCGSECTORSIZE, &dp->sector_size);
  if (dp->sector_size == 0)
    dp->sector_size = dp->sbuf.st_blksize;
  
  dp->sectors = 0;
  if (dp->sector_size)
    dp->sectors = dp->media_size / dp->sector_size;
  
  dp->stripe_size = 0;
  (void) ioctl(dp->fd, DIOCGSTRIPESIZE, &dp->stripe_size);

  if (dp->stripe_size == 0)
    dp->stripe_size = dp->sector_size;
  
  dp->stripe_offset = 0;
  (void) ioctl(dp->fd, DIOCGSTRIPEOFFSET, &dp->stripe_offset);

  dp->stripes = 0;
  if (dp->stripe_size)
    dp->stripes = dp->media_size / dp->stripe_size;
  
  dp->fw_sectors = 0;
  (void) ioctl(dp->fd, DIOCGFWSECTORS, &dp->fw_sectors);
  
  dp->fw_heads = 0;
  (void) ioctl(dp->fd, DIOCGFWHEADS, &dp->fw_heads);

  dp->tracks = 0;
  if (dp->fw_heads && dp->fw_sectors && dp->stripe_size)
    dp->tracks = dp->media_size / (dp->fw_heads * dp->fw_sectors * dp->stripe_size);
  
  dp->front_reserved = 0;
  (void) ioctl(dp->fd, DIOCGFRONTSTUFF, &dp->front_reserved);
  
  dp->provider_name = NULL;
  pnbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGPROVIDERNAME, pnbuf) == 0) {
    dp->provider_name = strndup(pnbuf, sizeof(pnbuf));
    if (!dp->provider_name)
      goto Fail;
    free(dp->name);
    dp->name = strdup(dp->provider_name);
    if (!dp->name)
      goto Fail;
  }
  
  dp->ident = NULL;
  idbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGIDENT, idbuf) == 0) {
    dp->ident = strndup(idbuf, sizeof(idbuf));
    if (!dp->ident)
      goto Fail;
  }
  
  dp->physical_path = NULL;
  pnbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGPHYSPATH, pnbuf) == 0) {
    dp->physical_path = strndup(pnbuf, sizeof(pnbuf)); 
    if (!dp->physical_path)
      goto Fail;
  }

  cam = cam_open_device(dp->path, O_RDWR);
  if (cam) {
    char buf[256];
    
    snprintf(buf, sizeof(buf), "/%s%u/bus%u/scbus%u/target%u/lun%lu",
	     cam->sim_name, cam->sim_unit_number, cam->bus_id,
	     cam->path_id, cam->target_id, cam->target_lun);
    dp->cam.path = strdup(buf);
    cam_close_device(cam);
  }

  dp->next = drives;
  drives = dp;
  
  return dp;

 Fail:
  s_errno = errno;
  drive_close(dp);
  errno = s_errno;
  return NULL;
}




int
drives_load(void) {
  size_t bsize;
  char *buf;
  char *cp;
  int n, rc = 0;

  
  if (sysctlbyname("kern.disks", NULL, &bsize, NULL, 0) < 0) {
    if (f_debug)
      fprintf(stderr, "%s: Error: Unable to get number of drives from kernel: %s\n",
	      argv0, strerror(errno));
    return -1;
  }

  buf = malloc(bsize);
  if (!buf) {
    if (f_debug)
      fprintf(stderr, "%s: Error: malloc(%ld bytes) failed: %s\n",
	      argv0, bsize, strerror(errno));
    return -1;
  }
  
  if (sysctlbyname("kern.disks", buf, &bsize, NULL, 0) < 0) {
    if (f_debug)
      fprintf(stderr, "%s: Error: Unable to get list of drives from kernel: %s\n",
	      argv0, strerror(errno));
    return -1;
  }

  n = 0;
  while ((cp = strsep(&buf, " ")) != NULL) {
    if (drive_open(cp) == NULL) {
      if (f_debug)
	fprintf(stderr, "%s: Error: %s: Opening drive: %s\n", argv0, cp, strerror(errno));
      rc = -1;
    } else
      ++n;
  }
  
  return rc < 0 ? rc : n;
}


void
drives_print(int f_idx) {
  DRIVE *dp = drives;
  int i = 0;
  
  while (dp) {
    drive_print(stdout, f_idx ? ++i : 0, dp);
    dp = dp->next;
  }
}

char *
strtrim(char *buf) {
  int i;

  
  if (!buf)
    return NULL;
  
  while (isspace(*buf))
    ++buf;

  i = strlen(buf);
  while (i > 0 && isspace(buf[i-1]))
    --i;
  buf[i] = '\0';
  return buf;
}



DRIVE *
drive_select(const char *name) {
  DRIVE *dp;
  int i, d;
  char buf[80];


  do {
    if (!name) {
      fprintf(stderr, "Specify drive (enter it's number or name): ");
      if (fgets(buf, sizeof(buf), stdin) == NULL)
	exit(1);
      name = strtrim(buf);
    }
    
    if (sscanf(name, "%d", &d) == 1) {
      i = 0;
      for (dp = drives; dp && ++i != d; dp = dp->next)
	;
    } else {
      for (dp = drives; dp && strcmp(name, dp->name) != 0; dp = dp->next)
	;
    }

    name = NULL;
  } while (!dp);

  return dp;
}



void
test_pstatus(FILE *fp,
	     TEST *tp,
	     struct timespec *now,
	     off_t bno,
	     off_t real_bno) {
  long time_left;
  off_t bytes_done, bytes_left, bytes_total, bytes_rate_avg, bytes_done_pct;
  char ibuf[256], tbuf[256], pbuf[256];


  bytes_done  = bno * tp->b_size;
  bytes_total = tp->b_length * tp->b_size;
  bytes_left  = bytes_total - bytes_done;
  
  bytes_done_pct = bytes_total ? (100 * bytes_done) / bytes_total : 0;
  
  bytes_rate_avg = rate_get(&tp->rate);

  if (tp->timeout)
    time_left = tp->timeout - ts_delta(now, &tp->t0, NULL, NULL);
  else
    time_left = bytes_rate_avg ? bytes_left / bytes_rate_avg : 0;
  
  fprintf(fp, "    %15ld",
	  bno);
  if (f_verbose)
    fprintf(fp, " : %15ld",
	    real_bno);
  fprintf(fp, " : %7sB @ %5sB/s : %4ld%% done",
	  int2str(bytes_done, pbuf, sizeof(pbuf), 0),
	  int2str(bytes_rate_avg, ibuf, sizeof(ibuf), 0),
	  bytes_done_pct);
  
  if (time_left)
    fprintf(fp, " (%s left)",
	    time2str(time_left, tbuf, sizeof(tbuf)));

  fprintf(fp, "                     ");
}


volatile int abort_f = 0;


void
sigint_handler(int sig) {
  signal(SIGINT, sigint_handler);
  abort_f = 1;
}
  

#if 0
int
do_drive_delete_all(DRIVE *dp) {
  int rc;
  off_t i;

  
  for (i = 0; i < dp->media_size; i += dp->delete_size) {
    rc = drive_delete(dp, i, dp->delete_size);
    if (rc < 0) {
      fprintf(stderr, "%s: Error: %s: Drive Delete %ld bytes @ %ld failed: %s\n",
	      argv0, dp->path, dp->delete_size, i, strerror(errno));
      exit(1);
    }
  }

  return 0;
}
#endif


/* Return number of equal bytes from the start */
size_t
mem_verify(unsigned char *a,
	   unsigned char *b,
	   size_t s) {
  size_t p = 0;


  while (p < s && a[p] == b[p])
    ++p;

  return p;
}


/* Run a test sequence */
int
test_seq(TEST *tp) {
  DRIVE *dp = tp->drive;
  struct timespec now;
  ssize_t rc;
  off_t b, b_pos, pos, bytes_done;
  int nw, n_write;
  double td;
  

  clock_gettime(CLOCK_REALTIME, &now);
  tp->t0 = now;
  tp->t1 = now;
  
  bytes_done = 0;

  for (b = 0; b < tp->b_length && (!tp->timeout || ts_delta(&now, &tp->t0, NULL, NULL) <= tp->timeout); ++b) {

    if (tp->blocks) {
      if (tp->blocks->s) {
	off_t b_offset = b / tp->blocks->s;
	b_pos = tp->blocks->v[b % tp->blocks->s] + b_offset * tp->blocks->s;
      } else
	b_pos = orand(tp->b_length);
    } else
      b_pos = b;

    if (f_reverse)
      b_pos = tp->b_length - b_pos - 1;
    
    b_pos += tp->b_start;
    pos = b_pos * tp->b_size;

    if (tp->flags & TEST_READ) {
      /* Read original data */
      
      rc = pread(dp->fd, tp->obuf, tp->b_size, pos);
      if (rc != tp->b_size) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ block %ld (offset %ld): %s\n",
		  argv0,
		  dp->path,
		  tp->b_size,
		  b_pos,
		  pos,
		  strerror(errno));
	  exit(1);
	} else {
	  fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) @ block %ld (offset %ld)\n",
		  argv0,
		  dp->path,
		  rc,
		  b_pos,
		  pos);
	  exit(1);
	}
      }
    }
    
    if (tp->flags & TEST_RESTORE)
      /* Make sure we ignore Ctrl-C at this time in order to prevent data corruption */
      signal(SIGINT, sigint_handler);
    
    n_write = 1;
    if (tp->flags & TEST_PURGE)
      n_write = NUM_PURGE_PATTERNS+1;
    
    for (nw = 0; nw < n_write; nw++) {
      if (tp->flags & TEST_PURGE) {
	if (nw < NUM_PURGE_PATTERNS)
	  pattern_fill(tp->wbuf, tp->b_size, purge_patterns[nw], sizeof(purge_patterns[nw]));
	else {
	  int i;
	  
	  for (i = 0; i < tp->b_size; i++)
	    tp->wbuf[i] = (unsigned char) lrand48();
	}
      }

      switch (tp->transform) {
      case TRANSFORM_XOR:
	buf_xor(tp->wbuf, tp->b_size, tp->txd.xor);
	break;
      case TRANSFORM_ROR:
	buf_ror(tp->wbuf, tp->b_size, tp->txd.ror);
	break;
      case TRANSFORM_ROL:
	buf_rol(tp->wbuf, tp->b_size, tp->txd.rol);
	break;
      }
      
      if (tp->flags & TEST_WRITE) {
	/* Write new test data */
	
	rc = pwrite(dp->fd, tp->wbuf, tp->b_size, pos);
	if (rc != tp->b_size) {
	  if (rc < 0) {
	    fprintf(stderr, "%s: Error: %s: Writing %ld bytes @ block %ld (offset %ld): %s\n",
		    argv0,
		    dp->path,
		    tp->b_size,
		    b_pos,
		    pos,
		    strerror(errno));
	    exit(1);
	  } else {
	    fprintf(stderr, "%s: Error: %s: Short write (%ld bytes) @ block %ld (offset %ld)\n",
		    argv0,
		    dp->path,
		    rc,
		    b_pos,
		    pos);
	    exit(1);
	  }
	}
      }
      
      if (f_flush || (tp->flags & TEST_FLUSH)) {
	/* Force device to write out data to permanent storage */
	
	rc = drive_flush(dp);
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: FLUSH failed @ block %ld (offset %ld): %s\n",
		  argv0, dp->path, b_pos, pos, strerror(errno));
	  exit(1);
	}
      }
      
      if (tp->flags & TEST_VERIFY) {
	/* Read back block to verify the just written data */
	
	rc = pread(dp->fd, tp->rbuf, tp->b_size, pos);
	if (rc != tp->b_size) {
	  if (rc < 0) {
	    fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ block %ld (offset %ld): %s\n",
		    argv0,
		    dp->path,
		    tp->b_size,
		    b_pos,
		    pos,
		    strerror(errno));
	    exit(1);
	  } else {
	    fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) @ block %ld (offset %ld)\n",
		    argv0,
		    dp->path,
		    rc,
		    b_pos,
		    pos);
	    exit(1);
	  }
	}
	
	rc = mem_verify(tp->wbuf, tp->rbuf, tp->b_size);
	if (rc != tp->b_size) {
	  fprintf(stderr, "%s: Error: %s: Verify failed @ block %ld+%ld (offset %ld)\n",
		  argv0,
		  dp->path,
		  b_pos,
		  rc,
		  pos+rc);
	  exit(1);
	}
      }
    }
    
    if (f_trim && (tp->flags & TEST_TRIM)) {
      /* TRIM block from (SSD) device */
      
      rc = drive_delete(dp, pos, tp->b_size);
      if (rc < 0) {
	fprintf(stderr, "%s: Error: %s: TRIM failed @ block %ld (offset %ld): %s\n",
		argv0, dp->path, b_pos, pos, strerror(errno));
	exit(1);
      }
    }
    
    if (tp->flags & TEST_RESTORE) {
      /* Restore the original block */
      
      rc = pwrite(dp->fd, tp->obuf, tp->b_size, pos);
      if (rc != tp->b_size) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: Unable to restore original @ block %ld (offset %ld): %s\n",
		  argv0,
		  dp->path,
		  b_pos,
		  pos,
		  strerror(errno));
	  exit(1);
	} else {
	  fprintf(stderr, "%s: Fatal: %s: Short write (%ld bytes) while restoring original @ block %ld (offset %ld)\n",
		  argv0,
		  dp->path,
		  rc,
		  b_pos,
		  pos);
	  exit(1);
	}
      }	
      
      /* Read back block to verify the just written data */
      rc = pread(dp->fd, tp->rbuf, tp->b_size, pos);
      if (rc != tp->b_size) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ block %ld (offset %ld): %s\n",
		  argv0,
		  dp->path,
		  tp->b_size,
		  b_pos,
		  pos,
		  strerror(errno));
	  exit(1);
	} else {
	  fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) @ block %ld (offset %ld)\n",
		  argv0,
		  dp->path,
		  rc,
		  b_pos,
		  pos);
	  exit(1);
	}
      }
      
      rc = mem_verify(tp->obuf, tp->rbuf, tp->b_size);
      if (rc != tp->b_size) {
	fprintf(stderr, "%s: Error: %s: Original restore verify failed @ block %ld+%ld (offset %ld)\n",
		argv0,
		dp->path,
		b_pos,
		rc,
		pos+rc);
	exit(1);
      }
    }
    
    /* Now we can allow signals again */
    signal(SIGINT, SIG_DFL);
    if (abort_f) {
      fprintf(stderr, "\n*** Aborted ***\n");
      exit(1);
    }
      
    if (tp->digest.type) {
      unsigned char *tbuf = tp->obuf;
      
      if (tp->flags & TEST_WRITE)
	tbuf = tp->wbuf;
      
      if (tp->flags & TEST_VERIFY)
	tbuf = tp->rbuf;

      digest_update(&tp->digest, tbuf, tp->b_size);
    }

    bytes_done += tp->b_size;
    clock_gettime(CLOCK_REALTIME, &now);
    
    td = ts_delta(&now, &tp->t1, NULL, NULL);
    if (td*1000 > f_update_freq) {
      if (td) {
	off_t bytes_rate;
	
	bytes_rate = bytes_done / td;
	rate_update(&tp->rate, bytes_rate);

	bytes_done = 0;
	tp->t1 = now;
      }

      test_pstatus(stderr, tp, &now, b, b_pos);
      fputc('\r', stderr);
    }
    
    tp->last_b_pos = b_pos;
    tp->last_pos   = pos;
  }
  
  test_pstatus(stderr, tp, &now, b, b_pos);
  fputc('\n', stderr);
  return 0;
}





void                                                                          
spin(FILE *fp) {
  static char *dials = "-\\|/";
  static time_t t_last = 0;
  time_t t;

  
  time(&t);
  if (t_last != t) {
    putc(dials[(unsigned int) t_last & 3], fp);
    putc('\b', fp);
    t_last = t;
  }
}





void
buf_print(FILE *fp,
	 unsigned char *buf,
	 size_t size,
	 int mode) {
  size_t i, j;

  
  i = 0;
  while (i < size) {
    if (mode) {
      putc(' ', fp);
      putc(' ', fp);
      if (mode > 1)
	fprintf(fp, "%08lX : ", i);
    }
    
    for (j = 0; j < 16 && i+j < size; j++) {
      if (mode) {
	if (j % 8 == 0)
	  putc(' ', fp);
	if (j)
	  putc(' ', fp);
      }
      fprintf(fp, "%02X", buf[i+j]);
    }

    if (mode > 2) {
      putc(' ', fp);
      putc(':', fp);
      putc(' ', fp);
      for (j = 0; j < 16 && i+j < size; j++) {
	if (j % 8 == 0)
	  putc(' ', fp);
	if (j)
	  putc(' ', fp);
      	putc(isprint(buf[i+j]) ? buf[i+j] : '?', fp);
      }
    }

    i += j;
    if (mode)
      putc('\n', fp);
  }
}


void
act_help(FILE *fp) {
  fprintf(fp, "COMMANDS:\n");
  fprintf(fp, "  drive            Select new drive\n");
  fprintf(fp, "  current          Describe current drive\n");
  fprintf(fp, "  config           Display test configuration\n");
  
  fprintf(fp, "  read             Read-only test [doesn't harm OS]\n");
  fprintf(fp, "  refresh          Read+Rewrite test [doesn't harm data]\n");
  fprintf(fp, "  verify           Read+Rewrite+Read test [doesn't harm data]\n");
  fprintf(fp, "  test             Read+Write+Read+Restore pattern test [doesn't harm data]\n");
  fprintf(fp, "  write            Write-only pattern test [corrupts data]\n");
  fprintf(fp, "  compare          Write+Read pattern test [corrupts data]\n");
  fprintf(fp, "  purge            Multi-pass Write+Read NCSC-TG-025/DoD 5220.22-M purge [corrupts data]\n");
  fprintf(fp, "  trim             Clean drive via TRIM commands [corrupts data]\n");

  fprintf(fp, "  digest           Display last calculated digest (if enabled)\n");
  fprintf(fp, "  print            Print last block buffer\n");
  
  fprintf(fp, "  exit             Quit this program\n");
  fprintf(fp, "  !<cmd>           Execute <cmd> shell command\n");
  fprintf(fp, "  #<text>          Comment (ignored)\n");
  
  fprintf(fp, "\nNOTES:\n");
  fprintf(fp, "  - Beware of using any of the write tests on SSD devices. Due\n");
  fprintf(fp, "    to the way they operate (with remapping of blocks for wear levelling)\n");
  fprintf(fp, "    you will not test what you intend and instead just make them fail faster.\n\n");
  fprintf(fp, "  - Beware that the Shuffle (-R with size > 0) option allocates a lot of RAM,\n");
  fprintf(fp, "    typically 8 bytes times the number of blocks of the device. One the other hand\n");
  fprintf(fp, "    it guarantees that all blocks in the requested range will be visited.\n");
}

void
usage(FILE *fp) {
  fprintf(fp, "[drvtool, version %s, by Peter Eriksson <pen@lysator.liu.se>]\n\n", version);
  fprintf(fp, "USAGE:\n  %s [<options>] [<device> [<commands>]]\n",
	  argv0);
  fprintf(fp, "\nOPTIONS:\n");
  fprintf(fp, "  -h               Display this information\n");
  fprintf(fp, "  -V               Print program verson and exit\n");
  fprintf(fp, "  -d               Increase debug info level\n");
  fprintf(fp, "  -v               Increase verbosity\n");
  fprintf(fp, "  -w               Open device in R/W mode (needed for write tests)\n");
  fprintf(fp, "  -y               Answer yes to all questions\n");
  fprintf(fp, "  -r               Reverse block order\n");
  fprintf(fp, "  -f               Flush device write buffer\n");
  fprintf(fp, "  -t               Enable sending TRIM commands to device\n");
  fprintf(fp, "  -R <size>        Enable Shuffled (size of deck)/Random (size 0) order\n");
  fprintf(fp, "  -X <type>        Transform type (XOR,ROL,ROR) [NONE]\n");
  fprintf(fp, "  -D <type>        Digest (checksum) type (CRC32,ADLER32,MD5,SHA256,SHA384,SHA512) [NONE]\n");
  fprintf(fp, "  -T <time>        Test timeout (0 = no limit) [%ld]\n",    f_timeout);
  fprintf(fp, "  -P <num>         Number of passes (0 = no limit) [%d]\n", f_passes);
  fprintf(fp, "  -S <pos>         Starting block offset [0]\n");
  fprintf(fp, "  -L <size>        Number of blocks [ALL]\n");
  fprintf(fp, "  -B <size>        Block size [NATIVE]\n");
  putc('\n', fp);
  act_help(fp);
}


int
prompt_yes(const char *msg,
	   ...) {
  char buf[80], *cp;
  va_list ap;


  if (f_yes)
    return 1;
  
  while (1) {
    va_start(ap, msg);
    vfprintf(stderr, msg, ap);
    va_end(ap);
    fprintf(stderr, " [yes|no]? ");

    buf[0] = 0;
    if (fgets(buf, sizeof(buf), stdin) == NULL)
      return -1;
    
    for (cp = buf+strlen(buf); cp > buf && isspace(cp[-1]); --cp)
      ;
    *cp = '\0';
    for (cp = buf; isspace(*cp); ++cp)
      ;
    
    if (strcmp(cp, "yes") == 0)
      return 1;
    if (strcmp(cp, "no") == 0)
      return 0;

    fprintf(stderr, "*** Invalid answer ***\n");
  }

  return -1;
}


int
str2digest(const char *s) {
  if (strcasecmp(s, "NONE") == 0)
    return DIGEST_NONE;
  
  if (strcasecmp(s, "ADLER32") == 0 || strcasecmp(s, "ADLER-32") == 0)
    return DIGEST_ADLER32;

  if (strcasecmp(s, "CRC32") == 0 || strcasecmp(s, "CRC-32") == 0)
    return DIGEST_CRC32;

  if (strcasecmp(s, "MD5") == 0 || strcasecmp(s, "MD-5") == 0)
    return DIGEST_MD5;

  if (strcasecmp(s, "256") == 0 || strcasecmp(s, "SHA256") == 0 || strcasecmp(s, "SHA-256") == 0)
    return DIGEST_SHA256;

  if (strcasecmp(s, "384") == 0 || strcasecmp(s, "SHA384") == 0 || strcasecmp(s, "SHA-384") == 0)
    return DIGEST_SHA384;

  if (strcasecmp(s, "512") == 0 || strcasecmp(s, "SHA512") == 0 || strcasecmp(s, "SHA-512") == 0)
    return DIGEST_SHA512;

  return -1;
}


int
str2transform(const char *s,
	      union transform_txd *d) {
  int v;

  if (strcasecmp(s, "NONE") == 0)
    return TRANSFORM_NONE;
  
  if (strncasecmp(s, "XOR", 3) == 0) {
    s += 3;
    if (*s == '-')
      ++s;

    v = 0xFF;
    if (*s && str2int(s, &v) < 0)
      return -1;
    
    d->xor = v;
    return TRANSFORM_XOR;
  }

  if (strncasecmp(s, "ROR", 3) == 0) {
    s += 3;
    if (*s == '-')
      ++s;

    v = 1;
    if (*s && str2int(s, &v) < 0)
      return -1;
    
    d->xor = v;
    return TRANSFORM_ROR;
  }

  if (strncasecmp(s, "ROL", 3) == 0) {
    s += 3;
    if (*s == '-')
      ++s;

    v = 1;
    if (*s &&str2int(s, &v) < 0)
      return -1;
    
    d->xor = v;
    return TRANSFORM_ROL;
  }

  return -1;
}


int
act_print(FILE *fp,
	  TEST *tp) {
  char b1[80], b2[80];

  
  if (tp->last_b_pos < 0) {
    fprintf(stderr, "%s: Error: No data block used yet\n", argv0);
    return 1;
  }
    
  fprintf(fp, "DATA BUFFER @ block %ld (offset %sB) size %sB:\n",
	  tp->last_b_pos,
	  int2str(tp->last_pos, b1, sizeof(b1), 0),
	  int2str(tp->b_size, b2, sizeof(b2), 0));
  
  buf_print(fp, tp->rbuf, tp->b_size, 3);
  return 0;
}


int
test_print_config(FILE *fp,
		  TEST *tp) {
  char buf[80];

  
  fprintf(fp, "TEST CONFIGURATION:\n");

  fprintf(fp, "  Analyze entire drive?          %s\n", tp->b_length == tp->b_total ? "Yes" : "No");
  if (tp->b_length != tp->b_total) {
    fprintf(fp, "  Starting block number:         %ld\n", tp->b_start);
    fprintf(fp, "  Ending block number:           %ld\n", tp->b_end-1);
  }
  
  fprintf(fp, "  Number of blocks:              %ld (%sB)\n",
	  tp->b_length,
	  int2str(tp->b_length * tp->b_size, buf, sizeof(buf), 0));
  
  fprintf(fp, "  Loop continously?              %s\n", tp->passes == 0 && tp->timeout == 0 ? "Yes" : "No");
  if (tp->passes)
    fprintf(fp, "  Number of passes:              %d\n", tp->passes);
  if (tp->timeout)
    fprintf(fp, "  Time limit:                    %lds\n", tp->timeout);

#if 0  
  fprintf(fp, "  Repair defective blocks?       %s\n", tp->flags.repair ? "Yes" : "No");
  fprintf(fp, "  Stop after first error?        %s\n", tp->flags.fail_fast ? "Yes" : "No");
  fprintf(fp, "  Use random bit patterns?       %s\n", tp->flags.random_patterns ? "Yes" : "No");
#endif
  
  fprintf(fp, "  Number of sectors per block:   %ld (%sB)\n",
	  tp->b_size / tp->drive->sector_size,
	  int2str(tp->b_size, buf, sizeof(buf), 0));
#if 0
  fprintf(fp, "  Enable extended messages?      %s\n", tp->flags.verbose ? "Yes" : "No");
  fprintf(fp, "  Restore defect list?           %s\n", tp->flags.restore_defects ? "Yes" : "No");
  fprintf(fp, "  Restore disk label?            %s\n", tp->flags.restore_label ? "Yes" : "No");
#endif

  return 0;
}


int
act_digest(FILE *fp,
	   TEST *tp) {
  switch (tp->digest.type) {
  case DIGEST_ADLER32:
    printf("ADLER32 DIGEST:\n  %08lX\n",
	   tp->digest.ctx.adler32);
    break;
    
  case DIGEST_CRC32:
    printf("CRC32 DIGEST:\n  %08lX\n",
	   tp->digest.ctx.crc32);
    break;
    
  case DIGEST_MD5:
    printf("MD5 DIGEST:\n");
    buf_print(stdout, tp->digest.buffer.last.md5, 16, 1);
    break;
      
  case DIGEST_SHA256:
    printf("SHA256 DIGEST:\n");
    buf_print(stdout, tp->digest.buffer.last.sha256, 32, 1);
    break;
      
  case DIGEST_SHA384:
    printf("SHA384 DIGEST:\n");
    buf_print(stdout, tp->digest.buffer.last.sha384, 48, 1);
    break;
    
  case DIGEST_SHA512:
    printf("SHA512 DIGEST:\n");
    buf_print(stdout, tp->digest.buffer.last.sha512, 64, 1);
    break;
  }

  return 0;
}


int
test_action(TEST *tp,
	    const char *act,
	    const char *arg) {
  int (*tstfun)(TEST *tp);
  unsigned int pass;
  char tbuf[256], bbuf[256], sbuf[256];
  char *tstname = "?";
  off_t blocks = tp->b_length;
  off_t bytes = blocks * tp->b_size;
  int rc = -1;

  
  if (strcmp(act, "help") == 0 || strcmp(act, "?") == 0) {
    
    act_help(stdout);
    return 0;
    
  } else if (strcmp(act, "config") == 0) {
    
    return test_print_config(stdout, tp);

  } else if (strcmp(act, "print") == 0) {
    
    return act_print(stdout, tp);

  } else if (strcmp(act, "current") == 0) {

    drive_print(stdout, 0, tp->drive);
    return 0;

  } else if (strcmp(act, "digest") == 0) {

    return act_digest(stdout, tp);

  } else if (strcmp(act, "drive") == 0) {

    tp->drive = drive_select(arg);
    return 0;

#if 0
  } else if (strcmp(act, "trim") == 0) {
    
    rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		    tstname);
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      exit(0);
    }
    
    test_trim;

#endif
  } else if (strcmp(act, "exit") == 0) {
    int v = 0;
    
    if (arg)
      sscanf(arg, "%d", &v);
    exit(v);
    
  } else if (strcmp(act, "read") == 0) {
    
    tstname = "READ";
    tp->rbuf = tp->obuf;
    tp->wbuf = tp->obuf;
    tp->flags = TEST_READ;
    tstfun = test_seq;
    
  } else if (strcmp(act, "refresh") == 0) {
    
    tstname = "REFRESH";
    tp->wbuf = tp->obuf;
    tp->rbuf = tp->wbuf;
    tp->flags = TEST_READ|TEST_WRITE;
    tstfun = test_seq;
    
  } else if (strcmp(act, "verify") == 0) {
    
    tstname = "VERIFY";
    tp->wbuf = tp->obuf;
    tp->flags = TEST_READ|TEST_WRITE|TEST_VERIFY;
    tstfun = test_seq;
    
  } else if (strcmp(act, "pattern") == 0) {
    
    tstname = "PATTERN";
    tp->flags = TEST_READ|TEST_PATTERN|TEST_WRITE|TEST_VERIFY|TEST_RESTORE;
    tstfun = test_seq;
    
  } else if (strcmp(act, "write") == 0) {
    
    tstname = "WRITE";
    
    rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		    tstname);
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      exit(0);
    }
    
    tp->rbuf = tp->wbuf;
    tp->flags = TEST_WRITE|TEST_PATTERN;
    tstfun = test_seq;
    
  } else if (strcmp(act, "compare") == 0) {
    
    tstname = "COMPARE";
    
    rc = prompt_yes("About to start %s. This will corrupt any data on the drive.\nContinue?",
		    tstname);
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      return 1;
    }
    
    tp->flags = TEST_WRITE|TEST_PATTERN|TEST_VERIFY;
    tstfun = test_seq;
    
  } else if (strcmp(act, "purge") == 0) {
    
    tstname = "PURGE";
    
    rc = prompt_yes("About to start %s. This will corrupt any data on the drive.\nContinue?",
		    tstname);
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      return 1;
    }
    
    tp->flags = TEST_WRITE|TEST_PURGE|TEST_VERIFY;
    tstfun = test_seq;
    
  } else if (strcmp(act, "trim") == 0) {
    
    tstname = "TRIM";
    
    rc = prompt_yes("About to start %s. This will corrupt any data on the drive.\nContinue?",
		    tstname);
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      return 1;
    }
    
    tp->flags = TEST_TRIM;
    tstfun = test_seq;
    
  }  else {
    
    fprintf(stderr, "%s: Error: %s: Invalid action\n", argv0, act);
    return 1;
    
  }
  
  printf("%s %s TEST (%sB / %s blocks @ %sB):\n",
	 tp->blocks ? (tp->blocks->s ? "SHUFFLED" : "RANDOM") : "SEQUENTIAL",
	 tstname,
	 int2str(bytes,  tbuf, sizeof(tbuf), 0),
	 int2str(blocks, bbuf, sizeof(bbuf), 0),
	 int2str(tp->b_size, sbuf, sizeof(sbuf), 1));
  
  for (pass = 1; !tp->passes || pass <= tp->passes; ++pass) {
    if (tp->flags & TEST_PATTERN) {
      memcpy(sel_pattern, test_patterns[(pass-1) % NUM_TEST_PATTERNS], sizeof(sel_pattern));
	pattern_fill(tp->wbuf, tp->b_size, sel_pattern, sizeof(sel_pattern));
	
	printf("  Pass %u [", pass);
	buf_print(stdout, sel_pattern, sizeof(sel_pattern), 0);
	printf("]:\n");
    } else if (tp->passes > 1)
      printf("  Pass %u:\n", pass);

    if (tp->digest_type)
      digest_init(&tp->digest, tp->digest_type);
    
    rc = (*tstfun)(tp);

    if (tp->digest.type) {
      switch (tp->digest.type) {
      case DIGEST_ADLER32:
	tp->digest.buffer.last.adler32 = tp->digest.ctx.adler32;
	if (pass == 1)
	  tp->digest.buffer.first.adler32 = tp->digest.ctx.adler32;
	else {
	  if (tp->digest.buffer.first.adler32 != tp->digest.buffer.last.adler32) {
	    fprintf(stderr, "%s: Error: ADLER32 Digest mismatch pass 1 (%08lX) vs pass %d (%08lX)\n",
		    argv0,
		    tp->digest.buffer.first.adler32,
		    pass,
		    tp->digest.buffer.last.adler32);
	    return 1;
	  }
	}
	break;
	
      case DIGEST_CRC32:
	tp->digest.buffer.last.crc32 = tp->digest.ctx.crc32;
	if (pass == 1)
	  tp->digest.buffer.first.crc32 = tp->digest.ctx.crc32;
	else {
	  if (tp->digest.buffer.first.crc32 != tp->digest.buffer.last.crc32) {
	    fprintf(stderr, "%s: Error: CRC32 Digest mismatch pass 1 (%08lX) vs pass %d (%08lX)\n",
		    argv0,
		    tp->digest.buffer.first.crc32,
		    pass,
		    tp->digest.buffer.last.crc32);
	    return 1;
	  }
	}
	break;
	
      case DIGEST_MD5:
	MD5Final(tp->digest.buffer.last.md5, &tp->digest.ctx.md5);
	if (pass == 1)
	  memcpy(tp->digest.buffer.first.md5,
		 tp->digest.buffer.last.md5,
		 sizeof(tp->digest.buffer.first.md5));
	else {
	  if (memcmp(tp->digest.buffer.first.md5,
		     tp->digest.buffer.last.md5,
		     sizeof(tp->digest.buffer.first.md5)) != 0) {
	    fprintf(stderr, "%s: Error: MD5 Digest mismatch between passes\n", argv0);
	    return 1;
	  }
	}
	break;
	
      case DIGEST_SHA256:
	SHA256_Final(tp->digest.buffer.last.sha256, &tp->digest.ctx.sha256);
	if (pass == 1)
	  memcpy(tp->digest.buffer.first.sha256,
		 tp->digest.buffer.last.sha256,
		 sizeof(tp->digest.buffer.first.sha256));
	else {
	  if (memcmp(tp->digest.buffer.first.sha256,
		     tp->digest.buffer.last.sha256,
		     sizeof(tp->digest.buffer.first.sha256)) != 0) {
	    fprintf(stderr, "%s: Error: SHA256 Digest mismatch between passes\n", argv0);
	    return 1;
	  }
	}
	break;
	
      case DIGEST_SHA384:
	SHA384_Final(tp->digest.buffer.last.sha384, &tp->digest.ctx.sha384);
	if (pass == 1)
	  memcpy(tp->digest.buffer.first.sha384,
		 tp->digest.buffer.last.sha384,
		 sizeof(tp->digest.buffer.first.sha384));
	else {
	  if (memcmp(tp->digest.buffer.first.sha384,
		     tp->digest.buffer.last.sha384,
		     sizeof(tp->digest.buffer.first.sha384)) != 0) {
	    fprintf(stderr, "%s: Error: SHA384 Digest mismatch between passes\n", argv0);
	    return 1;
	  }
	}
	break;
	
      case DIGEST_SHA512:
	SHA512_Final(tp->digest.buffer.last.sha512, &tp->digest.ctx.sha512);
	if (pass == 1)
	  memcpy(tp->digest.buffer.first.sha512,
		 tp->digest.buffer.last.sha512,
		 sizeof(tp->digest.buffer.first.sha512));
	else {
	  if (memcmp(tp->digest.buffer.first.sha512,
		     tp->digest.buffer.last.sha512,
		     sizeof(tp->digest.buffer.first.sha512)) != 0) {
	    fprintf(stderr, "%s: Error: SHA512 Digest mismatch between passes\n", argv0);
	    return 1;
	  }
	}
	break;
      }
    }
  }
  
  return rc;
}




int
main(int argc,
     char *argv[]) {
  DRIVE *dp = NULL;
  TEST tst;
  int i, j, rc;
  char *arg;

  char *s_digest = NULL;
  char *s_transform = NULL;
  char *s_time = NULL;
  char *s_passes = NULL;
  char *s_start = NULL;
  char *s_length = NULL;
  char *s_bsize = NULL;
  char *s_rsize = NULL;
  
  argv0 = argv[0];

  for (i = 1; i < argc && argv[i][0] == '-'; i++) {
    arg = NULL;
    for (j = 1; argv[i][j]; j++) {
      switch (argv[i][j]) {
      case 'h':
	usage(stdout);
	exit(0);
	
      case 'w':
	++f_update;
	break;

      case 'v':
	++f_verbose;
	break;

      case 'V':
	printf("[drvtool, version %s, by Peter Eriksson <pen@lysator.liu.se>]\n", version);
	exit(0);

      case 'r':
	++f_reverse;
	break;

      case 'f':
	++f_flush;
	break;

      case 'i':
	++f_ibase;
	break;

      case 'y':
	++f_yes;
	break;

      case 't':
	++f_trim;
	break;
	
      case 'd':
	++f_debug;
	break;

      case 'D':
	if (argv[i][j+1])
	  s_digest = argv[i]+j+1;
	else if (argv[i+1])
	  s_digest = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -D: Missing digest type\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'X':
	if (argv[i][j+1])
	  s_transform = argv[i]+j+1;
	else if (argv[i+1])
	  s_transform = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -X: Missing transform type\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'P':
	if (argv[i][j+1])
	  s_passes = argv[i]+j+1;
	else if (argv[i+1])
	  s_passes = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -P: Missing number of passes\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'S':
	if (argv[i][j+1])
	  s_start = argv[i]+j+1;
	else if (argv[i+1])
	  s_start = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -S: Missing start position\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'L':
	if (argv[i][j+1])
	  s_length = argv[i]+j+1;
	else if (argv[i+1])
	  s_length = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -L: Missing length\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'B':
	if (argv[i][j+1])
	  s_bsize = argv[i]+j+1;
	else if (argv[i+1])
	  s_bsize = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -B: Missing block size\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'R':
	if (argv[i][j+1])
	  s_rsize = argv[i]+j+1;
	else if (argv[i+1])
	  s_rsize = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -R: Missing max shuffle-array size\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      case 'T':
	if (argv[i][j+1])
	  s_time = argv[i]+j+1;
	else if (argv[i+1])
	  s_time = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -T: Missing time\n", argv0);
	  exit(1);
	}
	goto NextArg;
	
      default:
	fprintf(stderr, "%s: Error: -%c: Invalid switch at %d:%d\n", argv0, argv[i][j], i, j);
	exit(1);
      }
    }

  NextArg:;
  }

  printf("[drvtool, version %s, by Peter Eriksson <pen@lysator.liu.se>]\n\n", version);
    
  if (i >= argc) {
    if (drives_load() < 0) {
      fprintf(stderr, "%s: Error: Unable to open drive(s): %s\n",
	      argv0, strerror(errno));
      exit(1);
    }
  } else {
    dp = drive_open(argv[i]);
    if (!dp) {
      fprintf(stderr, "%s: Error: %s: Unable to open drive: %s\n",
	      argv0, argv[i], strerror(errno));
      exit(1);
    }

    ++i;
  }

  if (!dp) {
    puts("AVAILABLE DRIVE SELECTIONS:");
    drives_print(1);
    dp = drive_select(NULL);
  } else {
    puts("DRIVE SELECTED:");
    drives_print(0);
  }

  putchar('\n');
  if (dp->flags.is_ssd && f_update && !f_yes) {
    rc = prompt_yes("\n*** SSD DETECTED ***\nBeware of any tests that write data to the drive! Proceed anyway?");
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      exit(0);
    }
  }

  test_init(&tst, dp);
  
  if (s_bsize) {
    off_t v;
    
    rc = str2bytes(s_bsize, &v, 1, &tst);
    if (rc < 1 || test_set_bsize(&tst, v) < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid block size\n",
	      argv0, s_bsize);
      exit(1);
    }
  }
  
  if (s_start) {
    off_t v = 0;
    
    rc = str2blocks(s_start, &v, 0, &tst);
    if (rc < 1 || test_set_start(&tst, v) < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid start block\n",
	      argv0, s_start);
      exit(1);
    }
  }
  
  if (s_length) {
    off_t v = 0;
    
    if (strcasecmp(s_length, "ALL") == 0) {
      v = tst.b_total;
      rc = 1;
    } else
      rc = str2blocks(s_length, &v, 0, &tst);
    
    if (rc < 1 || test_set_length(&tst, v) < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid length\n",
	      argv0, s_length);
      exit(1);
    }
  }
  
  if (s_passes) {
    rc = str2int(s_passes, &tst.passes);
    if (rc < 1) {
      fprintf(stderr, "%s: Error: %s: Invalid number of passes\n",
	      argv0, s_passes);
      exit(1);
    }
  }
  
  if (s_time) {
    rc = str2time(s_time, &tst.timeout);
    if (rc < 1) {
      fprintf(stderr, "%s: Error: %s: Invalid timeout\n",
	      argv0, arg);
      exit(1);
    }
  }
  
  if (s_digest) {
    tst.digest_type = str2digest(s_digest);
    if (tst.digest_type < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid digest type\n",
	      argv0, s_digest);
      exit(1);
    }
  }
  
  if (s_transform) {
    tst.transform = str2transform(s_transform, &tst.txd);
    if (tst.transform < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid transform type\n",
	      argv0, s_transform);
      exit(1);
    }
  }

  if (s_rsize) {
    off_t v = 0;

    rc = str2off(s_rsize, &v, 0, NULL);
    if (rc < 1) {
      fprintf(stderr, "%s: Error: %s: Invalid shuffle-array size\n",
	      argv0, s_rsize);
      exit(1);
    }

    tst.blocks = blocks_create(v);
    blocks_shuffle(tst.blocks);
  }
  
  if (f_verbose) {
    test_print_config(stdout, &tst);
  }

  if (i == argc) {
    char buf[256], *bp, *act;

    do {

      if (isatty(fileno(stdin)))
	fprintf(stderr, "\n[%s]> ", tst.drive->name);
      
      if (fgets(buf, sizeof(buf), stdin) == NULL)
	exit(1);
      
      bp = strtrim(buf);
      if (!*bp || *bp == '#')
	continue;

      if (*bp == '!') {
	system(bp+1);
	continue;
      }
      
      act = strsep(&bp, " \t\r\n");
      if (!act)
	continue;
      
      rc = test_action(&tst, act, bp);
    } while(1);
  } else {
    for (; i < argc; i++) {
      putchar('\n');
      
      rc = test_action(&tst, argv[i], NULL);
      if (rc)
	break;
    }
  }
  
  return rc;
}
