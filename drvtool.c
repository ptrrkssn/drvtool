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
#include <sys/stat.h>
#include <sys/errno.h>
#include <sha224.h>
#include <sha256.h>
#include <sha512.h>
#include <sys/sysctl.h>
#include <camlib.h>
#include <geom/geom_disk.h>

#include "drvtool.h"


char *argv0 = "drvtool";

long f_update_freq = 500;

int f_yes = 0;
int f_ibase = 0;
int f_flush = 0;
int f_delete = 0;
int f_update = 0;
int f_verbose = 0;
int f_random = 0;

int f_print = 0;


int d_passes = 1;


BLOCKS *d_blocks = NULL;


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


DISK *disks = NULL;


void
buf_xor(unsigned char *buf,
	size_t bufsize,
	int b) {
  while (bufsize-- > 0)
    *buf++ ^= b;
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

  bp->s = 0;
  bp->v = calloc(s, sizeof(bp->v[0]));
  if (bp->v == NULL)
    return NULL;

  bp->s = s;

  for (i = 0; i < s; i++)
    bp->v[i] = i;

  return bp;
}


static inline void
off_swap(off_t *a,
	 off_t *b) {
  off_t t = *a;
  
  *a = *b;
  *b = t;
}

off_t
orand(off_t size) {
  off_t r;

  r = (((off_t) lrand48() << 31) | ((off_t) lrand48() << 31)) | lrand48();
  return r % size;
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
dev_cam_sysctl(DEVICE *dp,
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
dev_geom_sysctl(DEVICE *dp,
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
  DEVICE *dev;
  off_t ns, tb, o_start, o_length;

  
  if (!tp || !tp->dev)
    return -1;

  dev = tp->dev;

  o_start = tp->b_start * tp->b_size;
  o_length = tp->b_length * tp->b_size;
  
  /* Make sure block size is >= sector size */
  if (bsize < dev->sector_size)
    return -1;

  /* Make sure block size is a multiple of the sector size */
  ns = bsize / dev->sector_size;
  if (ns * dev->sector_size != bsize)
    return -1;

  /* Make sure block size and total media size is compatible */
  tb = dev->media_size / bsize;
  if (tb * bsize != dev->media_size)
    return -1;

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
  if (!tp || !tp->dev)
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
  if (!tp || !tp->dev)
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
test_init(TEST *tp,
	  DEVICE *dp) {
  tp->dev = dp;
  
  rate_init(&tp->rate);

  tp->flags  = 0;
  tp->passes = d_passes;
  tp->t_max  = 0;

  tp->digest  = 0;
  tp->crypto  = 0;
  
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
  
  if (i == 'i')
    base = 1024;
  
  switch (toupper(c)) {
  case 'K':
    *vp *= base;
    break;
    
  case 'M':
    *vp *= base*base;
    if (rest)
      ++*rest;
    break;
    
  case 'G':
    *vp *= base*base*base;
    if (rest)
      ++*rest;
    break;
    
  case 'T':
    *vp *= base*base*base*base;
    if (rest)
      ++*rest;
    break;
    
  case 'P':
    *vp *= base*base*base*base;
    if (rest)
      ++*rest;
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
  DEVICE *dp = tp->dev;
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
  
  /* Make sure it's a multiple of the sector size */
  v = *vp / dp->sector_size;
  if (v * dp->sector_size != *vp)
    return -1;

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
dev_flush(DEVICE *dp) {
  int rc;


  rc = ioctl(dp->fd, DIOCGFLUSH);
  return rc;
}


int
dev_delete(DEVICE *dp,
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
dev_close(DEVICE *dp) {
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
dev_print(FILE *fp,
	  int idx,
	  DEVICE *dp) {
  char sbuf[256], bbuf[256], xbuf[256];
  

  if (idx)
    fprintf(fp, "  %4d.\t", idx);
  else
    fprintf(fp, "\t");
  
  fprintf(fp, "%s", dp->name);

  if (dp->ident && dp->ident[0])
    fprintf(fp, " <%s>", dp->ident);

  fprintf(fp, " : %sB (%s sectors @ %sn",
	  int2str(dp->media_size, sbuf, sizeof(sbuf), 0),
	  int2str(dp->sectors, bbuf, sizeof(bbuf), 0),
	  int2str(dp->stripe_size, xbuf, sizeof(xbuf), 2));
  
  if (dp->sector_size != dp->stripe_size)
    fprintf(fp, "/%se",
	    int2str(dp->sector_size, xbuf, sizeof(xbuf), 2));
  
  putc(')', fp);
  
  putc(' ', fp);
  putc(':', fp);
  if (dp->flags.is_ssd)
    fprintf(fp, " SSD");
  if (dp->flags.is_open)
    fprintf(fp, " OPEN");
  
  putc('\n', fp);

  if (f_verbose) {
    putc('\t', fp);
    
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




DEVICE *
dev_open(const char *name) {
  char pnbuf[MAXPATHLEN];
  char idbuf[DISK_IDENT_SIZE];
  char gbuf[256];
  int s_errno;
  DEVICE *dp;
  int is_rotating = -1;
  size_t isrs = sizeof(is_rotating);
  struct cam_device *cam;
  u_int geom_flags;
  
  
  dp = malloc(sizeof(*dp));
  if (!dp)
    return NULL;

  dp->name = strdup(name);
  if (!dp->name)
    goto Fail;
  
  dp->fd = -1;
  if (*name == '/') {
    dp->path = strdup(name);
    
    dp->name = strrchr(name, '/');
    dp->name++;
  }
  else
    dp->path = strxdup("/dev/", name, NULL);

  dev_cam_sysctl(dp, "rotating", &is_rotating, &isrs);
  dp->flags.is_ssd = (is_rotating ? 0 : 1);

  geom_flags = 0;
  isrs = sizeof(gbuf);
  dev_geom_sysctl(dp, "flags", gbuf, &isrs);
  if (sscanf(gbuf, "%02x", &geom_flags) == 1) 
    dp->flags.is_open = (geom_flags & 0x0002) ? 1 : 0;

  
  dp->fd = open(dp->path, (f_update ? O_RDWR : O_RDONLY), 0);
  if (dp->fd < 0)
    goto Fail;

  if (fstat(dp->fd, &dp->sbuf) < 0)
    goto Fail;

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
    
    snprintf(buf, sizeof(buf), "%s%u bus %u scbus %u target %u lun %lu",
	     cam->sim_name, cam->sim_unit_number, cam->bus_id,
	     cam->path_id, cam->target_id, cam->target_lun);
    dp->cam.path = strdup(buf);
    cam_close_device(cam);
  }

  return dp;

 Fail:
  s_errno = errno;
  dev_close(dp);
  errno = s_errno;
  return NULL;
}





int
disks_add(const char *name) {
  DEVICE *dev;
  DISK *dip;

  
  dev = dev_open(name);
  if (!dev)
    return -1;
  
  dip = malloc(sizeof(*dip));
  if (!dip)
    return -1;

  dip->name = strdup(name);
  dip->dev = dev;
  
  dip->next = disks;
  disks = dip;

  return 0;
}


int
disks_load(void) {
  size_t bsize;
  char *buf;
  char *cp;
  
  
  if (sysctlbyname("kern.disks", NULL, &bsize, NULL, 0) < 0)
    return -1;

  buf = malloc(bsize);
  if (!buf)
    return -1;
  
  if (sysctlbyname("kern.disks", buf, &bsize, NULL, 0) < 0)
    return -1;

  while ((cp = strsep(&buf, " ")) != NULL) {
    disks_add(cp);
  }
  
  return 0;
}


void
disks_print(void) {
  DISK *dip = disks;
  int i = 0;
  
  while (dip) {
    dev_print(stdout, ++i, dip->dev);
    dip = dip->next;
  }
}




void
test_pstatus(FILE *fp,
	     TEST *tp,
	     struct timespec *now,
	     off_t bno) {
  long time_left;
  off_t bytes_done, bytes_left, bytes_total, bytes_rate_avg, bytes_done_pct;
  char ibuf[256], tbuf[256], pbuf[256];


  bytes_done  = bno * tp->b_size;
  bytes_total = tp->b_length * tp->b_size;
  bytes_left  = bytes_total - bytes_done;
  
  bytes_done_pct = bytes_total ? (100 * bytes_done) / bytes_total : 0;
  
  bytes_rate_avg = rate_get(&tp->rate);

  if (tp->t_max)
    time_left = tp->t_max - ts_delta(now, &tp->t0, NULL, NULL);
  else
    time_left = bytes_rate_avg ? bytes_left / bytes_rate_avg : 0;
  
  fprintf(fp, "    %15ld : %7sB @ %5sB/s : %4ld%% done",
	  bno, 
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
  


int
test_trim(TEST *tp) {
  DEVICE *dp = tp->dev;
  int rc;
  off_t i;
  
  for (i = tp->b_start; i < tp->b_end; i++) {
    off_t pos = i * tp->b_size;
    
    rc = dev_delete(dp, pos, tp->b_size);
    if (rc < 0) {
      fprintf(stderr, "%s: Error: %s: TRIM %ld bytes @ %ld failed: %s\n",
	      argv0, dp->path, tp->b_size, pos, strerror(errno));
      exit(1);
    }
  }

  return 0;
}
  

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
  DEVICE *dp = tp->dev;
  struct timespec now;
  ssize_t rc;
  off_t b, b_pos, pos, bytes_done;
  int nw, n_write;
  double td;
  

  clock_gettime(CLOCK_REALTIME, &now);
  tp->t0 = now;
  tp->t1 = now;
  
 
  bytes_done = 0;

  for (b = 0; b < tp->b_length && (!tp->t_max || ts_delta(&now, &tp->t0, NULL, NULL) <= tp->t_max); ++b) {

    if (tp->blocks && b < tp->blocks->s)
      b_pos = tp->blocks->v[b];
    else if (f_random)
      b_pos = orand(tp->b_length);
    else
      b_pos = b;

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
      /* Ignore Ctrl-C at this time in order to prevent data corruption */
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

      switch (tp->crypto) {
      case TEST_CRYPTO_XOR:
	buf_xor(tp->wbuf, tp->b_size, 0xFF);
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
	
	rc = dev_flush(dp);
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
    
    if (f_delete && (tp->flags & TEST_DELETE)) {
      /* TRIM block from (SSD) device */
      
      rc = dev_delete(dp, pos, tp->b_size);
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
      
    if (tp->digest) {
      unsigned char *tbuf = tp->obuf;
      
      if (tp->flags & TEST_WRITE)
	tbuf = tp->wbuf;
      
      if (tp->flags & TEST_VERIFY)
	tbuf = tp->rbuf;
      
      switch (tp->digest) {
      case TEST_DIGEST_SHA224:
	SHA224_Update(&tp->ctx.sha224, tbuf, rc);
	break;
	
      case TEST_DIGEST_SHA256:
	SHA256_Update(&tp->ctx.sha256, tbuf, rc);
	break;
	
      case TEST_DIGEST_SHA512:
	SHA512_Update(&tp->ctx.sha512, tbuf, rc);
	break;
      }
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

      test_pstatus(stderr, tp, &now, b);
      fputc('\r', stderr);
    }
  }
  
  test_pstatus(stderr, tp, &now, b);
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
      if (j)
	putc(' ', fp);
      fprintf(fp, "%02X", buf[i+j]);
    }

    if (mode > 2) {
      putc(' ', fp);
      putc(':', fp);
      putc(' ', fp);
      for (j = 0; j < 16 && i+j < size; j++) {
	putc(isprint(buf[i+j]) ? buf[i+j] : '?', fp);
	putc(' ', fp);
      }
    }

    i += j;
    if (mode)
      putc('\n', fp);
  }
}


void
usage(FILE *fp) {
  fprintf(fp, "Usage:\n  %s [<options>] [<device> [<actions>]]\n",
	  argv0);
  fprintf(fp, "\nOptions:\n");
  fprintf(fp, "  -h               Display this information\n");
  fprintf(fp, "  -v               Increase verbosity [NO]\n");
  fprintf(fp, "  -w               Open device in R/W mode (needed for write tests) [RO]\n");
  fprintf(fp, "  -y               Answer yes to all questions [NO]\n");
  fprintf(fp, "  -r               Random block order (-rr for shuffled order) [NO]\n");
  fprintf(fp, "  -f               Flush device write buffer [NO]\n");
  fprintf(fp, "  -d               Enable sending TRIM commands to device [NO]\n");
  fprintf(fp, "  -p               Print last block [NO]\n");
  fprintf(fp, "  -C <type>        Crypto type (XOR) [NONE]\n");
  fprintf(fp, "  -D <type>        Digest (checksum) type (SHA224 - SHA256 - SHA512) [NONE]\n");
  fprintf(fp, "  -T <time>        Test time limit [NONE]\n");
  fprintf(fp, "  -P <num>         Number of passes [%d]\n", d_passes);
  fprintf(fp, "  -S <pos>         Starting block offset [FIRST]\n");
  fprintf(fp, "  -L <size>        Number of blocks [ALL]\n");
  fprintf(fp, "  -B <size>        Block size [NATIVE]\n");
  fprintf(fp, "\nActions:\n");
  fprintf(fp, "  read             Read-only test [doesn't harm OS]\n");
  fprintf(fp, "  refresh          Read+Rewrite test [doesn't harm data]\n");
  fprintf(fp, "  verify           Read+Rewrite+Read test [doesn't harm data]\n");
  fprintf(fp, "  test             Read+Write+Read+Restore pattern test [doesn't harm data]\n");
  fprintf(fp, "  write            Write-only pattern test [corrupts data]\n");
  fprintf(fp, "  compare          Write+Read pattern test [corrupts data]\n");
  fprintf(fp, "  purge            Multi-pass Write+Read NCSC-TG-025/DoD 5220.22-M purge [corrupts data]\n");
  fprintf(fp, "  delete           Send device TRIM commands [corrupts data]\n");
  fprintf(fp, "\nNotes:\n");
  fprintf(fp, "  - Beware of using any of the write tests on SSD devices. Due\n");
  fprintf(fp, "    to the way they operate (with remapping of blocks for wear levelling)\n");
  fprintf(fp, "    you will not test what you intend and instead just make them fail faster.\n");
  fprintf(fp, "  - Beware that the Shuffle (-rr) option allocates a lot of RAM, typically\n");
  fprintf(fp, "    8 bytes times the number of blocks of the device but it guarantees that\n");
  fprintf(fp, "    all blocks in the requested range will be visited.\n");
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
  if (strcasecmp(s, "224") == 0 || strcasecmp(s, "SHA224") == 0 || strcasecmp(s, "SHA-224") == 0)
    return TEST_DIGEST_SHA224;
  
  if (strcasecmp(s, "256") == 0 || strcasecmp(s, "SHA256") == 0 || strcasecmp(s, "SHA-256") == 0)
    return TEST_DIGEST_SHA256;

  if (strcasecmp(s, "512") == 0 || strcasecmp(s, "SHA512") == 0 || strcasecmp(s, "SHA-512") == 0)
    return TEST_DIGEST_SHA512;

  return -1;
}


int
str2crypto(const char *s) {
  if (strcasecmp(s, "XOR") == 0)
    return TEST_CRYPTO_XOR;

  return -1;
}



int
main(int argc,
     char *argv[]) {
  DEVICE *dev = NULL;
  TEST tst;
  int i, j, rc;
  char *arg;
  unsigned char o_digest[64], digest[64];
  int (*tstfun)(TEST *tp);

  char *s_digest = NULL;
  char *s_crypto = NULL;
  char *s_time = NULL;
  char *s_passes = NULL;
  char *s_start = NULL;
  char *s_length = NULL;
  char *s_bsize = NULL;
  
  
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

      case 'p':
	++f_print;
	break;
	
      case 'v':
	++f_verbose;
	break;

      case 'r':
	++f_random;
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

      case 'd':
	++f_delete;
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
	
      case 'C':
	if (argv[i][j+1])
	  s_crypto = argv[i]+j+1;
	else if (argv[i+1])
	  s_crypto = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -C: Missing crypto type\n", argv0);
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

  if (i >= argc) {
    disks_load();
    
    puts("AVAILABLE DISK SELECTIONS:");
    disks_print();
    exit(0);
  }
  
  if (disks_add(argv[i]) < 0) {
    fprintf(stderr, "%s: Error: %s: Unable to open device: %s\n",
	    argv0, argv[i], strerror(errno));
    exit(1);
  }
  
  ++i;

  dev = disks->dev;
  puts("SELECTED DISK:");
  dev_print(stdout, 0, dev);
  putchar('\n');

  if (dev->flags.is_ssd && f_update && !f_yes) {
    rc = prompt_yes("*** DEVICE IS SSD ***\nBeware of any tests that write data to the device! Proceed anyway?");
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      exit(0);
    }
  }
  
  if (dev->flags.is_open && f_update && !f_yes) {
    rc = prompt_yes("*** DEVICE IS IN USE ***\nProceed anyway?");
    if (rc != 1) {
      fprintf(stderr, "*** Aborted ***\n");
      exit(0);
    }
  }
  
  test_init(&tst, dev);
  
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
    rc = str2time(s_time, &tst.t_max);
    if (rc < 1) {
      fprintf(stderr, "%s: Error: %s: Invalid time\n",
	      argv0, arg);
      exit(1);
    }
  }
  
  if (s_digest) {
    tst.digest = str2digest(s_digest);
    if (tst.digest < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid digest type\n",
	      argv0, s_digest);
      exit(1);
    }
  }
  
  if (s_crypto) {
    tst.crypto = str2crypto(s_crypto);
    if (tst.crypto < 0) {
      fprintf(stderr, "%s: Error: %s: Invalid crypto type\n",
	      argv0, s_crypto);
      exit(1);
    }
  }
  
  if (f_random > 1) {
    if (!tst.blocks)
      tst.blocks = blocks_create(tst.b_length);
    
    blocks_shuffle(tst.blocks);
  }
  
  if (f_verbose > 1) {
    printf("Blocks:\n");
    printf("  Size:   %lu\n", tst.b_size);
    printf("  Start:  %ld\n", tst.b_start);
    printf("  Length: %ld\n", tst.b_length);
    printf("  End:    %ld\n", tst.b_end);
    printf("Device:\n");
    printf("  Sector Size:   %ld\n", tst.dev->sector_size);
    printf("  Stripe Size:   %ld\n", tst.dev->stripe_size);
    printf("  Stripe Offset: %ld\n", tst.dev->stripe_offset);
    printf("  Front Stuff:   %ld\n", tst.dev->front_reserved);
  }

  for (; i < argc; i++) {
    unsigned int pass;
    char tbuf[256], bbuf[256], sbuf[256];
    char *tstname = "?";
    off_t blocks = tst.b_length;
    off_t bytes = blocks * tst.b_size;
    int rc;

    
    if (strcmp(argv[i], "help") == 0) {
      
      usage(stdout);
      exit(0);
      
    } else if (strcmp(argv[i], "read") == 0) {
      
      tstname = "Read";
      tst.rbuf = tst.obuf;
      tst.flags = TEST_READ;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "refresh") == 0) {
      
      tstname = "Refresh";
      tst.wbuf = tst.obuf;
      tst.rbuf = tst.wbuf;
      tst.flags = TEST_READ|TEST_WRITE;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "verify") == 0) {
      
      tstname = "Refresh+Verify";
      tst.wbuf = tst.obuf;
      tst.flags = TEST_READ|TEST_WRITE|TEST_VERIFY;
      tstfun = test_seq;

    } else if (strcmp(argv[i], "test") == 0) {
      
      tstname = "Pattern";
      tst.flags = TEST_READ|TEST_PATTERN|TEST_WRITE|TEST_VERIFY|TEST_RESTORE;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "write") == 0) {

      tstname = "Write";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tst.rbuf = tst.wbuf;
      tst.flags = TEST_WRITE|TEST_PATTERN;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "compare") == 0) {
      
      tstname = "Write+Read+Verify";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tst.flags = TEST_WRITE|TEST_PATTERN|TEST_VERIFY;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "purge") == 0) {
      
      tstname = "Purge";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tst.flags = TEST_WRITE|TEST_PURGE|TEST_VERIFY;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "delete") == 0) {
      
      tstname = "TRIM Delete";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tst.flags = TEST_DELETE;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "delete-all") == 0) {
      
      tstname = "TRIM Delete-All";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tstfun = test_trim;
      
    }  else {
      
      fprintf(stderr, "%s: Error: %s: Invalid action\n", argv0, argv[i]);
      exit(1);

    }
    
    printf("%s %s Test (%sB / %s blocks @ %sB):\n",
	   f_random ? (f_random > 1 ? "Shuffled" : "Random") : "Sequential",
	   tstname,
	   int2str(bytes,  tbuf, sizeof(tbuf), 0),
	   int2str(blocks, bbuf, sizeof(bbuf), 0),
	   int2str(tst.b_size, sbuf, sizeof(sbuf), 1));
    
    for (pass = 1; pass <= tst.passes; ++pass) {
      if (tst.flags & TEST_PATTERN) {
	memcpy(sel_pattern, test_patterns[(pass-1) % NUM_TEST_PATTERNS], sizeof(sel_pattern));
	pattern_fill(tst.wbuf, tst.b_size, sel_pattern, sizeof(sel_pattern));
	
	printf("  Pass %u [", pass);
	buf_print(stdout, sel_pattern, sizeof(sel_pattern), 0);
	printf("]:\n");
      } else if (tst.passes > 1)
	printf("  Pass %u:\n", pass);
      
      switch (tst.digest) {
      case TEST_DIGEST_SHA224:
	SHA224_Init(&tst.ctx.sha224);
	break;
	
      case TEST_DIGEST_SHA256:
	SHA256_Init(&tst.ctx.sha256);
	break;
	
      case TEST_DIGEST_SHA512:
	SHA512_Init(&tst.ctx.sha512);
	break;
      }

      rc = (*tstfun)(&tst);
  
      switch (tst.digest) {
      case TEST_DIGEST_SHA224:
	SHA224_Final(digest, &tst.ctx.sha224);

	if (pass == tst.passes) {
	  printf("SHA224 Digest:\n");
	  buf_print(stdout, digest, 32, 1);
	}
	
	if (pass == 1)
	  memcpy(o_digest, digest, 32);
	else {
	  if (memcmp(o_digest, digest, 32) != 0) {
	    fprintf(stderr, "%s: Error: SHA224 Digest mismatch between passes\n", argv0);
	    exit(1);
	  }
	}
	break;
	
      case TEST_DIGEST_SHA256:
	SHA256_Final(digest, &tst.ctx.sha256);

	if (pass == tst.passes) {
	  printf("SHA256 Digest:\n");
	  buf_print(stdout, digest, 32, 1);
	}
	
	if (pass == 1)
	  memcpy(o_digest, digest, 32);
	else {
	  if (memcmp(o_digest, digest, 32) != 0) {
	    fprintf(stderr, "%s: Error: SHA256 Digest mismatch between passes\n", argv0);
	    exit(1);
	  }
	}
	break;
	
      case TEST_DIGEST_SHA512:
	SHA512_Final(digest, &tst.ctx.sha512);
	
	if (pass == tst.passes) {
	  printf("SHA512 Digest:\n");
	  buf_print(stdout, digest, 64, 1);
	}

	if (pass == 1)
	  memcpy(o_digest, digest, 64);
	else {
	  if (memcmp(o_digest, digest, 64) != 0) {
	    fprintf(stderr, "%s: Error: SHA512 Digest mismatch between passes\n", argv0);
	    exit(1);
	  }
	}
	break;
      }
    }
    
    if (f_print) {
      printf("Last Data Block (%lu bytes):\n", tst.b_size);
      buf_print(stdout, tst.rbuf, tst.b_size, 3);
    }
  }
  
  exit(0);
}
