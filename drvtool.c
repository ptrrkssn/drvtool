/*
** drvtool.c 
**
** Utility to analyze/test/purge hard drives
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


char *argv0 = "drvtool";

long f_update_freq = 500;

int f_yes = 0;
int f_ibase = 0;
int f_flush = 0;
int f_delete = 0;
int f_update = 0;
int f_verbose = 0;

int f_print = 0;

int d_passes = 1;


typedef unsigned char PATTERN[4];

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



#define CHECK_SHA224 1
#define CHECK_SHA256 2
#define CHECK_SHA512 3


#define RATE_BUF_SIZE 10

typedef struct {
  unsigned int f;
  unsigned int p;
  off_t dv[RATE_BUF_SIZE];
} RATE;



#define TYPE_UNKNOWN 0
#define TYPE_DISK    1
#define TYPE_SSD     2


typedef struct {
  int fd;
  char *path;
  struct stat sbuf;

  int type;
  
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
  
  char *provider_name;
  char *ident;
  char *physical_path;
} DEVICE;


#define TEST_READ     0x0001
#define TEST_WRITE    0x0002
#define TEST_VERIFY   0x0004
#define TEST_RESTORE  0x0008
#define TEST_PATTERN  0x0010
#define TEST_PURGE    0x0020
#define TEST_FLUSH    0x0040
#define TEST_DELETE   0x0080


typedef struct {
  DEVICE *dev;
  
  RATE rate;           /* Transfer rate buffer        */

  unsigned int flags;  /* Test flags                  */
  
  int passes;          /* Number of passes            */
  
  struct timespec t0;  /* Start time                  */
  struct timespec t1;  /* Last update                 */
  
  time_t t_max;        /* Max length of test run      */
  
  off_t b_size;       /* Size of each block          */
  off_t b_last;       /* Size of last read block     */
  
  off_t  start;        /* First postion               */
  off_t  length;       /* Data length                 */
  off_t  end;          /* End of data                 */
  
  off_t  pos;          /* Current block position      */
  
  unsigned char *obuf; /* Block buffer, original data */
  unsigned char *wbuf; /* Block buffer, data to write */
  unsigned char *rbuf; /* Block buffer, data read     */

  int checksum;
  union {
    SHA224_CTX sha224;
    SHA256_CTX sha256;
    SHA512_CTX sha512;
  } ctx;
} TEST;



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


int
dev_sysctl(DEVICE *dp,
	   const char *vname,
	   void *vp,
	   size_t *vs) {
  char nbuf[128], *pname, *cp;
  int unit;

  
  if (!dp || !dp->provider_name)
    return -1;
  
  pname = dp->provider_name;
  cp = pname+strlen(pname)-1;
  while (cp >= pname && isdigit(*cp))
    --cp;
  ++cp;
  
  if (sscanf(cp, "%d", &unit) != 1)
    return -1;

  *cp = '\0';

  snprintf(nbuf, sizeof(nbuf), "kern.cam.%s.%d.%s", pname, unit, vname);
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
test_set_start(TEST *tp,
	       off_t start) {
  if (!tp || !tp->dev)
    return -1;

  if (start < 0 || start > tp->dev->media_size)
    return -1;
  
  tp->start = start;
  tp->pos   = tp->start;

  if (tp->length + tp->start > tp->dev->media_size)
    tp->length = tp->dev->media_size - tp->start;
  
  tp->end = tp->start + tp->length;

  return 0;
  
}

int
test_set_length(TEST *tp,
		off_t length) {
  if (!tp || !tp->dev)
    return -1;

  /* Negative length -> Count from end of disk */
  if (length < 0)
    length += tp->dev->media_size;
  
  if (length < 0 || length+tp->start > tp->dev->media_size)
    return -1;
  
  tp->length = length;
  tp->end    = tp->start + tp->length;

  if (tp->pos > tp->end)
    tp->pos = tp->end;
  
  return 0;
}

int
test_init(TEST *tp,
	  DEVICE *dp) {
  tp->dev = dp;
  
  rate_init(&tp->rate);

  tp->flags  = 0;
  tp->passes = d_passes;
  
  tp->b_size = dp->stripe_size;
  
  tp->start = 0;
  tp->pos   = 0;

  tp->length = dp->media_size - tp->start;
  tp->end    = tp->start + tp->length;

  tp->obuf = malloc(tp->b_size);
  if (!tp->obuf)
    return -1;
  
  tp->wbuf = malloc(tp->b_size);
  if (!tp->wbuf)
    return -1;
  
  tp->rbuf = malloc(tp->b_size);
  if (!tp->rbuf)
    return -1;

  return 0;
}


int
str2int(const char *str,
	int *vp) {
  char c;
  int rc;
  
  
  c = 0;  
  rc = sscanf(str, "%d%c", vp, &c);
  if (rc < 1)
    return rc;
  
  switch (toupper(c)) {
  case 0:
    break;
    
  case 'K':
    *vp *= 1000;
    break;
    
  case 'M':
    *vp *= 1000000;
    break;
    
  case 'G':
    *vp *= 1000000000;
    break;
    
  default:
    return -1;
  }

  return 1;
}

int
str2off(const char *str,
	off_t *vp,
	int f_b2f,
	DEVICE *dp) {
  char cbuf[4], *cp = NULL;
  int rc;
  off_t base = 1000;

  
  if (f_ibase || f_b2f) {
    base = 1024;
    f_b2f = 1;
  }
  
  memset(cbuf, 0, sizeof(cbuf));
  
  rc = sscanf(str, "%ld%3s", vp, cbuf);
  if (rc < 1)
    return rc;

  if (rc > 1 && cbuf[0]) {
    cp = cbuf;

    switch (toupper(*cp++)) {
    case 0:
      break;

    case 'B':
    case 'E':
    case 'S':
      --cp;
      break;
      
    case 'K':
      if (*cp == 'i') {
	base = 1024;
	++cp;
      }
      *vp *= base;
      break;
      
    case 'M':
      if (*cp == 'i') {
	base = 1024;
	++cp;
      }
      *vp *= base*base;
      break;
      
    case 'G':
      if (*cp == 'i') {
	base = 1024;
	++cp;
      }
      *vp *= base*base*base;
      break;
      
    case 'T':
      if (*cp == 'i') {
	base = 1024;
	++cp;
      }
      *vp *= base*base*base*base;
      break;
      
    case 'P':
      if (*cp == 'i') {
	base = 1024;
	++cp;
      }
      *vp *= (unsigned long long) base*base*base*base;
      break;
      
    default:
      return -1;
    }
  }

  if (dp && cp) {
    off_t v;

    switch (toupper(*cp)) {
    case 0:
      break;
      
    case 'B':
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

  
  if (f_ibase || b2f) {
    base = 1024;
    b2f = 1;
  }
  
  if (llabs(b) < 10000) {
    snprintf(buf, bufsize, "%ld", b);
    return buf;
  }
  
  b /= base;
  if (llabs(b) < 10000) {
    snprintf(buf, bufsize, "%ldK%s", b, b2f ? "i" : "");
    return buf;
  }
  
  b /= base;
  if (llabs(b) < 10000) {
    snprintf(buf, bufsize, "%ldM%s", b, b2f ? "i" : "");
    return buf;
  }
  
  b /= base;
  if (llabs(b) < 10000) {
    snprintf(buf, bufsize, "%ldG%s", b, b2f ? "i" : "");
    return buf;
  }
  
  b /= base;
  if (llabs(b) < 10000) {
    snprintf(buf, bufsize, "%ldT%s", b, b2f ? "i" : "");
    return buf;
  }
  
  snprintf(buf, bufsize, "%ldP%s", b, b2f ? "i" : "");
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
	  DEVICE *dp) {
  char sbuf[256];
  char bbuf[256];
  

  fprintf(fp, "%s", dp->path);

  if (dp->provider_name)
    fprintf(fp, " (%s)", dp->provider_name);

  switch (dp->type) {
  case TYPE_DISK:
    fprintf(fp, " DISK");
    break;
  case TYPE_SSD:
    fprintf(fp, " SSD");
    break;
  }
  
  if (f_verbose && dp->physical_path)
    fprintf(fp, " {%s}", dp->physical_path);

  if (dp->ident)
    fprintf(fp, " [%s]", dp->ident);
  
  fprintf(fp, ": %sB (%s sectors @ %ldn",
		 int2str(dp->media_size, sbuf, sizeof(sbuf), 0),
		 int2str(dp->sectors, bbuf, sizeof(bbuf), 0),
		 dp->stripe_size);
  
  if (dp->sector_size != dp->stripe_size)
    fprintf(fp, "/%lde", dp->sector_size);
  
  putc(')', fp);
  
  if (f_verbose && dp->fw_heads && dp->tracks && dp->fw_sectors)
    fprintf(fp, " {%u heads, %u tracks/head, %u sectors/track}",
	    dp->fw_heads,
	    dp->tracks,
	    dp->fw_sectors);
}



DEVICE *
dev_open(const char *path) {
  char pnbuf[MAXPATHLEN];
  char idbuf[DISK_IDENT_SIZE];
  int s_errno;
  DEVICE *dp;
  int is_rotating = -1;
  size_t isrs = sizeof(is_rotating);
  
  
  dp = malloc(sizeof(*dp));
  if (!dp)
    return NULL;

  dp->fd = -1;
  if (*path == '/')
    dp->path = strdup(path);
  else
    dp->path = strxdup("/dev/", path, NULL);
  
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

  dp->type = TYPE_UNKNOWN;
  dev_sysctl(dp, "rotating", &is_rotating, &isrs);
  switch (is_rotating) {
  case 0:
    dp->type = TYPE_SSD;
    break;

  case 1:
    dp->type = TYPE_DISK;
    break;
  }
  
  return dp;

 Fail:
  s_errno = errno;
  dev_close(dp);
  errno = s_errno;
  return NULL;
}






void
test_pstatus(FILE *fp,
	     TEST *tp,
	     struct timespec *now) {
  long time_left;
  off_t bytes_done, bytes_left, bytes_total, bytes_rate_avg, bytes_done_pct;
  char ibuf[256], tbuf[256], pbuf[256];


  bytes_done = tp->pos - tp->start;
  bytes_left = tp->length - bytes_done;
  bytes_total = tp->length;
  bytes_done_pct = bytes_total ? (100 * bytes_done) / bytes_total : 0;
  
  bytes_rate_avg = rate_get(&tp->rate);

  if (tp->t_max)
    time_left = tp->t_max - ts_delta(now, &tp->t0, NULL, NULL);
  else
    time_left = bytes_rate_avg ? bytes_left / bytes_rate_avg : 0;
  
  fprintf(fp, "    %15sB (%sB/s): %ld%% done",
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


  rc = dev_delete(dp, tp->start, tp->length);
  if (rc < 0) {
    fprintf(stderr, "%s: Error: %s: TRIM %ld bytes @ %ld failed: %s\n",
	    argv0, dp->path, tp->length, tp->start, strerror(errno));
    exit(1);
  }

  return 0;
}
  
  
int
test_seq(TEST *tp) {
  DEVICE *dp = tp->dev;
  struct timespec now;
  ssize_t len, rc;
  off_t opos;
  int nw, n_write;
  double td;
  

  clock_gettime(CLOCK_REALTIME, &now);
  tp->t0 = now;
  tp->t1 = now;
  
  tp->pos = tp->start;
  opos = tp->pos;
  
  while ((len = tp->end - tp->pos) > 0 &&
	 (!tp->t_max || ts_delta(&now, &tp->t0, NULL, NULL) <= tp->t_max)) {
    
    if (len > tp->b_size)
      len = tp->b_size;

    if (tp->flags & TEST_READ) {
      /* Read original data */
      
      rc = pread(dp->fd, tp->obuf, len, tp->pos);
      if (rc != len) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ %ld: %s\n",
		  argv0,
		  dp->path,
		  len,
		  tp->pos,
		  strerror(errno));
	  exit(1);
	} else {
	  if (rc < dp->sector_size)
	    fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) at %ld\n",
		    argv0,
		    dp->path,
		    rc,
		    tp->pos);
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
	  pattern_fill(tp->wbuf, len, purge_patterns[nw], sizeof(purge_patterns[nw]));
	else {
	  uint32_t rv = mrand48();
	  
	  pattern_fill(tp->wbuf, len, (unsigned char *) &rv, sizeof(rv));
	}
      }
      
      if (tp->flags & TEST_WRITE) {
	/* Write new test data */
	
	rc = pwrite(dp->fd, tp->wbuf, len, tp->pos);
	if (rc != len) {
	  if (rc < 0) {
	    fprintf(stderr, "%s: Error: %s: Writing %ld bytes @ %ld: %s\n",
		    argv0,
		    dp->path,
		    len,
		    tp->pos,
		    strerror(errno));
	    exit(1);
	  } else {
	    if (rc < dp->sector_size)
	      fprintf(stderr, "%s: Error: %s: Short write (%ld bytes) at %ld\n",
		      argv0,
		      dp->path,
		      rc,
		      tp->pos);
	    exit(1);
	  }
	}
      }
      
      if (f_flush || (tp->flags & TEST_FLUSH)) {
	/* Force device to write out data to permanent storage */
	
	rc = dev_flush(dp);
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: FLUSH failed @ %ld: %s\n",
		  argv0, dp->path, tp->pos, strerror(errno));
	  exit(1);
	}
      }
      
      if (tp->flags & TEST_VERIFY) {
	/* Read back block to verify the just written data */
	
	rc = pread(dp->fd, tp->rbuf, len, tp->pos);
	if (rc != len) {
	  if (rc < 0) {
	    fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ %ld: %s\n",
		    argv0,
		    dp->path,
		    len,
		    tp->pos,
		    strerror(errno));
	    exit(1);
	  } else {
	    if (rc < dp->sector_size)
	      fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) at %ld\n",
		      argv0,
		      dp->path,
		      rc,
		      tp->pos);
	    exit(1);
	  }
	}

	if (memcmp(tp->wbuf, tp->rbuf, len) != 0) {
	  fprintf(stderr, "%s: Error: %s: Verify for block at @ %ld\n",
		  argv0,
		  dp->path,
		  tp->pos);
	  exit(1);
	}
      }
    }

    if (f_delete && (tp->flags & TEST_DELETE)) {
      /* TRIM block from (SSD) device */
      
      rc = dev_delete(dp, tp->pos, len);
      if (rc < 0) {
	fprintf(stderr, "%s: Error: %s: TRIM failed @ %ld: %s\n",
		argv0, dp->path, tp->pos, strerror(errno));
	exit(1);
      }
    }
    
    if (tp->flags & TEST_RESTORE) {
      /* Restore the original block */
      
      rc = pwrite(dp->fd, tp->obuf, len, tp->pos);
      if (rc != len) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Fatal: %s: Unable to restore original block @ %ld: %s\n",
		  argv0,
		  dp->path,
		  tp->pos,
		  strerror(errno));
	  exit(1);
	} else {
	  if (rc < dp->sector_size)
	    fprintf(stderr, "%s: Fatal: %s: Short write (%ld bytes) while restoring original block @ %ld\n",
		    argv0,
		    dp->path,
		    rc,
		    tp->pos);
	  exit(1);
	}
      }	
      
      /* Read back block to verify the just written data */
      
      rc = pread(dp->fd, tp->rbuf, len, tp->pos);
      if (rc != len) {
	if (rc < 0) {
	  fprintf(stderr, "%s: Error: %s: Reading %ld bytes @ %ld: %s\n",
		  argv0,
		  dp->path,
		  len,
		  tp->pos,
		  strerror(errno));
	  exit(1);
	} else {
	  if (rc < dp->sector_size)
	    fprintf(stderr, "%s: Error: %s: Short read (%ld bytes) at %ld\n",
		    argv0,
		    dp->path,
		    rc,
		    tp->pos);
	  exit(1);
	}
      }
      
      if (memcmp(tp->obuf, tp->rbuf, len) != 0) {
	fprintf(stderr, "%s: Fatal: %s: Verify failed while restoring original block @ %ld\n",
		argv0,
		dp->path,
		tp->pos-len);
	exit(1);
      }
    }
    
    /* Now we can allow signals again */
    signal(SIGINT, SIG_DFL);
    if (abort_f) {
      fprintf(stderr, "\n*** Aborted ***\n");
      exit(1);
    }
      
    if (tp->checksum) {
      unsigned char *tbuf = tp->obuf;
      
      if (tp->flags & TEST_WRITE)
	tbuf = tp->wbuf;
      
      if (tp->flags & TEST_VERIFY)
	tbuf = tp->rbuf;
      
      switch (tp->checksum) {
      case CHECK_SHA224:
	SHA224_Update(&tp->ctx.sha224, tbuf, rc);
	break;
	
      case CHECK_SHA256:
	SHA256_Update(&tp->ctx.sha256, tbuf, rc);
	break;
	
      case CHECK_SHA512:
	SHA512_Update(&tp->ctx.sha512, tbuf, rc);
	break;
      }
    }

    tp->b_last = len;
    tp->pos += len;
    
    clock_gettime(CLOCK_REALTIME, &now);
    
    td = ts_delta(&now, &tp->t1, NULL, NULL);
    if (td*1000 > f_update_freq) {
      off_t bytes = tp->pos - opos;
      
      if (td) {
	off_t bytes_rate;
	
	bytes_rate = bytes / td;
	rate_update(&tp->rate, bytes_rate);
	
	opos = tp->pos;
	tp->t1 = now;
      }

      test_pstatus(stderr, tp, &now);
      fputc('\r', stderr);
    }
  }
  
  test_pstatus(stderr, tp, &now);
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
p_buffer(FILE *fp,
	 unsigned char *buf,
	 size_t size,
	 int mode) {
  unsigned long i;

  
  for (i = 0; i < size; i++) {
    if (mode && !(i%16)) { 
      putc('\n', fp);
      putc(' ', fp);
      if (mode > 1)
	fprintf(fp, "%08lX : ", i);
    }
    if (mode)
      putc(' ', fp);
    fprintf(fp, "%02X", buf[i]);
  }
  if (mode)
    putc('\n', fp);
}


void
usage(FILE *fp) {
  fprintf(fp, "Usage:\n  %s [<options>] [<actions>]\n",
	  argv0);
  fprintf(fp, "\nOptions:\n");
  fprintf(fp, "  -h               Display this information\n");
  fprintf(fp, "  -v               Increase verbosity\n");
  fprintf(fp, "  -w               Open device in R/W mode (needed for write tests)\n");
  fprintf(fp, "  -f               Flush device write buffer\n");
  fprintf(fp, "  -d               Send TRIM commands to device\n");
  fprintf(fp, "  -p               Print last block\n");
  fprintf(fp, "  -C <type>        Checksum type (SHA224 - SHA256 - SHA512)\n");
  fprintf(fp, "  -D <device>      Disk device\n");
  fprintf(fp, "  -T <time>        Limit test time\n");
  fprintf(fp, "  -P <num>         Passes [%d]\n", d_passes);
  fprintf(fp, "  -S <off>         Start offset [0]\n");
  fprintf(fp, "  -L <off>         Length [ALL]\n");
  fprintf(fp, "  -B <size>        Block size [<native>]\n");
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
str2checksum(const char *s) {
  if (strcasecmp(s, "224") == 0 || strcasecmp(s, "SHA224") == 0 || strcasecmp(s, "SHA-224") == 0)
    return CHECK_SHA224;
  
  if (strcasecmp(s, "256") == 0 || strcasecmp(s, "SHA256") == 0 || strcasecmp(s, "SHA-256") == 0)
    return CHECK_SHA256;

  if (strcasecmp(s, "512") == 0 || strcasecmp(s, "SHA512") == 0 || strcasecmp(s, "SHA-512") == 0)
    return CHECK_SHA512;

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

  char *s_checksum = NULL;
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

      case 'C':
	if (argv[i][j+1])
	  s_checksum = argv[i]+j+1;
	else if (argv[i+1])
	  s_checksum = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -C: Missing checksum type\n", argv0);
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
	
      case 'D':
	if (argv[i][j+1])
	  arg = argv[i]+j+1;
	else if (argv[i+1])
	  arg = argv[++i];
	else {
	  fprintf(stderr, "%s: Error: -D: Missing device\n", argv0);
	  exit(1);
	}
	
	dev = dev_open(arg);
	if (!dev) {
	  fprintf(stderr, "%s: Error: %s: Unable to open device: %s\n",
		  argv0, arg, strerror(errno));
	  exit(1);
	}
	dev_print(stdout, dev);
	putchar('\n');
	goto NextArg;

      default:
	fprintf(stderr, "%s: Error: -%c: Invalid switch at %d:%d\n", argv0, argv[i][j], i, j);
	exit(1);
      }
    }

  NextArg:;
  }

  if (dev) {
    test_init(&tst, dev);

    if (s_bsize) {
      rc = str2off(s_bsize, &tst.b_size, 1, dev);
      if (rc < 1) {
	fprintf(stderr, "%s: Error: %s: Invalid block size\n",
		argv0, s_bsize);
	exit(1);
      }
    }
    
    if (s_start) {
      off_t v = 0;
      
      rc = str2off(s_start, &v, 0, dev);
      if (rc < 1 || test_set_start(&tst, v) < 0) {
	fprintf(stderr, "%s: Error: %s: Invalid start position\n",
		argv0, s_start);
	exit(1);
      }
    }

    if (s_length) {
      off_t v = 0;

      if (strcasecmp(s_length, "ALL") == 0 || strcasecmp(s_length, "0E") == 0) {
	v = tst.dev->media_size - tst.start;
	rc = 1;
      } else
	rc = str2off(s_length, &v, 0, dev);
      
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

    if (s_checksum) {
      tst.checksum = str2checksum(s_checksum);
      if (tst.checksum < 0) {
	fprintf(stderr, "%s: Error: %s: Invalid checksum type\n",
		argv0, s_checksum);
	exit(1);
      }
    }
	
    if (f_verbose) {
      printf("Blocks:\n");
      printf("  Size:   %lu\n", tst.b_size);
      printf("  Start:  %ld\n", tst.start/tst.b_size);
      printf("  Length: %ld\n", tst.length/tst.b_size);
      printf("Device:\n");
      printf("  Stripe Offset: %ld\n", tst.dev->stripe_offset);
      printf("  Front Stuff:   %ld\n", tst.dev->front_reserved);
    }
  }
  
  for (; i < argc; i++) {
    unsigned int pass;
    char tbuf[256], bbuf[256], sbuf[256];
    char *tstname = "?";
    off_t bytes = tst.length;
    off_t blocks = bytes / tst.b_size;
    int rc;

    
    if (strcmp(argv[i], "help") == 0) {
      
      usage(stdout);
      exit(0);
      
    } else if (strcmp(argv[i], "read") == 0) {
      
      tstname = "Sequential Read Test";
      tst.rbuf = tst.obuf;
      tst.flags = TEST_READ;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "refresh") == 0) {
      
      tstname = "Sequential Refresh Test";
      tst.wbuf = tst.obuf;
      tst.flags = TEST_READ|TEST_WRITE;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "verify") == 0) {
      
      tstname = "Sequential Refresh+Verify Test";
      tst.wbuf = tst.obuf;
      tst.flags = TEST_READ|TEST_WRITE|TEST_VERIFY;
      tstfun = test_seq;

    } else if (strcmp(argv[i], "test") == 0) {
      
      tstname = "Sequential Pattern Test";
      tst.flags = TEST_READ|TEST_PATTERN|TEST_WRITE|TEST_VERIFY|TEST_RESTORE;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "write") == 0) {

      tstname = "Sequential Write Test";
      
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
      
      tstname = "Sequential Write+Read+Verify Test";
      
      rc = prompt_yes("About to start %s. This will corrupt any data on the device.\nContinue?",
		      tstname);
      if (rc != 1) {
	fprintf(stderr, "*** Aborted ***\n");
	exit(0);
      }
      
      tst.flags = TEST_WRITE|TEST_PATTERN|TEST_VERIFY;
      tstfun = test_seq;
      
    } else if (strcmp(argv[i], "purge") == 0) {
      
      tstname = "Sequential Purge";
      
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
    
    printf("%s (%sB / %s blocks @ %sB):\n",
	   tstname,
	   int2str(bytes,  tbuf, sizeof(tbuf), 0),
	   int2str(blocks, bbuf, sizeof(bbuf), 0),
	   int2str(tst.b_size, sbuf, sizeof(sbuf), 1));
    
    for (pass = 1; pass <= tst.passes; ++pass) {
      if (tst.flags & TEST_PATTERN) {
	memcpy(sel_pattern, test_patterns[(pass-1) % NUM_TEST_PATTERNS], sizeof(sel_pattern));
	pattern_fill(tst.wbuf, tst.b_size, sel_pattern, sizeof(sel_pattern));
	
	printf("  Pass %u [", pass);
	p_buffer(stdout, sel_pattern, sizeof(sel_pattern), 0);
	printf("]:\n");
      } else if (tst.passes > 1)
	printf("  Pass %u:\n", pass);
      
      switch (tst.checksum) {
      case CHECK_SHA224:
	SHA224_Init(&tst.ctx.sha224);
	break;
	
      case CHECK_SHA256:
	SHA256_Init(&tst.ctx.sha256);
	break;
	
      case CHECK_SHA512:
	SHA512_Init(&tst.ctx.sha512);
	break;
      }

      rc = (*tstfun)(&tst);
  
      switch (tst.checksum) {
      case CHECK_SHA224:
	SHA224_Final(digest, &tst.ctx.sha224);

	if (pass == tst.passes) {
	  printf("SHA224 Digest:");
	  p_buffer(stdout, digest, 32, 1);
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
	
      case CHECK_SHA256:
	SHA256_Final(digest, &tst.ctx.sha256);

	if (pass == tst.passes) {
	  printf("SHA256 Digest:");
	  p_buffer(stdout, digest, 32, 1);
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
	
      case CHECK_SHA512:
	SHA512_Final(digest, &tst.ctx.sha512);
	
	if (pass == tst.passes) {
	  printf("SHA512 Digest:");
	  p_buffer(stdout, digest, 64, 1);
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
      printf("Last Data Block (%lu bytes @ %lu):", tst.b_last, tst.pos-tst.b_last);
      p_buffer(stdout, tst.rbuf, tst.b_last, 2);
    }
  }
  
  exit(0);
}
