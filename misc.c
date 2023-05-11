/*
 * misc.c - Misc utility functions
 *
 * Copyright (c) 2019-2020, Peter Eriksson <pen@lysator.liu.se>
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

#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <pwd.h>
#include <grp.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <termios.h>

#include "drvtool.h"

#if defined(HAVE_LIBEDIT)
#include <histedit.h>
#include <editline/readline.h>
#elif defined(HAVE_LIBREADLINE)
#include <readline/readline.h>
#include <readline/history.h>
#else
extern char *readline(const char *prompt);
#endif

#define NEW(vp) ((vp) = malloc(sizeof(*(vp))))

/*
 * Calculate the difference between two struct timespec, returns elapsed time i microseconds.
 * Also returns the elapsed time and a unit as a string.
 */
long
ts_delta(struct timespec *x,
	 const struct timespec *y,
	 long *res,
	 char **unit) {
  struct timespec r;
  
  /* Avoid overflow of r.tv_nsec */
  if (x->tv_nsec < y->tv_nsec) {
    x->tv_nsec += 1000000000L;
    x->tv_sec  -= 1;
  }
  
  r.tv_sec  = x->tv_sec - y->tv_sec;
  r.tv_nsec = x->tv_nsec - y->tv_nsec;
  
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
  return r.tv_sec * 1000000 + r.tv_nsec / 1000;
}


#if 0
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
#endif


int
prompt_user(char *buf,
	    size_t bufsize,
	    int echo,
	    const char *prompt,
	    ...) {
  struct termios oflags, nflags;
  int i;
  char *res;
  va_list ap;
  

  va_start(ap, prompt);
  i = vfprintf(stderr, prompt, ap);
  va_end(ap);
  if (i < 0)
    return -1;
  
  if (!echo) {
    if (tcgetattr(fileno(stdin), &oflags) < 0)
      return -1;
    
    nflags = oflags;
    nflags.c_lflag &= ~ECHO;
    nflags.c_lflag |= ECHONL;
    
    if (tcsetattr(fileno(stdin), TCSANOW, &nflags) < 0) {
      return -1;
    }
  }
  
  res = fgets(buf, bufsize, stdin);

  if (res) {
    /* Remove trailing newline(s) */
    i = strlen(buf)-1;
    while (i >= 0 && (buf[i] == '\n' || buf[i] == '\r'))
      --i;
    buf[i+1] = '\0';
  }
  
  if (!echo) {
    /* restore terminal */
    if (tcsetattr(fileno(stdin), TCSANOW, &oflags) <0)
      return -1;
  }
  
  return res ? i : 0;
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
data_print(FILE *fp,
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


int
prompt_yes(const char *msg,
	   ...) {
  char buf[80], *cp;
  va_list ap;


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
