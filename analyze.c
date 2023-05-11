/*
** analyze.c - Disk tests
**
** Copyright (c) 2020-2021, Peter Eriksson <pen@lysator.liu.se>
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
#include <sys/errno.h>

#include "drvtool.h"


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
test_set_bsize(TEST *tp,
	       off_t bsize) {
  DRIVE *dev;
  off_t ns, tb, o_start, o_length;
  off_t d;
  
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
      return -2;

    /* Make sure block size and total media size is compatible */
    d = dev->media_size - tb * bsize;
    if (d) {
      fprintf(stderr, "%s: Warning: Block Size and Media Size mismatch by %ld bytes\n",
	      argv0, d);
#if 0
      return -3;
#endif
    }
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
  
  if (length < 0) {
    errno = EINVAL;
    return -1;
  }
  if (tp->b_start + length > tp->b_total) {
    fprintf(stderr, "ERR: START=%ld, LEN=%ld, TOT=%ld\n", tp->b_start, length, tp->b_total);
    errno = EOVERFLOW;
    return -1;
  }
  
  tp->b_length = length;
  tp->b_end    = tp->b_start + tp->b_length;

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

  fprintf(fp, "          ");
}


volatile int abort_f = 0;


void
sigint_handler(int sig) {
  signal(SIGINT, sigint_handler);
  abort_f = 1;
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

    b_pos = blocks_lookup(tp->blocks, b, tp->b_length);
    
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

      transform_apply(&tp->transform, tp->wbuf, tp->b_size);
      
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
      
    if (f_digest) {
      unsigned char *tbuf = tp->obuf;
      
      if (tp->flags & TEST_WRITE)
	tbuf = tp->wbuf;
      
      if (tp->flags & TEST_VERIFY)
	tbuf = tp->rbuf;

      digest_update(&tp->digest.data, tbuf, tp->b_size);
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
      if (f_verbose)
	fputc('\n', stderr);
      else
	fputc('\r', stderr);
    }
    
    tp->last_b_pos = b_pos;
    tp->last_pos   = pos;
  }
  
  test_pstatus(stderr, tp, &now, b, b_pos);
  fputc('\n', stderr);
  return 0;
}


int
test_print_config(FILE *fp,
		  TEST *tp) {
  char buf[80];
  off_t c, h, s;
  
  
  fprintf(fp, "TEST CONFIGURATION:\n");

  fprintf(fp, "  Analyze entire drive?          %s\n",
	  tp->b_length == tp->b_total ? "Yes" : "No");
  
  if (tp->b_length != tp->b_total) {
    
    fprintf(fp, "  Starting block number:         %-8ld", tp->b_start);
    if (f_verbose && blocks2chs(tp->b_start, &c, &h, &s, tp->drive) == 1)
      fprintf(fp, " [%ld/%ld/%ld]", c, h, s);
    putc('\n', fp);
    
    fprintf(fp, "  Ending block number:           %-8ld", tp->b_end-1);
    if (f_verbose && blocks2chs(tp->b_end-1, &c, &h, &s, tp->drive) == 1)
      fprintf(fp, " [%ld/%ld/%ld]", c, h, s);
    putc('\n', fp);
  }
  
  fprintf(fp, "  Number of blocks:              %-8ld",
	  tp->b_length);
  
  if (f_verbose && blocks2chs(tp->b_length, &c, &h, &s, tp->drive) == 1)
    fprintf(fp, " [%ld/%ld/%ld]", c, h, s);
  
  fprintf(fp, " (%sB)", 
	  int2str(tp->b_length * tp->b_size, buf, sizeof(buf), 0));
  putc('\n', fp);
  
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
  
  fprintf(fp, "  Number of sectors per block:   %-8ld (%sB)\n",
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
  DIGEST_TYPE type = digest_typeof(&tp->digest.data);
  
  if (type == DIGEST_TYPE_NONE) {
    fprintf(stderr, "%s: Error: DIGEST not selected\n", argv0);
    return 1;
  }
  
  printf("%s DIGEST:\n", digest_type2str(type));
  data_print(stdout,
	    tp->digest.buffer.last.data,
	    tp->digest.buffer.last.len,
	    1);
  
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

    drive_print(stdout, 0, tp->drive, 2);
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
      data_print(stdout, sel_pattern, sizeof(sel_pattern), 0);
      printf("]:\n");
    } else {
      if (tp->passes > 1)
	printf("  Pass %u:\n", pass);
    }

    if (f_digest)
      digest_init(&tp->digest.data, f_digest);
    
    rc = (*tstfun)(tp);

    if (f_digest) {
      int dlen;
      
      dlen = digest_final(&tp->digest.data,
			  tp->digest.buffer.last.data,
			  sizeof(tp->digest.buffer.last.data));
      if (dlen < 0) {
	fprintf(stderr, "%s: Error: %s Digest final failed: %s\n",
		argv0,
		digest_type2str(digest_typeof(&tp->digest.data)),
		strerror(errno));
	exit(1);
      }

      tp->digest.buffer.last.len = dlen;
      
      if (pass == 1) {
	memcpy(tp->digest.buffer.first.data,
	       tp->digest.buffer.last.data, 
	       tp->digest.buffer.last.len);
	tp->digest.buffer.first.len = tp->digest.buffer.last.len;
      }
      else {
	if (tp->digest.buffer.first.len != tp->digest.buffer.last.len ||
	    memcmp(tp->digest.buffer.first.data,
		   tp->digest.buffer.last.data, 
		   tp->digest.buffer.last.len) != 0) {
	  fprintf(stderr, "%s: Error: %s Digest mismatch between pass 1 and pass %d\n",
		  digest_type2str(digest_typeof(&tp->digest.data)),
		  argv0,
		  pass);
	  if (f_verbose) {
	    printf("PASS 1 %s DIGEST (%lu bytes):\n",
		   digest_type2str(digest_typeof(&tp->digest.data)),
		   tp->digest.buffer.last.len);
	    data_print(stdout,
		      tp->digest.buffer.first.data,
		      tp->digest.buffer.first.len,
		      1);
	    printf("PASS %d %s DIGEST (%lu bytes):\n",
		   pass,
		   digest_type2str(digest_typeof(&tp->digest.data)),
		   tp->digest.buffer.last.len);
	    data_print(stdout,
		      tp->digest.buffer.last.data,
		      tp->digest.buffer.last.len,
		      1);
	  }
	  return 1;
	}
      }
    }
  }
  
  return rc;
}



int
read_cmd(int argc,
	 char **argv) {
  
}

static COMMAND read_command =
  { "read-test",      read_cmd, NULL, "", "Read-Only test" };

COMMAND *drive_commands[] =
  {
   &read_command,
   NULL,
  };
