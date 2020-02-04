/*
** seq.c - Sequence functions
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
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <errno.h>

#include "seq.h"



const char *
seq_type2str(SEQ_TYPE type) {
  switch (type) {
  case SEQ_TYPE_NONE:
    return "NONE";
    
  case SEQ_TYPE_RANGE:
    return "RANGE";
    
  case SEQ_TYPE_ARRAY:
    return "ARRAY";
    
  case SEQ_TYPE_LIST: /* Special type - internally equivalent to ARRAY */
    return "ARRAY";
  }

  return NULL;
}



int
_seq_init(SEQ *sp,
	  SEQ_TYPE type,
	  va_list ap) {
  off_t first, last;
  size_t i;
  
  
  memset(sp, 0, sizeof(*sp));
  
  sp->type = type;
  sp->entries = 0;
  
  sp->next = sp;
  sp->prev = sp;
  
  switch (type) {
  case SEQ_TYPE_NONE:
    break;

  case SEQ_TYPE_RANGE:
    first = va_arg(ap, off_t);
    last  = va_arg(ap, off_t);
    sp->data.range.first = first;
    sp->data.range.last = last;
    sp->entries = llabs(last - first) + 1;
    break;

  case SEQ_TYPE_ARRAY:
    sp->entries = va_arg(ap, size_t);
    sp->data.array = malloc(sp->entries * sizeof(sp->data.array[0]));
    if (!sp->data.array)
      return -1;
    memcpy(sp->data.array, va_arg(ap, off_t *), sp->entries * sizeof(sp->data.array[0]));
    break;

  case SEQ_TYPE_LIST:
    sp->entries = va_arg(ap, size_t);
    sp->data.array = malloc(sp->entries * sizeof(sp->data.array[0]));
    if (!sp->data.array)
      return -1;
    for (i = 0; i < sp->entries; i++)
      sp->data.array[i] = va_arg(ap, off_t);
    sp->type = SEQ_TYPE_ARRAY;
    break;
    
  default:
    return -1;
  }

  return 0;
}


SEQ *
_seq_create(SEQ_TYPE type,
	    va_list ap) {
  SEQ *sp;
  

  sp = malloc(sizeof(*sp));
  if (!sp)
    return NULL;

  if (_seq_init(sp, type, ap) < 0) {
    free(sp);
    return NULL;
  }
  
  return sp;
}


SEQ *
seq_create(SEQ_TYPE type,
	   ...) {
  va_list ap;

  
  va_start(ap, type);
  return _seq_create(type, ap);
}

int
_seq_destroy(SEQ *sp) {
  if (!sp)
    return -1;


  if (sp->next)
    sp->next->prev = sp->prev;
  if (sp->prev)
    sp->prev->next = sp->next;
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
    break;
    
  case SEQ_TYPE_RANGE:
    sp->data.range.first = 0;
    sp->data.range.last = 0;
    break;
    
  case SEQ_TYPE_ARRAY:
    if (sp->data.array) {
      free(sp->data.array);
      sp->data.array = NULL;
    }
    break;
    
  default:
    break;
  }
  
  sp->type = SEQ_TYPE_NONE;
  sp->entries = 0;

  free(sp);
  return 1;
}

 
int
seq_destroy(SEQ *sp) {
  if (!sp)
    return -1;

  while (sp != sp->next) {
    SEQ *next = sp->next;
    
    if (_seq_destroy(sp) < 0)
      return -1;
    
    sp = next;
  }

  return 0;
}



/* 
 *      A0      ->      B1(A0)             (if SEQ_TYPE_NONE)
 *      A1      ->      A1 - B1
 * AP - A1      -> AP - A1 - B1
 *      A1 - AN ->      A1 - B1 - AN
 * AP - A1 - AN -> AP - A1 - B1 - AN
 *
 *      A0      ->      B1(A0) - B2        (if SEQ_TYPE_NONE)
 *      A1      ->      A1 - B1 - B2
 * AP - A1      -> AP - A1 - B1 - B2
 *      A1 - AN ->      A1 - B1 - B2 - AN
 * AP - A1 - AN -> AP - A1 - B1 - B2 - AN
 */
int
seq_splice(SEQ *a,
	   SEQ *b) {
  SEQ *al, *bl, *an, *ap, *bn, *bp;

  
  if (!a || !b || a == b) {
#ifdef DEBUG
    fprintf(stderr, "SEQ_SPLICE: FATA: A=%p, B=%p\n", a, b);
#endif
    errno = EINVAL;
    return -1;
  }

  an = a->next;
  ap = a->prev;
  bn = b->next;
  bp = b->prev;
  
  if (a->type == SEQ_TYPE_NONE) {
    if (a->next != a || a->prev != a) {
#ifdef DEBUG
      fprintf(stderr, "INTERNAL ERROR: NONE segment not alone\n");
#endif
      abort();
    }

#ifdef DEBUG
    fprintf(stderr, "SPLICE: NONE-segment (A=%p, B=%p, AN=%p, BN=%p, AP=%p, BP=%p)\n",
	    a, b, an, bn, ap, bp);
#endif
    
    a->type = b->type;
    a->entries = b->entries;
    memcpy(&a->data, &b->data, sizeof(a->data));

    a->next = (bn == b ? a : bn);
    a->prev = (bp == b ? a : bp);
    
    free(b);

#ifdef DEBUG
    fprintf(stderr, "SPLICED %p: T=%d, E=%ld, P=%p, N=%p\n",
	    a, a->type, a->entries, a->prev, b->prev);
#endif
    
    return 0;
  }
  
  al = a->prev;
  an = a->next;
  bl = b->prev;
  bn = b->next;
  
  a->next  = b;
  b->prev  = a;
  
  bl->next = an;
  an->prev = bl;
  
  return 0;
}


int
seq_append(SEQ *sp,
	   SEQ_TYPE type,
	   ...) {
  SEQ *nsp;
  va_list ap;
  
  
  if (!sp) {
    errno = EFAULT;
    return -1;
  }

  va_start(ap, type);

  nsp = _seq_create(type, ap);
  if (!nsp) {
#ifdef DEBUG
    fprintf(stderr, "%s: seq_create() failed: %s\n",
	    seq_type2str(type), strerror(errno));
#endif
    return -1;
  }

#ifdef DEBUG
  fprintf(stderr, "SEQ_APPEND: seq_splice(sp=%p, nsp=%p)\n", sp, nsp);
#endif
  return seq_splice(sp, nsp);
}



int
_seq_foreach(SEQ *sp,
	     int (*f)(SEQ *sp, void *xp),
	     void *xp) {
  int rc;
  SEQ *head;

  if (!sp)
    return -1;

  head = sp;
  do {
    rc = (*f)(sp, xp);
    if (rc)
      return rc;

    sp = sp->next;
  } while (sp != head);
  
  return 0;
}


int
_seq_expand(SEQ *sp,
	    void *xp) {
  off_t *ov, o;
  size_t i;
  int d;
  

  if (!sp)
    return -1;
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
  case SEQ_TYPE_ARRAY:
    break;
    
  case SEQ_TYPE_RANGE:
    ov = calloc(sp->entries, sizeof(*ov));
    if (!ov)
      return -1;
    
    o = sp->data.range.first;
    d = sp->data.range.first < sp->data.range.last ? 1 : -1;
    
    sp->type = SEQ_TYPE_ARRAY;
    
    for (i = 0; i < sp->entries; i++) {
      ov[i] = o;
      o += d;
    }
    
    sp->data.array = ov;
    break;
    
  default:
    return -1;
  }
  
  return 0;
}



int
_seq_compact(SEQ *sp,
	     void *xp) {
  off_t first, last;
  size_t i;
  int d;

  
  if (!sp)
    return -1;

  switch (sp->type) {
  case SEQ_TYPE_NONE:
  case SEQ_TYPE_RANGE:
    break;
    
  case SEQ_TYPE_ARRAY:
    if (sp->entries < 2)
      break;
    
    /* Get range potential direction */
    d = sp->data.array[0] < sp->data.array[1] ? 1 : -1;
    
    /* Check range for monotone +1/-1 step */
    first = sp->data.array[0];
    last = first + d;
    for (i = 1; i < sp->entries && last == sp->data.array[i]; i++)
      last += d;

    last -= d;
    
    /* Range not complete */
    if (i < sp->entries) {
      if (i > 2) {
	SEQ *nsp = seq_create(SEQ_TYPE_ARRAY, sp->entries-i, sp->data.array+i);
	if (!nsp)
	  return -1;

	free(sp->data.array);
	sp->type = SEQ_TYPE_RANGE;
	sp->entries = i;
	sp->data.range.first = first;
	sp->data.range.last = last;

#ifdef DEBUG
	fprintf(stderr, "_SEQ_COMPACT: seq_splice(sp=%p, nsp=%p)\n", sp, nsp);
#endif
	if (seq_splice(sp, nsp) < 0)
	  return -1;
      }
    } else {
      free(sp->data.array);
      sp->type = SEQ_TYPE_RANGE;
      sp->data.range.first = first;
      sp->data.range.last = last;
    }
    break;
    
  default:
    return -1;
  }
  
  return 0;
}


int
_seq_merge(SEQ *sp,
	   void *xp) {
  SEQ *nsp, *head;
  int d, nd;
  size_t i, ne;
  off_t *nov;

  
  if (!sp)
    return -1;

  head = sp;
  
  while (sp->next != head && sp->type == sp->next->type) {
    nsp = sp->next;

    switch (sp->type) {
    case SEQ_TYPE_NONE:
      abort();
      return -1;
      
    case SEQ_TYPE_RANGE:
      d  =  sp->data.range.first <=  sp->data.range.last ? 1 : -1;
      nd = nsp->data.range.first <= nsp->data.range.last ? 1 : -1;
      if (d != nd || sp->data.range.last + d != nsp->data.range.first)
	return 0;
      
#ifdef DEBUG
      fprintf(stderr, "_SEQ_MERGE(RANGE) A=%ld-%ld, B=%ld-%ld\n",
	      sp->data.range.first, sp->data.range.last,
	      nsp->data.range.first, nsp->data.range.last);
#endif
      
      sp->data.range.last = nsp->data.range.last;
      sp->entries = llabs(sp->data.range.last - sp->data.range.first) + 1;

      sp->next = nsp->next;
      nsp->next->prev = sp;
      
      free(nsp);
      break;
      
    case SEQ_TYPE_ARRAY:
#ifdef DEBUG
      fprintf(stderr, "_SEQ_MERGE(ARRAY) AE=%ld, BE=%ld\n", 
	      sp->entries, nsp->entries);
#endif
      
      ne  = sp->entries + nsp->entries;
      nov = realloc(sp->data.array, ne * sizeof(*nov));
      if(!nov)
	return -1;
      for (i = 0; i < nsp->entries; i++)
	nov[sp->entries + i] = nsp->data.array[i];
      
      sp->data.array = nov;
      sp->entries = ne;
      
      sp->next = nsp->next;
      nsp->next->prev = sp;
      
      free(nsp->data.array);
      free(nsp);
      break;
      
    default:
      return -1;
    }
  }
  
  return 0;
}


int
seq_get(SEQ *sp,
	ssize_t i,
	off_t *vp) {
  SEQ *head = sp;
  
  
  if (i >= 0) {

    head = sp;    
    while (i >= sp->entries) {
      i -= sp->entries;
      sp = sp->next;
      
      if (sp == head)
	return 0;
    }
    
    switch (sp->type) {
    case SEQ_TYPE_NONE:
      return 0;
      
    case SEQ_TYPE_RANGE:
      if (sp->data.range.first < sp->data.range.last)
	*vp = sp->data.range.first + i;
      else
	*vp = sp->data.range.first - i;
      return 1;

    case SEQ_TYPE_ARRAY:
      *vp = sp->data.array[i];
      return 1;
      
    default:
      return -1;
    }
    
  } 

  /* Negative - backwards */
  i = -i-1;
  
  head = sp;
  while (i >= sp->entries) {
    i -= sp->entries;
    sp = sp->prev;
    if (sp == head)
      return 0;
  }

  switch (sp->type) {
  case SEQ_TYPE_NONE:
    return 0;
    
  case SEQ_TYPE_RANGE:
    if (sp->data.range.first < sp->data.range.last)
      *vp = sp->data.range.last - i;
    else
      *vp = sp->data.range.last + i;
    return 1;
    
  case SEQ_TYPE_ARRAY:
    *vp = sp->data.array[sp->entries-i-1];
    return 1;
    
  default:
    return -1;
  }
}

    

static inline void
off_swap(off_t *a,
	 off_t *b) {
#if 0
  off_t t = *a;
  
  *a = *b;
  *b = t;
#else
  if (a != b) {
    *a = *a ^ *b;
    *b = *b ^ *a;
    *a = *a ^ *b;
  }
#endif
}


static inline size_t
size_rand(size_t size) {
  off_t r;

  
  r = ((off_t) lrand48() << 31) | lrand48();
  return r % size;
}


/* 
 * Shuffle using Fisher-Yates shuffe algorithm.
 */
int
_seq_shuffle(SEQ *sp,
	     void *xp) {
  size_t i;


  if (!sp)
    return -1;
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
    break;
    
  case SEQ_TYPE_RANGE:
    if (_seq_expand(sp, NULL) < 0)
      return -1;
    
  case SEQ_TYPE_ARRAY:
    for (i = sp->entries-1; i > 0; i--) {
      size_t j = size_rand(i+1);
      
      off_swap(&sp->data.array[i], &sp->data.array[j]);
    }
    break;
    
  default:
    return -1;
  }

  return 0;
}


typedef int (*SORTFUN)(const void *a, const void *b);


static int
off_cmp(const void *va,
	const void *vb) {
  off_t a = * (off_t *) va;
  off_t b = * (off_t *) vb;

  return a-b;
}


int
_seq_sort(SEQ *sp,
	  void *xp) {
  SORTFUN sf = xp ? (SORTFUN) xp : off_cmp;
  off_t t;

  
  if (!sp)
    return -1;
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
    break;
    
  case SEQ_TYPE_RANGE:
    if ((*sf)(&sp->data.range.first, &sp->data.range.last) > 0) {
      t = sp->data.range.first;
      sp->data.range.first = sp->data.range.last;
      sp->data.range.last = t;
    }
    break;
    
  case SEQ_TYPE_ARRAY:
    qsort(sp->data.array, sp->entries, sizeof(off_t), sf);
    break;

  default:
    return -1;
  }
  
  return 0;
}


/*
 * Check if A contains V
 */
int
_seq_contains_off(SEQ *a,
		  off_t tv) {
  off_t ai, av;


  for (ai = 0; seq_get(a, ai, &av) == 1; ai++)
    if (av == tv)
      return 1;

  return 0;
}

/*
 * Check if A contains all values in B
 */
int
seq_contains(SEQ *a,
	     SEQ *b) {
  off_t bi, bv;

  for (bi = 0; seq_get(b, bi, &bv) == 1; bi++)
    if (_seq_contains_off(a, bv) != 1)
      return 0;

  return 1;
}

int
_seq_uniq(SEQ *sp,
	  void *xp) {
  size_t i, j;

  
  if (!sp)
    return -1;
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
    break;
    
  case SEQ_TYPE_RANGE:
    break;
    
  case SEQ_TYPE_ARRAY:
    i = 0;
    for (i = 0; i < sp->entries-1; i++) {
      while (i < sp->entries-1 && sp->data.array[i] == sp->data.array[i+1]) {
	for (j = i; j < sp->entries-1; j++)
	  sp->data.array[j] = sp->data.array[j+1];
	sp->entries--;
      }
    }
    break;
    
  default:
    return -1;
  }
  
  return 0;
}


int
_seq_print(SEQ *sp,
	   void *xp) {
  size_t i = 0, ns = 0;
  off_t v = -1;
  FILE *fp = xp ? (FILE *) xp : stdout;


  if (!sp)
    return -1;
  
  fprintf(fp, "Segment %ld @ %p [prev=%p, next=%p] (%ld entries):\n  ",
	  ns++,
	  sp,
	  sp->prev,
	  sp->next,
	  sp->entries);
  
  switch (sp->type) {
  case SEQ_TYPE_NONE:
    fputs("-\n", fp);
    break;

  case SEQ_TYPE_RANGE:
    fprintf(fp, "%ld - %ld\n", sp->data.range.first, sp->data.range.last);
    break;
    
  case SEQ_TYPE_ARRAY:
    for (i = 0; i < sp->entries && seq_get(sp, i, &v) == 1; i++) {
      if (i) {
	putc(',', fp);
	putc(' ', fp);
      }
      fprintf(fp, "%ld", v);
    }
    putc('\n', fp);
    break;
    
  default:
    return -1;
  }

  return 0;
}

int
seq_merge(SEQ *sp) {
  return _seq_foreach(sp, _seq_merge, NULL);
}


int
_seq_entries(SEQ *sp,
	     void *xp) {
  size_t *ep = (size_t *) xp;

  
  if (!sp)
    return -1;
  
  *ep += sp->entries;
  return 0;
}

ssize_t
seq_entries(SEQ *sp,
	    size_t *ne) {
  size_t e;
  int rc;

  
  if (!ne)
    ne = &e;

  *ne = 0;
  rc = _seq_foreach(sp, _seq_entries, (void *) ne);
  if (rc < 0)
    return rc;

  return *ne;
}


int
seq_expand(SEQ *sp) {
  return _seq_foreach(sp, _seq_expand, NULL);
}


int
seq_compact(SEQ *sp) {
  return _seq_foreach(sp, _seq_compact, NULL);
}


int
seq_shuffle(SEQ *sp) {
  return _seq_foreach(sp, _seq_shuffle, NULL);
}


int
seq_sort(SEQ *sp,
	 int (*cmpfun)(const off_t *v1, const off_t *v2)) {
  return _seq_foreach(sp, _seq_sort, (SORTFUN) cmpfun);
}


int
seq_uniq(SEQ *sp) {
  return _seq_foreach(sp, _seq_uniq, NULL);
}


ssize_t
seq_compare(SEQ *a,
	    SEQ *b) {
  off_t ai, bi, av, bv;
  size_t d;
  int rc;
  
  
  ai = bi = 0;
  while ((d = (rc = seq_iter(a, 1, &ai, &av)) - seq_iter(b, 1, &bi, &bv)) == 0 && rc) {
    d = av - bv;
    if (d)
      return d;
  }

  return d;
}


int
seq_print(SEQ *sp,
	  FILE *fp) {
  return _seq_foreach(sp, _seq_print, (void *) fp);
}


int
seq_iter(SEQ *sp,
	 int d,
	 off_t *ip,
	 off_t *v) {
  int rc;

  rc = seq_get(sp, (d > 0 ? *ip : -(1+*ip)), v);
  ++*ip;
  return rc;
}


