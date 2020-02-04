/*
** blocks.c - Blocks functions
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

#include <stdlib.h>

#include "blocks.h"

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


void
blocks_free(BLOCKS *bp) {
  if (bp->v)
    free(bp->v);
  bp->v = NULL;
  bp->s = 0;
  free(bp);
}


static inline off_t
off_rand(off_t size) {
  off_t r;

  r = ((off_t) lrand48() << 31) | lrand48();
  return r % size;
}


static inline void
off_swap(off_t *a,
	 off_t *b) {
#if 1
  off_t t = *a;
  
  *a = *b;
  *b = t;
#else
  if (a != b) {
    *a ^= *b;
    *b ^= *a;
    *a ^= *b;
  }
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
    off_t j = off_rand(i+1);
    
    off_swap(&bp->v[i], &bp->v[j]);
  }
}


off_t
blocks_lookup(BLOCKS *bp,
	      off_t pos,
	      off_t max) {
  if (!bp)
    return pos;
  
  if (bp->s) {
    off_t b_offset = pos / bp->s;
    return bp->v[pos % bp->s] + b_offset * bp->s;
  }
  
  return off_rand(max);
}
