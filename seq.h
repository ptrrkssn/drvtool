/*
** seq.h - Number sequence manipulation functions
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

#ifndef SEQ_H
#define SEQ_H 1

#include <sys/types.h>


typedef enum seqtype {
  SEQ_TYPE_NONE  = 0,
  SEQ_TYPE_RANGE = 1,
  SEQ_TYPE_ARRAY = 2,
  SEQ_TYPE_LIST  = 3,
} SEQ_TYPE;


typedef struct seq {
  SEQ_TYPE type;
  size_t   entries;
  
  union {
    struct {
      off_t first;
      off_t last;
    } range;
    off_t *array;
  } data;
  
  struct seq *prev;
  struct seq *next;
} SEQ;


/*
 * Create a new sequence. Initialize with:
 * - SEQ_TYPE_NONE (no data)
 * - RANGE (off_t first, off_t last)
 * - ARRAY (size_t entries, off_t *array)
 * - LIST (size_t entries, off_t v1, ..., off_t vN)
 */
extern SEQ *
seq_create(SEQ_TYPE type, ...);

/*
 * Deallocate a sequence
 */
extern int
seq_destroy(SEQ *a);

/*
 * Append more RANGE, ARRAY or LIST data to a sequence
 */
extern int
seq_append(SEQ *a,
	   SEQ_TYPE type, ...);

/*
 * Insert sequence B after sequence A
 */
extern int
seq_splice(SEQ *a,
	   SEQ *b);

/*
 * Compare two sequences for equivalence
 */
extern ssize_t
seq_compare(SEQ *a,
	    SEQ *b);

/*
 * Return the total number of entries in a sequence
 */
extern ssize_t
seq_entries(SEQ *a,
	    size_t *ep);

/*
 * Merge adjoining continous ARRAY and RANGE segments
 */
extern int
seq_merge(SEQ *a);

/*
 * Sort all values each ARRAY and RANGE segments (individually)
 * If cmpfun is NULL then the sort will be in numerically increasing order.
 */
extern int
seq_sort(SEQ *a,
	 int (*cmpfun)(const off_t *v1, const off_t *v2));

/*
 * Remove duplicate continous values in ARRAY segments
 */
extern int
seq_uniq(SEQ *a);

/*
 * Convert all RANGE segments into ARRAY:s
 */
extern int
seq_expand(SEQ *a);

/*
 * Convert all ARRAY segments into RANGEs (where possible)
 */
extern int
seq_compact(SEQ *a);


/*
 * Randomize the contents of all ARRAY segments in the sequence
 */
extern int
seq_shuffle(SEQ *a);


/*
 * Iterate over all values in sequence A
 * if d is > 0 start from the first value else if < 0 start from the end.
 */
extern int
seq_iter(SEQ *a, int d, off_t *ip, off_t *vp);


/*
 * Get value at index I from A
 * Returns 1 if value found, 0 if outside sequence range and 
 * else -1 if an error occured.
 */
extern int
seq_get(SEQ *a,
	ssize_t i,
	off_t *vp);

/*
 * Check if sequence A contains all values in sequence B
 * Returns 1 if true, 0 if false and else -1 in case of error.
 */
extern int
seq_contains(SEQ *a,
	     SEQ *b);

/*
 * Dump internal representation of a sequence
 */
extern int
seq_print(SEQ *a,
	  FILE *fp);

#endif
