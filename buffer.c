/*
** buffer.c - Buffer handling stuff
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

#include "buffer.h"


int
buf_init(BUFFER *bp,
	 size_t size) {
  size_t n_size = BUFFER_SLACKIFY(size+1);
  bp->data = malloc(n_size);
  if (!bp->data)
    return -1;
  
  bp->size = n_size;
  bp->length = 0;
  bp->data[bp->length] = '\0';

  return 0;
}


void
buf_destroy(BUFFER *bp) {
  bp->length = 0;
  
  if (bp->data) {
    bp->data[bp->length] = '\0';
    free(bp->data);
  }
  
  bp->data = NULL;
  bp->size = 0;
}


int
buf_resize(BUFFER *bp,
	   size_t size) {
  size_t n_size = BUFFER_SLACKIFY(size+1);
  void *n_data = realloc(bp->data, n_size);
  if (!n_data)
    return -1;

  bp->size = n_size;
  
  if (bp->length > n_size) {
    bp->length = n_size;
    bp->data[bp->length] = '\0';
  }

  return bp->length;
}


int
buf_truncate(BUFFER *bp,
	     size_t size) {
  if (bp->length > size) {
    bp->length = size;
    bp->data[bp->length] = '\0';
  }

  return bp->length;
}


char *
buf_get(BUFFER *bp,
	size_t *size) {
  if (size)
    *size = bp->length;
  
  return bp->data;
}


int
buf_set(BUFFER *bp,
	unsigned char *data,
	size_t size) {
  if (size > bp->size) {
    if (buf_resize(bp, size) < 0)
      return -1;
  }

  memcpy(bp->data, data, size);
  bp->length = size;
  bp->data[bp->length] = '\0';


  return bp->length;
}


int
buf_add(BUFFER *bp,
	unsigned char *data,
	size_t size) {
  size_t n_length = bp->length + size;
  
  if (n_length > bp->size) {
    if (buf_resize(bp, n_length) < 0)
      return -1;
  }

  memcpy(bp->data + bp->length, data, size);
  bp->length += size;
  bp->data[bp->length] = '\0';

  return bp->length;
}



	
