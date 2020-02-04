/*
** transform.c - Data transformation functions
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
#include <string.h>
#include "transform.h"



static int
transform_str2type(const char *type,
		   union transform_txd *d) {
  int v;


  if (strcasecmp(type, "NONE") == 0)
    return TRANSFORM_TYPE_NONE;
  
  if (strncasecmp(type, "XOR", 3) == 0) {
    type += 3;
    if (*type == '-')
      ++type;

    v = 0xFF;
    if (*type && !(sscanf(type, "0x%x", &v) == 1 || sscanf(type, "%d", &v) == 1))
      return -1;
    
    d->xor = v;
    return TRANSFORM_TYPE_XOR;
  }

  if (strncasecmp(type, "ROR", 3) == 0) {
    type += 3;
    if (*type == '-')
      ++type;

    v = 1;
    if (*type && !(sscanf(type, "0x%x", &v) == 1 || sscanf(type, "%d", &v) == 1))
      return -1;
    
    d->xor = v;
    return TRANSFORM_TYPE_ROR;
  }

  if (strncasecmp(type, "ROL", 3) == 0) {
    type += 3;
    if (*type == '-')
      ++type;

    v = 1;
    if (*type && !(sscanf(type, "0x%x", &v) == 1 || sscanf(type, "%d", &v) == 1))
      return -1;
    
    d->xor = v;
    return TRANSFORM_TYPE_ROL;
  }

  return -1;
}


int
transform_init(TRANSFORM *tp,
	       const char *s_type) {
  memset(tp, 0, sizeof(*tp));
  tp->type = transform_str2type(s_type, &tp->txd);
  return tp->type;
}


static void
data_xor(unsigned char *buf,
	 size_t bufsize,
	 int b) {
  while (bufsize-- > 0)
    *buf++ ^= b;
}

static void
data_ror(unsigned char *buf,
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

static void
data_rol(unsigned char *buf,
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


int
transform_apply(TRANSFORM *tp,
		unsigned char *buf,
		size_t bufsize) {

  switch (tp->type) {
  case TRANSFORM_TYPE_NONE:
    break;
  case TRANSFORM_TYPE_XOR:
    data_xor(buf, bufsize, tp->txd.xor);
    break;
  case TRANSFORM_TYPE_ROR:
    data_ror(buf, bufsize, tp->txd.ror);
    break;
  case TRANSFORM_TYPE_ROL:
    data_rol(buf, bufsize, tp->txd.rol);
    break;
  }

  return 0;
}


