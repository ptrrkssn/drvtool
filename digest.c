/*
** digest.h - Digest/Checksum functions
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

#include <errno.h>
#include <string.h>
#include <zlib.h>

#include <arpa/inet.h>

#include "digest.h"

int
digest_init(DIGEST *dp,
	    DIGEST_TYPE type) {
  memset(dp, 0, sizeof(*dp));

  dp->state = DIGEST_STATE_NONE;
  
  switch (dp->type) {
  case DIGEST_TYPE_NONE:
    break;
    
  case DIGEST_TYPE_ADLER32:
    dp->ctx.crc32 = adler32_z(0L, NULL, 0);
    break;
    
  case DIGEST_TYPE_CRC32:
    dp->ctx.adler32 = crc32_z(0L, NULL, 0);
    break;
    
  case DIGEST_TYPE_MD5:
    MD5Init(&dp->ctx.md5);
    break;
    
  case DIGEST_TYPE_SKEIN256:
    SKEIN256_Init(&dp->ctx.skein256);
    break;
    
  case DIGEST_TYPE_SHA256:
    SHA256_Init(&dp->ctx.sha256);
    break;
    
  case DIGEST_TYPE_SHA384:
    SHA384_Init(&dp->ctx.sha384);
    break;
    
  case DIGEST_TYPE_SHA512:
    SHA512_Init(&dp->ctx.sha512);
    break;

  default:
    return -1;
  }

  dp->type = type;
  dp->state = DIGEST_STATE_INIT;
  return 0;
}



int
digest_update(DIGEST *dp,
	      unsigned char *buf,
	      size_t bufsize) {

  if (!dp)
    return -1;

  switch (dp->state) {
  case DIGEST_STATE_INIT:
  case DIGEST_STATE_UPDATE:
    switch (dp->type) {
    case DIGEST_TYPE_ADLER32:
      dp->ctx.adler32 = adler32_z(dp->ctx.adler32, buf, bufsize);
      break;
      
    case DIGEST_TYPE_CRC32:
      dp->ctx.adler32 = crc32_z(dp->ctx.crc32, buf, bufsize);
      break;
      
    case DIGEST_TYPE_MD5:
      MD5Update(&dp->ctx.md5, buf, bufsize);
      break;
      
    case DIGEST_TYPE_SKEIN256:
      SKEIN256_Update(&dp->ctx.skein256, buf, bufsize);
      break;
      
    case DIGEST_TYPE_SHA256:
      SHA256_Update(&dp->ctx.sha256, buf, bufsize);
      break;
      
    case DIGEST_TYPE_SHA384:
      SHA384_Update(&dp->ctx.sha384, buf, bufsize);
      break;
      
    case DIGEST_TYPE_SHA512:
      SHA512_Update(&dp->ctx.sha512, buf, bufsize);
      break;
      
    default:
      return -1;
    }
    break;
    
  default:
    return -1;
  }
  
  dp->state = DIGEST_STATE_UPDATE;
  return 0;
}


ssize_t
digest_final(DIGEST *dp,
	     unsigned char *buf,
	     size_t bufsize) {
  ssize_t rlen = -1;

  
  switch (dp->state) {
  case DIGEST_STATE_INIT:
  case DIGEST_STATE_UPDATE:
    
    switch (dp->type) {
    case DIGEST_TYPE_NONE:
      rlen = 0;
      break;
      
    case DIGEST_TYPE_ADLER32:
      if (bufsize < DIGEST_BUFSIZE_ADLER32) {
	errno = EOVERFLOW;
	return -1;
      }
      * (uint32_t *) buf = htonl(dp->ctx.adler32);
      rlen = DIGEST_BUFSIZE_ADLER32;
      break;
      
    case DIGEST_TYPE_CRC32:
      if (bufsize < DIGEST_BUFSIZE_CRC32) {
	errno = EOVERFLOW;
	return -1;
      }
      * (uint32_t *) buf = htonl(dp->ctx.crc32);
      rlen = DIGEST_BUFSIZE_CRC32;
      break;
      
    case DIGEST_TYPE_MD5:
      if (bufsize < DIGEST_BUFSIZE_MD5) {
	errno = EOVERFLOW;
	return -1;
      }
      MD5Final(buf, &dp->ctx.md5);
      rlen = DIGEST_BUFSIZE_MD5;
      break;
      
    case DIGEST_TYPE_SKEIN256:
      if (bufsize < DIGEST_BUFSIZE_SKEIN256) {
	errno = EOVERFLOW;
	return -1;
      }
      SKEIN256_Final(buf, &dp->ctx.skein256);
      rlen = DIGEST_BUFSIZE_SKEIN256;
      break;
      
    case DIGEST_TYPE_SHA256:
      if (bufsize < DIGEST_BUFSIZE_SHA256) {
	errno = EOVERFLOW;
	return -1;
      }
      SHA256_Final(buf, &dp->ctx.sha256);
      rlen = DIGEST_BUFSIZE_SHA256;
      break;
      
    case DIGEST_TYPE_SHA384:
      if (bufsize < DIGEST_BUFSIZE_SHA384) {
	errno = EOVERFLOW;
	return -1;
      }
      SHA384_Final(buf, &dp->ctx.sha384);
      rlen = DIGEST_BUFSIZE_SHA384;
      break;
      
    case DIGEST_TYPE_SHA512:
      if (bufsize < DIGEST_BUFSIZE_SHA512) {
	errno = EOVERFLOW;
	return -1;
      }
      SHA512_Final(buf, &dp->ctx.sha512);
      rlen = DIGEST_BUFSIZE_SHA512;
      break;
    }
    break;
    
  default:
    errno = EINVAL;
    return -1;
  }
  
  dp->state = DIGEST_STATE_FINAL;
  return rlen;
}


DIGEST_TYPE
digest_str2type(const char *s) {
  if (strcasecmp(s, "NONE") == 0)
    return DIGEST_TYPE_NONE;
  
  if (strcasecmp(s, "ADLER32") == 0 || strcasecmp(s, "ADLER-32") == 0)
    return DIGEST_TYPE_ADLER32;

  if (strcasecmp(s, "CRC32") == 0 || strcasecmp(s, "CRC-32") == 0)
    return DIGEST_TYPE_CRC32;

  if (strcasecmp(s, "MD5") == 0 || strcasecmp(s, "MD-5") == 0)
    return DIGEST_TYPE_MD5;

  if (strcasecmp(s, "SKEIN256") == 0 || strcasecmp(s, "SKEIN-256") == 0)
    return DIGEST_TYPE_SKEIN256;

  if (strcasecmp(s, "SHA256") == 0 || strcasecmp(s, "SHA-256") == 0)
    return DIGEST_TYPE_SHA256;

  if (strcasecmp(s, "SHA384") == 0 || strcasecmp(s, "SHA-384") == 0)
    return DIGEST_TYPE_SHA384;

  if (strcasecmp(s, "SHA512") == 0 || strcasecmp(s, "SHA-512") == 0)
    return DIGEST_TYPE_SHA512;

  return -1;
}


const char *
digest_type2str(DIGEST_TYPE type) {
  switch (type) {
  case DIGEST_TYPE_NONE:
    return "NONE";

  case DIGEST_TYPE_ADLER32:
    return "ADLER32";

  case DIGEST_TYPE_CRC32:
    return "CRC32";

  case DIGEST_TYPE_MD5:
    return "MD5";

  case DIGEST_TYPE_SKEIN256:
    return "SKEIN256";
    
  case DIGEST_TYPE_SHA256:
    return "SHA256";
    
  case DIGEST_TYPE_SHA384:
    return "SHA384";
    
  case DIGEST_TYPE_SHA512:
    return "SHA512";

  default:
    return NULL;
  }
}


void
digest_destroy(DIGEST *dp) {
  unsigned char tbuf[DIGEST_BUFSIZE_MAX];

  (void) digest_final(dp, tbuf, sizeof(tbuf));
  memset(dp, 0, sizeof(*dp));
  dp->state = DIGEST_STATE_NONE;
  dp->type  = DIGEST_TYPE_NONE;
}

DIGEST_TYPE
digest_typeof(DIGEST *dp) {
  if (!dp)
    return DIGEST_TYPE_NONE;
  
  return dp->type;
}

DIGEST_STATE
digest_stateof(DIGEST *dp) {
  if (!dp)
    return DIGEST_STATE_NONE;
  
  return dp->state;
}
