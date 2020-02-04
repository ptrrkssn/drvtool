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

#ifndef DIGEST_H
#define DIGEST_H 1

#include <sys/types.h>

#include <md5.h>
#include <skein.h>
#include <sha256.h>
#include <sha384.h>
#include <sha512.h>


typedef enum {
	      DIGEST_TYPE_INVALID  = -1,
	      DIGEST_TYPE_NONE     = 0,
	      DIGEST_TYPE_ADLER32  = 1,
	      DIGEST_TYPE_CRC32    = 2,
	      DIGEST_TYPE_MD5      = 3,
	      DIGEST_TYPE_SKEIN256 = 4,
	      DIGEST_TYPE_SHA256   = 5,
	      DIGEST_TYPE_SHA384   = 6,
	      DIGEST_TYPE_SHA512   = 7,
} DIGEST_TYPE;


typedef enum {
	      DIGEST_STATE_NONE    = 0,
	      DIGEST_STATE_INIT    = 1,
	      DIGEST_STATE_UPDATE  = 2,
	      DIGEST_STATE_FINAL   = 3,
} DIGEST_STATE;


typedef struct digest {
  DIGEST_TYPE  type;
  DIGEST_STATE state;
  union {
    uint32_t     crc32;
    uint32_t     adler32;
    MD5_CTX      md5;
    SKEIN256_CTX skein256;
    SHA256_CTX   sha256;
    SHA384_CTX   sha384;
    SHA512_CTX   sha512;
  } ctx;
} DIGEST;


/*
 * Result buffer sizes
 */
#define DIGEST_BUFSIZE_ADLER32  sizeof(uint32_t)
#define DIGEST_BUFSIZE_CRC32    sizeof(uint32_t)
#define DIGEST_BUFSIZE_MD5      16
#define DIGEST_BUFSIZE_SKEIN256 32
#define DIGEST_BUFSIZE_SHA256   32
#define DIGEST_BUFSIZE_SHA384   48  
#define DIGEST_BUFSIZE_SHA512   64

#define DIGEST_BUFSIZE_MAX      64


extern int
digest_init(DIGEST *dp,
	    const char *stype);

extern void
digest_destroy(DIGEST *dp);

extern int
digest_update(DIGEST *dp,
	      unsigned char *buf,
	      size_t bufsize);

extern ssize_t
digest_final(DIGEST *dp,
	     unsigned char *buf,
	     size_t bufsize);


extern DIGEST_TYPE
digest_typeof(DIGEST *dp);

extern DIGEST_STATE
digest_stateof(DIGEST *dp);


extern DIGEST_TYPE
digest_str2type(const char *str);

extern const char *
digest_type2str(DIGEST_TYPE type);

#endif
