/*
** drvtool.h
*/

#ifndef DRVTOOL_H
#define DRVTOOL_H 1

typedef struct {
  off_t *v;
  off_t s;
} BLOCKS;


typedef unsigned char PATTERN[4];


#define TEST_DIGEST_SHA256  1
#define TEST_DIGEST_SHA384  2
#define TEST_DIGEST_SHA512  3

#define TEST_DIGEST_ADLER32 4
#define TEST_DIGEST_CRC32   5

#define TEST_TRANSFORM_XOR  1
#define TEST_TRANSFORM_ROR  2
#define TEST_TRANSFORM_ROL  3


#define RATE_BUF_SIZE 10

typedef struct {
  unsigned int f;
  unsigned int p;
  off_t dv[RATE_BUF_SIZE];
} RATE;


typedef struct {
  char *name;
  char *path;
  
  int fd;
  struct stat sbuf;

  struct {
    unsigned int is_file : 1;
    unsigned int is_ssd  : 1;
#if 0
    unsigned int is_open : 1;
#endif
  } flags;
  
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
  
  struct {
    char *path;
  } cam;
  
} DEVICE;



typedef struct disk {
  struct disk *next;
  char *name;
  DEVICE *dev;
} DISK;




#define TEST_READ     0x0001
#define TEST_WRITE    0x0002
#define TEST_VERIFY   0x0004
#define TEST_RESTORE  0x0008
#define TEST_PATTERN  0x0010
#define TEST_PURGE    0x0020
#define TEST_FLUSH    0x0040
#define TEST_DELETE   0x0080
#define TEST_CRYPT    0x0100


typedef struct {
  DEVICE *dev;
  
  RATE rate;           /* Transfer rate buffer        */

  unsigned int flags;  /* Test flags                  */
  
  int passes;          /* Number of passes            */
  
  time_t t_max;        /* Max length of test run      */
  
  off_t b_size;        /* Size of each block          */
  off_t b_total;       /* Device size in # blocks     */
  
  off_t b_start;       /* First block #               */
  off_t b_length;      /* Number of blocks            */
  off_t b_end;         /* Last+1 block #              */
  
  int transform;
  union transform_txd {
    unsigned char xor;
    unsigned char ror;
    unsigned char rol;
  } txd;
    
  int digest;
  union digest_ctx {
    SHA256_CTX sha256;
    SHA384_CTX sha384;
    SHA512_CTX sha512;
    unsigned long adler32;
  } ctx;

  BLOCKS *blocks;
  
  struct timespec t0;  /* Start time                  */
  struct timespec t1;  /* Last update                 */
  
  unsigned char *obuf; /* Block buffer, original data */
  unsigned char *wbuf; /* Block buffer, data to write */
  unsigned char *rbuf; /* Block buffer, data read     */

  off_t last_b_pos;
  off_t last_pos;
} TEST;


#endif
