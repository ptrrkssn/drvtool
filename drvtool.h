/*
** drvtool.h
*/

#ifndef DRVTOOL_H
#define DRVTOOL_H 1

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

#include "rate.h"
#include "blocks.h"
#include "digest.h"
#include "transform.h"



typedef unsigned char PATTERN[4];




typedef struct drive {
  char *name;
  char *path;
  
  int fd;
  struct stat sbuf;

  struct {
    unsigned int is_file : 1;
    unsigned int is_ssd  : 1;
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
    char *vendor;
    char *product;
    char *revision;
  } cam;

  struct drive *next;
} DRIVE;




#define TEST_READ     0x0001
#define TEST_WRITE    0x0002
#define TEST_VERIFY   0x0004
#define TEST_RESTORE  0x0008
#define TEST_PATTERN  0x0010
#define TEST_PURGE    0x0020
#define TEST_FLUSH    0x0040
#define TEST_TRIM     0x0080




typedef struct test {
  DRIVE *drive;
  
  RATE rate;           /* Transfer rate buffer        */

  unsigned int flags;  /* Test flags                  */
  
  int    passes;       /* Number of passes            */
  time_t timeout;      /* Max length of test run      */
  
  off_t b_size;        /* Size of each block          */
  off_t b_total;       /* Device size in # blocks     */
  
  off_t b_start;       /* First block #               */
  off_t b_length;      /* Number of blocks            */
  off_t b_end;         /* Last+1 block #              */

  TRANSFORM transform;

  struct {
    DIGEST data;
    struct {;
      struct {
	unsigned char data[DIGEST_BUFSIZE_MAX];
	size_t len;
      } first;
      struct {
	unsigned char data[DIGEST_BUFSIZE_MAX];
	size_t len;
      } last;
    } buffer;
  } digest;
  
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
