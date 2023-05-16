/*
** drives.c - Load all drives
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
#include <sys/disk.h>
#include <sys/errno.h>
#include <sys/sysctl.h>
#include <camlib.h>
#include <dev/nvme/nvme.h>

#include <sys/ioctl.h>

#include "drvtool.h"


#if defined(HAVE_LIBEDIT)
#include <histedit.h>
#include <editline/readline.h>
#elif defined(HAVE_LIBREADLINE)
#include <readline/readline.h>
#include <readline/history.h>
#else
extern char *readline(const char *prompt);
#endif

static DRIVE *all_drives = NULL;
DRIVE *selected_drive = NULL;


int
drive_nvme_getinfo(DRIVE *dp,
		   const char *path) {
  struct nvme_pt_command pt;
  struct nvme_controller_data cdata;
  char *cp;
  int fd, i;
  char pbuf[1024];


  if (sscanf(path, "/dev/nvd%d", &i) == 1) {
    sprintf(pbuf, "/dev/nvme%d", i);
    path = pbuf;
  }

  fd = open(path, O_RDWR);
  if (fd < 0) {
    return -1;
  }
  
  memset(&pt, 0, sizeof(pt));
  pt.cmd.opc = NVME_OPC_IDENTIFY;
  pt.cmd.cdw10 = 1;
  pt.buf = &cdata;
  pt.len = sizeof(cdata);
  pt.is_read = 1;
  
  if (ioctl(fd, NVME_PASSTHROUGH_CMD, &pt) < 0)
    return -2;
  
  if (nvme_completion_is_error(&pt.cpl))
    return -3;

  for (i = 0; i < NVME_MODEL_NUMBER_LENGTH && isspace(((const char*)cdata.mn)[i]); i++)
    ;
  dp->vendor = strndup(((const char*)cdata.mn)+i, NVME_MODEL_NUMBER_LENGTH-i);
  cp = strchr(dp->vendor, ' ');
  if (cp) {
    *cp++ = '\0';
    while (*cp && isspace(*cp))
      ++cp;
    dp->product = strdup(cp);
  } else
    dp->product = strdup("");
  for (i = strlen(dp->product)-1; i >= 0 && isspace(dp->product[i]); i--)
    ;
  dp->product[i+1] = '\0';
  
  for (i = 0; i < NVME_FIRMWARE_REVISION_LENGTH && isspace(((const char *)cdata.fr)[i]); i++)
    ;
  dp->revision = strndup((const char *) cdata.fr+i, NVME_FIRMWARE_REVISION_LENGTH-i);
  for (i = strlen(dp->revision)-1; i >= 0 && isspace(dp->revision[i]); i--)
    ;
  dp->revision[i+1] = '\0';
  
  for (i = 0; i < NVME_SERIAL_NUMBER_LENGTH && isspace(((const char *)cdata.sn)[i]); i++)
    ;
  dp->ident = strndup((const char *) cdata.sn+i, NVME_SERIAL_NUMBER_LENGTH-i);
  for (i = strlen(dp->ident)-1; i >= 0 && isspace(dp->ident[i]); i--)
    ;
  dp->ident[i+1] = '\0';
  
  close(fd);

  dp->flags.is_ssd = 1;
  return 0;
}


int
drive_cam_sysctl(DRIVE *dp,
	       const char *vname,
	       void *vp,
	       size_t *vs) {
  char nbuf[128], *pname, *cp;
  int unit;

  
  if (!dp || !dp->name)
    return -1;
  
  pname = dp->name;
  cp = pname+strlen(pname)-1;
  while (cp >= pname && isdigit(*cp))
    --cp;
  ++cp;
  
  if (sscanf(cp, "%d", &unit) != 1)
    return -1;

  snprintf(nbuf, sizeof(nbuf), "kern.cam.%.*s.%d.%s", (int) (cp-pname), pname, unit, vname);
  return sysctlbyname(nbuf, vp, vs, NULL, 0);
}

int
drive_geom_sysctl(DRIVE *dp,
		const char *vname,
		void *vp,
		size_t *vs) {
  char nbuf[128], *pname, *cp;
  int unit;

  
  if (!dp || !dp->name)
    return -1;
  
  pname = dp->name;
  cp = pname+strlen(pname)-1;
  while (cp >= pname && isdigit(*cp))
    --cp;
  ++cp;
  
  if (sscanf(cp, "%d", &unit) != 1)
    return -1;

  snprintf(nbuf, sizeof(nbuf), "kern.geom.disk.%s.%s", pname, vname);
  return sysctlbyname(nbuf, vp, vs, NULL, 0);
}

#if 0
static int
str2int(const char *str,
	int *vp) {
  off_t ov;
  int rc;

  rc = str2off(&str, &ov);
  if (rc < 1)
    return rc;

  if (ov < INT_MIN || ov > INT_MAX) {
    errno = ERANGE;
    return -1;
  }
  
  *vp = ov;
  return rc;
}
#endif


int
drive_str2bytes(const char *str,
		off_t *vp,
		TEST *tp) {
  DRIVE *dp = tp->drive;
  off_t v, cv, hv, sv;
  int rc;
  
  
  rc = str2off(&str, vp);
  if (rc < 1)
    return rc;

  switch (toupper(*str)) {
  case 0: /* Blocks */
    *vp *= tp->b_size;
    break;

  case '%': /* % of disk */
    *vp = *vp * tp->b_total * tp->b_size / 100;
    break;

  case 'B': /* Bytes */
    break;

  case 'C': /* Cylinders (Tracks) */
    if (!dp || !dp->fw_sectors) {
      errno = EINVAL;
      return -1;
    }
    *vp *= dp->sector_size * dp->fw_sectors;
    break;
    
  case 'N': /* Native sector size */
    *vp *= dp->stripe_size;
    break;
    
  case 'S': /* (Emulated) Sector size */
    *vp *= dp->sector_size;
    break;

  case '/': /* Cylinders/Heads/Sectors */
    cv = *vp;
    if (!dp || !dp->fw_sectors) {
      errno = EINVAL;
      return -1;
    }
    ++str;
    if (str2off(&str, &hv) != 1) {
      errno = EINVAL;
      return -1;
    }
    if (*str != '/') {
      errno = EINVAL;
      return -1;
    }
    ++str;
    if (str2off(&str, &sv) != 1) {
      errno = EINVAL;
      return -1;
    }
    *vp = cv * hv * sv * dp->sector_size;
    break;
    
  default:
    return -1;
  }

  if (!dp->flags.is_file) {
    /* Make sure it's a multiple of the sector size */
    v = *vp / dp->sector_size;
    if (v * dp->sector_size != *vp) {
      errno = EINVAL;
      return -1;
    }

    if (*vp > dp->media_size) {
      errno = EOVERFLOW;
      return -1;
    }
  }

  return 1;
}

int
drive_str2blocks(const char *str,
		 off_t *vp,
		 TEST *tp) {
  int rc;
  off_t b;
  
  
  rc = drive_str2bytes(str, vp, tp);
  if (rc < 1)
    return rc;

  b = *vp / tp->b_size;
  if (b * tp->b_size != *vp) {
    errno = EINVAL;
    return -1;
  }

  *vp = b;
  return 1;
}


int
drive_blocks2chs(off_t b,
		 off_t *cv,
		 off_t *hv,
		 off_t *sv,
		 DRIVE *dp) {
  if (!dp)
    return 0;

  if (!dp->fw_heads || !dp->fw_sectors || (dp->fw_heads == 255 && dp->fw_sectors == 63))
    return 0;
  
  *cv = b / (dp->fw_heads*dp->fw_sectors);
  
  b -= (*cv * dp->fw_heads*dp->fw_sectors);

  *hv = b / dp->fw_sectors;
  
  b -= (*hv * dp->fw_sectors);
  
  *sv = b;
  return 1;
}



int
drive_flush(DRIVE *dp) {
  int rc;


  rc = ioctl(dp->fd, DIOCGFLUSH);
  return rc;
}


int
drive_delete(DRIVE *dp,
	   off_t off,
	   off_t len) {
  int rc;
  off_t iov[2];

  iov[0] = off;
  iov[1] = len;

  rc = ioctl(dp->fd, DIOCGDELETE, iov);
  return rc;
}

int
drive_close(DRIVE *dp) {
  int rc;

  
  if (dp->path) {
    free(dp->path);
    dp->path = NULL;
  }

  if (dp->provider_name) {
    free(dp->provider_name);
    dp->provider_name = NULL;
  }

  if (dp->ident) {
    free(dp->ident);
    dp->ident = NULL;
  }

  if (dp->physical_path) {
    free(dp->physical_path);
    dp->physical_path = NULL;
  }

  rc = 0;
  if (dp->fd != -1) {
    rc = close(dp->fd);
    dp->fd = -1;
  }
  
  free(dp);
  return rc;
}

void
p_flag(FILE *fp,
       int *n,
       const char *s) {
  if (*n)
    putc(',', fp);
  fputs(s, fp);
  ++*n;
}

void
drive_print(FILE *fp,
	    int idx,
	    DRIVE *dp,
	    int verbose) {
  char b1[80], b2[80], b3[80];
  struct winsize w;

  if (verbose > 1) {
    fprintf(fp, "%-14s\t%s\n", "Name:", dp->name);
    if (idx)
      fprintf(fp, "%-14s\t%d\n", "Drive#:", idx);
    if (dp->vendor)
      fprintf(fp, "%-14s\t%s\n", "Vendor:", dp->vendor);
    if (dp->product)
      fprintf(fp, "%-14s\t%-16s\n", "Product:", dp->product);
    if (dp->revision)
      fprintf(fp, "%-14s\t%-8s\n", "Revision:", dp->revision);
    if (dp->ident)
      fprintf(fp, "%-14s\t%-20s\n", "Ident:", dp->ident);
    
    int2str(dp->stripe_size, b1, sizeof(b1), 2);
    strcat(b1, "n");
    if (dp->sector_size != dp->stripe_size) {
      strcat(b1, "/");
      int2str(dp->sector_size, b2, sizeof(b2), 2);
      strcat(b1, b2);
      strcat(b1, "e");
    }
    
    fprintf(fp, "%-14s\t%sB", "Media Size:", 
	    int2str(dp->media_size, b2, sizeof(b2), 0));
    
    if (ioctl(1, TIOCGWINSZ, &w) == 0 && w.ws_col > 110) {
      fprintf(fp, " (%s @ %s)\n",
	      int2str(dp->media_size / dp->stripe_size, b3, sizeof(b3), 0),
	      b1);

      {
	int nf = 0;
	fprintf(fp, "%-14s\t", "Flags:");
      
	if (dp->flags.is_file)
	  p_flag(fp, &nf, "FILE");
	
	if (dp->flags.is_ssd)
	  p_flag(fp, &nf, "SSD");
	
	if (dp->flags.is_ro) 
	  p_flag(fp, &nf, "RO");
      }
      putc('\n', fp);
    }

    if (dp->cam.path)
      fprintf(fp, "%-14s\t%s\n", "CAM Path:", dp->cam.path);

    if (dp->physical_path)
      fprintf(fp, "%-14s\t%s\n", "Physical Path:", dp->physical_path);
    
    if (dp->fw_heads && dp->tracks && dp->fw_sectors) {
      fprintf(fp, "%-14s\t%u/%u/%u%s\n",
	      "T/H/S:",
	      dp->tracks,
	      dp->fw_heads,
	      dp->fw_sectors,
	      ((dp->fw_heads == 255) && (dp->fw_sectors == 63)) ? " (Simulated)" : "");
    }
  
  } else {
    if (idx)
      fprintf(fp, "%3d. ", idx);
    else
      fprintf(fp, "%3s  ", "");
    
    fprintf(fp, "%-4s", dp->name);
    
    fprintf(fp, " : %-8s %-16s %-8s",
	    dp->vendor ? dp->vendor : "",
	    dp->product ? dp->product : "",
	    dp->revision ? dp->revision : "");
    
    fprintf(fp, " : %-20s", dp->ident ? dp->ident : "");
    
    int2str(dp->stripe_size, b1, sizeof(b1), 2);
    strcat(b1, "n");
    if (dp->sector_size != dp->stripe_size) {
      strcat(b1, "/");
      int2str(dp->sector_size, b2, sizeof(b2), 2);
      strcat(b1, b2);
      strcat(b1, "e");
    }
    
    fprintf(fp, " : %6sB",
	    int2str(dp->media_size, b2, sizeof(b2), 0));
    
    if (ioctl(1, TIOCGWINSZ, &w) == 0 && w.ws_col > 110) {
      fprintf(fp, " : %6s sectors @ %-12s",
	      int2str(dp->media_size / dp->stripe_size, b3, sizeof(b3), 0),
	      b1);
      
      if (w.ws_col > 117) {
	int nf = 0;
	
	fprintf(fp, " : ");
	
	if (dp->flags.is_file)
	  p_flag(fp, &nf, "FILE");
	
	if (dp->flags.is_ssd)
	  p_flag(fp, &nf, "SSD");
	
	if (dp->flags.is_ro) 
	  p_flag(fp, &nf, "RO");
      }
    }
    
    
    putc('\n', fp);
    
    if (verbose) {
      fprintf(fp, "%11s : ", "");
      
      if (dp->cam.path)
	fprintf(fp, "%s ", dp->cam.path);
      
      if (dp->physical_path)
	fprintf(fp, "{%s} ", dp->physical_path);
      
      if (verbose > 1)
	if (dp->fw_heads && dp->tracks && dp->fw_sectors)
	  fprintf(fp, "{%u/%u/%u C/H/S %s} ",
		  dp->tracks,
		  dp->fw_heads,
		  dp->fw_sectors,
		  ((dp->fw_heads == 255) && (dp->fw_sectors == 63)) ? "(Simulated)" : "");
      
      putc('\n', fp);
      putc('\n', fp);
    }
  }
}




DRIVE *
drive_open(const char *name,
	   int rwmode) {
  char pnbuf[MAXPATHLEN];
  char idbuf[DISK_IDENT_SIZE];
  int s_errno, rc;
  DRIVE *dp;
  int is_rotating = -1;
  size_t isrs = sizeof(is_rotating);
  struct cam_device *cam;
  char *cp;
  
  
  dp = malloc(sizeof(*dp));
  if (!dp)
    return NULL;

  dp->name = strdup(name);
  if (!dp->name)
    goto Fail;
  
  dp->fd = -1;
  if (*name == '/' || *name == '.') {
    dp->path = strdup(name);

    if (*name == '/') {
      dp->name = strrchr(name, '/');
      dp->name++;
    }
  }
  else
    dp->path = strxdup("/dev/", name, NULL);

  drive_cam_sysctl(dp, "rotating", &is_rotating, &isrs);
  dp->flags.is_ssd = (is_rotating ? 0 : 1);

  dp->fd = -1;
  dp->flags.is_ro = 1;
  
  if (!config.f_noupdate) {
    dp->fd = open(dp->path, O_RDWR|O_DIRECT, 0);
    if (dp->fd >= 0)
      dp->flags.is_ro = 0;
    else if (rwmode > 0)
      goto Fail;
  }
  if (dp->fd < 0) {
    dp->fd = open(dp->path, O_RDONLY|O_DIRECT, 0);
    if (dp->fd < 0)
      goto Fail;
 }

  if (fstat(dp->fd, &dp->sbuf) < 0)
    goto Fail;

  dp->flags.is_file = S_ISREG(dp->sbuf.st_mode) ? 1 : 0;
  
  dp->media_size = 0;
  (void) ioctl(dp->fd, DIOCGMEDIASIZE, &dp->media_size);
  if (dp->media_size == 0)
    dp->media_size = dp->sbuf.st_size;
  
  dp->sector_size = 0;
  (void) ioctl(dp->fd, DIOCGSECTORSIZE, &dp->sector_size);
  if (dp->sector_size == 0)
    dp->sector_size = dp->sbuf.st_blksize;
  
  dp->sectors = 0;
  if (dp->sector_size)
    dp->sectors = dp->media_size / dp->sector_size;
  
  dp->stripe_size = 0;
  (void) ioctl(dp->fd, DIOCGSTRIPESIZE, &dp->stripe_size);

  if (dp->stripe_size == 0)
    dp->stripe_size = dp->sector_size;
  
  dp->stripe_offset = 0;
  (void) ioctl(dp->fd, DIOCGSTRIPEOFFSET, &dp->stripe_offset);

  dp->stripes = 0;
  if (dp->stripe_size)
    dp->stripes = dp->media_size / dp->stripe_size;
  
  dp->fw_sectors = 0;
  (void) ioctl(dp->fd, DIOCGFWSECTORS, &dp->fw_sectors);
  
  dp->fw_heads = 0;
  (void) ioctl(dp->fd, DIOCGFWHEADS, &dp->fw_heads);

  dp->tracks = 0;
  if (dp->fw_heads && dp->fw_sectors && dp->stripe_size)
    dp->tracks = dp->media_size / (dp->fw_heads * dp->fw_sectors * dp->stripe_size);

  dp->front_reserved = 0;
#ifdef DIOCGFRONTSTUFF
  (void) ioctl(dp->fd, DIOCGFRONTSTUFF, &dp->front_reserved);
#endif
  
  dp->provider_name = NULL;
  pnbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGPROVIDERNAME, pnbuf) == 0) {
    dp->provider_name = strndup(pnbuf, sizeof(pnbuf));
    if (!dp->provider_name)
      goto Fail;
    free(dp->name);
    dp->name = strdup(dp->provider_name);
    if (!dp->name)
      goto Fail;
  }
  
  dp->ident = NULL;
  idbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGIDENT, idbuf) == 0) {
    dp->ident = strndup(idbuf, sizeof(idbuf));
    if (!dp->ident)
      goto Fail;
  }
  
  dp->physical_path = NULL;
  pnbuf[0] = '\0';
  if (ioctl(dp->fd, DIOCGPHYSPATH, pnbuf) == 0) {
    dp->physical_path = strndup(pnbuf, sizeof(pnbuf)); 
    if (!dp->physical_path)
      goto Fail;
  }

  cam = cam_open_device(dp->path, O_RDWR);
  if (cam) {
    char buf[256];
    
    snprintf(buf, sizeof(buf), "/%s%u/bus%u/scbus%u/target%u/lun%lu",
	     cam->sim_name, cam->sim_unit_number, cam->bus_id,
	     cam->path_id, cam->target_id, cam->target_lun);
    dp->cam.path = strdup(buf);

    dp->vendor   = strndup(cam->inq_data.vendor, 8);
    cp = strchr(dp->vendor, ' ');
    if (cp)
      *cp = '\0';
    dp->product  = strndup(cam->inq_data.product, 16);
    cp = strchr(dp->product, ' ');
    if (cp)
      *cp = '\0';
    dp->revision = strndup(cam->inq_data.revision, 4);
    cp = strchr(dp->revision, ' ');
    if (cp)
      *cp = '\0';
    
    cam_close_device(cam);
  } else {
    rc = drive_nvme_getinfo(dp, dp->path);
    if (rc < 0)
      fprintf(stderr, "Error getting nvme data: %d\n", rc);
  }
  

  dp->next = all_drives;
  all_drives = dp;
  
  return dp;

 Fail:
  s_errno = errno;
  drive_close(dp);
  errno = s_errno;
  return NULL;
}




int
drives_load(int rwmode) {
  size_t bsize;
  char *buf;
  char *cp;
  int n, rc = 0;

  
  if (sysctlbyname("kern.disks", NULL, &bsize, NULL, 0) < 0) {
    if (config.f_debug)
      fprintf(stderr, "%s: Error: Unable to get number of drives from kernel: %s\n",
	      argv0, strerror(errno));
    return -1;
  }

  buf = malloc(bsize);
  if (!buf) {
    if (config.f_debug)
      fprintf(stderr, "%s: Error: malloc(%ld bytes) failed: %s\n",
	      argv0, bsize, strerror(errno));
    return -1;
  }
  
  if (sysctlbyname("kern.disks", buf, &bsize, NULL, 0) < 0) {
    if (config.f_debug)
      fprintf(stderr, "%s: Error: Unable to get list of drives from kernel: %s\n",
	      argv0, strerror(errno));
    return -1;
  }

  n = 0;
  while ((cp = strsep(&buf, " ")) != NULL) {
    if (drive_open(cp, rwmode) == NULL) {
      if (errno == EPERM && rwmode > 0)
	continue; /* Skip RO drives */
      
      if (config.f_debug)
	fprintf(stderr, "%s: Error: %s: Opening drive: %s\n", argv0, cp, strerror(errno));
      rc = -1;
    } else
      ++n;
  }
  
  return rc < 0 ? rc : n;
}


void
drives_print(int f_idx) {
  DRIVE *dp = all_drives;
  int i = 0;
  
  while (dp) {
    drive_print(stdout, f_idx ? ++i : 0, dp, config.f_verbose);
    dp = dp->next;
  }
}

char *
strtrim(char *buf) {
  int i;

  
  if (!buf)
    return NULL;
  
  while (isspace(*buf))
    ++buf;

  i = strlen(buf);
  while (i > 0 && isspace(buf[i-1]))
    --i;
  buf[i] = '\0';
  return buf;
}



DRIVE *
drive_select(const char *name) {
  DRIVE *dp;
  int i, d;


  do {
    if (!name) {
      char *cp;
      cp = readline("Specify drive (number or name): ");
      if (!cp)
	exit(1);
      name = strtrim(cp);
    }
    
    if (sscanf(name, "%d", &d) == 1) {
      i = 0;
      for (dp = all_drives; dp && ++i != d; dp = dp->next)
	;
    } else {
      for (dp = all_drives; dp && strcmp(name, dp->name) != 0; dp = dp->next)
	;
    }

    name = NULL;
  } while (!dp);

  if (dp)
    selected_drive = dp;
  
  return dp;
}





int
select_cmd(int argc,
	   char **argv) {
  DRIVE *dp = NULL;

  
  if (argc > 1) {
    dp = drive_select(argv[1]);
    if (dp) {
      puts("Selected drive:");
      drive_print(stdout, 0, selected_drive, 0);
    }
  }
  else {
    puts("AVAILABLE DRIVE SELECTIONS:");
    drives_print(1); 
    dp = drive_select(NULL);
  }
  
  return dp ? 0 : 1;
}

int
current_cmd(int argc,
	    char **argv) {
  drive_print(stdout, 0, selected_drive, 2);
  return 0;
}

COMMAND select_command =
  { "select-drive",      select_cmd, NULL, "[<drive>]", "Select drive" };
COMMAND current_command =
  { "current-drive",     current_cmd, NULL, "", "Display selected drive" };

COMMAND *drive_commands[] =
  {
   &select_command,
   &current_command,
   NULL,
  };
