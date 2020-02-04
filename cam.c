#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <camlib.h>


int
main(int argc,
     char *argv[]) {
  struct cam_device *cdp;
  union ccb *ccbp;

  cdp = cam_open_device(argv[1], O_RDWR);
  if (!cdp) {
    fprintf(stderr, "%s: Error: %s: cam_open_device: %s\n",
	    argv[0], argv[1], strerror(errno));
    exit(1);
  }
  
  ccbp = cam_getccb(cdp);
  if (!ccbp) {
    fprintf(stderr, "%s: Error: %s: cam_getccb: %s\n",
	    argv[0], argv[1], strerror(errno));
    exit(1);
  }


  if (cam_send_ccb(cdp, ccbp) < 0) {
    fprintf(stderr, "%s: Error: %s: cam_send_ccb: %s\n",
	    argv[0], argv[1], strerror(errno));
    exit(1);
  }
  
}
