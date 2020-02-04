
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include <errno.h>

#include "../seq.h"
#include "../strval.h"




int
seq_append_str(SEQ *sp,
	       const char *str) {
  off_t first, last;
  size_t nr = 0;
  int rc;

  
  if (!str) {
    errno = EFAULT;
    return -1;
  }

  while (*str) {
    rc = str2off(&str, &first);
    if (rc < 1)
      return rc;
    
    if (*str == '-' || *str == ':') {
      ++str;
      
      rc = str2off(&str, &last);
      if (rc < 1)
	return rc;
    } else
      last = first;

    if (seq_append(sp, SEQ_TYPE_RANGE, first, last) < 0)
      return -1;
    
    ++nr;
    
    if (*str == ',')
      ++str;
  }

  return nr;
}



int
main(int argc,
     char *argv[]) {
  SEQ *a;
  off_t i, j;
  off_t o;
  
  
  srand48(time(NULL)^getpid());

  a = seq_create(SEQ_TYPE_NONE);
  if (!a) {
    fprintf(stderr, "Unable to create SEQ\n");
    exit(1);
  }

  for (i = 1; i < argc && strcmp(argv[i], "-") != 0; i++) {
    if (seq_append_str(a, argv[i]) < 0) {
      fprintf(stderr, "Unable to add range: %s\n", argv[i]);
      exit(1);
    }
  }

  puts("\nINPUT:");
  seq_print(a, stdout);

  seq_merge(a);
  puts("\nMERGED:");
  seq_print(a, stdout);

  seq_expand(a);
  puts("\nEXPANDED:");
  seq_print(a, stdout);

  seq_shuffle(a);
  puts("\nSHUFFLED:");
  seq_print(a, stdout);

  seq_merge(a);
  puts("\nMERGED:");
  seq_print(a, stdout);
  
  seq_sort(a, NULL);
  puts("\nSORTED:");
  seq_print(a, stdout);
  
  seq_uniq(a);
  puts("\nUNIQ:");
  seq_print(a, stdout);

  seq_compact(a);
  puts("\nCOMPACTED:");
  seq_print(a, stdout);

  seq_shuffle(a);
  puts("\nSHUFFLED:");
  seq_print(a, stdout);
  
  if (i+1 < argc) {
    SEQ *b;
    off_t v, vi;
    size_t np;
    ssize_t rc;
    
    ++i;
    b = seq_create(SEQ_TYPE_NONE);
    seq_append_str(b, argv[i]);

    puts("\nSELRANGE:");
    seq_print(b, stdout);
    
    rc = seq_compare(a, b);
    printf("\nCOMPARE:\n  rc = %ld\n", rc);

    puts("\nSELECTED:");
    np = 0;
    putchar(' ');
    putchar(' ');
    for (j = 0; seq_get(b, j, &vi) == 1; ++j) {
      if (seq_get(a, vi, &v) == 1) {
	if (np++) {
	  putchar(',');
	  putchar(' ');
	}
	printf("%ld", v);
      }
    }
    putchar('\n');
    
    puts("\nCONTAINS:");
    printf("  %s\n", seq_contains(a, b) ? "Yes" : "No");

    seq_destroy(b);
  }

  puts("\nITER-UP:");
  j = 0;
  putchar(' ');
  putchar(' ');
  while (seq_iter(a, 1, &j, &o) == 1) {
    if (j > 1) {
      putchar(',');
      putchar(' ');
    }
    printf("%ld", o);
  }
  putchar('\n');
  
  puts("\nITER-DOWN:");
  j = 0;
  putchar(' ');
  putchar(' ');
  while (seq_iter(a, -1, &j, &o) == 1) {
    if (j > 1) {
      putchar(',');
      putchar(' ');
    }
    printf("%ld", o);
  }
  putchar('\n');
  seq_destroy(a);
  
  return 0;
}

