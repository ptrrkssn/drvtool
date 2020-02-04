/*
 * strval.c - String to values conversions
 */

#include <stdio.h>
#include <ctype.h>

#include "strval.h"

int
str2off(const char **str,
	off_t *vp) {
  off_t base = 1000, vbm, sbm, v, sv;
  int rc = 0, rv = 0, sign = 1;
  char c, i;

  
  *vp = 0;

  if (**str == '-' && isdigit((*str)[1])) {
    ++*str;
    sign = -1;
  }
  
  while (**str && isdigit(**str)) {
    c = i = 0;

    if ((*str)[0] == '0' && (*str)[1] == 'x') {
      ++*str;
      ++*str;
      
      rc = sscanf(*str, "%lx%c%c", &v, &c, &i);
      if (rc < 1)
	return -1;
      
      while (**str && isxdigit(**str))
	++*str;
      
      rv = 1;
    } else {
      rc = sscanf(*str, "%ld%c%c", &v, &c, &i);
      if (rc < 1)
	return -1;
      
      while (**str && isdigit(**str))
	++*str;
      
      rv = 1;
    }

    if (c) {
      if (i == 'i') {
	++*str;
	base = 1024;
      }
    }

    switch (toupper(c)) {
    case 'K':
      vbm = base;
      sbm = 1;
      break;
      
    case 'M':
      vbm = base*base;
      sbm = base;
      break;
      
    case 'G':
      vbm = base*base*base;
      sbm = base*base;
      break;
      
    case 'T':
      vbm = base*base*base*base;
      sbm = base*base*base;
      break;
      
    case 'P':
      vbm = base*base*base*base;
      sbm = base*base*base;
      break;
      
    default:
      c = 0;
      vbm = 1;
      sbm = 1;
      break;
    }

    v *= vbm;
    
    if (c) {
      int nd = 0;
      ++*str;
      sv = 0;
      while (isdigit(**str)) {
	sv *= 10;
	sv += **str - '0';
	++nd;
	++*str;
      }
      switch (nd) {
      case 1:
	sv *= 100;
	break;
      case 2:
	sv *= 10;
	break;
      }
		     
      v += sv*sbm;
    }
    
    *vp += v;
    
    if (**str == '+')
      ++*str;
    else
      break;
  }

  *vp *= sign;
  return rv;
}

