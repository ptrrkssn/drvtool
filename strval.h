/*
 * strval.h - String to values conversions
 */

#ifndef STRVAL_H
#define STRVAL_H 1

#include <sys/types.h>

extern int
str2off(const char **str,
	off_t *vp);

#endif

