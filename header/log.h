#ifndef LOG_H
#define LOG_H

#include <stdio.h>

#define DEBUG 1

#ifdef DEBUG
#define LOG(...) printf(__VA_ARGS__); printf("\n");
#else
#define LOG(...) ;
#endif

#endif
