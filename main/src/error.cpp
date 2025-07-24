#include "error.h"

#include <cstdarg>
#include <cstdio>

void fatal(const char* fmt, ...) {
    va_list args;

    fprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    
    exit(EXIT_FAILURE);
}