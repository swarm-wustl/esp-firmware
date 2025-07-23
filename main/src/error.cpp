#include "error.h"

#include <stdarg.h>
#include <stdio.h>

void fatal(const char* fmt, ...) {
    va_list args;

    fprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    
    exit(EXIT_FAILURE);
}