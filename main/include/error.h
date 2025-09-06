#ifndef ERROR_H
#define ERROR_H

#include <cstdarg>
#include <cstdio>

enum class Result {
    SUCCESS
};

/*
- Prints a printf-formatted message to stderr
- Appends a newline character
- Exits the program
*/
inline void fatal(const char* fmt, ...) {
    va_list args;

    fprintf(stderr, fmt, args);
    fprintf(stderr, "\n");
    
    exit(EXIT_FAILURE);
}

inline void log(const char* fmt, ...) {
    va_list args;

    printf(fmt, args);
    printf("\n");
}

#endif