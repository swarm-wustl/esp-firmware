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

    va_start(args, fmt);     
    vfprintf(stderr, fmt, args); 
    va_end(args);           
    fprintf(stderr, "\n");
    
    exit(EXIT_FAILURE);
}

inline void log(const char* fmt, ...) {
    va_list args;

    va_start(args, fmt);  
    vprintf(fmt, args);
    va_end(args); 
    
    printf("\n");
}

#endif