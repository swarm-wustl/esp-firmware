#ifndef ERROR_H
#define ERROR_H

enum class Result {
    SUCCESS
};

/*
- Prints a printf-formatted message to stderr
- Appends a newline character
- Exits the program
*/
void fatal(const char* fmt, ...);

#endif