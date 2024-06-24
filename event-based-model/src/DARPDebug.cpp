#include "DARPH.h"

// declared in "DARPDebug.h"
void report_error(const char* format, ...)
{
    ///
    /// Prints the message and function name to stderr and terminates the program.
    ///

    va_list args; 
    va_start(args, format);
    vfprintf(stderr, format, args);
    va_end(args);
    fprintf(stderr, "\n");

    fprintf(stderr,"Exiting\n");
    exit(-1);
}


