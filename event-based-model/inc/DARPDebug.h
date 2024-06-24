
#ifndef _DARP_DEBUG_H
#define _DARP_DEBUG_H

#define DEBUG_ALL           0
#define VERIFY_ALL          1
#define VERBOSE             0
#define FILE_DEBUG     0 + DEBUG_ALL 

// Just sends message to stderr and exits
void report_error(const char*, ...);
#endif