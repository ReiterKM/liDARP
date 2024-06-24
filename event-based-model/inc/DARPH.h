// This is the primary include file for DARPH
#ifndef _DARPH_H
#define _DARPH_H

// Useful macros 
#define DARPH_MIN(X,Y)   ((X) < (Y) ?  (X) : (Y))
#define DARPH_MAX(X,Y)   ((X) < (Y) ?  (Y) : (X))
#define DARPH_ABS(a)     (((a) < 0) ? -(a) : (a))
#define DARPH_PLUS(a)    (((a) < 0) ?   0  : (a))

#define DARPH_STRING_SIZE     200
#define DARPH_DEPOT           0
#define DARPH_INFINITY        (1<<30)
#define DARPH_EPSILON         1E-5
#define STRINGWIDTH           8


#include <stdarg.h> // for va_list, va_start() and va_end() 
#include <stdio.h> // für fprintf
#include <vector> // event-based graph
#include <array> // event-based graph
#include <algorithm> // for erase-remove idiom (delete element by value from vector)
#include <unordered_map> 
#include <chrono> // measuring elapsed time
#include <iostream>
#include <iomanip> // format output (i.e. for table)
#include <utility> // for pair
#include <random> // for uniform random distribution (beta pr-set)
#include <cmath> // for pow()
#include <math.h> // für roundf
#include <fstream> // to read file while ignoring whitespaces
#include <string> 
#include <cstring> // strcpy()



#include "DARPUtils.h"
#include "DARPNode.h"
#include "DARPRoute.h"
#include "DARPDebug.h"
#include "DARP.h"
#include "HashFunction.h"
#include "DARPGraph.h"
#include "DARPSolver.h"
#include "RollingHorizon.h"

#endif
