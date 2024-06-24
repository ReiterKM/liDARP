#include "DARPH.h"

int DARPGetDimension(std::string filename)
{
    /// 
    /// Open up filename and scan for the number of nodes
    ///
    std::ifstream file;
    std::string line;
    int dimension;
    
    file.open(filename.c_str(),std::ios_base::in);
    if (!file)
    {
        fprintf(stderr, "Unable to open %s for reading\n", filename);
        exit(-1);
    }
    
    getline(file,line);
    std::istringstream f(line);
    f >> dimension; // first number is number of vehicles not dimemsion
    f >> dimension; // second number is dimension
#if FILE_DEBUG
    printf("Number of nodes 2n = %d\n", dimension); 
#endif

    file.close();
    return dimension;
}


