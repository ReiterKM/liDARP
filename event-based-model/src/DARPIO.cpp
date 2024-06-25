#include "DARPH.h"

std::tuple<int, int> DARPGetDimension(std::string filepath)
{
    /// 
    /// Open up filename and scan for the number of nodes
    ///
    std::ifstream file;
    std::string line;
    int dimension, veh_capacity, temp;
    
    file.open(filepath.c_str(), std::ios_base::in);
    if (!file)
    {
        fprintf(stderr, "Unable to open %s for reading\n", filepath);
        exit(-1);
    }
    
    getline(file,line);
    std::istringstream iss(line);
    // num_vehicles num_requests num_stations  max_time vehicle_capacity alpha beta
    iss >> temp >> dimension >> temp >> temp >> veh_capacity >> temp >> temp;

#if FILE_DEBUG
    printf("Number of nodes n = %d\n", dimension); 
#endif

    file.close();
    return { dimension, veh_capacity };
}

double Euclidean_Distance(std::vector<double> x, std::vector<double> y, int dimension){
    double dist = 0;
    for (int i = 0; i < dimension; ++i) {
        dist += (y[i] - x[i]) * (y[i] - x[i]);
    }
    return sqrt(dist);
}