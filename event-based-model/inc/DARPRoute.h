#ifndef _DARP_ROUTE_H
#define _DARP_ROUTE_H

struct DARPRoute
{
    ///
    /// Stores the information about a particular route. 
    ///
    int start = -1;
    int end = -1;
    bool has_customers = false;
    double departure_depot = -DARPH_INFINITY; // departure time at depot
    double return_depot = -DARPH_INFINITY; // return time to depot

    double duration_violation = 0;
    double ride_time_violation = 0;
    double tw_violation = 0;
    int load_violation = 0; 
    
    
    
    template<int Q>
    friend class DARPGraph;
    template<int Q>
    friend class RollingHorizon;
    friend class DARPSolver;
    friend class TabuSearch;
    
};

#endif