#ifndef _DARP_NODE_H
#define _DARP_NODE_H

// reine Datenklasse
struct DARPNode
{
    DARPNode() : 
    x{0}, y{0}, 
    demand{0}, 
    max_ride_time{DARPH_INFINITY}, 
    service_time{0},
    start_tw{0},
    end_tw{DARPH_INFINITY},
    departure_time{DARPH_INFINITY},
    arrival_time{DARPH_INFINITY},
    beginning_service{DARPH_INFINITY},
    waiting_time{DARPH_INFINITY}, 
    // important that this is -DARPH_INFINITY in computation of forward time slack
    ride_time{-DARPH_INFINITY}, // assign value if i >= n+1   
    vehicle_load{0} {} 
       
    double x;
    double y;
    int id;
    double tw_length;
    int demand; 
    double max_ride_time;
    double service_time;
    double start_tw; // lower bound of time window
    double end_tw; // upper bound of time window

    // attributes that change during the search
    double departure_time;
    double arrival_time;
    double beginning_service;
    double beginning_service_res = DARPH_INFINITY;
    double waiting_time;
    double ride_time;
    int vehicle_load;


    template<int Q>
    friend class DARPGraph;
    template<int Q>
    friend class RollingHorizon;
    friend class DARPSolver;
    
};

#endif