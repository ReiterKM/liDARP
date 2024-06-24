#ifndef _DARP_H
#define _DARP_H

class DARP {   
private:
    int instance_mode;
    
    // Problem parameters, description, etc.
    int num_requests;
    int num_nodes; // without depot
    int num_vehicles; 
    double max_route_duration; // = max duration of SERVICE  
    double planning_horizon;
    int veh_capacity;
    double **d; // The distance matrix d
    double **tt; // The travel times matrix tt 

    class DARPNode *nodes; // Array of nodes - contains coordinates, time windows, load

    // Solution storage
    class DARPRoute *route; // Array stores useful information about the routes in a solution   
    int *next_array;
    int *pred_array;
    int *route_num;
    bool *routed; // Indicates wether the user is in a route yet or not

    int rcardinality;
    std::vector<int> R; 
    // dynamic instance
    double *become_known_array;
    int last_static;
    int num_known_requests;
    std::vector<int> known_requests; 
    
    
public:
    
    DARP(int); // constructor for an n-node problem
    ~DARP(); 
    // copy/ move constructor and assignment/ move operator not defined, not needed in this project so far (not more than one instance created)
    
        
    int get_instance_mode() const {return instance_mode;}
    void set_instance_mode(int i) {instance_mode = i;}

    // // file processing
    void read_file(std::string infile, std::string data_directory, std::string instance);
    void transform_dynamic(double share_static_requests = 0.25, double beta = 60);
    // tighten time windows if necessary
    void preprocess();
    

    template<int Q>
    friend class DARPGraph;
    template<int Q>
    friend class RollingHorizon;
    friend class DARPSolver;
};

#endif