#ifndef _DARP_H
#define _DARP_H

class DARP {   
private:
    int instance_mode;
    
    // Problem parameters, description, etc.
    int num_requests; // number of requests
    int num_stations; // number of bus stations
    int num_nodes; // number of nodes, incl. depot
    int num_vehicles; // number of vehicles
    double max_route_duration; // = max duration of SERVICE
    double planning_horizon; // ? still needed?
    double alpha; // service promise alpha
    double beta; // service promise beta
    int veh_capacity;
    double **d; // The distance matrix d, which denotes distance between passengers
    double **tt; // The travel times matrix tt 
    std::vector<double> d_direct;

    class DARPNode *nodes; // Array of nodes - contains coordinates, time windows, load

    // Solution storage
    class DARPRoute *route; // Array stores useful information about the routes in a solution 
    std::vector<int> next_array;
    std::vector<int> pred_array;
    std::vector<int> route_num;
    std::vector<bool> routed; // Indicates wether the user is in a route yet or not

    int rcardinality;
    std::vector<int> R;
    std::vector<int> R_forward;
    std::vector<int> R_backward;
    // dynamic instance
    std::vector<double> become_known_array;
    int last_static;
    int num_known_requests;
    std::vector<int> known_requests; 
    int num_forward_requests{ 0 };
    int num_backward_requests{ 0 };
    
    
public:
    double turn_time; // time to turn around
    
    DARP(int); // constructor for an n-node problem
    ~DARP(); 
    // copy/ move constructor and assignment/ move operator not defined, not needed in this project so far (not more than one instance created)
    
        
    int get_instance_mode() const {return instance_mode;}
    void set_instance_mode(int i) {instance_mode = i;}

    // file processing
    void read_file(std::string infile, std::string data_directory, std::string instance);
    void generate_distances(); // for instance_mode 1
    void generate_distances(std::string infile, std::string data_directory, std::string instance, bool consider_shortcuts); // for instance_mode 2
    void generate_distances(std::string infile, std::string data_directory, std::string instance); // for instance_mode 3
    //void transform_dynamic(double share_static_requests = 0.25, double beta = 60);
    void transform_dynamic(double share_static_requests = 1, double beta = 60);
    // tighten time windows if necessary
    void set_time_windows();
    void preprocess();
    void set_alpha_and_beta(double alpha_in, double beta_in);
    

    template<int Q>
    friend class DARPGraph;
    template<int Q>
    friend class RollingHorizon;
    friend class DARPSolver;
};

#endif