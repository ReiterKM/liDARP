#ifndef _DARP_SOLVER_H
#define _DARP_SOLVER_H


class DARPSolver {

    
protected:
    int n; // num_requests
    // measuring model and solve time
    using clock = std::chrono::system_clock;
    using sec = std::chrono::duration<double>;
    
    // solve parameters
    const int notify_requests_sec = 45;
    const int pickup_delay_param = 5;

    // general evaluation
    double total_routing_costs;
    double total_excess_ride_time;
    double answered_requests;
    double avg_waiting_time;
    double avg_transportation_time;
    double avg_ride_time;
    double *time_to_answer; // array that stores time needed to answer request i 
    double avg_time_to_answer;

    // measuring efficiency of ridepooling systems
    double personenkm_gebucht;
    double personenkm_gefahren;
    double empty_mileage;
    double share_empty_mileage;
    double avg_detour_factor;
    double mean_occupancy;
    double system_efficiency;
    double pooling_factor;

    // save requests' status
    std::vector<int> dropped_off; // requests that have been dropped off in the last iteration (since the last reveal of new requests)
    std::vector<int> picked_up; // requests that have been picked up in the last iteration 
    std::vector<int> denied; // requests that have been dropped of in the last iteration
    std::vector<int> seekers; // scheduled requests, i.e. requests that have been accepted but haven't been picked up yet
    std::vector<int> all_picked_up; // requests that have been picked up in the last and in earlier iterations 
    std::vector<int> all_seekers; // and so on...
    std::vector<int> all_denied;
    std::vector<int> all_dropped_off;
    std::vector <int> next_new_requests; 
    std::vector <int> new_requests;

    // check feasibility pf paths
    int ***f; // path feasibilty matrix: 1 = feasible, 0 = infeasible   
    double ***incremental_costs; // path incremental costs matrix
    
public:

    DARPSolver(int);
    ~DARPSolver();
    // no copy/ move constructor or assignment/ move operator needed so far

    // check pairwise feasibility of paths
    void check_paths(DARP& D);
    void check_new_paths(DARP& D, double w1 = 1, double w2 = 60, double w3 = 0.1);

    // 8-step route evaluation scheme by Cordeau and Laporte (2003)
    bool update_vertices(DARP& D, DARPRoute&); // modified != tabu search
    bool update_vertices(DARP& D, DARPRoute&, int); // modified != tabu search
    bool eight_step(DARP& D, DARPRoute&); // modified != tabu search
    bool eight_step(DARP& D, DARPRoute&, int); // modified != tabu search
    
    // feasible path heuristic
    void find_min(double***, int, std::vector<std::array<int,3>>&) const;
    void choose_paths(int, double);

    // Solution display/debugging
    bool verify_routes(DARP& D, bool consider_excess_ride_time, const char*);
    void compute_stats(DARP& D);
    void detailed_file(DARP& D, std::string instance) const;
};
    
#endif