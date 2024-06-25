#include <ilcplex/ilocplex.h>
#include <ilconcert/iloexpression.h>
ILOSTLBEGIN

#ifndef _ROLLING_HORIZON_H
#define _ROLLING_HORIZON_H

// Solve DARP using Rollung Horizon Algorithm
template<int S>
class RollingHorizon : public DARPSolver {

private:
    typedef std::array<int,S> NODE;
    typedef std::array<std::array<int,S>,2> ARC;

    
    // solve time
    double total_time_model;
    double total_time_model_solve;
    // answer requests
    double *communicated_pickup;

    // map requests to variables
    std::unordered_map <int,int> rmap; // rmap[r] = idx for r in D.R (request to index)
    std::unordered_map <NODE,uint64_t,HashFunction<S>> vmap; //vmap[v] = idx for v in G.V_in U G.V_out (vertex to index)
    std::unordered_map <NODE,uint64_t,HashFunction<S>> vinmap; // vmap[v] = idx for v in G.V_in (pick-up vertex to index)
    std::unordered_map <NODE,uint64_t,HashFunction<S>> voutmap; // vmap[v] = idx for v in G.V_out (drop-off vertex to index)
    std::unordered_map <ARC,uint64_t,HashFunction<S>> amap; // amap[a] = idx for a in G.A (arc to index)
    std::unordered_map<NODE,int,HashFunction<S>>* vec_map;  // vec_map[i][w] = idx for i in D.R for w in G.V (drop-off vertex for each request to index)

    // active nodes and arcs
    std::pair<NODE,double>* active_node;
    ARC* active_arc; 
    std::vector<ARC> fixed_edges;
    std::vector<ARC> all_fixed_edges;

    int num_milps = 1; // counter for milps
    const double epsilon = 1e-7; // fix variables in interval of +-epsilon
    double time_passed;
    int modify_obj;
    double denied_timeout = 0; // counts number of requests denied due to timeout
    sec dur_model;
    sec dur_solve;


public:
    // Constructor
    RollingHorizon(int);
    ~RollingHorizon();
    // no copy/ move constructor or assignment/ move operator needed so far

    // maps - after creating nodes and arcs!
    void create_maps(DARP& D, DARPGraph<S>& G);
    void update_maps(const std::vector<int>&, DARP& D, DARPGraph<S>& G);

    // rolling horizon routines
    // before/ first solve
    void first_milp(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloNumVar& d_max, IloRangeArray& accept, IloRangeArray& 
        , IloRangeArray& time_window_ub, IloRangeArray& time_window_lb, IloArray<IloRangeArray>& max_ride_time, IloRangeArray& travel_time, IloRangeArray& flow_preservation, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x, IloRangeArray& pickup_delay, IloRange& num_tours, IloObjective& obj, IloExpr& obj1, IloExpr& obj2, IloExpr& obj3, IloExpr& obj4, const std::array<double,2>& w);
    void query_solution(DARP& D, DARPGraph<S>& G, IloNumArray& B_val, IloIntArray& p_val, IloIntArray& x_val, const std::array<double,2>& w);
    void update_request_sets();
    void erase_dropped_off(bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x);
    void erase_denied(bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x);
    void erase_picked_up(DARPGraph<S>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& fixed_B, IloRangeArray& fixed_x);
    void create_new_variables(bool heuristic, DARP& D, DARPGraph<S>& G, IloEnv& env, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloRangeArray& fixed_B, IloRangeArray& fixed_x, const std::array<double,2>& w);
    void update_milp(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, IloEnv& env, IloModel& model, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloNumVar& d_max, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& time_window_ub, IloRangeArray& time_window_lb, IloArray<IloRangeArray>& max_ride_time, IloRangeArray& travel_time, IloRangeArray& flow_preservation, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x, IloRangeArray& pickup_delay, IloRange& num_tours, IloObjective& obj, IloExpr& obj1, IloExpr& obj3, IloExpr& obj4, const std::array<double,2>& w);
    
    
    // after solve
    void update_graph_sets(bool consider_excess_ride_time, DARPGraph<S>& G, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val); // only for num_milps > 1
    void get_solution_values(bool consider_excess_ride_time, DARP& D, DARPGraph<S>& G, IloCplex& cplex, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d);
    void print_routes(DARP& D, DARPGraph<S>& G, IloNumArray& B_val, IloIntArray& x_val);

    // complete routine
    std::array<double,8> solve(bool accept_all, bool consider_excess_ride_time, bool dynamic, bool heuristic, int time_limit, DARP& D, DARPGraph<S>& G, const std::array<double,2>& w);
};
#endif