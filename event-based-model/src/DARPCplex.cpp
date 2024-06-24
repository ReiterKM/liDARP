#include "DARPH.h"

template <int Q>
std::array<double,3> RollingHorizon<Q>::solve(bool accept_all, bool consider_excess_ride_time, bool dynamic, bool heuristic, DARP& D, DARPGraph<Q>& G, const std::array<double,3>& w)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;
    
    bool solved, flag;
    double phi; // timelimit 
    int next_r;
    double tunnr, tusnr; // time until next new requests and second next new requests
    const double notify_requests_min = double(notify_requests_sec) / 60;
    
    /// ***************Cplex API objects****************
    // Cplex model
    IloCplex cplex;
    IloEnv env;
    IloModel model;

    // save solution values
    IloNumArray B_val, d_val;
    IloIntArray p_val, x_val;  

    // Variables
    IloNumVarArray B, x, p, d;
    IloNumVar d_max;
        
    // Constraints
    IloRangeArray accept, serve_accepted;
    IloRangeArray time_window_ub, time_window_lb;
    IloArray<IloRangeArray> max_ride_time;
    IloRangeArray travel_time;
    IloRangeArray flow_preservation;
    IloRangeArray excess_ride_time;
    IloRangeArray fixed_B, fixed_x;
    IloRangeArray pickup_delay;
    IloRange num_tours;
        
    // Objective function
    IloObjective obj;       
    IloExpr obj1; // routing costs
    IloExpr obj2; // answered requests
    IloExpr obj3; // excess ride time

    /// ************************************************

    // create Graph
    check_paths(D);
    G.create_graph(D,f);
    create_maps(D, G);

    
    // check if problem instance is dynamic
    // if yes, compute next new request and time until it is revealed
    if (dynamic)
    {
        std::cout << "Using pick-up delay limit of " << pickup_delay_param << " minutes.\n";

        time_passed = 0;
        std::cout << "time passed: " << time_passed << std::endl;
        std::cout << "notify_requests_sec: " << notify_requests_sec << std::endl;
        std::cout << "notify_requests_min: " << notify_requests_min << std::endl;

        next_r = D.last_static + 1;
        tunnr = DARPH_INFINITY;
        for (int i = next_r; i <= n; ++i)
        {
            if(std::find(D.known_requests.begin(), D.known_requests.end(), i) == D.known_requests.end())
            {
                // request i is not known yet
                if (D.become_known_array[i-1] < tunnr)
                {
                    tunnr = D.become_known_array[i-1];
                    next_r = i;
                }
            }
        }

        next_new_requests.push_back(next_r);
        if (next_r == D.last_static+1)
            D.last_static = next_r;
        
        // check if there are other requests which become known at the same time
        for (int i=D.last_static+1; i<=n; ++i)
        {
            if(i != next_r)
            {
                if (DARPH_ABS(D.become_known_array[i-1] - tunnr) < DARPH_EPSILON)
                {
                    next_new_requests.push_back(i);
                    if (i == D.last_static+1)
                        D.last_static = i;
                }
            }
        }
        time_passed = D.become_known_array[D.R[0]-1] + min(notify_requests_min, tunnr - D.become_known_array[D.R[0]-1]);
        phi = DARPH_MIN(notify_requests_sec, (tunnr - D.become_known_array[D.R[0]-1]) * 60);
        tunnr = tunnr - time_passed; 
        
        // compute time until second next new request: tusnr 
        // when next new request arrives we can fix routes only for min(notify_requests_min,tusnr) minutes, i.e. until time_passed + tunnr + min(notify_requests_min,tusnr)
        // --> time for computation min(notify_requests_min,tusnr)
        next_r = D.last_static + 1;
        tusnr = DARPH_INFINITY;
        for (int i=next_r; i<=n; ++i)
        {
            if(std::find(next_new_requests.begin(), next_new_requests.end(), i) == next_new_requests.end() && std::find(D.known_requests.begin(), D.known_requests.end(), i) == D.known_requests.end())
            {
                // request i is not known yet and no next new request
                if (D.become_known_array[i-1] < tusnr)
                {
                    tusnr = D.become_known_array[i-1];
                    next_r = i;
                }
            }
        }
        tusnr = tusnr - (tunnr + time_passed);
    


        std::cout << "--------------- MILP " << num_milps << " ---------------" << std::endl;
        std::cout << std::endl << "Time passed [m]: " << time_passed << std::endl;
        sort(D.known_requests.begin(), D.known_requests.end());
        std::cout << "Known requests: ";
        for (const auto& i : D.known_requests)
        {
            std::cout << i << " ";
        }
        std::cout << std::endl;

        if (D.num_known_requests < n)
        {
            std::cout << "Time until next new request(s)[m]: " << tunnr << std::endl;
            std::cout << "Next new request(s): ";
            for (const auto& i : next_new_requests)
            {
                std::cout << i << " ";
            }
        }
        else
            std::cout << "No new requests.\n";
        std::cout << std::endl << std::endl; 
    }


    
    try
    { 
        const auto before = clock::now();
        
        // create MILP from Graph
        first_milp(accept_all, consider_excess_ride_time, D, G, env, model, B_val, d_val, p_val, x_val, B, x, p, d, d_max, accept, serve_accepted, time_window_ub, time_window_lb, max_ride_time, travel_time, flow_preservation, excess_ride_time, fixed_B, fixed_x, pickup_delay, num_tours, obj, obj1, obj2, obj3, w);
        
        // Create the solver object
        cplex = IloCplex(model);
        //name << "MILP/MILP" << "_w0=" << w[0] << "_w1=" << w[1] << "_w_2=" << w[2] << ".lp";
        //cplex.exportModel(name.str().c_str());
        //name.str("");

        dur_model = clock::now() - before;
#if VERBOSE == 0
        cplex.setOut(env.getNullStream());
#endif
         
        cplex.setParam(IloCplex::Param::Simplex::Tolerances::Feasibility, 0.0001);
        if (dynamic)
        {
            phi = phi - dur_model.count();
            cplex.setParam(IloCplex::Param::TimeLimit, phi);
        }
        else
            cplex.setParam(IloCplex::Param::TimeLimit, 7200);
        
        solved = cplex.solve();
        

        dur_solve = clock::now() - before;
        
        if (solved)
        {
            // If CPLEX successfully solved the model, print the results
            get_solution_values(consider_excess_ride_time, D, G, cplex, B_val, d_val, p_val, x_val, B, x, p, d);
        
            while (dynamic && solved && D.num_known_requests < n)
            {   
  
                const auto before = clock::now();
                // when next new request arrives we can fix routes only for min(notify_requests_min,tusnr) minutes, i.e. until time_passed + tunnr + min(notify_requests_min,tusnr)
                // without tusnr: fix routes until time_passed + tunnr + notify_requests_min (oder weniger z.B. 0.15)
                std::cout << "time_passed " << time_passed << std::endl;
                std::cout << "tunnr " << tunnr << std::endl;
                std::cout << "tusnr " << tusnr << std::endl; 
                time_passed += tunnr + min(notify_requests_min, tusnr); // +notify_requests_min wegen 30s bis answer an new request
                std::cout << "time_passed " << time_passed << std::endl;
                phi = DARPH_MIN(notify_requests_sec, tusnr * 60);
                if (phi < notify_requests_sec)
                {
                    printf("Time for optimization < %d. New time: %f", notify_requests_sec, tusnr * 60);
                    if (phi < 3)
                    {
                        printf("WARNING: Time for optimimation < 3 seconds!");
                    }
                }
                
                modify_obj = 0;
                
                query_solution(D, G, B_val, p_val, x_val, w);
                const auto after_query_solution = clock::now();
                update_request_sets();
                const auto after_update_request_sets = clock::now();
                erase_dropped_off(consider_excess_ride_time, D, G, env, model, B_val, B, x, p, accept, serve_accepted, excess_ride_time, fixed_B, fixed_x);
                const auto after_erase_dropped_off = clock::now();
                erase_denied(consider_excess_ride_time, D, G, env, model, B_val, B, x, p, d, accept, serve_accepted, excess_ride_time, fixed_B, fixed_x);
                const auto after_erase_denied = clock::now();
                erase_picked_up(G, env, model, B_val, B, x, p, accept, serve_accepted, fixed_B, fixed_x);
                const auto after_erase_picked_up = clock::now();
                
                new_requests = next_new_requests; // this can be done only AFTER sorting the requests into groups
                next_new_requests.clear();

                create_new_variables(heuristic, D, G, env, B, x, p, d, fixed_B, fixed_x, w);
                const auto after_create_new_variables = clock::now();
                update_milp(accept_all, consider_excess_ride_time, D, G, env, model, B, x, p, d, d_max, accept, serve_accepted, time_window_ub, time_window_lb, max_ride_time, travel_time, flow_preservation, excess_ride_time, fixed_B, fixed_x, pickup_delay, num_tours, obj, obj1, obj3, w);  
                const auto after_update_milp = clock::now();

                const sec dur_query_solution = after_query_solution - before;
                const sec dur_update_request_sets = after_update_request_sets - after_query_solution;
                const sec dur_erase_dropped_off = after_erase_dropped_off - after_update_request_sets;
                const sec dur_erase_denied = after_erase_denied - after_erase_dropped_off;
                const sec dur_erase_picked_up = after_erase_picked_up - after_erase_denied;
                const sec dur_create_new_variables = after_create_new_variables - after_erase_picked_up;
                const sec dur_update_milp = after_update_milp - after_create_new_variables;       
                
                // compute next new request and time limit
                if (D.num_known_requests < n)
                {
                    tunnr = tusnr - min(notify_requests_min,tusnr);
                    next_new_requests.push_back(next_r);
                    if (next_r == D.last_static+1)
                        D.last_static = next_r;
                    
                    // check if there are other requests which become known at the same time
                    for (int i=D.last_static+1; i<=n; ++i)
                    {
                        if(i != next_r)
                        {
                            if (DARPH_ABS(D.become_known_array[i-1] - (time_passed + tunnr)) < DARPH_EPSILON)
                            {
                                next_new_requests.push_back(i);
                                if (i == D.last_static+1)
                                    D.last_static = i;
                            }
                        }
                    }
                    
                    if (D.num_known_requests < n-1)
                    {
                        // compute second next new request 
                        tusnr = DARPH_INFINITY;
                        next_r = D.last_static + 1;
                        for (int i=D.last_static+1; i<=n; ++i)
                        {
                            if(std::find(next_new_requests.begin(), next_new_requests.end(), i) == next_new_requests.end() && std::find(D.known_requests.begin(), D.known_requests.end(), i) == D.known_requests.end())
                            {
                                // requests i is not known yet and not a next new request
                                if (D.become_known_array[i-1] < tusnr)
                                {
                                    tusnr = D.become_known_array[i-1];
                                    next_r = i;
                                }
                            }
                        }
                        tusnr = tusnr - (time_passed + tunnr); 
                    }
                    else 
                        tusnr = notify_requests_min;
                }
                

                num_milps++;

                std::cout << "--------------- MILP " << num_milps << " ---------------" << std::endl;
                std::cout << std::endl << "Time passed [m]: " << time_passed << std::endl;
                sort(D.known_requests.begin(), D.known_requests.end());
                std::cout << "Known requests: ";
                for (const auto& i : D.known_requests)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                
                std::cout << "New request(s): ";
                for (const auto& i : new_requests)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "New dropped-off users: " << std::endl;
                for (const auto& i : dropped_off)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "All dropped-off: " << std::endl;
                for (const auto& i : all_dropped_off)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "All picked-up users: " << std::endl;
                for (const auto& i : all_picked_up)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "All seekers: " << std::endl;
                for (const auto& i : all_seekers)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "New denied: " << std::endl;
                for (const auto& i : denied)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                std::cout << "All denied: " << std::endl;
                for (const auto& i : all_denied)
                {
                    std::cout << i << " ";
                }
                std::cout << std::endl;
                
                if (D.num_known_requests < n)
                {
                    std::cout << "Time until next new request(s)[m]: " << tunnr << std::endl;
                    std::cout << "Next new request(s): ";
                    for (const auto& i : next_new_requests)
                    {
                        std::cout << i << " ";
                    }
                }
                else
                    std::cout << "No new requests.\n";
                std::cout << std::endl << std::endl; 

                //name << "MILP/MILP" << num_milps << ".lp";
                //cplex.exportModel(name.str().c_str());
                //name.str("");
                dur_model = clock::now() - before;

                if (dur_model.count() > 15)
                {
                    std::cout << "Duration model > 15.\n";
                    std::cout << dur_model.count() << "s\n" << std::endl;
                    if (dur_model.count() > phi)
                    {
                        std::cerr << "ERROR in MILP " << num_milps << ":\n"; 
                        std::cerr << "Time to load new model = " << dur_model.count() << "s > Time between new requests = " << phi << "\n";
                    }
                
                    std::cout << "query_solution: " << dur_query_solution.count() << "s\n"; 
                    std::cout << "update_request_sets: " << dur_update_request_sets.count() << "s\n"; 
                    std::cout << "erase_dropped_off: " << dur_erase_dropped_off.count() << "s\n"; 
                    std::cout << "erase_denied: " << dur_erase_denied.count() << "s\n"; 
                    std::cout << "erase_picked_up: " << dur_erase_picked_up.count() << "s\n"; 
                    std::cout << "create_new_variables: " << dur_create_new_variables.count() << "s\n"; 
                    std::cout << "update_milp: " << dur_update_milp.count() << "s\n" << std::endl; 
                }
                
                // optimization can only suceed if the time between two new requests is still greater than the time needed to load the new model
                phi = phi - dur_model.count();
                cplex.setParam(IloCplex::Param::TimeLimit, phi);
                solved = cplex.solve();
                dur_solve = clock::now() - before;

                if (solved)
                {
                    update_graph_sets(consider_excess_ride_time, G, B_val, d_val, p_val, x_val);
                    get_solution_values(consider_excess_ride_time, D, G, cplex, B_val, d_val, p_val, x_val, B, x, p, d);
                }
                else
                {
                    std::cerr << "\n\nCplex error!\n";
                    std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
                    std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
                }
                
                // build new model and solve again   
            } 
            
            if (dynamic)
            {
                // query solution one last time to determine status of new requests
                for (const auto& i : new_requests)
                {
                    flag = false;
                    if (p_val[rmap[i]] > 0.9)
                    { 
                        all_dropped_off.push_back(i);
                        // save active node to compute waiting time at pick-up node
                        for (const auto& v: G.V_i[i])
                        {
                            for (const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    active_arc[i-1] = a;
                                    active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                            if (flag)
                                break;
                        }
                    }
                    else
                    {
                        all_denied.push_back(i);
                    } 
                }
            }
            else
            {
                for (const auto& i : D.R)
                {
                    if (p_val[rmap[i]] < 0.1)
                        all_denied.push_back(i);
                }
            }
#if VERBOSE
            std::cout << "All denied: " << std::endl;
            for (const auto& i : all_denied)
            {
                std::cout << i << " ";
            }
            std::cout << std::endl;
#endif
            
            total_routing_costs = cplex.getValue(obj1);
            std::cout << "Total routing costs: " << total_routing_costs << std::endl; 
#if VERBOSE
            std::cout << "Average routing costs: " << roundf(total_routing_costs / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
            
            
            if (consider_excess_ride_time)
            {
                if (w[2] > DARPH_EPSILON)
                {
                    total_excess_ride_time = cplex.getValue(obj3);
                    std::cout << "Total excess ride time: " << total_excess_ride_time << std::endl; 
#if VERBOSE
                    std::cout << "Average excess ride time: " << roundf(total_excess_ride_time / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
                }
                else
                {
                    total_excess_ride_time = 0;
                    for (const auto& i: D.R)
                    {
                        if (std::find(all_denied.begin(), all_denied.end(), i) == all_denied.end())
                        {
                            total_excess_ride_time += D.nodes[n+i].beginning_service - D.nodes[n+i].start_tw;
                        }
                    }
                    std::cout << "Total excess ride time: " << total_excess_ride_time << std::endl; 
#if VERBOSE
                    std::cout << "Average excess ride time: " << roundf(total_excess_ride_time / (n - all_denied.size()) * 100) / 100 << std::endl; 
#endif
                }
            }

            if (dynamic)
                answered_requests = n - all_denied.size();
            else
            {
                answered_requests = n - cplex.getValue(obj2);
            }

            std::cout << "Number denied requests: " << n - answered_requests << std::endl;
#if VERBOSE
            std::cout << "Percentage denied requests: " << roundf(double(all_denied.size())/ n * 1000) / 1000 << std::endl;   
            std::cout << "Percentage denied requests due to timeout: " << roundf(denied_timeout / double(all_denied.size()) * 100) / 100 << std::endl;    
#endif
            if (dynamic)
            {
                avg_time_to_answer = 0;
                for (int i = 1; i<=n; ++i)
                {
                    avg_time_to_answer += time_to_answer[i-1];
                }
                avg_time_to_answer = avg_time_to_answer / n;
#if VERBOSE
                std::cout << "Average time to answer request: " << roundf(avg_time_to_answer * 100) / 100 << std::endl;   
#endif
            }
#if VERBOSE
            std::cout << "Total time to model: " << roundf(total_time_model * 100) / 100 << std::endl;
            std::cout << "Total time to model + solve: " << roundf(total_time_model_solve * 100) / 100 << std::endl; 
#endif
                     
        }
        else
        {
            std::cerr << "\n\nCplex error!\n";
            std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
            std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
        }

        // free memory of Cplex objects created in first_milp()
        obj1.end();
        obj2.end();
        obj3.end();
        env.end(); 
    }
    catch (IloException& ex) {
        cerr << "Error: " << ex << endl;
    }
    catch (...) {
        cerr << "Error" << endl;
    }

    
    
    
 #if VERIFY_ALL
    if (accept_all || (w[1]>0 && answered_requests!=0))
    {
        verify_routes(D, consider_excess_ride_time, "Test routes returned by dynamic MILP");
    }
#endif        

    std::array<double,3> obj_value = {total_routing_costs, n - answered_requests, total_excess_ride_time};
    return obj_value;
}


template<int Q>
void RollingHorizon<Q>::query_solution(DARP& D, DARPGraph<Q>& G, IloNumArray& B_val, IloIntArray& p_val, IloIntArray& x_val, const std::array<double,3>& w)
{      
    bool flag;

    if (num_milps < 2)
    {
        for (const auto& i: D.R)
        {
            flag = false;
            if (p_val[rmap[i]] > 0.9)
            {
                // determine dropped-off users until time time_passed
                // see if drop-off until time time_passed is possible
                if (time_passed >= D.nodes[n+i].end_tw)
                {
                    // drop-off time window has passed -> user must have been dropped off already
                    dropped_off.push_back(i);
                    // look for active pick-up node and active pick-up arc
                    for (const auto& v: G.V_i[i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[i-1] = a;
                                active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }
                    // look for active drop-off node and active drop-off arc
                    flag = false;
                    for (const auto& v: G.V_i[n+i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[n+i-1] = a;
                                active_node[n+i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }
                }
                else if (time_passed >= D.nodes[n+i].start_tw)
                {
                    // we are within the time window
                    // check if user has been dropped off yet
                    for (const auto& v: G.V_i[n+i])
                    {
                        if (time_passed >= B_val[vmap[v]])
                        {
                            for(const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    dropped_off.push_back(i);
                                    active_arc[n+i-1] = a;
                                    active_node[n+i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                        }
                        if (flag)
                            break;
                    }
                    if (flag)
                    {
                        // user has been dropped off yet --> look for active pick-up node and active pick-up arc 
                        flag = false;
                        for (const auto& v: G.V_i[i])
                        {
                            for (const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    active_arc[i-1] = a;
                                    active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                            if (flag)
                                break;
                        }
                    }
                    else
                    {
                        // we are within drop-off time window but user has not been dropped off yet
                        // check if user has been picked up yet (Überschneidungen pick-up und drop-off time window möglich z.B. request 2 in a2-20)
                        
                        for (const auto& v: G.V_i[i])
                        {
                            if (time_passed >= B_val[vmap[v]])
                            {
                                for (const auto& a: G.delta_in[v])
                                {
                                    if (x_val[amap[a]] > 0.9)
                                    {
                                        picked_up.push_back(i);
                                        active_arc[i-1] = a;
                                        active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                        flag = true;
                                        break;
                                    }
                                }
                                if (flag)
                                    break;
                            } 
                        }   
                        if (!flag)
                            seekers.push_back(i); 
                    }
                }
                else if (time_passed >= D.nodes[i].end_tw)
                {
                    // user must have been picked-up already
                    picked_up.push_back(i);
                    for (const auto& v: G.V_i[i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[i-1] = a;
                                active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }    
                }
                else if (time_passed >= D.nodes[i].start_tw)
                {
                    // we are within pick-up time window
                    // check if user has been picked-up already
                    for (const auto& v: G.V_i[i])
                    {
                        if (time_passed >= B_val[vmap[v]])
                        {
                            for (const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    picked_up.push_back(i);
                                    active_arc[i-1] = a;
                                    active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                        }
                        if (flag)
                            break;     
                    }
                    if (!flag)
                    {
                        // we are within pick-up time window but user has not been picked-up yet
                        seekers.push_back(i);
                    }
                }
                else
                {
                    // user's pick-up time window has not started yet
                    seekers.push_back(i);
                }

                if (flag)  
                {
                    // user has been picked_up or dropped_off
                    modify_obj += w[1];
                }
            }
            else
            {
                denied.push_back(i);
            } 
        }
    }

    
    
    // first step: determine status of new_requests in solution: denied, dropped_off, picked-up, seeker
    if (num_milps > 1)
    {
        for (const auto& i : new_requests)
        {
            flag = false;
            if (p_val[rmap[i]] > 0.9)
            { 
                // check if drop-off until time time_passed is possible
                if (time_passed >= D.nodes[n+i].end_tw)
                {
                    // drop-off time window has passed -> user must have been dropped off already
                    dropped_off.push_back(i);
                    // look for active pick-up node and active pick-up arc
                    for (const auto& v: G.V_i[i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[i-1] = a;
                                active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }
                    // look for active drop-off node and active drop-off arc
                    flag = false;
                    for (const auto& v: G.V_i[n+i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[n+i-1] = a;
                                active_node[n+i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }
                }
                else if (time_passed >= D.nodes[n+i].start_tw)
                {
                    // we are within the drop-off time window
                    // check if user has been dropped off yet
                    for (const auto& v: G.V_i[n+i])
                    {
                        if (time_passed >= B_val[vmap[v]])
                        {
                            for(const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    dropped_off.push_back(i);
                                    active_arc[n+i-1] = a;
                                    active_node[n+i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                        }
                        if (flag)
                            break;
                    }
                    if (flag)
                    {
                        // user has been dropped off yet --> look for active pick-up node and active pick-up arc 
                        flag = false;
                        for (const auto& v: G.V_i[i])
                        {
                            for (const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    active_arc[i-1] = a;
                                    active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                            if (flag)
                                break;
                        }
                    }
                    else
                    {
                        // we are within drop-off time window but user has not been dropped off yet
                        // check if user has been picked up yet (Überschneidungen pick up und drop off time window möglich)
                        
                        for (const auto& v: G.V_i[i])
                        {
                            if (time_passed >= B_val[vmap[v]])
                            {
                                for (const auto& a: G.delta_in[v])
                                {
                                    if (x_val[amap[a]] > 0.9)
                                    {
                                        picked_up.push_back(i);
                                        active_arc[i-1] = a;
                                        active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                        flag = true;
                                        break;
                                    }
                                }
                                if (flag)
                                    break;
                            } 
                        }
                        if (!flag)
                            seekers.push_back(i);   
                    }
                }
                else if (time_passed >= D.nodes[i].end_tw)
                {
                    // user must have been picked-up already
                    picked_up.push_back(i);
                    for (const auto& v: G.V_i[i])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                active_arc[i-1] = a;
                                active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    }    
                }
                else if (time_passed >= D.nodes[i].start_tw)
                {
                    // we are within pick-up time window
                    // check if user has been picked-up already
                    for (const auto& v: G.V_i[i])
                    {
                        if (time_passed >= B_val[vmap[v]])
                        {
                            for (const auto& a: G.delta_in[v])
                            {
                                if (x_val[amap[a]] > 0.9)
                                {
                                    picked_up.push_back(i);
                                    active_arc[i-1] = a;
                                    active_node[i-1] = make_pair(v,B_val[vmap[v]]);
                                    flag = true;
                                    break;
                                }
                            }
                        }
                        if (flag)
                            break;     
                    }
                    if (!flag)
                    {
                        // we are within pick-up time window but user has not been picked-up yet
                        seekers.push_back(i);
                    }
                }
                else
                {
                    // user's pick-up time window has not started yet
                    seekers.push_back(i);
                } 
                

                if (flag)
                {
                    // user has been picked-up or dropped-off
                    // update objective function
                    modify_obj += w[1];
                }
            }
            else
            {
                denied.push_back(i);
            } 
        }
    }

    // second: check if status of any other users in R \ {new_request} = {all_picked_up, all_seekers} has changed
    // i.e. all_picked_up --> dropped_off (objective function is not changed for these cases because it has been changed when user was moved to picked-up)
    // all_seekers --> dropped_off
    // all_seekers --> picked_up

    auto itr = std::begin(all_picked_up);
    while (itr != std::end(all_picked_up))
    {
        flag = false;
        
        // determine dropped-off users until time time_passed
        // see if drop-off until time time_passed is possible
        if (time_passed >= D.nodes[n+(*itr)].end_tw)
        {
            // drop-off time window has passed -> user must have been dropped off already
            dropped_off.push_back(*itr);
            // look for active drop-off node and active drop-off arc
            flag = false;
            for (const auto& v: G.V_i[n+(*itr)])
            {
                for (const auto& a: G.delta_in[v])
                {
                    if (x_val[amap[a]] > 0.9)
                    {
                        active_arc[n+(*itr)-1] = a;
                        active_node[n+(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                        flag = true;
                        break;
                    }
                }
                if (flag)
                    break;
            }
        }
        else if (time_passed >= D.nodes[n+(*itr)].start_tw)
        {
            // we are within the time window
            // check if user has been dropped off yet
            for (const auto& v: G.V_i[n+(*itr)])
            {
                if (time_passed >= B_val[vmap[v]])
                {
                    for(const auto& a: G.delta_in[v])
                    {
                        if (x_val[amap[a]] > 0.9)
                        {
                            dropped_off.push_back(*itr);
                            active_arc[n+(*itr)-1] = a;
                            active_node[n+(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                            flag = true;
                            break;
                        }
                    }
                }
                if (flag)
                    break;
            }
        }  
        // else: drop-off time window has not started yet: remain in picked_up      
        
        if (flag)
            itr = all_picked_up.erase(itr);
        else
            ++itr;
    }


    itr = std::begin(all_seekers);
    while (itr != std::end(all_seekers))
    {
        flag = false;

        // determine dropped-off users until time time_passed
        // see if drop-off until time time_passed is possible
        if (time_passed >= D.nodes[n+(*itr)].end_tw)
        {
            // drop-off time window has passed -> user must have been dropped off already
            dropped_off.push_back(*itr);
            // look for active pick-up node and active pick-up arc
            for (const auto& v: G.V_i[*itr])
            {
                for (const auto& a: G.delta_in[v])
                {
                    if (x_val[amap[a]] > 0.9)
                    {
                        active_arc[(*itr)-1] = a;
                        active_node[(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                        flag = true;
                        break;
                    }
                }
                if (flag)
                    break;
            }
            // look for active drop-off node and active drop-off arc
            flag = false;
            for (const auto& v: G.V_i[n+(*itr)])
            {
                for (const auto& a: G.delta_in[v])
                {
                    if (x_val[amap[a]] > 0.9)
                    {
                        active_arc[n+(*itr)-1] = a;
                        active_node[n+(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                        flag = true;
                        break;
                    }
                }
                if (flag)
                    break;
            }
        }
        else if (time_passed >= D.nodes[n+(*itr)].start_tw)
        {
            // we are within the time window
            // check if user has been dropped off yet
            for (const auto& v: G.V_i[n+(*itr)])
            {
                if (time_passed >= B_val[vmap[v]])
                {
                    for(const auto& a: G.delta_in[v])
                    {
                        if (x_val[amap[a]] > 0.9)
                        {
                            dropped_off.push_back(*itr);
                            active_arc[n+(*itr)-1] = a;
                            active_node[n+(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                            flag = true;
                            break;
                        }
                    }
                }
                if (flag)
                    break;
            }
            if (flag)
            {
                // user has been dropped off yet --> look for active pick-up node and active pick-up arc 
                flag = false;
                for (const auto& v: G.V_i[*itr])
                {
                    for (const auto& a: G.delta_in[v])
                    {
                        if (x_val[amap[a]] > 0.9)
                        {
                            active_arc[(*itr)-1] = a;
                            active_node[(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                            flag = true;
                            break;
                        }
                    }
                    if (flag)
                        break;
                }
            }
            else
            {
                // we are within drop-off time window but user has not been dropped off yet
                // check if user has been picked up yet (Überschneidungen pick up und drop off time window möglich)
                
                for (const auto& v: G.V_i[*itr])
                {
                    if (time_passed >= B_val[vmap[v]])
                    {
                        for (const auto& a: G.delta_in[v])
                        {
                            if (x_val[amap[a]] > 0.9)
                            {
                                picked_up.push_back(*itr);
                                active_arc[(*itr)-1] = a;
                                active_node[(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                                flag = true;
                                break;
                            }
                        }
                        if (flag)
                            break;
                    } 
                } 
            }
        }
        else if (time_passed >= D.nodes[*itr].end_tw)
        {
            // user must have been picked-up already
            picked_up.push_back(*itr);
            for (const auto& v: G.V_i[*itr])
            {
                for (const auto& a: G.delta_in[v])
                {
                    if (x_val[amap[a]] > 0.9)
                    {
                        active_arc[(*itr)-1] = a;
                        active_node[(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                        flag = true;
                        break;
                    }
                }
                if (flag)
                    break;
            }    
        }
        else if (time_passed >= D.nodes[*itr].start_tw)
        {
            // we are within pick-up time window
            // check if user has been picked-up already
            for (const auto& v: G.V_i[*itr])
            {
                if (time_passed >= B_val[vmap[v]])
                {
                    for (const auto& a: G.delta_in[v])
                    {
                        if (x_val[amap[a]] > 0.9)
                        {
                            picked_up.push_back(*itr);
                            active_arc[(*itr)-1] = a;
                            active_node[(*itr)-1] = make_pair(v,B_val[vmap[v]]);
                            flag = true;
                            break;
                        }
                    }
                }
                if (flag)
                    break;     
            }
        // if (!flag): we are within pick-up time window but user has not been picked-up yet: remain in seekers
        }
        // else: pick-up time window has not started yet: remain in seekers

        if (flag)
        {
            itr = all_seekers.erase(itr);
            modify_obj += w[1];
        }
        else
            ++itr;
    }

    // look for edges to fix
    for (const auto& i: picked_up)
    {
        // check if edge has been fixed previously weil sie schon befahren wurde
        if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), active_arc[i-1]) == all_fixed_edges.end())
        {
            fixed_edges.push_back(active_arc[i-1]);
        }
    }
    for (const auto& i: dropped_off)
    {
        // check if edge has been fixed previously weil sie schon befahren wurde oder user schon picked_up war
        if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), active_arc[i-1]) == all_fixed_edges.end())
        {
            fixed_edges.push_back(active_arc[i-1]);
        }
        if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), active_arc[n+i-1]) == all_fixed_edges.end())
        {
            fixed_edges.push_back(active_arc[n+i-1]);
        }
    }
    // delayed departure has begun - fix edge
    for (const auto& a: G.delta_out[G.depot])
    {
        // können mehrmals nacheinander auf demselben Streckenabschnitt sein wenn Zeit nicht weit voran geht, daher prüfen ob schon in all_fixed_edges
        if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), a) == all_fixed_edges.end())
        {
            if (x_val[amap[a]] > 0.9 && time_passed >= B_val[vmap[a[1]]] - G.t[a] && time_passed < B_val[vmap[a[1]]])
            {
                // delayed departure ist bereits geschehen
                fixed_edges.push_back(a);
                active_node[a[1][0]-1] = make_pair(a[1], B_val[vmap[a[1]]]);
            }
        }
    }
    // delayed departure of outgoing arcs of picked_up users
    for (const auto& i: picked_up)
    {    
        for (const auto& a : G.delta_out[active_node[i-1].first])
        {
            if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), a) == all_fixed_edges.end())
            {
                if (x_val[amap[a]] > 0.9 && time_passed >= B_val[vmap[a[1]]] - G.t[a] && time_passed < B_val[vmap[a[1]]])
                {
                    // delayed departure ist bereits geschehen
                    fixed_edges.push_back(a);
                    active_node[a[1][0]-1] = make_pair(a[1], B_val[vmap[a[1]]]);
                    //has_fixed_successor[i] = true;
                }
            }
        }
    }
    // delayed departure of outgoing arcs of dropped_off users
    for (const auto& i: dropped_off)
    {
        for (const auto& a : G.delta_out[active_node[n+i-1].first])
        {
            if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), a) == all_fixed_edges.end())
            {
                if (x_val[amap[a]] > 0.9 && time_passed >= B_val[vmap[a[1]]] - G.t[a] && time_passed < B_val[vmap[a[1]]])
                {
                    // delayed departure ist bereits geschehen
                    fixed_edges.push_back(a);
                    active_node[a[1][0]-1] = make_pair(a[1], B_val[vmap[a[1]]]);
                }
            }
        }
    }
}

template<int Q>
void RollingHorizon<Q>::update_request_sets() {
    all_picked_up.insert(all_picked_up.end(), picked_up.begin(), picked_up.end());
    all_dropped_off.insert(all_dropped_off.end(), dropped_off.begin(), dropped_off.end());
    all_seekers.insert(all_seekers.end(), seekers.begin(), seekers.end());
    all_denied.insert(all_denied.end(), denied.begin(), denied.end());
}



template<int Q>
void RollingHorizon<Q>::erase_dropped_off(bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;
    bool flag;

    // erase dropped-off users 
    for (const auto& i: dropped_off)
    {

        // remove from requests R and variables p_i
        D.R.erase(std::remove(D.R.begin(), D.R.end(), i), D.R.end());
        p[rmap[i]].end();
        accept[rmap[i]].end();
        serve_accepted[rmap[i]].end();
        // dont't delete variable d_i!
                        
        // remove nodes from V_in, V_i[i], fix variables B_v and remove corresponding constraints (except active node)
        for (const auto& v : G.V_i[i])
        {     
            if (v != active_node[i-1].first)
            {
                G.V_in.erase(std::remove(G.V_in.begin(), G.V_in.end(), v), G.V_in.end());
                if constexpr (Q==3)
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," <<  v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                fixed_B[vmap[v]] = IloRange(env, B_val[vmap[v]] - epsilon, B[vmap[v]], B_val[vmap[v]] + epsilon, name.str().c_str());
                model.add(fixed_B[vmap[v]]);
                name.str(""); 

                for (const auto& a: G.delta_in[v])
                {
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_in.erase(v);
                for (const auto& a: G.delta_out[v])
                {
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());                             
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_out.erase(v);

            }
            else
            {
                // v is active pick-up node 

                // erase all incoming arcs != active arc
                for (const auto& a: G.delta_in[v])
                {
                    if (a != active_arc[i-1])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());                              
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str(""); 

                    }
                }
                G.delta_in[v].clear();
                G.delta_in[v].push_back(active_arc[i-1]); 
            }
        }

        
        G.V_i[i].clear(); // !!!
        G.V_i[i].push_back(active_node[i-1].first);

        for (const auto& v : G.V_i[n+i])
        {
            if (v != active_node[n+i-1].first)
            {

                G.V_out.erase(std::remove(G.V_out.begin(), G.V_out.end(), v), G.V_out.end());
                if constexpr (Q==3)
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," <<  v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                fixed_B[vmap[v]] = IloRange(env, B_val[vmap[v]] - epsilon, B[vmap[v]], B_val[vmap[v]] + epsilon, name.str().c_str());
                model.add(fixed_B[vmap[v]]);
                name.str(""); 
                
                // erase arcs incident to v
                for (const auto& a: G.delta_in[v])
                {
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_in.erase(v);

                for (const auto& a: G.delta_out[v])
                {
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_out.erase(v);
            }
            else
            {

                // v is active drop-off node
                // don't end() excess_ride_time - needed for computation of d_i

                // erase all incoming arcs != active drop-off arc
                for (const auto& a: G.delta_in[v])
                {
                    if (a != active_arc[n+i-1])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                }
                G.delta_in[v].clear();
                G.delta_in[v].push_back(active_arc[n+i-1]);
            }  
        }

        G.V_i[n+i].clear(); // !!!
        G.V_i[n+i].push_back(active_node[n+i-1].first);

        // first for V_in
        // erase all nodes but active nodes
        auto itr = std::begin(G.V_in);
        while (itr != std::end(G.V_in))
        {

            flag = true;
            for (int k=1; k<=n; ++k)
            {
                if ((*itr) == active_node[k-1].first)
                    flag = false;
            }

            if (flag)
            {
                flag = false;
                for (int j=1; j<D.veh_capacity; ++j)
                {
                    if ((*itr)[j] == i)
                    { 
                        if constexpr (Q==3)
                            name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," << (*itr)[2] << ")";
                        else
                            name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," <<  (*itr)[2] << "," << (*itr)[3] << "," << (*itr)[4] << "," << (*itr)[5] << ")";
                        fixed_B[vmap[*itr]] = IloRange(env, B_val[vmap[*itr]] - epsilon, B[vmap[*itr]], B_val[vmap[*itr]] + epsilon, name.str().c_str());
                        model.add(fixed_B[vmap[*itr]]);
                        name.str(""); 
                        G.V_i[(*itr)[0]].erase(std::remove(G.V_i[(*itr)[0]].begin(), G.V_i[(*itr)[0]].end(), *itr), G.V_i[(*itr)[0]].end());
                    
                        // erase arcs incident to v
                        for (const auto& a: G.delta_in[*itr])
                        {
                            G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                            G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                            if constexpr (Q==3)
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                            else
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                            fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                            model.add(fixed_x[amap[a]]);  
                            name.str("");
                        }
                        G.delta_in.erase(*itr);

                        for (const auto& a: G.delta_out[*itr])
                        {
                            G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                            G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                            if constexpr (Q==3)
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                            else
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                            fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                            model.add(fixed_x[amap[a]]);  
                            name.str("");   
                        }
                        G.delta_out.erase(*itr);

                        itr = G.V_in.erase(itr);
                        flag = true;
                        break;
                    }
                }
            }
            if (!flag)
                ++itr;
        }

        // now for V_out
        itr = std::begin(G.V_out);
        while (itr != std::end(G.V_out))
        {

            flag = true;
            for (int k=n+1; k<=D.num_nodes; ++k)
            {
                if ((*itr) == active_node[k-1].first)
                    flag = false;
            }

            if (flag)
            {
                flag = false;
                for (int j=1; j<D.veh_capacity; ++j)
                {
                    if ((*itr)[j] == i)
                    {
                        if constexpr (Q==3)
                            name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," << (*itr)[2] << ")";
                        else
                            name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," <<  (*itr)[2] << "," << (*itr)[3] << "," << (*itr)[4] << "," << (*itr)[5] << ")";
                        fixed_B[vmap[*itr]] = IloRange(env, B_val[vmap[*itr]] - epsilon, B[vmap[*itr]], B_val[vmap[*itr]] + epsilon, name.str().c_str());
                        model.add(fixed_B[vmap[*itr]]);
                        name.str(""); 
                        
                        if (consider_excess_ride_time) 
                        {
                            excess_ride_time[voutmap[*itr]].setUB(D.nodes[(*itr)[0]].start_tw + B_val[vmap[*itr]]);
                        }
                        
                        G.V_i[(*itr)[0]].erase(std::remove(G.V_i[(*itr)[0]].begin(), G.V_i[(*itr)[0]].end(), *itr), G.V_i[(*itr)[0]].end());                                        

                        // erase arcs incident to v
                        for (const auto& a: G.delta_in[*itr])
                        {
                            G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                            G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                            if constexpr (Q==3)
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                            else
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                            fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                            model.add(fixed_x[amap[a]]);  
                            name.str("");   
                        }
                        G.delta_in.erase(*itr);

                        for (const auto& a: G.delta_out[*itr])
                        {
                            G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                            G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                            if constexpr (Q==3)
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                            else
                                name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                            fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                            model.add(fixed_x[amap[a]]);  
                            name.str(""); 
                        }
                        G.delta_out.erase(*itr);

                        itr = G.V_out.erase(itr);                                  
                        flag = true;
                        break;
                    }
                }
            }
            if (!flag)
                ++itr;
        }

    }
}


template<int Q>
void RollingHorizon<Q>::erase_denied(bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;
    bool flag;
    // erase denied users 
    for (const auto& i: denied)
    {
        // remove from requests R and variables p_i
        D.R.erase(std::remove(D.R.begin(), D.R.end(), i), D.R.end());
        p[rmap[i]].end();
        d[rmap[i]].end();
        serve_accepted[rmap[i]].end();
        accept[rmap[i]].end();

        // remove nodes from V and variables B_v
        for (const auto& v: G.V_i[i])
        {

            G.V_in.erase(std::remove(G.V_in.begin(), G.V_in.end(), v), G.V_in.end());
            if constexpr (Q==3)
                name << "fixed_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "fixed_B_(" << v[0] << "," << v[1] << "," <<  v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            fixed_B[vmap[v]] = IloRange(env, B_val[vmap[v]] - epsilon, B[vmap[v]], B_val[vmap[v]] + epsilon, name.str().c_str());
            model.add(fixed_B[vmap[v]]);
            name.str(""); 
                      
            // erase arcs incident to v
            for (const auto& a: G.delta_in[v])
            {
                G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                if constexpr (Q==3)
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                else
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                model.add(fixed_x[amap[a]]);  
                name.str("");
            }
            G.delta_in.erase(v);

            for (const auto& a: G.delta_out[v])
            {
                G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                if constexpr (Q==3)
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                else
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                model.add(fixed_x[amap[a]]);  
                name.str("");
            }
            G.delta_out.erase(v); 
        }
        G.V_i.erase(i);
        
        for (const auto& v: G.V_i[n+i])
        {

            G.V_out.erase(std::remove(G.V_out.begin(), G.V_out.end(), v), G.V_out.end());
            if constexpr (Q==3)
                name << "fixed_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "fixed_B_(" << v[0] << "," << v[1] << "," <<  v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            fixed_B[vmap[v]] = IloRange(env, B_val[vmap[v]] - epsilon, B[vmap[v]], B_val[vmap[v]] + epsilon, name.str().c_str());
            model.add(fixed_B[vmap[v]]);
            name.str(""); 

            // erase arcs incident to v
            for (const auto& a: G.delta_in[v])
            {
                G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());                           
                if constexpr (Q==3)
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                else
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                model.add(fixed_x[amap[a]]);  
                name.str("");
            }
            G.delta_in.erase(v);

            for (const auto& a: G.delta_out[v])
            {
                G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                if constexpr (Q==3)
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                else
                    name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                model.add(fixed_x[amap[a]]);  
                name.str("");
            }
            G.delta_out.erase(v);
                        
        }
        G.V_i.erase(n+i);

        // have to use itr here because we want to modify V and erase elements as we iterate through V
        // first for V_in
        auto itr = std::begin(G.V_in);
        while (itr != std::end(G.V_in))
        {
            flag = false;
            for (int j=1; j<D.veh_capacity; ++j)
            {
                if ((*itr)[j] == i)
                {

                    if constexpr (Q==3)
                        name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," << (*itr)[2] << ")";
                    else
                        name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," <<  (*itr)[2] << "," << (*itr)[3] << "," << (*itr)[4] << "," << (*itr)[5] << ")";
                    fixed_B[vmap[*itr]] = IloRange(env, B_val[vmap[*itr]] - epsilon, B[vmap[*itr]], B_val[vmap[*itr]] + epsilon, name.str().c_str());
                    model.add(fixed_B[vmap[*itr]]);
                    name.str("");
                    
                    G.V_i[(*itr)[0]].erase(std::remove(G.V_i[(*itr)[0]].begin(), G.V_i[(*itr)[0]].end(), *itr), G.V_i[(*itr)[0]].end());
                                
                    // erase arcs incident to v
                    for (const auto& a: G.delta_in[*itr])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                    G.delta_in.erase(*itr);

                    for (const auto& a: G.delta_out[*itr])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                    G.delta_out.erase(*itr);

                    itr = G.V_in.erase(itr);
                    flag = true;
                    break;
                }
            }
            if (!flag)
                ++itr;
        }

        // now for V_out
        itr = std::begin(G.V_out);
        while (itr != std::end(G.V_out))
        {
            flag = false;
            for (int j=1; j<D.veh_capacity; ++j)
            {
                if ((*itr)[j] == i)
                { 
                                    
                    if constexpr (Q==3)
                        name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," << (*itr)[2] << ")";
                    else
                        name << "fixed_B_(" << (*itr)[0] << "," << (*itr)[1] << "," <<  (*itr)[2] << "," << (*itr)[3] << "," << (*itr)[4] << "," << (*itr)[5] << ")";
                    fixed_B[vmap[*itr]] = IloRange(env, B_val[vmap[*itr]] - epsilon, B[vmap[*itr]], B_val[vmap[*itr]] + epsilon, name.str().c_str());
                    model.add(fixed_B[vmap[*itr]]);
                    name.str(""); 
                    if (consider_excess_ride_time) 
                    {
                        excess_ride_time[voutmap[*itr]].setUB(D.nodes[(*itr)[0]].start_tw + B_val[vmap[*itr]]);
                    }
                    
                    G.V_i[(*itr)[0]].erase(std::remove(G.V_i[(*itr)[0]].begin(), G.V_i[(*itr)[0]].end(), *itr), G.V_i[(*itr)[0]].end());

                    // erase arcs incident to v
                    for (const auto& a: G.delta_in[*itr])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                    G.delta_in.erase(*itr);

                    for (const auto& a: G.delta_out[*itr])
                    {
                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                    G.delta_out.erase(*itr);

                    itr = G.V_out.erase(itr);
                    flag = true;
                    break;
                }
            }
            if (!flag)
                ++itr;
        }
    }
}



template<int Q>
void RollingHorizon<Q>::erase_picked_up(DARPGraph<Q>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& fixed_B, IloRangeArray& fixed_x)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;
    // erase pick-up nodes corresponding to on-board users 
    for (const auto& i: picked_up)
    {
      
        // remove variable p_i and constraints 
        p[rmap[i]].end();
        accept[rmap[i]].end();
        serve_accepted[rmap[i]].end();
                     
        // erase all pick-up nodes but active node
        for (const auto& v: G.V_i[i])
        { 
            if (v != active_node[i-1].first)
            {

                G.V_in.erase(std::remove(G.V_in.begin(), G.V_in.end(), v), G.V_in.end());
        
                if constexpr (Q==3)
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
                else
                    name << "fixed_B_(" << v[0] << "," << v[1] << "," <<  v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
                fixed_B[vmap[v]] = IloRange(env, B_val[vmap[v]] - epsilon, B[vmap[v]], B_val[vmap[v]] + epsilon, name.str().c_str());
                model.add(fixed_B[vmap[v]]);
                name.str("");
                
                // erase arcs incident to v
                for (const auto& a: G.delta_in[v])
                { 
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_in.erase(v);
                for (const auto& a: G.delta_out[v])
                {
                    G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                    G.delta_in[a[1]].erase(std::remove(G.delta_in[a[1]].begin(), G.delta_in[a[1]].end(), a), G.delta_in[a[1]].end());
                    if constexpr (Q==3)
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                    else
                        name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                    fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                    model.add(fixed_x[amap[a]]);  
                    name.str("");
                }
                G.delta_out.erase(v);
                             
            }
            else
            {                              
                // erase all incoming arcs != active arc
                for (const auto& a: G.delta_in[v])
                {
                    if (a != active_arc[i-1])
                    {

                        G.A.erase(std::remove(G.A.begin(), G.A.end(), a), G.A.end());
                        G.delta_out[a[0]].erase(std::remove(G.delta_out[a[0]].begin(), G.delta_out[a[0]].end(), a), G.delta_out[a[0]].end());
                        if constexpr (Q==3)
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
                        else
                            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
                        fixed_x[amap[a]] = IloRange(env, 0, x[amap[a]], 0, name.str().c_str());
                        model.add(fixed_x[amap[a]]);  
                        name.str("");
                    }
                }
                G.delta_in[v].clear();
                G.delta_in[v].push_back(active_arc[i-1]); 
            } 
        }
        G.V_i[i].clear(); // !!!
        G.V_i[i].push_back(active_node[i-1].first);   
    }
}



template<int Q>
void RollingHorizon<Q>::create_new_variables(bool heuristic, DARP& D, DARPGraph<Q>& G, IloEnv& env, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloRangeArray& fixed_B, IloRangeArray& fixed_x, const std::array<double,3>& w)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    check_new_paths(D, w[0], w[1], w[2]);
    if (heuristic)
    {
        choose_paths(10, 0.25);  // min(10, 0.25 * num_feas_paths) paths allowed
        std::cout << std::endl << "Num of feas paths: 25%, but at least 10\n";
    }
    
    G.create_new_nodes(D, f, new_requests);
    G.create_new_arcs(D, f, new_requests, all_seekers);
    update_maps(new_requests, D, G);

    B.add(G.num_new_nodes, IloNumVar(env,0,IloInfinity,ILOFLOAT)); 
    for (const auto& v: G.V_in_new)
    {
        if constexpr (Q==3)
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str("");
    }
    for (const auto& v: G.V_out_new)
    {
        if constexpr (Q==3)
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str("");
    } 
    fixed_B.add(G.num_new_nodes, IloRange());
    
    x.add(G.num_new_arcs, IloNumVar(env,0,1,ILOBOOL));
    for (const auto& a: G.A_new)
    {
        if constexpr (Q==3)
            name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        x[amap[a]] = IloNumVar(env,0,1,ILOBOOL,name.str().c_str());
        name.str("");
    }
    fixed_x.add(G.num_new_arcs, IloRange());
    

    // add new requests to R
    // this has to be done AFTER checking compatibility with all other requests in R 
    p.add(new_requests.size(),IloNumVar(env,0,1,ILOBOOL));
    d.add(new_requests.size(),IloNumVar(env, 0, IloInfinity, ILOFLOAT));
    for (const auto& i : new_requests)
    {
        D.R.push_back(i);
        D.rcardinality++;
        D.known_requests.push_back(i);
        D.num_known_requests++;
        rmap[i] = D.num_known_requests - 1;
        name << "p_" << i;
        p[rmap[i]] = IloNumVar(env,0,1,ILOBOOL,name.str().c_str());
        name.str("");
        name << "d_" << i;
        d[rmap[i]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str("");
    }
}


template<int Q>
void RollingHorizon<Q>::update_milp(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, IloEnv& env, IloModel& model, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloNumVar& d_max, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& time_window_ub, IloRangeArray& time_window_lb, IloArray<IloRangeArray>& max_ride_time, IloRangeArray& travel_time, IloRangeArray& flow_preservation, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x, IloRangeArray& pickup_delay, IloRange& num_tours, IloObjective& obj, IloExpr& obj1, IloExpr& obj3, const std::array<double,3>& w)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;   
    IloExpr expr(env);
    
    // fix variable B_w for new fixed edges 
    for (const auto& a: fixed_edges)
    {
        if constexpr (Q==3)
            name << "fixed_B_(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "fixed_B_(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        fixed_B[vmap[a[1]]] = IloRange(env, active_node[a[1][0]-1].second - epsilon, B[vmap[a[1]]], active_node[a[1][0]-1].second + epsilon, name.str().c_str());
        model.add(fixed_B[vmap[a[1]]]);
        name.str("");  
    }

    // fix variable x_a for new fixed_edges
    for (const auto& a: fixed_edges)
    {
        all_fixed_edges.push_back(a);
        // fix with constraint
        if constexpr (Q==3)
            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "fixed_x(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        fixed_x[amap[a]] = IloRange(env, 1, x[amap[a]], 1, name.str().c_str());
        model.add(fixed_x[amap[a]]);  
        name.str("");
    }
 
    accept.add(new_requests.size(),IloRange());
    if (accept_all)
    {
        for (const auto& i : new_requests)
        {
            name << "accept_" << i;
            accept[rmap[i]] = IloRange(env, 1, p[rmap[i]], 1, name.str().c_str()); 
            model.add(accept[rmap[i]]);
            name.str("");
        }
    }
    
    // fixed variables p_i = 1 for all i in seekers
    if (!accept_all)
    {
        for (const auto& i: seekers)
        {
            name << "accept_" << i;
            accept[rmap[i]] = IloRange(env,1,p[rmap[i]],1,name.str().c_str()); 
            model.add(accept[rmap[i]]);
            name.str("");
        }
    }

    // pick-up time communicated to user may not be delayed by more than pickup_delay minutes
    // pick-up time for picked_up oder dropped_off users is fixed anyway --> fix time for seekers
    pickup_delay.add(G.V_in_new.size(),IloRange()); // add space in constraints array for new pick_up_nodes
    for (const auto& i : seekers)
    {
        for (const auto& v: G.V_i[i])
        {
            if constexpr (Q==3)
                name << "pickup_delay_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "pickup_delay_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")"; 
            for (const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
            pickup_delay[vinmap[v]] = IloRange(env, 0, -B[vmap[v]] + (1-expr) * D.nodes[i].end_tw + expr * (communicated_pickup[i-1] + pickup_delay_param), IloInfinity, name.str().c_str());
            expr.clear();
            name.str("");
            model.add(pickup_delay[vinmap[v]]);
        }

    }
                    
    for (const auto & i : all_seekers)
    {
        for (const auto& v: G.V_i_new[i])
        {   
            if constexpr (Q==3)
                name << "pickup_delay_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "pickup_delay_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")"; 
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
            pickup_delay[vinmap[v]] = IloRange(env, 0, -B[vmap[v]] + (1-expr) * D.nodes[i].end_tw + expr * (communicated_pickup[i-1] + pickup_delay_param), IloInfinity, name.str().c_str());
            expr.clear();
            name.str("");
            model.add(pickup_delay[vinmap[v]]);
        }   
    }
    
    std::cout << "modify obj " << modify_obj << std::endl;
    // update objective function
    expr = obj.getExpr();
    for (const auto& a: G.A_new)
    {
        expr += w[0] * G.c[a] * x[amap[a]];
        obj1 += G.c[a] * x[amap[a]];
    }
    
    if (!accept_all)
    {
        // constant w[1] has been removed for all users, for which the variable p_i has been removed and equals 1
        // i.e. picked-up or dropped-off users (but not dropped-off users which have previously had the picked-up status)
        expr -= modify_obj;
        for (const auto& i : new_requests)
        {
            expr += w[1];
            expr -= w[1] * p[rmap[i]];
            // obj2 doesn't contain the correct number of denied requests because p[rmap[i]] has been deleted for all dropped off and picked up and denied users
        }
    }
    if (consider_excess_ride_time)
    {
        for (const auto& i : new_requests)
        {
            expr += w[2] * d[rmap[i]];
            obj3 += d[rmap[i]];
        }
    }
    obj.setExpr(expr);
    expr.clear();

    
    // update 'flow preservation' constraints
    // for every old node v add arcs in delta_in, delta_out to expr 
    // depot
    expr = flow_preservation[vmap[G.depot]].getExpr();
    for (const auto& a: G.delta_in_new[G.depot])
    {
        expr += x[amap[a]];
    }
    for (const auto& a: G.delta_out_new[G.depot])
    {
        expr -= x[amap[a]];
    }
    flow_preservation[vmap[G.depot]].setExpr(expr);
    expr.clear();

    // V_in
    for (const auto& v: G.V_in)
    {
        if (!G.delta_in_new[v].empty() || !G.delta_out_new[v].empty())
        {
            expr = flow_preservation[vmap[v]].getExpr();
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
            for (const auto& a: G.delta_out_new[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]].setExpr(expr);
            expr.clear();
        }
    }

    
    // V_out
    for (const auto& v: G.V_out)
    {
        if (!G.delta_in_new[v].empty() || !G.delta_out_new[v].empty())
        {
            //anzahl_vout += 1;
            //const auto ux1 = clock::now();
            
            expr = flow_preservation[vmap[v]].getExpr();
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
            for (const auto& a: G.delta_out_new[v])
            {
                expr -= x[amap[a]];
            }
            flow_preservation[vmap[v]].setExpr(expr);
            expr.clear();
        }
    }

    // add constraint for every new node
    flow_preservation.add(G.num_new_nodes,IloRange());
    for (const auto& v: G.V_in_new)
    {
        if constexpr (Q==3)
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        for(const auto& a: G.delta_in_new[v])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out_new[v])
        {
            expr -= x[amap[a]];
        }
        flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
        model.add(flow_preservation[vmap[v]]);
        expr.clear();
        name.str("");
    }

    for (const auto& v: G.V_out_new)
    {
        if constexpr (Q==3)
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        for(const auto& a: G.delta_in_new[v])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out_new[v])
        {
            expr -= x[amap[a]];
        }
        flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
        model.add(flow_preservation[vmap[v]]);
        expr.clear();
        name.str("");
    }          


    // every accepted request is served
    for (const auto& i: all_seekers)
    {
        expr = serve_accepted[rmap[i]].getExpr();
        // 1. add new arcs in delta_in to expr 
        // 2. add new nodes with v_1 = i and corresponding ingoing arcs (new nodes are not contained in V_i)
        for (const auto& v: G.V_i[i])
        {
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
        }
        for (const auto& v: G.V_i_new[i])
        {
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
        }
        serve_accepted[rmap[i]].setExpr(expr);
        expr.clear();
    }

    // add constraint for new requests 
    serve_accepted.add(new_requests.size(),IloRange());
    for (const auto& i : new_requests)
    {
        for (const auto& v: G.V_i_new[i])
        {
            for (const auto& a: G.delta_in_new[v])
            {
                expr += x[amap[a]];
            }
        }
        expr -= p[rmap[i]];
        name << "serve_accepted_" << i;
        serve_accepted[rmap[i]] = IloRange(env,0,expr,0,name.str().c_str());
        model.add(serve_accepted[rmap[i]]);
        expr.clear();
        name.str("");
    }
    
    // number of tours
    expr = num_tours.getExpr();
    for (const auto& a: G.delta_out_new[G.depot])
    {
        expr += x[amap[a]];
    }
    num_tours.setExpr(expr);
    expr.clear();
    

    // travel time arc a
    // for all new arcs
    travel_time.add(G.num_new_arcs, IloRange());
    for (const auto& a: G.A_new)
    {
        if constexpr (Q==3)
            name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        if (a[0] != G.depot)
        {
            // check if node a[0] has been reached already
            if ((a[0][0] <= n && std::find(all_picked_up.begin(), all_picked_up.end(), a[0][0]) != all_picked_up.end())||(a[0][0] > n && std::find(all_dropped_off.begin(), all_dropped_off.end(), a[0][0] - n) != all_dropped_off.end()))
            {
                expr = -B[vmap[a[1]]] + time_passed + D.nodes[a[0][0]].service_time + G.t[a] - (time_passed - D.nodes[a[1][0]].start_tw + G.t[a] + D.nodes[a[0][0]].service_time) * (1 - x[amap[a]]);
                travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
                model.add(travel_time[amap[a]]);
            }
            else
            {
                expr = -B[vmap[a[1]]] + B[vmap[a[0]]] + D.nodes[a[0][0]].service_time + G.t[a] - (D.nodes[a[0][0]].end_tw - D.nodes[a[1][0]].start_tw + G.t[a] + D.nodes[a[0][0]].service_time) * (1 - x[amap[a]]);
                travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
                model.add(travel_time[amap[a]]);
            }
        }
        else
        {
            expr = -B[vmap[a[1]]] + G.t[a] * x[amap[a]];
            travel_time[amap[a]] = IloRange(env,expr,-time_passed,name.str().c_str());
            model.add(travel_time[amap[a]]);
        }
        expr.clear();
        name.str("");
    }

    // old edges that are not fixed and start in depot
    for (const auto& a: G.delta_out[G.depot])
    {
        if (std::find(all_fixed_edges.begin(), all_fixed_edges.end(), a) == all_fixed_edges.end())
        {
            travel_time[amap[a]].setUB(-time_passed);
        }
    }  


    IloExpr inner_expr(env);
    // time constraints pick-up and drop-off
    // old time constraints: add new arcs from delta_in 
    for (const auto& v: G.V_in)
    {
        if (!G.delta_in_new[v].empty())
        {
            expr = time_window_lb[vmap[v]].getExpr();

            for (const auto& a: G.delta_in_new[v])
            {
                inner_expr += x[amap[a]];
            }
            expr -= D.nodes[v[0]].tw_length * inner_expr;
            time_window_lb[vmap[v]].setExpr(expr);
            inner_expr.clear();
            expr.clear();
        }
    }
    for (const auto& v: G.V_out)
    {
        if (!G.delta_in_new[v].empty())
        {
            expr = time_window_ub[vmap[v]].getExpr();

            for (const auto& a: G.delta_in_new[v])
            {
                inner_expr += x[amap[a]];
            }
            expr += D.nodes[v[0]-n].tw_length * inner_expr;
            time_window_ub[vmap[v]].setExpr(expr);
            inner_expr.clear();
            expr.clear();
        }
    }
    inner_expr.end();

    // time window constraints for new nodes
    time_window_lb.add(G.num_new_nodes, IloRange());
    time_window_ub.add(G.num_new_nodes, IloRange());
    for (const auto& v: G.V_in_new)
    {
        for (const auto& a: G.delta_in_new[v])
        {
            expr += x[amap[a]];
        }
        if constexpr (Q==3)
            name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        time_window_lb[vmap[v]] = IloRange(env, -B[vmap[v]] + D.nodes[v[0]].start_tw + D.nodes[v[0]].tw_length * (1 - expr),0,name.str().c_str()); 
        model.add(time_window_lb[vmap[v]]);
        name.str("");
        expr.clear();

        
        if constexpr (Q==3)
            name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        time_window_ub[vmap[v]] = IloRange(env,B[vmap[v]],D.nodes[v[0]].end_tw, name.str().c_str());
        model.add(time_window_ub[vmap[v]]);
        name.str("");
    }
    for (const auto& v: G.V_out_new)
    {
        for (const auto& a: G.delta_in_new[v])
        {
            expr += x[amap[a]];
        }
        if constexpr (Q==3)
            name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        time_window_ub[vmap[v]] = IloRange(env,0,-B[vmap[v]] + D.nodes[v[0]-n].max_ride_time + D.nodes[v[0]-n].start_tw + D.nodes[v[0]-n].service_time + D.nodes[v[0]-n].tw_length * expr, IloInfinity, name.str().c_str());
        model.add(time_window_ub[vmap[v]]);
        name.str("");
        expr.clear();

        if constexpr (Q==3)
            name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        time_window_lb[vmap[v]] = IloRange(env,D.nodes[v[0]].start_tw,B[vmap[v]], IloInfinity, name.str().c_str());
        model.add(time_window_lb[vmap[v]]);
        name.str("");
    }

    // time constraints maximum ride time 
    // ride time constraints for dropped-off or denied users have been deleted completely
    // ride time constraints for all picked-up users have been deleted (all inactive pick-up nodes)/ modified (active pick-up node)
    // --> need to add constraints for new drop-off nodes
    // modify constraints for picked-up users (add new drop-off nodes)
    for (const auto& i: all_picked_up)
    {
        if (!G.V_i_new[n+i].empty())
        {
            max_ride_time[vinmap[active_node[i-1].first]].add(G.V_i_new[n+i].size(),IloRange());
            for (const auto& w: G.V_i_new[n+i])
            {
                if constexpr (Q==3)
                    name << "max_ride_time_(" << active_node[i-1].first[0] << "," << active_node[i-1].first[1] << "," << active_node[i-1].first[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                else
                    name << "max_ride_time_(" << active_node[i-1].first[0] << "," << active_node[i-1].first[1] << "," << active_node[i-1].first[2] << "," << active_node[i-1].first[3] << "," << active_node[i-1].first[4] << "," << active_node[i-1].first[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                max_ride_time[vinmap[active_node[i-1].first]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (active_node[i-1].second + D.nodes[i].service_time), D.nodes[i].max_ride_time, name.str().c_str());
                model.add(max_ride_time[vinmap[active_node[i-1].first]][vec_map[i-1][w]]);
                name.str("");
            }
        }
    }

    // modify constraints for seekers (add new drop-off nodes)
    for (const auto& i: all_seekers)
    {
        for (const auto& v: G.V_i[i])
        {
            if (!G.V_i_new[n+i].empty())
            {
                max_ride_time[vinmap[v]].add(G.V_i_new[n+i].size(),IloRange());
                for (const auto& w: G.V_i_new[n+i])
                {
                    if constexpr (Q==3)
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                    else
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                    max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time), D.nodes[i].max_ride_time, name.str().c_str());
                    model.add(max_ride_time[vinmap[v]][vec_map[i-1][w]]);
                    name.str("");
                }
            }
        }
    }


    // add new space to array for new pick-up nodes (this has to be done first because of vinmap)
    for (const auto& i: all_seekers)
    {
        if (!G.V_i_new[i].empty())
        {
            max_ride_time.add(G.V_i_new[i].size(),IloRangeArray());
        }
    }
    // add space for new requests' pick-up nodes
    for (const auto& i : new_requests)
    {
        max_ride_time.add(G.V_i_new[i].size(), IloRangeArray()); 
    }
    // for each of the new pick-up nodes, create the constraints
    for (const auto& i: all_seekers)
    {
        // new pick-up nodes
        for (const auto& v: G.V_i_new[i])
        {
            // add IloRangeArray with size equal to number of drop-off nodes (old and new and already deleted)
            // only for still existing drop-off nodes an actual constraint will be added 
            // --> therefore constraints have to be added to model one on one, otherwise empty constraints are added which results in Segmentaion Fault
            max_ride_time[vinmap[v]] = IloRangeArray(env,vec_map[i-1].size());
            
            // old drop-off nodes
            for (const auto& w: G.V_i[n+i])
            {
                if constexpr (Q==3)
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                    else
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time),D.nodes[i].max_ride_time, name.str().c_str());
                model.add(max_ride_time[vinmap[v]][vec_map[i-1][w]]);
                name.str("");
            }
            
            // new drop-off nodes
            if (!G.V_i_new[n+i].empty())
            {
                for (const auto& w: G.V_i_new[n+i])
                {
                    if constexpr (Q==3)
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                    else
                        name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                    max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time), D.nodes[i].max_ride_time, name.str().c_str());
                    model.add(max_ride_time[vinmap[v]][vec_map[i-1][w]]);
                    name.str("");
                }
            }
        }
    }

    // for each of the new requests' pick-up nodes, create the constraints
    for (const auto& i : new_requests)
    {
        for (const auto& v: G.V_i_new[i])
        {
            max_ride_time[vinmap[v]] = IloRangeArray(env,G.V_i_new[n+i].size());
            
            for (const auto& w: G.V_i_new[n+i])
            {
                if constexpr (Q==3)
                    name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                else
                    name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time),D.nodes[i].max_ride_time, name.str().c_str());
                name.str("");
            }
            model.add(max_ride_time[vinmap[v]]);
        }
    }
    

    // add excess ride time constraints 
    // - for new requests
    // - for new drop-off nodes of seekers and picked-up users
    // add space for for new drop-off nodes
    excess_ride_time.add(G.V_out_new.size(), IloRange()); 
    if (consider_excess_ride_time)
    {
        for (const auto& v: G.V_out_new)
        {
            if constexpr (Q==3)
                name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            excess_ride_time[voutmap[v]] = IloRange(env, -IloInfinity, B[vmap[v]]-d[rmap[v[0]-n]], D.nodes[v[0]].start_tw,name.str().c_str()); 
            name.str("");
            model.add(excess_ride_time[voutmap[v]]);
        }
    }
    expr.end();
}


template<int Q>
void RollingHorizon<Q>::first_milp(bool accept_all, bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, IloEnv& env, IloModel& model, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d, IloNumVar& d_max, IloRangeArray& accept, IloRangeArray& serve_accepted, IloRangeArray& time_window_ub, IloRangeArray& time_window_lb, IloArray<IloRangeArray>& max_ride_time, IloRangeArray& travel_time, IloRangeArray& flow_preservation, IloRangeArray& excess_ride_time, IloRangeArray& fixed_B, IloRangeArray& fixed_x, IloRangeArray& pickup_delay, IloRange& num_tours, IloObjective& obj, IloExpr& obj1, IloExpr& obj2, IloExpr& obj3, const std::array<double,3>& w)
{
    // We use this stringstream to create variable and constraint names
    std::stringstream name;

    model = IloModel(env);

        
    // Variables
    // array for variables B_v
    B = IloNumVarArray(env, G.vcardinality);
    if constexpr (Q==3)
        B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0)");
    else
        B[vmap[G.depot]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, "B_(0,0,0,0,0,0)");
    for (const auto& v: G.V_in)
    {
        if constexpr (Q==3)
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str(""); // Clean name
    }
    for (const auto& v: G.V_out)
    {
        if constexpr (Q==3)
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        B[vmap[v]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str(""); // Clean name
    }
    // array for variables x_a
    x = IloNumVarArray(env, G.acardinality);
    for (const auto& a: G.A)
    {
        if constexpr (Q==3)
            name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "x_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "), (" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        x[amap[a]] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str());
        name.str(""); // Clean name
    }
    // array for variables p_i
    p = IloNumVarArray(env, D.rcardinality);
    for (const auto& i: D.R)
    {
        name << "p_" << i;
        p[rmap[i]] = IloNumVar(env, 0, 1, ILOBOOL, name.str().c_str()); 
        name.str(""); // Clean name
    }
    // array for variables d_i
    d = IloNumVarArray(env, D.rcardinality);
    for (const auto& i: D.R)
    {
        name << "d_" << i;
        d[rmap[i]] = IloNumVar(env, 0, IloInfinity, ILOFLOAT, name.str().c_str());
        name.str(""); // Clean name
    }
    d_max = IloNumVar(env,0,IloInfinity, ILOFLOAT, name.str().c_str());

    
    // to save solution values
    B_val = IloNumArray(env, G.vcardinality);
    p_val = IloIntArray(env, D.rcardinality);
    x_val = IloIntArray(env, G.acardinality);
    d_val = IloNumArray(env, D.rcardinality);
    
    
    
    // Constraints
    accept = IloRangeArray(env, D.rcardinality);
    flow_preservation = IloRangeArray(env, G.vcardinality);
    serve_accepted = IloRangeArray(env, D.rcardinality);
    travel_time = IloRangeArray(env, G.acardinality);
    time_window_ub = IloRangeArray(env, G.vcardinality);
    time_window_lb = IloRangeArray(env, G.vcardinality);
    max_ride_time = IloArray<IloRangeArray>(env, G.vincardinality);
    fixed_B = IloRangeArray(env, G.vcardinality);
    fixed_x = IloRangeArray(env, G.acardinality);
    excess_ride_time = IloRangeArray(env, G.voutcardinality);
    pickup_delay = IloRangeArray(env, G.vincardinality);
    
    
    
    if (accept_all)
    {
        for (const auto& i : D.R)
        {
            name << "accept_" << i;
            accept[rmap[i]] = IloRange(env,1,p[rmap[i]],1,name.str().c_str()); 
            name.str(""); // Clean name
        }   
        model.add(accept);
    }
        
    obj1 = IloExpr(env); // routing costs
    obj2 = IloExpr(env); // answered requests
    obj3 = IloExpr(env); // excess ride time
    
    
    IloExpr expr(env);

    // Create objective function
    obj1 += 0;
    for (const auto& a: G.A)
    {   
        obj1 += G.c[a] * x[amap[a]]; 
    }
    
    obj2 += 0;
    if (!accept_all)
    {
        obj2 += D.rcardinality; 
        for (const auto& i : D.R)
        {
            obj2 -= p[rmap[i]]; 
        }
    }
    
    obj3 += 0;
    if (consider_excess_ride_time)
    {
        for (const auto& i: D.R)
        {
            obj3 += d[rmap[i]];
        }
    }
    
    expr = w[0] * obj1 + w[1] * obj2 + w[2] * obj3; 
    obj = IloObjective(env, expr, IloObjective::Minimize);
    model.add(obj);
    expr.clear();

   
              
    // Constraints
    // 'flow preservation' constraints
    // depot
    for (const auto& a: G.delta_in[G.depot])
    {
        expr += x[amap[a]];
    }
    for(const auto& a: G.delta_out[G.depot])
    {
        expr -= x[amap[a]];
    }
    if constexpr (Q==3)
        flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0)");
    else
        flow_preservation[vmap[G.depot]] = IloRange(env,0,expr,0,"flow_preservation_(0,0,0,0,0,0)");
    expr.clear();

    // V_in
    for (const auto& v: G.V_in)
    {
        if constexpr (Q==3)
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";

        for(const auto& a: G.delta_in[v])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out[v])
        {
            expr -= x[amap[a]];
        }
        flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
        expr.clear();
        name.str("");
    }
    // V_out
    for (const auto& v: G.V_out)
    {
        if constexpr (Q==3)
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << ")";
        else
            name << "flow_preservation_B_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
        for(const auto& a: G.delta_in[v])
        {
            expr += x[amap[a]];
        }
        for(const auto& a: G.delta_out[v])
        {
            expr -= x[amap[a]];
        }
        flow_preservation[vmap[v]] = IloRange(env,0,expr,0,name.str().c_str());
        expr.clear();
        name.str("");
    }
    model.add(flow_preservation);
    
    // every request accepted has to be served
    for (const auto& i : D.R)
    {
        name << "serve_accepted_" << i;
        for (const auto& v: G.V_i[i])
        {
            for (const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
        }
        expr -= p[rmap[i]];
        serve_accepted[rmap[i]] = IloRange(env,0,expr,0,name.str().c_str()); 
        expr.clear();
        name.str("");
    }
    model.add(serve_accepted);
    

    // number of vehicles
    for (const auto& a: G.delta_out[G.depot])
    {
        expr += x[amap[a]];
    }
    num_tours = IloRange(env, expr, D.num_vehicles, "number_tours");
    model.add(num_tours);
    expr.clear();
    
    // travel time arc a 
    for (const auto& a: G.A)
    {   
        if constexpr (Q==3)
            name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << ")";
        else
            name << "travel_time_(" << a[0][0] << "," << a[0][1] << "," << a[0][2] << "," << a[0][3] << "," << a[0][4] << "," << a[0][5] << "),(" << a[1][0] << "," << a[1][1] << "," << a[1][2] << "," << a[1][3] << "," << a[1][4] << "," << a[1][5] << ")";
        if (a[0] != G.depot)
        {
            expr = -B[vmap[a[1]]] + B[vmap[a[0]]] + D.nodes[a[0][0]].service_time + G.t[a] - (D.nodes[a[0][0]].end_tw - D.nodes[a[1][0]].start_tw + G.t[a] + D.nodes[a[0][0]].service_time) * (1 - x[amap[a]]);
            travel_time[amap[a]] = IloRange(env,expr,0,name.str().c_str());
        }
        else
        {
            expr = -B[vmap[a[1]]] + G.t[a] * x[amap[a]];
            travel_time[amap[a]] = IloRange(env,expr,-time_passed,name.str().c_str());
        }
        expr.clear();
        name.str("");
    }
    model.add(travel_time);
    
    // time constraints pick-up and drop-off
    
    // return to depot of last vehicle
    if constexpr (Q==3)
        time_window_ub[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0)");
    else
        time_window_ub[vmap[G.depot]] = IloRange(env,B[vmap[G.depot]],D.max_route_duration,"time_window_ub_(0,0,0,0,0,0)");

    // inbound: tw_length = l_i - e_i 
    // outbound: tw_length = l_n+i - e_n+i 
    for (const auto& i : D.R)
    {
        for (const auto& v: G.V_i[i])
        {
            for (const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            if constexpr (Q==3)
                name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            time_window_lb[vmap[v]] = IloRange(env,-B[vmap[v]] + D.nodes[i].start_tw + D.nodes[i].tw_length * (1 - expr),0,name.str().c_str()); 
            expr.clear();
            name.str("");

            if constexpr (Q==3)
                name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            time_window_ub[vmap[v]] = IloRange(env,B[vmap[v]],D.nodes[i].end_tw,name.str().c_str());
            name.str("");
        }
        for (const auto& v: G.V_i[n+i])
        {
            for (const auto& a: G.delta_in[v])
            {
                expr += x[amap[a]];
            }
            if constexpr (Q==3)
                name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "time_window_ub_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            time_window_ub[vmap[v]] = IloRange(env, 0, -B[vmap[v]] + D.nodes[i].max_ride_time + D.nodes[i].start_tw + D.nodes[i].service_time + D.nodes[i].tw_length * expr, IloInfinity, name.str().c_str());
            expr.clear();
            name.str("");

            if constexpr (Q==3)
                name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "time_window_lb_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            time_window_lb[vmap[v]] = IloRange(env,D.nodes[n+i].start_tw,B[vmap[v]], IloInfinity, name.str().c_str());
            name.str("");
        }      
    }
    model.add(time_window_ub);
    model.add(time_window_lb);

    // maximum ride time
    for (const auto& i: D.R)
    {
        for (const auto& v: G.V_i[i])
        {
            max_ride_time[vinmap[v]] = IloRangeArray(env,G.V_i[n+i].size());
            for (const auto& w: G.V_i[n+i])
            {
                if constexpr (Q==3)
                    name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << ")";
                else
                    name << "max_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ") -- ("<< w[0] << "," << w[1] << "," << w[2] << "," << w[3] << "," << w[4] << "," << w[5] << ")";
                max_ride_time[vinmap[v]][vec_map[i-1][w]] = IloRange(env,B[vmap[w]] - (B[vmap[v]] + D.nodes[i].service_time),D.nodes[i].max_ride_time,name.str().c_str());
                name.str("");
            }
            model.add(max_ride_time[vinmap[v]]);
        }
    }
    

    // excess ride time constraints
    if (consider_excess_ride_time)
    {
        for (const auto& v: G.V_out)
        {
            if constexpr (Q==3)
                name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << ")";
            else
                name << "excess_ride_time_(" << v[0] << "," << v[1] << "," << v[2] << "," << v[3] << "," << v[4] << "," << v[5] << ")";
            excess_ride_time[voutmap[v]] = IloRange(env, -IloInfinity, B[vmap[v]]-d[rmap[v[0]-n]], D.nodes[v[0]].start_tw,name.str().c_str()); 
            name.str("");
        }
        model.add(excess_ride_time);
    }

    // Free the memory used by expr
    expr.end(); 
}



template<int Q>
void RollingHorizon<Q>::print_routes(DARP& D, DARPGraph<Q>& G, IloNumArray& B_val, IloIntArray& x_val)
{
    // output solution we have so far

    bool flag;
    int route_count = 0;    
    int current;

    std::vector<ARC> cycle_arcs;
    std::vector<ARC> cycle;
    ARC b = {};
    
    for (const auto& a: G.A)
    {
        if (x_val[amap[a]] > 0.9)
        {
            cycle_arcs.push_back(a);
        }
    }

    while (!cycle_arcs.empty())
    {
        std::cout << std::endl;
        flag = false;
        for (const auto& a: cycle_arcs)
        {
            if (a[0] == G.depot)
            {
                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "Start";
                b = a;
                cycle_arcs.erase(std::remove(cycle_arcs.begin(), cycle_arcs.end(), a), cycle_arcs.end());
                
                current = b[1][0];
                D.route[route_count].start = current;
                D.route[route_count].has_customers = true;
                D.routed[current] = true;
                D.route_num[current] = route_count;
                if (route_count == 0)
                {
                    D.next_array[DARPH_DEPOT] = -current;
                    D.pred_array[current] = DARPH_DEPOT;
                }
                else
                {
                    D.next_array[D.route[route_count-1].end] = -current;
                    D.pred_array[current] = -D.route[route_count-1].end;
                }
                // as long as we haven't found the last arc in the dicycle, look for the next arc in the dicycle and
                // add it to the tour
                while (b[1] != G.depot)
                {
                    for (const auto& f: cycle_arcs)
                    {
                        if (f[0] == b[1])
                        {
                            cycle.push_back(f);
                            if (f[0][0] > n)
                            {
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "out:";
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << f[0][0] - n;
                            }
                            else
                            {
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "in:";
                                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << f[0][0];
                            }
                            
                            if (f[1] != G.depot)
                            {
                                D.next_array[current] = f[1][0];
                                D.pred_array[f[1][0]] = current;
                                current = f[1][0];
                                D.routed[current] = true;
                                D.route_num[current] = route_count;
                            }
                    
                            for (int i = 0; i<2; ++i)
                            {
                                for (int j= 0; j<D.veh_capacity; ++j)
                                {
                                    b[i][j] = f[i][j];
                                }
                            }
                            cycle_arcs.erase(std::remove(cycle_arcs.begin(), cycle_arcs.end(), f), cycle_arcs.end());
                            break; 
                        }
                    }
                }

                D.route[route_count].end = current;
                D.next_array[current] = DARPH_DEPOT;

                std::cout << std::endl;
                std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << time_passed;
                for (const auto& a: cycle)
                {
                    std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << "Time:";
                    std::cout << std::left << setw(STRINGWIDTH) << setfill(' ') << B_val[vmap[a[0]]];
                    D.nodes[a[0][0]].beginning_service = B_val[vmap[a[0]]];
                }
                std::cout << std::endl;
                cycle.clear();
                flag = true;
                route_count++;
            }
            if (flag)
                break;
        }
    }
    D.pred_array[DARPH_DEPOT] = -D.route[route_count-1].end;
    std::cout << std::endl;
    cycle_arcs.clear();
}


template<int Q>
void RollingHorizon<Q>::update_graph_sets(bool consider_excess_ride_time, DARPGraph<Q>& G, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val)
{   
    // update sets
    dropped_off.clear(); 
    picked_up.clear(); 
    denied.clear();
    seekers.clear();
    fixed_edges.clear();

    for (const auto& e: G.V_i_new)
    {
        G.V_i[e.first].insert(G.V_i[e.first].end(), e.second.begin(), e.second.end());
    }
    G.V_i_new.clear();

    for (const auto& e: G.delta_in_new)
    {
        G.delta_in[e.first].insert(G.delta_in[e.first].end(), e.second.begin(), e.second.end());
    }
    for (const auto& e: G.delta_out_new)
    {
        G.delta_out[e.first].insert(G.delta_out[e.first].end(), e.second.begin(), e.second.end());
    }
    G.delta_in_new.clear();
    G.delta_out_new.clear();

    G.V_in.insert(G.V_in.end(), G.V_in_new.begin(), G.V_in_new.end());
    G.V_out.insert(G.V_out.end(), G.V_out_new.begin(), G.V_out_new.end());
    G.V_in_new.clear();
    G.V_out_new.clear();
    B_val.add(G.num_new_nodes, IloNum());
    G.num_new_nodes = 0;
    
    G.A.insert(G.A.end(), G.A_new.begin(), G.A_new.end());
    G.A_new.clear();
    x_val.add(G.num_new_arcs, IloInt());
    G.num_new_arcs = 0;

    p_val.add(new_requests.size(), IloInt());
    if (consider_excess_ride_time)
        d_val.add(new_requests.size(), IloNum());
}


template<int Q>
void RollingHorizon<Q>::get_solution_values(bool consider_excess_ride_time, DARP& D, DARPGraph<Q>& G, IloCplex& cplex, IloNumArray& B_val, IloNumArray& d_val, IloIntArray& p_val, IloIntArray& x_val, IloNumVarArray& B, IloNumVarArray& x, IloNumVarArray& p, IloNumVarArray& d)
{
  
    // If CPLEX successfully solved the model, print the results
    std::cout << "\nCplex success!\n";
    std::cout << "\tStatus: " << cplex.getStatus() << "\n";
    std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";    
    std::cout << "\tRelative MIP Gap: " << cplex.getMIPRelativeGap() << "\n";
    std::cout << "\tModel MILP " << dur_model.count() << "s" << std::endl;
    std::cout << "\tModel + Solve MILP " << dur_solve.count() << "s" << std::endl;  
    
    // get solution values
    B_val[vmap[G.depot]] = cplex.getValue(B[vmap[G.depot]]);
    for (const auto& v: G.V_in)
    {
        B_val[vmap[v]] = cplex.getValue(B[vmap[v]]);
    }
    for (const auto& v: G.V_out)
    {
        B_val[vmap[v]] = cplex.getValue(B[vmap[v]]);
    }
    for (const auto& a: G.A)
    {
        if (cplex.getValue(x[amap[a]]) > 0.9)
        {
            x_val[amap[a]] = 1;
        }
        else 
        {
            x_val[amap[a]] = 0;
        }
    }
    for (const auto& i : D.R)
    {
        if(std::find(all_picked_up.begin(), all_picked_up.end(), i) == all_picked_up.end())
        {
            if (cplex.getValue(p[rmap[i]]) > 0.9)
            {
                p_val[rmap[i]] = 1;
            }
            else
            {
                p_val[rmap[i]] = 0;
            }
        }  
        if (consider_excess_ride_time)
        {
            d_val[rmap[i]] = cplex.getValue(d[rmap[i]]);
        }
    }

    print_routes(D, G, B_val, x_val);

    
    // important: execute function print_routes first to assgin value to D.nodes[i].beginning_service
    if (num_milps > 1)
    {
        for (const auto& i: new_requests)
        {
            communicated_pickup[i-1] = D.nodes[i].beginning_service;
#if VERBOSE
            std::cout << "communicated pick-up request " << i << ": " << communicated_pickup[i-1] << std::endl;
#endif
        
            time_to_answer[i-1] = dur_solve.count();
            if (cplex.getValue(p[rmap[i]]) < 0.9 && time_to_answer[i-1] > notify_requests_sec - 0.001)
            {
                denied_timeout += 1;
            } 
        }

    }
    else
    {
        for (const auto& i: D.R)
        {
            if (p_val[rmap[i]] > 0.9)
            {
                communicated_pickup[i-1] = D.nodes[i].beginning_service;
#if VERBOSE
                std::cout << "communicated pick-up request " << i << ": " << communicated_pickup[i-1] << std::endl;
#endif
            }
                
            time_to_answer[i-1] = dur_solve.count();
        }  
    }
    
    total_time_model += dur_model.count();
    total_time_model_solve += dur_solve.count();  
}



template class RollingHorizon<3>;
template class RollingHorizon<6>;