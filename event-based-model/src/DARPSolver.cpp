#include "DARPH.h"


DARPSolver::DARPSolver(int num_requests) : n{num_requests} {
    time_to_answer = new double[n];

    // Allocate memory for feasibility matrix
    f = new int**[n+1];
    for (int i = 0; i < n+1; i++)
    {
        // Allocate memory blocks for rows of each 2D array
        f[i] = new int*[n+1];
        for (int j = 0; j < n+1; j++) 
        {
            // Allocate memory blocks for columns of each 2D array
            f[i][j] = new int[2];
            f[i][j][0] = 1; // default = feasible --> when no heuristic is used
            f[i][j][1] = 1;
        }
    }

    incremental_costs = new double**[n+1];
    for (int i = 0; i < n+1; i++)
    {
        // Allocate memory blocks for rows of each 2D array
        incremental_costs[i] = new double*[n+1];
        for (int j = 0; j < n+1; j++) 
        {
            // Allocate memory blocks for columns of each 2D array
            incremental_costs[i][j] = new double[2];
            incremental_costs[i][j][0] = DARPH_INFINITY;
            incremental_costs[i][j][1] = DARPH_INFINITY;
        }
    }
}


DARPSolver::~DARPSolver() {
    delete[] time_to_answer;

     // deallocate memory f
    for (int i=0; i<n; ++i)
    {
        for (int j=0; j<n; ++j)
        {
            delete[] f[i][j];
        }
        delete[] f[i];
    }
    delete[] f;

    // deallocate memory incremental_costs
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            delete[] incremental_costs[i][j];
        }
        delete[] incremental_costs[i];
    }
    delete[] incremental_costs;
}



void DARPSolver::check_paths(DARP& D)
{
    ///
    /// check paths only for first requests (in dynamic problem |D.R|=1)
    ///
    DARPRoute path;
    
    // first check if problem instance is infeasible due to e_0 + t_0i > l_i
    for (const auto& i: D.R)
    {
        if (D.nodes[DARPH_DEPOT].start_tw + D.tt[DARPH_DEPOT][i] > D.nodes[i].end_tw)
        {
            fprintf(stderr, "Warning: request %d is infeasible because upper bound of pick-up time window [%f, %f] is to early to reach pick-up from depot.", i, D.nodes[i].start_tw, D.nodes[i].end_tw);
            report_error("%s: Infeasible pick-up time window detected.\n", __FUNCTION__);
        }
    }
    // Initialize feasibilty matrix for j = 0
    for (int i=1; i<=n; i++)
    {
        f[i][0][0] = 1;
        f[i][0][1] = 1;
        f[0][i][0] = 1;
        f[0][i][1] = 1;
    }
    
    for (const auto& i: D.R)
    {
        for (const auto& j: D.R)
        {
            if (j != i)
            {
                if (D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i] > D.nodes[i].end_tw || (D.nodes[i].demand + D.nodes[j].demand > D.veh_capacity) || (D.nodes[i].direction != D.nodes[j].direction))
                {
                    f[i][j][0] = 0;
                    f[i][j][1] = 0;
                }
                else if ((D.nodes[i].direction == 0 && D.nodes[i].bus_station < D.nodes[j].bus_station) || (D.nodes[i].direction == 1 && D.nodes[i].bus_station > D.nodes[j].bus_station)) {
                    // infeasible due to line precedence
                    f[i][j][0] = 0;
                    f[i][j][1] = 0;
                }
                else if (D.nodes[n+j].bus_station == D.nodes[i].bus_station) {
                    // infeasible as we require passengers to alight before the next can enter
                    f[i][j][0] = 0;
                    f[i][j][1] = 0;
                }
                else
                {
                    // test path 0
                    // j --- i --- n+j --- n+i
                    path.start = j;
                    D.next_array[j] = i;
                    D.next_array[i] = n+j;
                    D.next_array[n+j] = n+i;
                    D.next_array[n+i] = -1; // mark the end of the path
                    path.end = n+i;

                    if (eight_step(D, path)) {
                        // check line precedence
                        if (D.nodes[j].direction == 0) {
                            if ((D.nodes[j].bus_station <= D.nodes[i].bus_station) && (D.nodes[n + j].bus_station <= D.nodes[n + i].bus_station)) {
                                f[i][j][0] = 1;
                            }
                            else {
                                f[i][j][0] = 0;
                            }
                        }
                        else {
                            if ((D.nodes[j].bus_station >= D.nodes[i].bus_station) && (D.nodes[n + j].bus_station >= D.nodes[n + i].bus_station)) {
                                f[i][j][0] = 1;
                            }
                            else {
                                f[i][j][0] = 0;
                            }
                        }
                    }
                    else {
                        f[i][j][0] = 0;

                    }
                    // test path 1
                    // j --- i --- n+i --- n+j
                    path.start = j;
                    D.next_array[j] = i;
                    D.next_array[i] = n+i;
                    D.next_array[n+i] = n+j;
                    D.next_array[n+j] = -1;
                    path.end = n+j;

                    if (eight_step(D, path)) {
                        // check line precedence
                        if (D.nodes[j].direction == 0) {
                            if ((D.nodes[j].bus_station <= D.nodes[i].bus_station) && (D.nodes[n + i].bus_station <= D.nodes[n + j].bus_station)) {
                                f[i][j][1] = 1;
                            }
                            else {
                                f[i][j][1] = 0;
                            }
                        }
                        else {
                            if ((D.nodes[j].bus_station >= D.nodes[i].bus_station) && (D.nodes[n + i].bus_station >= D.nodes[n + j].bus_station)) {
                                f[i][j][1] = 1;
                            }
                            else {
                                f[i][j][1] = 0;
                            }
                        }
                    }
                    else {
                        f[i][j][1] = 0;
                    }
                }
            }
        }
    }
}


void DARPSolver::check_new_paths(DARP& D, double w1, double w2, double w3)
{
    DARPRoute path;
    for (const auto& i: all_seekers)
    {
        for (const auto& j: new_requests)
        {
            if (D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i] > D.nodes[i].end_tw || (D.nodes[i].demand + D.nodes[j].demand > D.veh_capacity))
            {
                f[i][j][0] = 0;
                f[i][j][1] = 0;
            }
            else 
            {
                // test path 0
                // j --- i --- n+j --- n+i
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+j;
                D.next_array[n+j] = n+i;
                D.next_array[n+i] = -1; // mark the end of the path
                path.end = n+i;

                if (eight_step(D, path))
                {
                    f[i][j][0] = 1;
                    incremental_costs[i][j][0] = w1 * (D.d[j][i] + D.d[i][n+j] + D.d[n+j][n+i]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+j]) - D.nodes[n+j].start_tw + D.nodes[n+j].service_time + D.tt[n+j][n+i] - D.nodes[n+i].start_tw);
                }
                else
                    f[i][j][0] = 0;
                
                // test path 1
                // j --- i --- n+i --- n+j
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+i;
                D.next_array[n+i] = n+j;
                D.next_array[n+j] = -1;
                path.end = n+j;

                if (eight_step(D, path))
                {
                    f[i][j][1] = 1;
                    incremental_costs[i][j][1] = w1 * (D.d[j][i] + D.d[i][n+i] + D.d[n+i][n+j]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+i]) - D.nodes[n+i].start_tw + D.nodes[n+i].service_time + D.tt[n+i][n+j] - D.nodes[n+j].start_tw);
                }
                else
                    f[i][j][1] = 0;  
            }
        }   
    }
    for (const auto& i: all_picked_up)
    {
        for (const auto& j: new_requests)
        {
            f[i][j][0] = 0;
            f[i][j][1] = 0;
        }
    }

    for (const auto & i: new_requests)
    {
        for (const auto& j: all_seekers)
        {
            if (D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i] > D.nodes[i].end_tw || (D.nodes[i].demand + D.nodes[j].demand > D.veh_capacity))
            {
                f[i][j][0] = 0;
                f[i][j][1] = 0;
            }
            else
            {
                // test path 0
                // j --- i --- n+j --- n+i
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+j;
                D.next_array[n+j] = n+i;
                D.next_array[n+i] = -1; // mark the end of the path
                path.end = n+i;
                
                if (eight_step(D, path))
                {
                    f[i][j][0] = 1;
                    incremental_costs[i][j][0] = w1 * (D.d[j][i] + D.d[i][n+j] + D.d[n+j][n+i]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+j]) - D.nodes[n+j].start_tw + D.nodes[n+j].service_time + D.tt[n+j][n+i] - D.nodes[n+i].start_tw);
                }    
                else
                    f[i][j][0] = 0;
                
                // test path 1
                // j --- i --- n+i --- n+j
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+i;
                D.next_array[n+i] = n+j;
                D.next_array[n+j] = -1;
                path.end = n+j;

                if (eight_step(D, path))
                {
                    f[i][j][1] = 1;
                    incremental_costs[i][j][1] = w1 * (D.d[j][i] + D.d[i][n+i] + D.d[n+i][n+j]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+i]) - D.nodes[n+i].start_tw + D.nodes[n+i].service_time + D.tt[n+i][n+j] - D.nodes[n+j].start_tw);
                }    
                else
                    f[i][j][1] = 0;  
            }  
        }
    }

    for (const auto & i : new_requests)
    {    
        for (const auto& j: all_picked_up)
        {
            if (D.nodes[j].beginning_service + D.nodes[j].service_time + D.tt[j][i] > D.nodes[i].end_tw || (D.nodes[i].demand + D.nodes[j].demand > D.veh_capacity))
            {
                f[i][j][0] = 0;
                f[i][j][1] = 0;
            }
            else
            {
                // test path 0
                // j --- i --- n+j --- n+i
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+j;
                D.next_array[n+j] = n+i;
                D.next_array[n+i] = -1; // mark the end of the path
                path.end = n+i;
                
                if (eight_step(D, path,j))
                {
                    f[i][j][0] = 1;
                    incremental_costs[i][j][0] = w1 * (D.d[j][i] + D.d[i][n+j] + D.d[n+j][n+i]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].beginning_service + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+j]) - D.nodes[n+j].start_tw + D.nodes[n+j].service_time + D.tt[n+j][n+i] - D.nodes[n+i].start_tw);
                }
                else
                    f[i][j][0] = 0;
                
                // test path 1
                // j --- i --- n+i --- n+j
                path.start = j;
                D.next_array[j] = i;
                D.next_array[i] = n+i;
                D.next_array[n+i] = n+j;
                D.next_array[n+j] = -1;
                path.end = n+j;

                if (eight_step(D, path,j))
                {
                    f[i][j][1] = 1;
                    incremental_costs[i][j][1] = w1 * (D.d[j][i] + D.d[i][n+i] + D.d[n+i][n+j]) + w3 * (2 * (DARPH_MAX(D.nodes[i].start_tw, D.nodes[j].start_tw + D.nodes[j].service_time + D.tt[j][i]) + D.nodes[i].service_time + D.tt[i][n+i]) - D.nodes[n+i].start_tw + D.nodes[n+i].service_time + D.tt[n+i][n+j] - D.nodes[n+j].start_tw);
                }    
                else
                    f[i][j][1] = 0;  
            }
        }
    }
}


bool DARPSolver:: eight_step(DARP& D, DARPRoute& path)
{
    ///
    /// MODIFIED TO CHECK feasibility of user pairs i, j
    /// evaluation of route veh using the 8-step evaluation scheme 
    /// by Cordeau and Laporte (2003)
    ///

    
    // Step 1 
    path.departure_depot = D.nodes[DARPH_DEPOT].start_tw;
      
    // Step 2 - compute A_i, W_i, B_i and D_i for each vertex v_i in the route
    // if infeasibility is detected during the computation of an "earliest possible"-schedule, return false
    if (!update_vertices(D, path))
        return false;
    
        
    // Step 3 - compute Forward Time Slack at depot
    double fts = DARPH_INFINITY;
    double waiting = 0;
    double temp;
    int c = path.start;
    
    while (c > 0)
    {
        if (c <= D.num_requests && D.nodes[c].ride_time > DARPH_EPSILON)
            report_error("Ride time > 0 assigned to nodes[%d].ride_time", c);
        waiting += D.nodes[c].waiting_time;
        temp =  waiting + DARPH_PLUS(D.nodes[c].end_tw - D.nodes[c].beginning_service);
        if (temp < fts)
            fts = temp;  
        c = D.next_array[c];
    }
    // we have to account for time window and maximum duration of route in fts
    // no waiting time at depot --> we do not increase waiting 
    temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[DARPH_DEPOT].end_tw - path.return_depot, 
            D.max_route_duration - (path.return_depot - path.departure_depot)));
    if (temp < fts)
        fts = temp;

    // Step 4
    path.departure_depot = D.nodes[DARPH_DEPOT].start_tw + DARPH_MIN(fts, waiting);

    // Step 5 - update A_i, W_i, B_i and D_i for each vertex v_i in the route
    update_vertices(D, path);

    // Step 6 - compute L_i for each request assigned to the route
    // this is done in update_vertices already
    
    // Step 7 
    // reset c 
    c = path.start;
    while (c > 0)
    {
        if (c <= D.num_requests)
        {
            
            // Step 7a) - compute F_c
            double fts = DARPH_INFINITY;
            double ride_time_slack;
            double waiting = 0;
            double temp = DARPH_PLUS(D.nodes[c].end_tw - D.nodes[c].beginning_service);
            fts = temp;
            int i = D.next_array[c];
            while (i > 0)
            {   
                waiting += D.nodes[i].waiting_time;

                ride_time_slack = DARPH_INFINITY;
                if (i>=n+1)
                {
                    // computation of ride time slack only if vertex i-n is before c in the route
                    int pred = D.pred_array[c];
                    while (pred > 0 && pred < D.num_nodes+1)
                    {
                        if (pred == i-n)
                        {
                            ride_time_slack = D.nodes[i].max_ride_time - D.nodes[i].ride_time;
                            break;
                        }
                        else
                        {
                            pred = D.pred_array[pred];
                        }
                    }
                }

                temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[i].end_tw - D.nodes[i].beginning_service, ride_time_slack));
                if (temp < fts)
                    fts = temp;
                i = D.next_array[i];
            }
            // we have to account for time window and maximum duration of route
            // no waiting time at depot --> we do not increase waiting 
            temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[DARPH_DEPOT].end_tw - path.return_depot, 
                D.max_route_duration - (path.return_depot - path.departure_depot)));
            if (temp < fts)
                fts = temp;
            
            // Step 7b) - Set B_c = B_c + min(F_c, sum W_p); D_j = B_j + d_j
            D.nodes[c].beginning_service += DARPH_MIN(fts, waiting);
            D.nodes[c].departure_time = D.nodes[c].beginning_service + D.nodes[c].service_time;
            D.nodes[c].waiting_time = DARPH_MAX(0,D.nodes[c].beginning_service - D.nodes[c].arrival_time);

            // Step 7c) - update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_c in the route
            update_vertices(D, path, c);

            // Step 7d) - update ride time L_i for each request whose destination vertex is after v_c
            // already done in update_vertices(veh, c)

        }

        c = D.next_array[c];
    }

    // Step 8 - compute violation route duration, time window and ride time constraints
    
    // duration violation
    if ((path.return_depot - path.departure_depot) > D.max_route_duration)
    {
        return false;    
    }
    
    // ride time violation and time window violation
    c = path.start;
    while(c>0)
    {
        if (c > D.num_requests && D.nodes[c].ride_time > D.nodes[c].max_ride_time)
        {
            return false;
        }
        if (D.nodes[c].beginning_service -0.001 > D.nodes[c].end_tw || D.nodes[c].beginning_service +0.001 < D.nodes[c].start_tw)
        {
            int e = path.start;
            std::cerr << std::endl;
            std::cerr << "Path: ";
            while (e>0)
            {
                std::cerr << e << " ";
                e = D.next_array[e];
            }
            std::cerr << std::endl;
            fprintf(stderr, "Problem in eight_step() routine. Routine should have stopped already due to time window infeasibility.");
            fprintf(stderr, "Node %d: Beginning of service is %f. Time window is [%f, %f].", c, D.nodes[c].beginning_service, D.nodes[c].start_tw, D.nodes[c].end_tw);
            report_error("%s: Error in code of eight_step(int j) detected.\n", __FUNCTION__);
            return false;
        }

        c = D.next_array[c];
    }

    return true;
}   


bool DARPSolver::update_vertices(DARP& D, DARPRoute& path)
{
    ///
    /// Update A_i, W_i, B_i and D_i for each vertex v_i in the route 
    /// in 8-step routine
    ///

    int c = path.start; // c - current
    int s = D.next_array[c]; // s - successor
    D.nodes[c].arrival_time = path.departure_depot + D.tt[DARPH_DEPOT][c];
    D.nodes[c].beginning_service = DARPH_MAX(D.nodes[c].arrival_time, D.nodes[c].start_tw);
    D.nodes[c].waiting_time = DARPH_MAX(0,D.nodes[c].beginning_service - D.nodes[c].arrival_time);
    D.nodes[c].departure_time = D.nodes[c].beginning_service + D.nodes[c].service_time;
    // check for infeasibility of tws (this is the earliest possible schedule)
    if (D.nodes[c].beginning_service > D.nodes[c].end_tw + DARPH_EPSILON)
        return false; 

    // in the first iteration of the loop s should be positive because for each passenger 
    // there are at least two nodes (i and n+i) in the route 
    // when the end of the route is reached s is negative (new route) or zero (DARPH_DEPOT)
    while (s > 0)
    {
        D.nodes[s].arrival_time = D.nodes[c].departure_time + D.tt[c][s];
        D.nodes[s].beginning_service = DARPH_MAX(D.nodes[s].arrival_time, D.nodes[s].start_tw);
        D.nodes[s].waiting_time = DARPH_MAX(0,D.nodes[s].beginning_service - D.nodes[s].arrival_time);
        D.nodes[s].departure_time = D.nodes[s].beginning_service + D.nodes[s].service_time;

        // modification of routine to update ride times immedialely
        if (s >= D.num_requests + 1)
            D.nodes[s].ride_time = D.nodes[s].beginning_service - D.nodes[s - D.num_requests].departure_time;

        // check for infeasibility of tws (this is the earliest possible schedule)
        if (D.nodes[s].beginning_service > D.nodes[s].end_tw + DARPH_EPSILON)
            return false; 
        c = s; 
        s = D.next_array[c];
    } 
    path.return_depot = D.nodes[c].departure_time + D.tt[c][DARPH_DEPOT];
    return true;
}


bool DARPSolver::update_vertices(DARP& D, DARPRoute& path, int j)
{
    ///
    /// Update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_j in the route 
    ///
    int c = j;
    int s = D.next_array[c]; // s - successor
        
    // in the first iteration of the loop s should be positive because c is a pick-up node 
    // when the beginning of a new route is reached s is negative or zero (DARPH_DEPOT)
    while (s > 0)
    {
        D.nodes[s].arrival_time = D.nodes[c].departure_time + D.tt[c][s];
        D.nodes[s].beginning_service = DARPH_MAX(D.nodes[s].arrival_time, D.nodes[s].start_tw);
        D.nodes[s].waiting_time = DARPH_MAX(0,D.nodes[s].beginning_service - D.nodes[s].arrival_time);
        D.nodes[s].departure_time = D.nodes[s].beginning_service + D.nodes[s].service_time;

        // modification of routine to update ride times immediately
        if (s >= D.num_requests + 1)
            D.nodes[s].ride_time = D.nodes[s].beginning_service - D.nodes[s - D.num_requests].departure_time;

        
        c = s;
        s = D.next_array[c];
    } 
    path.return_depot = D.nodes[c].departure_time + D.tt[c][DARPH_DEPOT];
    return true;
}



bool DARPSolver::eight_step(DARP& D, DARPRoute& path, int j)
{
    ///
    /// MODIFIED TO CHECK feasibility of user pairs i, j going in from node j, i.e. the departure time at node j is fixed
    /// evaluation of route veh using the 8-step evaluation scheme 
    /// by Cordeau and Laporte (2003)
    ///

    // Step 1 
    D.nodes[j].departure_time = D.nodes[j].beginning_service + D.nodes[j].service_time;
      
    // Step 2 - compute A_i, W_i, B_i and D_i for each vertex v_i in the route
    // if infeasibility is detected during the computation of an "earliest possible"-schedule, return false
    ///
    /// Update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_j in the route 
    ///
    int c = j;
    int s = D.next_array[c]; // s - successor
        
    // in the first iteration of the loop s should be positive because c is a pick-up node 
    // when the beginning of a new route is reached s is negative or zero (DARPH_DEPOT)
    while (s > 0)
    {
        D.nodes[s].arrival_time = D.nodes[c].departure_time + D.tt[c][s];
        D.nodes[s].beginning_service = DARPH_MAX(D.nodes[s].arrival_time, D.nodes[s].start_tw);
        D.nodes[s].waiting_time = DARPH_MAX(0,D.nodes[s].beginning_service - D.nodes[s].arrival_time);
        D.nodes[s].departure_time = D.nodes[s].beginning_service + D.nodes[s].service_time;

        // modification of routine to update ride times immediately
        if (s >= D.num_requests + 1)
            D.nodes[s].ride_time = D.nodes[s].beginning_service - D.nodes[s - D.num_requests].departure_time;

        // check for infeasibility of tws (this is the earliest possible schedule)
        if (D.nodes[s].beginning_service > D.nodes[s].end_tw + DARPH_EPSILON)
            return false; 
        
        c = s;
        s = D.next_array[c];
    } 
    path.return_depot = D.nodes[c].departure_time + D.tt[c][DARPH_DEPOT];
        
    
    // Step 7 
    // reset c 
    c = D.next_array[j];
    while (c > 0)
    {
        if (c <= D.num_requests)
        {
            if (c <= D.num_requests && D.nodes[c].ride_time > DARPH_EPSILON)
                report_error("Ride time > 0 assigned to nodes[%d].ride_time", c);
            // Step 7a) - compute F_c
            double fts = DARPH_INFINITY;
            double ride_time_slack;
            double waiting = 0;
            double temp = DARPH_PLUS(D.nodes[c].end_tw - D.nodes[c].beginning_service);
            fts = temp;
            int i = D.next_array[c];
            while (i > 0)
            {   
                waiting += D.nodes[i].waiting_time;
                ride_time_slack = DARPH_INFINITY;
                if (i>=n+1)
                {
                    // computation of ride time slack only if vertex i-n is before c in the route
                    int pred = D.pred_array[c];
                    while (pred > 0)
                    {
                        if (pred == i-n)
                        {
                            ride_time_slack = D.nodes[i].max_ride_time - D.nodes[i].ride_time;
                            break;
                        }
                        else
                        {
                            pred = D.pred_array[pred];
                        }
                    }
                }

                temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[i].end_tw - D.nodes[i].beginning_service, ride_time_slack));
                if (temp < fts)
                    fts = temp;
                i = D.next_array[i];
            }
            // we have to account for time window and maximum duration of route
            // no waiting time at depot --> we do not increase waiting 
            temp = waiting + DARPH_PLUS(DARPH_MIN(D.nodes[DARPH_DEPOT].end_tw - path.return_depot, 
                D.max_route_duration - (path.return_depot - path.departure_depot)));
            if (temp < fts)
                fts = temp;
            
            // Step 7b) - Set B_c = B_c + min(F_c, sum W_p); D_j = B_j + d_j
            D.nodes[c].beginning_service += DARPH_MIN(fts, waiting);
            D.nodes[c].departure_time = D.nodes[c].beginning_service + D.nodes[c].service_time;
            D.nodes[c].waiting_time = DARPH_MAX(0,D.nodes[c].beginning_service - D.nodes[c].arrival_time);

            // Step 7c) - update A_i, W_i, B_i and D_i for each vertex v_i that comes after v_c in the route
            update_vertices(D, path, c);

            // Step 7d) - update ride time L_i for each request whose destination vertex is after v_c
            // already done in update_vertices(c)

        }

        c = D.next_array[c];
    }

    // Step 8 - compute violation route duration, time window and ride time constraints
    
    // duration violation (doesn't make sense because we don't know the beginning of the path)
    // if ((path.return_depot - path.departure_depot) > D.max_route_duration)
    // {
    //     return false;    
    // }
    
    // ride time violation and time window violation
    c = path.start;
    while(c>0)
    {
        if (c > D.num_requests && D.nodes[c].ride_time > D.nodes[c].max_ride_time)
        {
            return false;
        }
        if (D.nodes[c].beginning_service -0.001 > D.nodes[c].end_tw || D.nodes[c].beginning_service +0.001 < D.nodes[c].start_tw)
        {
            fprintf(stderr, "Problem in eight_step(int j) routine. Routine should have stopped already due to time window infeasibility.");
            fprintf(stderr, "Node %d: Beginning of service is %f. Time window is [%f, %f].", c, D.nodes[c].beginning_service, D.nodes[c].start_tw, D.nodes[c].end_tw);
            report_error("%s: Error in code of eight_step(int j) detected.\n", __FUNCTION__);
            return false;
        }

        c = D.next_array[c];
    }

    return true;
}   



void DARPSolver::find_min(double*** pointer, int i, std::vector<std::array<int,3>>& path_list) const
{
    int min_value = DARPH_INFINITY;
    std::array<int,3> min_index, index;   

    for (const auto& j: all_seekers)
    {
        index = {i,j,0};
        if (f[i][j][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {    
            if (pointer[i][j][0] < min_value)
            {
                min_value = pointer[i][j][0];
                min_index = {i,j,0};
            }
        }
        index = {i,j,1};
        if (f[i][j][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {
            if (pointer[i][j][1] < min_value)
            {
                min_value = pointer[i][j][1];
                min_index = {i,j,1};
            }
        }
        index = {j,i,0};
        if (f[j][i][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {
            if (pointer[j][i][0] < min_value)
            {
                min_value = pointer[j][i][0];
                min_index = {j,i,0};
            }
        }
        index = {j,i,1};
        if (f[j][i][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {
            if (pointer[j][i][1] < min_value)
            {
                min_value = pointer[j][i][1];
                min_index = {j,i,1};
            }
        }
    }
    for (const auto& j: all_picked_up)
    {
        index = {i,j,0};
        if (f[i][j][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {    
            if (pointer[i][j][0] < min_value)
            {
                min_value = pointer[i][j][0];
                min_index = {i,j,0};
            }
        }
        index = {i,j,1};
        if (f[i][j][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
        {
            if (pointer[i][j][1] < min_value)
            {
                min_value = pointer[i][j][1];
                min_index = {i,j,1};
            }
        }
    }
    path_list.push_back(min_index);
}


void DARPSolver::choose_paths(int min_feas_paths_allowed, double percentage_feas_paths_allowed)
{
    std::vector <std::array<int,3> > path_list;
    int num_feas_paths;
    int num_feas_paths_allowed;
    std::array<int,3> index;

    
    for (const auto& i: new_requests)
    {
        // count feasible paths
        num_feas_paths = 0;
        for (const auto& j: all_seekers)
        {
            num_feas_paths += f[i][j][0] + f[i][j][1] + f[j][i][0] + f[j][i][1];
        }
        for (const auto& j: all_picked_up)
        {
            num_feas_paths += f[i][j][0] + f[i][j][1];
        }

        num_feas_paths_allowed = DARPH_MAX(min_feas_paths_allowed, percentage_feas_paths_allowed * num_feas_paths);
        
        if (num_feas_paths > num_feas_paths_allowed)
        {    
            path_list.clear();
            for (int j = 1; j<=num_feas_paths_allowed; j++)
            {
                find_min(incremental_costs, i, path_list);
            }
#if VERBOSE
            std::cout << std::endl << "request " << i << std::endl;
            std::cout << "Number of feasible paths allowed: " << num_feas_paths_allowed << std::endl;
            for (const auto& e: path_list)
            {
                if (e[2] == 0)
                {
                    // [i][j][0] <=> j --- i --- n+j --- n+i
                    std::cout << e[0] << " " << e[1] << " " << e[2] << std::endl;
                    std::cout << e[1] << " --- " << e[0] << " --- " << "n+" << e[1] << " --- " << "n+" << e[0] << std::endl;
                }
                else
                {
                    // [i][j][1] <=> j --- i --- n+i --- n+j   
                    std::cout << e[0] << " " << e[1] << " " << e[2] << std::endl;
                    std::cout << e[1] << " --- " << e[0] << " --- " << "n+" << e[0] << " --- " << "n+" << e[1] << std::endl;
                }
                
            }
#endif
           
            // "remove" other paths
            for (const auto& j: all_seekers)
            {
                index = {i,j,0};
                if (f[i][j][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {    
                    f[i][j][0] = 0;
                }
                index = {i,j,1};
                if (f[i][j][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {
                    f[i][j][1] = 0;
                }
                index = {j,i,0};
                if (f[j][i][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {
                    f[j][i][0] = 0; 
                }
                index = {j,i,1};
                if (f[j][i][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {
                    f[j][i][1] = 0;
                }
            }
            for (const auto& j: all_picked_up)
            {
                index = {i,j,0};
                if (f[i][j][0] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {    
                    f[i][j][0] = 0;
                }
                index = {i,j,1};
                if (f[i][j][1] == 1 && std::find(path_list.begin(), path_list.end(), index) == path_list.end())
                {
                    f[i][j][1] = 0; 
                }
            }
        }
    }
}


bool DARPSolver::verify_routes(DARP& D, bool consider_excess_ride_time, const char *message)
{
    ///
    /// This debugging function manually calculates the objective function of the current
    /// solution and the route values, etc., and compares them with the claimed
    /// value.  Returns false if any inconsistencies are found and prints
    /// the message.  Returns true with no output otherwise.
    ///

    double len{0};
    double excess{0};
    int counted_routes{0};
    int next_node, current_node, current_route, route_start, current_start, current_end;
    
    
    for (int i=1; i<=n; i++)
    {
        D.routed[i] = false;
    }

    // First check the pred/next arrays for consistency
    current_node = DARPH_DEPOT;
    next_node = DARPH_ABS(D.next_array[current_node]);
    while(next_node != DARPH_DEPOT)
    {
        if(DARPH_ABS(D.pred_array[next_node])!= current_node)
        {
            fprintf(stderr,"%d->%d??\nNext: %d->%d\nPred:%d->%d",current_node,next_node,
                current_node,D.next_array[current_node],next_node,D.pred_array[next_node]);

            report_error("%s: Next/pred inconsistency\n",__FUNCTION__);
        }
        current_node = next_node;
        next_node = DARPH_ABS(D.next_array[current_node]);
    }

    // repeat for the node ahead of DARPH_DEPOT and DARPH_DEPOT
    if(DARPH_ABS(D.pred_array[next_node]) != current_node)
    {
        fprintf(stderr,"%d->%d??\nNext: %d->%d\nPred:%d->%d",current_node,next_node,
            current_node,D.next_array[current_node],next_node,D.pred_array[next_node]);

        report_error("%s: Next/pred inconsistency depot\n",__FUNCTION__);
    }


    // compute route length and compare with total_routing_costs
    route_start = -D.next_array[DARPH_DEPOT];
    if(route_start < 0)
    {
        fprintf(stderr,"next[DARPH_DEPOT] is incorrect\n");
        report_error(message,__FUNCTION__);

    }
    
    current_node = route_start;
    current_route = D.route_num[current_node];
    current_start = D.route[current_route].start;
    current_end = D.route[current_route].end;
    counted_routes++;

    if(route_start != current_start)
    {
        fprintf(stderr,"Error in initial route start:  %d != %d\n",route_start, current_start);
        report_error(message,__FUNCTION__);
    }
    

    
    D.routed[current_node] = true;

    if (D.nodes[current_node].beginning_service + 0.01 < D.nodes[current_node].start_tw || D.nodes[current_node].beginning_service - 0.01 > D.nodes[current_node].end_tw)
    {
        fprintf(stderr,"Error in time windows of node %d. Time window [%f,%f]. Beginning of service at node: %f!\n",current_node,D.nodes[current_node].start_tw,D.nodes[current_node].end_tw, D.nodes[current_node].beginning_service);
        report_error(message,__FUNCTION__);
    }

    D.route[current_route].departure_depot = D.nodes[current_node].beginning_service - D.tt[DARPH_DEPOT][current_node];
    if (D.route[current_route].departure_depot + 0.01 < D.nodes[DARPH_DEPOT].start_tw)
    {
        fprintf(stderr, "Error in schedule of route %d. Departure at depot at %f is earlier than start of service at %f.\n", current_route, D.route[current_route].departure_depot, D.nodes[DARPH_DEPOT].start_tw);
        report_error(message,__FUNCTION__);
    }

    D.nodes[current_node].vehicle_load = D.nodes[current_node].demand;
    if (D.nodes[current_node].vehicle_load > D.veh_capacity)
    {
        fprintf(stderr, "Error in vehicle load of first node %d of route %d.\n", current_node, current_route);
        fprintf(stderr, "Demand node %d: %d", current_node, D.nodes[current_node].demand);
    }
   

    len += D.d[DARPH_DEPOT][current_node];
        
    while(D.next_array[current_node] != DARPH_DEPOT)
    {
        // When we finally get a route beginning at 0, this is the last route
        // and there is no next route, so break out

        if(D.next_array[current_node] == current_node)
        {
            // We've entered a loop
            fprintf(stderr,"Self loop found in next array(%d)\n",current_node);
            report_error("%s: Self loop!\n",__FUNCTION__);
        }

        if(D.next_array[current_node]>0)
        {
            // Next node is somewhere in the middle of a route

            next_node = D.next_array[current_node];
            if (next_node <= n)
            {
                D.routed[next_node] = true;
            }
            
            // Make sure current_node and next_node have the same route #
            if(D.route_num[current_node] != D.route_num[next_node])
            {
                fprintf(stderr,"Route # error for %d and %d: %d!=%d\n",current_node, next_node,
                    D.route_num[current_node],D.route_num[next_node]);

                report_error(message);
            }

            // check feasibility of beginning of service time
            if (D.nodes[next_node].beginning_service + 0.01 < D.nodes[next_node].start_tw || D.nodes[next_node].beginning_service - 0.01 > D.nodes[next_node].end_tw)
            {
                fprintf(stderr,"Error in time windows of node %d. Time window [%f,%f]. Beginning of service at node: %f!\n",next_node,D.nodes[next_node].start_tw,D.nodes[next_node].end_tw,D.nodes[next_node].beginning_service);
                report_error(message,__FUNCTION__);
            }


            // Make sure the route schedule is consistent
            D.nodes[current_node].departure_time = D.nodes[next_node].beginning_service - D.tt[current_node][next_node];
            if (D.nodes[current_node].departure_time + 0.01 < (D.nodes[current_node].beginning_service + D.nodes[current_node].service_time))
            {
                fprintf(stderr, "Next: %d -> %d\n", current_node, next_node);
                fprintf(stderr, "Error in schedule of route %d. Departure at node %d at %f is earlier than possible: Beginning of service %f. Service time %f.\n", current_route, current_node, D.nodes[current_node].departure_time, D.nodes[current_node].beginning_service, D.nodes[current_node].service_time);
                report_error(message,__FUNCTION__);
            }

            len+=D.d[current_node][next_node];

            // make sure vehicle load is feasible
            D.nodes[next_node].vehicle_load = D.nodes[current_node].vehicle_load + D.nodes[next_node].demand;
            if (D.nodes[next_node].vehicle_load > D.veh_capacity)
            {
                fprintf(stderr, "Error in vehicle load of node %d of route %d.\n", next_node, current_route);
                fprintf(stderr, "Demand node %d: %d", next_node, D.nodes[next_node].demand);
            }

           
            // Make sure ride time is feasible
            // Compute excess ride time
            
            if (next_node > n)
            {
                D.nodes[next_node].ride_time = D.nodes[next_node].beginning_service - (D.nodes[next_node-n].beginning_service + D.nodes[next_node-n].service_time);
                if (D.nodes[next_node].ride_time - 0.01 > D.nodes[next_node].max_ride_time)
                {
                    fprintf(stderr, "Error in ride time of request %d\n", next_node-n);
                    fprintf(stderr, "Beginning of service at pick-up node: %f. Service time: %f\n", D.nodes[next_node-n].beginning_service, D.nodes[next_node-n].service_time);
                    fprintf(stderr, "Beginning of service at drop-off node: %f.\n", D.nodes[next_node].beginning_service);
                    fprintf(stderr, "Max ride time: %f", D.nodes[next_node].max_ride_time);
                    report_error(message,__FUNCTION__);
                }
                if (consider_excess_ride_time)
                {    
                    excess += D.nodes[next_node].beginning_service - D.nodes[next_node].start_tw;
                }
            } 
            
            
        
            current_node=next_node;
 
        }
        else
        {
            // We must have a non-positive "next" node indicating the beginning of a new route

            len += D.d[current_node][DARPH_DEPOT];

            current_route = D.route_num[current_node];
            current_end = D.route[current_route].end;

            if(current_node != current_end)
            {
                fprintf(stderr,"Error in route ends: %d!=%d\n",current_node, current_end);
                report_error(message);
            }

            D.route[current_route].return_depot = D.nodes[current_node].beginning_service + D.nodes[current_node].service_time + D.tt[current_node][DARPH_DEPOT];
            // check maximum route length
            if (D.max_route_duration < (D.route[current_route].return_depot - D.route[current_route].departure_depot) - 0.01)
            {
                fprintf(stderr, "Error in route duration of route %d.\n", current_route);
                fprintf(stderr, "Max route duration: %f. Departure depot: %f. Return depot: %f/n", D.max_route_duration, D.route[current_route].departure_depot, D.route[current_route].return_depot);
                report_error(message,__FUNCTION__);
            }

            route_start = - (D.next_array[current_node]);
            current_route = D.route_num[route_start];
            current_start = D.route[current_route].start;
            current_end = D.route[current_route].end;
            counted_routes++;

            if(route_start != current_start)
            {
                fprintf(stderr,"Route %d:  %d != %d\n",current_route, route_start, current_start);
                report_error(message);
            }

            current_node = route_start;
            D.routed[current_node] = true;

            if (D.nodes[current_node].beginning_service + 0.01 < D.nodes[current_node].start_tw || D.nodes[current_node].beginning_service - 0.01 > D.nodes[current_node].end_tw)
            {
                fprintf(stderr,"Error in time windows of node %d. Time window [%f,%f]. Beginning of service at node: %f!\n",current_node,D.nodes[current_node].start_tw,D.nodes[current_node].end_tw, D.nodes[current_node].beginning_service);
                report_error(message,__FUNCTION__);
            }

            D.route[current_route].departure_depot = D.nodes[current_node].beginning_service - D.tt[DARPH_DEPOT][current_node];
            if (D.route[current_route].departure_depot + 0.01 < D.nodes[DARPH_DEPOT].start_tw)
            {
                fprintf(stderr, "Error in schedule of route %d. Departure at depot at %f is earlier than start of service at %f.\n", current_route, D.route[current_route].departure_depot, D.nodes[DARPH_DEPOT].start_tw);
                report_error(message,__FUNCTION__);
            }

            len+=D.d[DARPH_DEPOT][current_node];
            
            D.nodes[current_node].vehicle_load = D.nodes[current_node].demand;
            if (D.nodes[current_node].vehicle_load > D.veh_capacity)
            {
                fprintf(stderr, "Error in vehicle load of first node %d of route %d.\n", current_node, current_route);
                fprintf(stderr, "Demand node %d: %d\n", current_node, D.nodes[current_node].demand);
            }
        }
    }

    
    // We're at the end of the Solution!
    len+=D.d[current_node][DARPH_DEPOT];
    current_route=D.route_num[current_node];
    current_end = D.route[current_route].end;

    if(current_node != current_end)
    {
        fprintf(stderr,"Error in route ends: %d!=%d\n",current_node, current_end);
        report_error(message);
    }

    D.route[current_route].return_depot = D.nodes[current_node].beginning_service + D.nodes[current_node].service_time + D.tt[current_node][DARPH_DEPOT];
    // check maximum route length
    if (D.max_route_duration < (D.route[current_route].return_depot - D.route[current_route].departure_depot) - 0.01)
    {
        fprintf(stderr, "Error in route duration of route %d.\n", current_route);
        fprintf(stderr, "Max route duration: %f. Departure depot: %f. Return depot: %f/n", D.max_route_duration, D.route[current_route].departure_depot, D.route[current_route].return_depot);
        report_error(message,__FUNCTION__);
    }
    
    if(DARPH_ABS(len-total_routing_costs)>=.01)
    {
        fprintf(stderr,"Routing costs error: calculated(%f)!=claimed(%f)\n",len, total_routing_costs);
        report_error(message);
    }

    if (consider_excess_ride_time)
    {
        if (DARPH_ABS(excess-total_excess_ride_time) >=.01)
        {    
            fprintf(stderr,"Excess ride time error: calculated(%f)!=claimed(%f)\n",excess, total_excess_ride_time);
            report_error(message);
        }
    }
    

    // check if pick-up and drop-off are in the same route
    // check if pick-up is before drop-off
    for (int i=1; i<=n; ++i)
    {
        if (D.route_num[i] != D.route_num[i+n])
        {
            fprintf(stderr, "Node %d is in route %d, but node %d is in route %d\n", i, D.route_num[i], n + i, D.route_num[n + i]);
        }
        if (D.nodes[i].beginning_service > D.nodes[n+i].beginning_service)
        {
            fprintf(stderr, "Drop-off node %d is located before pick-up node %d!\n", n+i, i);
        }
    }


    fprintf(stderr, "DARP::verify_routes: Verification at '%s' passed\n", message);
    fprintf(stdout, "DARP::verify_routes: Verification at '%s' passed\n", message);
    return true;
}


void DARPSolver::compute_stats(DARP& D)
{
    std::cout << "Computing stats: ...\n";

    // average trip length in solution
    avg_waiting_time = 0;
    avg_ride_time = 0;
    avg_transportation_time = 0;
    for (int i = 1; i<=n; ++i)
    {
        if (D.routed[i])
        {
            if (D.nodes[i].request_type == 0) {
                avg_waiting_time += D.nodes[i].beginning_service - D.nodes[i].start_tw;
            }
            else if (D.nodes[i].request_type == 1) {
                avg_waiting_time += D.nodes[n+i].beginning_service - D.nodes[n+i].start_tw;
            }
            
            avg_ride_time += D.nodes[n+i].beginning_service - D.nodes[i].departure_time;
            avg_transportation_time += D.nodes[n+i].beginning_service - D.nodes[i].start_tw;
        }
    }
    avg_waiting_time = avg_waiting_time / answered_requests;
    avg_ride_time = avg_ride_time / answered_requests;
    avg_transportation_time = avg_transportation_time / answered_requests;
    std::cout << "Average waiting time: " << roundf(avg_waiting_time * 100) / 100 << std::endl;
    std::cout << "Average ride time (of solution): " << roundf(avg_ride_time * 100) / 100 << std::endl; 
    std::cout << "Average transportation time: " << roundf(avg_transportation_time * 100) / 100 << std::endl;

    int current_node = DARPH_DEPOT;
    int next_node = DARPH_ABS(D.next_array[DARPH_DEPOT]);

    empty_mileage = D.d[current_node][next_node];
    while (next_node != 0)
    {
        current_node = next_node;
        next_node = D.next_array[next_node];
        if (next_node < 0)
        {
            empty_mileage += D.d[current_node][DARPH_DEPOT];
            next_node = DARPH_ABS(next_node);
            empty_mileage += D.d[DARPH_DEPOT][next_node];
        }
        else if (D.nodes[current_node].vehicle_load == 0)
        {
            empty_mileage += D.d[current_node][next_node];
        } 
    }
    
    pax_km_driven = 0;
    pax_km_booked = 0;
    for (int i=1; i<=n; i++)
    {
        if (D.routed[i])
        {    
            pax_km_booked += D.nodes[i].demand * D.d[i][n+i];
            
            current_node = i;
            next_node = D.next_array[current_node];
            while (next_node != n+i)
            {
                pax_km_driven += D.nodes[i].demand * D.d[current_node][next_node];
                current_node = next_node;
                next_node = D.next_array[next_node];
            }
            pax_km_driven += D.nodes[i].demand * D.d[current_node][next_node];
        }
    }

    // for pooling factor
    pooling_factor = 0;
    for (int i=1; i<=n; i++)
    {
        if (D.routed[i])
        {
            // compute booked kilometers without detours
            pooling_factor += D.d[i][n+i];
        } 
    }
    pooling_factor = pooling_factor / total_routing_costs; // gebuchte km / total routing costs

    avg_detour_factor = pax_km_driven / pax_km_booked;
    mean_occupancy = pax_km_driven / (total_routing_costs - empty_mileage);
    share_empty_mileage = empty_mileage / total_routing_costs;
    system_efficiency = pax_km_booked / total_routing_costs;
    if (DARPH_ABS(system_efficiency - 1/avg_detour_factor * mean_occupancy * (1-share_empty_mileage)) > DARPH_EPSILON)
    {
        std::cout << "Error in system efficiency!\n";
        std::cout << "System efficiency: " << system_efficiency << " != " << 1/avg_detour_factor * mean_occupancy * (1-share_empty_mileage) << std::endl;
    }

    std::cout << "Average detour factor: " << avg_detour_factor << std::endl;
    std::cout << "Mean occupancy: " << mean_occupancy << std::endl;
    std::cout << "Share empty mileage: " << share_empty_mileage << std::endl;
    std::cout << "System efficiency: " << system_efficiency << std::endl;
    std::cout << "Pooling factor: " << pooling_factor << std::endl;
    
    return;
}


void DARPSolver::detailed_file(DARP& D, std::string instance, std::array<double,8> solution) const
{
    int current_node, next_node;
    double km_gefahren;
    std::fstream details;
    std::string filename = "output/" + instance + "_details.txt";
   
    details.open(filename, std::ios::out);
    if (!details)
    {
        std::cerr << "File could not be opened." << std::endl;
        exit(-1);
    }
    
    details << instance << std::endl;

    details << "total time & model time & status & MIP Gap" << std::endl;
    details << solution[4] << " & " << solution[5] << " & " << solution[6] << " & " << solution[7] << std::endl;

    details << "obj value & total routing costs & rejected requests & amount of vehicles & direct pax km" << std::endl;
    details << obj_val << " & " << total_routing_costs << " & " << n - answered_requests << " & " << num_buses_required << " & " << pax_km_direct << std::endl;

    details << "avg_waiting_time & avg_ride_time & avg_transportation_time & avg_detour & mean_occupancy & share_empty_mileage & efficiency & pooling_factor" << std::endl;
    details << avg_waiting_time << " & " << avg_ride_time << " & " << avg_transportation_time << " & " << avg_detour_factor << " & " << mean_occupancy << " & " << share_empty_mileage << " & " << system_efficiency << " & " << pooling_factor << std::endl;

    details << " & denied & e_i & l_i & e_{n+i} & l_{n+i} & departure & arrival & regret & waiting_time & ride_time & transportation time & km_gebucht & km_gefahren & Sitze" << std::endl;
    for (int i=1; i<=n; ++i)
    {
        details << i <<  " & " << (1-D.routed[i]) << " & " << D.nodes[i].start_tw << " & " << D.nodes[i].end_tw << " & " << D.nodes[n+i].start_tw << " & " << D.nodes[n+i].end_tw << " & "; 
        if (D.routed[i])
        {
            details << D.nodes[i].beginning_service << " & " << D.nodes[n+i].beginning_service << " & ";

            // regret
            details << D.nodes[n+i].beginning_service - D.nodes[n+i].start_tw << " & ";
            // waiting time 
            details << D.nodes[i].beginning_service - D.nodes[i].start_tw << " & ";
            // ride time
            details << D.nodes[n+i].beginning_service - D.nodes[i].departure_time << " & ";
            // transportation time 
            details << D.nodes[n+i].beginning_service - D.nodes[i].start_tw << " & ";
            // km gebucht
            details << D.d[i][n+i] << " & ";
            // km gefahren 
            current_node = i;
            next_node = D.next_array[current_node];
            km_gefahren = 0;
            while (next_node != n+i)
            {
                km_gefahren += D.d[current_node][next_node];
                current_node = next_node;
                next_node = D.next_array[next_node];
            }
            km_gefahren += D.d[current_node][next_node];

            details << km_gefahren << " & ";            
        }
        else
        {
            details << " - & - & - & - & - & - & - & - & "; 
        }
         // Sitze
        details << D.nodes[i].demand << std::endl;
    }
    details.close();
    std::cout << "Detail file saved sucesfully." << std::endl;
}