#include "DARPH.h"


DARP::DARP(int n) : num_requests{n}, num_nodes{2*n} {
    ///
    /// Constructor for an n-node problem
    ///

    // Set these to default values--they may change once we read the file
    num_vehicles = DARPH_INFINITY;
    max_route_duration = DARPH_INFINITY;
    planning_horizon = DARPH_INFINITY;

    d = new double* [num_nodes+1];
    d[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        d[i] = d[i-1] + (num_nodes+1) ;

    tt = new double* [num_nodes+1];
    tt[0] = new double [(num_nodes+1)*(num_nodes+1)];
    for(int i = 1; i < num_nodes+1; i++)
        tt[i] = tt[i-1] + (num_nodes+1) ;
    

    nodes = new DARPNode[num_nodes+1];
    
    next_array = new int[num_nodes+1];
    pred_array = new int[num_nodes+1];
    route_num = new int[num_nodes+1];
    routed = new bool[num_nodes+1];
    for (int i = 0; i < num_nodes+1; i++)
    {
        routed[i] = false;
    }

    // Allocate memory for route array when problem is loaded 
    route = NULL;
    
    become_known_array = new double[num_requests];
}


DARP::~DARP() {
    // Constructor fertig
    
    delete [] d[0];
    delete [] d;
    delete [] tt[0];
    delete [] tt;
    delete [] become_known_array;
    delete [] nodes;
    delete [] next_array;
    delete [] pred_array;
    delete [] route_num;
    delete [] routed;
    
}


void DARP::preprocess()
{
    ///
    /// this functions tightens the time windows of the non-critical vertex 
    /// according to Cordeau [2006]
    ///
    int n = num_requests;
    
    for (int i = 1; i<=n; ++i)
    {
        
        if (nodes[i].start_tw < DARPH_EPSILON && nodes[i].end_tw >= planning_horizon)
        {
            nodes[i].end_tw = DARPH_MAX(0,nodes[n+i].end_tw - tt[i][n+i]- nodes[i].service_time);
            nodes[i].start_tw = DARPH_MAX(0,nodes[n+i].start_tw - nodes[i].max_ride_time - nodes[i].service_time);
            if (nodes[i].end_tw <= nodes[i].start_tw)
            {
                report_error("%s: Time window preprocessing at node %d leads to error in time windows.\n",__FUNCTION__, i);
            }
        }
        if (nodes[n+i].start_tw < DARPH_EPSILON && nodes[n+i].end_tw >= planning_horizon)
        {
            nodes[n+i].start_tw = nodes[i].start_tw + nodes[i].service_time + tt[i][n+i];
            nodes[n+i].end_tw = nodes[i].end_tw + nodes[i].service_time + nodes[i].max_ride_time;
        }
    }
#if FILE_DEBUG
    for(int i=1; i<=2*n; ++i)
    {
        std::cout << i << "  " << nodes[i].x << "  " << nodes[i].y << "  " << nodes[i].start_tw << "  " << nodes[i].end_tw << std::endl;
    }
#endif
}



void DARP::read_file(std::string infile, std::string data_directory, std::string instance)
{   
    ///
    /// Currently reads file in format of the pr-set (Cordeau and Laporte, 2003).
    /// For another type of test instance pay attention to nodes[i].max_ride_time.
    ///

    double temp_max_ride_time;
    int i, j;
    double val;

    std::ifstream file;
    std::string line;
    file.open(infile.c_str(), std::ios_base::in);
    if (!file)
        report_error("%s: file error\n", __FUNCTION__);

    // Read first line of file, which contains the following data:
    // number of vehicles, number of nodes, maximum route duration, vehicle capacity, maximum ride time 
    getline(file,line);
    std::istringstream f(line);
    f >> num_vehicles >> num_nodes >> max_route_duration >> veh_capacity >> temp_max_ride_time;
    
    if (instance_mode == 1)
    {
        // The following lines have to be changed if there are individual maximum ride times
        for (i=1; i<=num_nodes; ++i)
        {
            nodes[i].max_ride_time = temp_max_ride_time;
        }
        
        // Read in the next lines until EOF is reached, which contain the data
        // id x y service_time load start_tw end_tw
        i = 0;
        while(i<=num_nodes)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            iss >> nodes[i].id >> nodes[i].x >> nodes[i].y >> nodes[i].service_time >> nodes[i].demand >> nodes[i].start_tw >> nodes[i].end_tw;
            i++;
        }
        for (i=1; i<=num_requests/2; ++i)
        {
            // first half of requests is outbound
            nodes[i].tw_length = nodes[num_requests+i].end_tw - nodes[num_requests+i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length;
        }
        for (i=num_requests/2+1; i<=num_requests; ++i)
        {
            // second half of requests is inbound
            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length;
        }

        planning_horizon = nodes[0].end_tw;
    }
    else
    {
        // Read in the next lines until EOF is reached, which contain the data
        // id service_time load start_tw end_tw max_ride_time
        i = 0;
        while(i<=num_nodes)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            iss >> nodes[i].id >> nodes[i].service_time >> nodes[i].demand >> nodes[i].start_tw >> nodes[i].end_tw >> nodes[i].max_ride_time;
            std::cout << nodes[i].id << " " << nodes[i].service_time << " " << nodes[i].demand << " " << nodes[i].start_tw << " " << nodes[i].end_tw << " " << nodes[i].max_ride_time << std::endl;
            i++;
        }
        // all requests are inbound
        for (i=1; i<=num_requests; i++)
        {
            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests+i].tw_length = nodes[i].tw_length; 
        }  

        planning_horizon = max_route_duration;
    }
   
    file.close();

    // check if requested load is greater than the vehicle capacity
    for (i=0; i<=num_requests; ++i)
    {
        if (nodes[i].demand > veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[i].demand, veh_capacity);
            report_error("%s: Infeasible number of requested seats detected.\n", __FUNCTION__);
        }
        if (nodes[num_requests+i].demand < -veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[num_requests+i].demand, veh_capacity);
            report_error("%s: Infeasible number of requested seats detected.\n", __FUNCTION__);
        }
    }
   
    // Memory for route array is allocated
    route = new DARPRoute[num_vehicles];
    
    // Create distance and travel time matrix 
    if (instance_mode == 1)
    {
        for(i=0; i<=num_nodes; ++i)
        {
            for(j=0; j<=num_nodes; j++)
            {
                val = sqrt((nodes[i].x - nodes[j].x) * (nodes[i].x - nodes[j].x) 
                        + (nodes[i].y - nodes[j].y) * (nodes[i].y - nodes[j].y));
                d[i][j] = roundf(val * 100) / 100; 
                tt[i][j] = d[i][j];
            }  
        }
    }    
    else if (instance_mode == 2)
    {
        std::string path_to_costs = data_directory + instance + "_c_a.txt";
    

        file.open(path_to_costs.c_str(), std::ios_base::in);
        if (!file)
            report_error("%s: costs file error\n", __FUNCTION__);

        for(i=0; i<num_nodes+1; ++i)
        {
            std::getline(file,line);
            std::istringstream iss(line);
            for(j=0; j<num_nodes+1; j++)
            {
                iss >> d[i][j];
                val = 1.8246 * d[i][j] + 2.3690; // based on linear regression with data = all completed rides (Jan, Feb 21)
                // val = 2.3634 * d[i][j] + 0.2086;   // night time (März - Sep 2021, 22-3:59h)
                tt[i][j] = roundf(val * 100) / 100;
            }    
        }
        file.close();
    }
        
  
    
    return;
}



void DARP::transform_dynamic(double share_static_requests, double beta)
{
    if (instance_mode == 1)
    {
        last_static = (int) (share_static_requests  * num_requests);
        
        for (int i=1; i<=last_static; ++i)
        {
            become_known_array[i-1] = 0;
            R.push_back(i);
        }
        
        for (int i=last_static+1; i<=num_requests; i++)
        {
            become_known_array[i-1] = DARPH_MAX(0,DARPH_MIN(nodes[i].end_tw, nodes[num_requests+i].end_tw - tt[i][num_requests+i] - nodes[i].service_time) - beta);

            if (become_known_array[i-1] < DARPH_EPSILON)
            {
                // request is known from beginning
                R.push_back(i);
            }
        }

        if (R.empty())
        {
            double first_time = become_known_array[0];
            int first_request = 1;
            for (int i = 2; i<=num_requests; ++i)
            {
                if (become_known_array[i-1] < first_time)
                {
                    first_time = become_known_array[i-1];
                    first_request = i;
                }
            }
            if (first_request == 1)
                last_static = 1;
            else 
                last_static = 0;
            R.push_back(first_request);
            
            // check if there are other requests which become known at the same time
            for (int i=last_static+1; i<=num_requests; ++i)
            {
                if(i != first_request)
                {
                    if (DARPH_ABS(become_known_array[i-1] - first_time) < DARPH_EPSILON)
                    {
                        R.push_back(i);
                        if (i == last_static+1)
                            last_static = i;
                    }
                }
            }
        }
    }
    else
    {
        // WSW
        for (int i = 1; i<=num_requests; ++i)
        {
            become_known_array[i-1] = nodes[i].start_tw;
        }
        double first_time = become_known_array[0];
        int first_request = 1;
        for (int i = 2; i<=num_requests; ++i)
        {
            if (become_known_array[i-1] < first_time)
            {
                first_time = become_known_array[i-1];
                first_request = i;
            }
        }
        if (first_request == 1)
            last_static = 1;
        else 
            last_static = 0;
        R.push_back(first_request);
        
        // check if there are other requests which become known at the same time
        for (int i=last_static+1; i<=num_requests; ++i)
        {
            if(i != first_request)
            {
                if (DARPH_ABS(become_known_array[i-1] - first_time) < DARPH_EPSILON)
                {
                    R.push_back(i);
                    if (i == last_static+1)
                        last_static = i;
                }
            }
        }
         
    }
    rcardinality = R.size();
    known_requests = R;
    num_known_requests = rcardinality;
}
