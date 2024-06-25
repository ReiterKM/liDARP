#include "DARPH.h"


DARP::DARP(int n) : num_requests{ n }, num_nodes{ 2 * n }, next_array(num_nodes + 1), pred_array(num_nodes + 1), route_num(num_nodes + 1), routed(num_nodes + 1), become_known_array(num_requests) {
    ///
    /// Constructor for an n-node problem
    ///

    // Set these to default values--they may change once we read the file
    num_vehicles = DARPH_INFINITY;
    max_route_duration = DARPH_INFINITY;
    planning_horizon = DARPH_INFINITY;

    d = new double* [num_nodes + 1];
    d[0] = new double[(num_nodes + 1) * (num_nodes + 1)];
    for (int i = 1; i < num_nodes + 1; i++)
        d[i] = d[i - 1] + (num_nodes + 1);

    std::fill(d[0], d[0] + (num_nodes + 1) * (num_nodes + 1), 0.0);

    d_direct.resize(num_requests + 1);
    std::fill(d_direct.begin(), d_direct.end(), 0);

    tt = new double* [num_nodes + 1];
    tt[0] = new double[(num_nodes + 1) * (num_nodes + 1)];
    for (int i = 1; i < num_nodes + 1; i++)
        tt[i] = tt[i - 1] + (num_nodes + 1);

    std::fill(tt[0], tt[0] + (num_nodes + 1) * (num_nodes + 1), 0.0);

    nodes = new DARPNode[num_nodes + 1];

    for (int i = 0; i < num_nodes + 1; i++)
    {
        routed[i] = false;
    }

    // Allocate memory for route array when problem is loaded 
    route = NULL;
}


DARP::~DARP() {
    // Constructor fertig

    delete[] d[0];
    delete[] d;
    delete[] tt[0];
    delete[] tt;
    become_known_array.clear();
    next_array.clear();
    route_num.clear();
    routed.clear();
    pred_array.clear();
    delete[] nodes;
}


void DARP::preprocess()
{
    ///
    /// this functions tightens the time windows of the non-critical vertex 
    /// according to Cordeau [2006]
    ///
    int n = num_requests;

    for (int i = 1; i <= n; ++i)
    {

        if (nodes[i].start_tw < DARPH_EPSILON && nodes[i].end_tw >= planning_horizon)
        {
            nodes[i].end_tw = DARPH_MAX(0, nodes[n + i].end_tw - tt[i][n + i] - nodes[i].service_time);
            nodes[i].start_tw = DARPH_MAX(0, nodes[n + i].start_tw - nodes[i].max_ride_time - nodes[i].service_time);
            if (nodes[i].end_tw <= nodes[i].start_tw)
            {
                report_error("%s: Time window preprocessing at node %d leads to error in time windows.\r\n", __FUNCTION__, i);
            }
        }
        if (nodes[n + i].start_tw < DARPH_EPSILON && nodes[n + i].end_tw >= planning_horizon)
        {
            nodes[n + i].start_tw = nodes[i].start_tw + nodes[i].service_time + tt[i][n + i];
            nodes[n + i].end_tw = nodes[i].end_tw + nodes[i].service_time + nodes[i].max_ride_time;
        }
    }
#if FILE_DEBUG
    for (int i = 1; i <= 2 * n; ++i)
    {
        std::cout << i << "  " << nodes[i].bus_station << "  " << nodes[i].start_tw << "  " << nodes[i].end_tw << std::endl;
    }
#endif
}

void DARP::set_alpha_and_beta(double alpha_in, double beta_in) {
    alpha = alpha_in;
    beta = beta_in;
}

void DARP::read_file(std::string infile, std::string data_directory, std::string instance)
{
    ///
    /// Currently reads file in a very specific format (see README).
    /// For another type of test instance pay attention to nodes[i].max_ride_time.
    ///

    double temp_max_ride_time;
    int i, j;
    double val;
    int temp_half_num_nodes;
    double temp;

    std::ifstream file;
    std::string line;
    file.open(infile.c_str(), std::ios_base::in);
    if (!file)
        report_error("%s: file error\n", __FUNCTION__);

    // Read first line of file, depending on the input format
    getline(file, line);
    std::istringstream iss(line);

    iss >> num_vehicles >> temp_half_num_nodes >> num_stations >> max_route_duration >> veh_capacity >> alpha >> beta;
    num_nodes = 2 * temp_half_num_nodes;

    // depot
    nodes[0].id = 0;

    // Read in the next lines until EOF is reached, which contain the data
    // id origin destination demand earliest_start_time latest_arrival_time
    i = 1;

    string start_tw, end_tw;

    while (i <= num_requests)
    {
        std::getline(file, line);
        std::istringstream iss(line);

        iss >> nodes[i].id >> nodes[i].bus_station >> nodes[i + num_requests].bus_station >> nodes[i].service_time >> nodes[i].demand >> start_tw >> end_tw;

        nodes[i].start_tw = std::atof(start_tw.c_str());
        nodes[i + num_requests].end_tw = std::atof(end_tw.c_str());

        nodes[i + num_requests].id = nodes[i].id + num_requests;
        nodes[i + num_requests].demand = -1 * nodes[i].demand;
        nodes[i + num_requests].service_time = nodes[i].service_time;

        nodes[i].end_tw = max_route_duration;
        nodes[i + num_requests].start_tw = 0;

        i++;
    }

    planning_horizon = max_route_duration;

    file.close();

    // check if requested load is greater than the vehicle capacity
    for (i = 0; i <= num_requests; ++i)
    {
        if (nodes[i].demand > veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[i].demand, veh_capacity);
            report_error("%s: Infeasible number of requested seats detected.\r\n", __FUNCTION__);
        }
        if (nodes[num_requests + i].demand < -veh_capacity)
        {
            fprintf(stderr, "Problem instance is infeasible due to excess load: demand of request %d is %d, vehicle capacity is %d\n", i, nodes[num_requests + i].demand, veh_capacity);
            report_error("%s: Infeasible number of requested seats detected.\r\n", __FUNCTION__);
        }
    }

    // assign direction to nodes
    nodes[0].direction = NULL;
    for (i = 1; i <= num_requests; ++i) {
        if (nodes[i].bus_station < nodes[i + num_requests].bus_station) {
            nodes[i].direction = 0;
            nodes[i + num_requests].direction = 0;
        }
        else if (nodes[i].bus_station > nodes[i + num_requests].bus_station) {
            nodes[i].direction = 1;
            nodes[i + num_requests].direction = 1;
        }
        else {
            // add to F per default, but notify user
            cout << "Warning: request " << i << " has same origin and destination. Added to forward direction per default." << endl;
            nodes[i].direction = 0;
            nodes[i + num_requests].direction = 0;
        }
    }
    cout << endl;
}

void DARP::set_time_windows(){
    double temp_max_ride_time;
        
    // time window lengths
    // Note: these are already tightened, hence no need for pre-processing.
    for (int i = 1; i <= num_requests; ++i)
    {
        // service promise: max ride time is fastest time * alpha;
        nodes[0].max_ride_time = max_route_duration;

        temp_max_ride_time = tt[i][i + num_requests] * alpha;
        nodes[i].max_ride_time = temp_max_ride_time;
        nodes[i + num_requests].max_ride_time = max_route_duration;

        // (T1), inbound request
        if (!(std::isnan(nodes[i].start_tw)) && std::isnan(nodes[i + num_requests].end_tw))
        {
            nodes[i].end_tw = nodes[i].start_tw + beta;
            nodes[i + num_requests].start_tw = std::max(0.0, nodes[i].start_tw + tt[i][i + num_requests] + nodes[i].service_time);
            nodes[i + num_requests].end_tw = std::min(max_route_duration, nodes[i].end_tw + nodes[i].max_ride_time + nodes[i].service_time);

            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests + i].tw_length = nodes[num_requests + i].end_tw - nodes[num_requests + i].start_tw;

            //nodes[i].tw_length = beta;
            //nodes[i + num_requests].tw_length = beta; //nodes[i + num_requests].end_tw - nodes[i + num_requests].start_tw;

            nodes[i].request_type = 0;
            nodes[i + num_requests].request_type = 0;
        }
        // (T2), outbound request
        else if ((std::isnan(nodes[i].start_tw)) && !(std::isnan(nodes[i + num_requests].end_tw)))
        {
            nodes[i + num_requests].start_tw = nodes[i + num_requests].end_tw - beta;
            nodes[i].start_tw = max(0.0, nodes[i + num_requests].start_tw - nodes[i].max_ride_time - nodes[i].service_time);
            nodes[i].end_tw = min(max_route_duration, nodes[i + num_requests].end_tw - tt[i][i + num_requests] - nodes[i + num_requests].service_time);

            nodes[i].tw_length = nodes[i].end_tw - nodes[i].start_tw;
            nodes[num_requests + i].tw_length = nodes[num_requests + i].end_tw - nodes[num_requests + i].start_tw;

            nodes[i].request_type = 1;
            nodes[i + num_requests].request_type = 1;
        }
        // (T3), no time window given
        else if (!(std::isnan(nodes[i].start_tw)) && !(std::isnan(nodes[i + num_requests].end_tw)))
        {
            nodes[i].end_tw = min(max_route_duration, max(0.0, nodes[i + num_requests].end_tw - tt[i][i + num_requests] - nodes[i + num_requests].service_time));
            nodes[i + num_requests].start_tw = max(0.0, nodes[i].start_tw + tt[i][i + num_requests] + nodes[i + num_requests].service_time);

            // custom TW length: l_i- - (e_i+ + L_i + s_i)
            nodes[i].tw_length = nodes[i + num_requests].end_tw - (nodes[i].start_tw + nodes[i].max_ride_time + nodes[i].service_time);
            nodes[num_requests + i].tw_length = nodes[i].tw_length;

            nodes[i].request_type = 2;
            nodes[i + num_requests].request_type = 2;
        }
        else
        {
            report_error("%s: Infeasible time window detected.\r\n", __FUNCTION__);
        }

#if VERBOSE
        cout << "i=" << i << " : [" << nodes[i].start_tw << ", " << nodes[i].end_tw << "], TW = " << nodes[i].tw_length << ", in " << nodes[i].max_ride_time << endl;
        cout << "i+n=" << i + num_requests << ": [" << nodes[i + num_requests].start_tw << ", " << nodes[i + num_requests].end_tw << "], TW = " << nodes[i + num_requests].tw_length << ", in " << nodes[i + num_requests].max_ride_time << endl;
#endif
    }
    // set depot end times
    nodes[0].end_tw = max_route_duration;
} 

void DARP::generate_distances() {
    int i, j;
    double val;

    // Memory for route array is allocated
    route = new DARPRoute[num_vehicles];
    // Create distance and travel time matrix 
    for (i = 1; i <= num_nodes; ++i)
    {
        for (j = 1; j <= num_nodes; j++)
        {
            if (i == j) {
                d[i][j] = 0;
                tt[i][j] = 0;
            }
            else {
                val = std::abs(nodes[i].bus_station - nodes[j].bus_station);
                d[i][j] = roundf(val * 100) / 100;
                tt[i][j] = d[i][j];
            }
        }
    }
    for (i = 1; i <= num_requests + 1; ++i)
    {
        d_direct[i] = d[i][i + num_requests];
    }
}

void DARP::generate_distances(std::string infile, std::string data_directory, std::string instance) {
    int idx, i, j;
    double val;
    int bus_station;

    std::ifstream file;
    std::string line;
    std::string file_path = data_directory + instance + ".txt";
    file.open(file_path.c_str(), std::ios_base::in);
    if (!file)
        report_error("%s: Error opening distance file\r\n", __FUNCTION__);

    // distance file should have the following format
    // d_{1,1} d_{1,2} d_{1,3} ... d_{1,H}
    // d_{2,1} d_{2,2} d_{2,3} ... d_{2,H}
    // ...
    // d_{H,1} d_{H,2} d_{H,3} ... d_{H,H}

    // Memory for route array is allocated
    route = new DARPRoute[num_vehicles];
        
    std::vector<std::vector<double> > station_distances(num_stations + 1, std::vector<double>(num_stations + 1, 0));
    for (i = 1; i < num_stations + 1; ++i)
    {
        std::getline(file, line);
        std::istringstream iss(line);

        for (j = 1; j < num_stations + 1; j++)
        {
            iss >> station_distances[i][j];
        }
    }
    file.close();

    // assign distance matrix, based on requests' stations
    for (i = 1; i <= num_nodes; ++i)
    {
        for (j = 1; j <= num_nodes; ++j)
        {
            if (nodes[i].bus_station == nodes[j].bus_station) {
                d[i][j] = 0;
                tt[i][j] = 0;
            }
            else {
                d[i][j] = station_distances[nodes[i].bus_station][nodes[j].bus_station];
                tt[i][j] = d[i][j];
            }
        }
    }
    for (i = 1; i <= num_requests; ++i)
    {
        d_direct[i] = d[i][i + num_requests];
    }

#if FILE_DEBUG
    std::cout << "Distance matrix: " << std::endl;
    for (int i = 0; i <= num_nodes; ++i) {
        for (int j = 0; j <= num_nodes; ++j) {
            std::cout << d[i][j] << " ";
        }
        std::cout << std::endl;
    }
#endif
}

void DARP::generate_distances(std::string infile, std::string data_directory, std::string instance, bool consider_shortcuts) {
    int idx, i, j, k;
    int i_station, j_station;
    double val;

    std::ifstream file;
    std::string line;
    std::string file_path = data_directory + instance + ".txt";
    file.open(file_path.c_str(), std::ios_base::in);
    if (!file)
        report_error("%s: Error opening distance file\r\n", __FUNCTION__);

    // Memory for route array is allocated
    route = new DARPRoute[num_vehicles];

    std::vector<std::vector<double> > station_location(num_stations + 1, std::vector<double>(2));
    // read station locations from file
    for (i = 0; i < num_stations + 1; ++i) {
        std::getline(file, line);
        std::istringstream iss(line);
        iss >> idx >> station_location[idx][0] >> station_location[idx][1];
        std::cout << i << " : " << station_location[i][0] << ", " << station_location[i][1] << endl;
    }
    file.close();

    // assign distance matrix, based on requests' stations
    for (i = 1; i <= num_nodes; ++i)
    {
        for (j = 1; j <= num_nodes; ++j)
        {
            i_station = nodes[i].bus_station;
            j_station = nodes[j].bus_station;

            if (i_station == j_station) {
                d[i][j] = 0;
                tt[i][j] = 0;
            }
            else {
                if (consider_shortcuts) {
                    val = Euclidean_Distance(station_location[i_station], station_location[j_station], 2);
                }
                else {
                    if (abs(j_station - i_station) == 1) {
                        val = Euclidean_Distance(station_location[i_station], station_location[j_station], 2);
                    }
                    else {
                        // distance i -> j = sum (dist(i,k) for k = i to j)
                        val = 0;
                        int start = DARPH_MIN(i_station, j_station);
                        int end = DARPH_MAX(i_station, j_station);
                        for (k = start; k <= end - 1; ++k) {
                            val += Euclidean_Distance(station_location[k], station_location[k+1], 2);
                        }                        
                    }
                }
                d[i][j] = roundf(val * 100) / 100;
                tt[i][j] = d[i][j];
            }
        }
    }
    for (i = 1; i <= num_requests; ++i)
    {
        i_station = nodes[i].bus_station;
        j_station = nodes[i + num_requests].bus_station;
        val = Euclidean_Distance(station_location[i_station], station_location[j_station], 2);
        d_direct[i] = roundf(val * 100) / 100;
    }
}

void DARP::transform_dynamic(double share_static_requests, double beta)
{
    last_static = (int)(share_static_requests * num_requests);

    for (int i = 1; i <= last_static; ++i)
    {
        become_known_array.push_back(0);
        R.push_back(i);
    }

    for (int i = last_static + 1; i <= num_requests; ++i)
    {
        become_known_array[i - 1] = DARPH_MAX(0, DARPH_MIN(nodes[i].end_tw, nodes[num_requests + i].end_tw - tt[i][num_requests + i] - nodes[i].service_time) - beta);

        if (become_known_array[i - 1] < DARPH_EPSILON)
        {
            // request is known from beginning
            R.push_back(i);
        }
    }

    if (R.empty())
    {
        double first_time = become_known_array[0];
        int first_request = 1;
        for (int i = 2; i <= num_requests; ++i)
        {
            if (become_known_array[i - 1] < first_time)
            {
                first_time = become_known_array[i - 1];
                first_request = i;
            }
        }
        if (first_request == 1)
            last_static = 1;
        else
            last_static = 0;
        R.push_back(first_request);

        // check if there are other requests which become known at the same time
        for (int i = last_static + 1; i <= num_requests; ++i)
        {
            if (i != first_request)
            {
                if (DARPH_ABS(become_known_array[i - 1] - first_time) < DARPH_EPSILON)
                {
                    R.push_back(i);
                    if (i == last_static + 1)
                        last_static = i;
                }
            }
        }
    }
    rcardinality = R.size();
    known_requests = R;
    num_known_requests = rcardinality;
}