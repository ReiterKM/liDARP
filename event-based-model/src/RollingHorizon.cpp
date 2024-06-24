#include "DARPH.h"

template<int Q>
RollingHorizon<Q>::RollingHorizon(int num_requests) : DARPSolver{num_requests}  
{
    communicated_pickup = new double[num_requests];

    vec_map = new std::unordered_map<NODE,int,HashFunction<Q>>[n]; 
    
    active_node = new std::pair<NODE,double>[2*num_requests];
    
    active_arc = new ARC[2*num_requests];
    
}

template<int Q>
RollingHorizon<Q>::~RollingHorizon() {
    delete[] communicated_pickup;
    delete[] vec_map;
    delete[] active_node;
    delete[] active_arc;
}


template<int Q>
void RollingHorizon<Q>::create_maps(DARP& D, DARPGraph<Q>& G) {

    uint64_t count = 0;
    uint64_t count_vout = 0;
    
    // add depot node to vmap
    vmap[G.depot] = count;
    count ++;
    for (const auto& v: G.V_in)
    {
        vmap[v] = count;
        vinmap[v] = count - 1;
        count ++;

        G.V_i[v[0]].push_back(v);
    }
    G.vincardinality = count - 1;
    count_vout = 0;
    for (const auto& v: G.V_out)
    {
        vmap[v] = count;
        voutmap[v] = count_vout;
        count ++;
        count_vout ++;

        G.V_i[v[0]].push_back(v);
    }
    G.vcardinality = count;
    G.voutcardinality = count_vout; 
    

       
    // for each request create an unordered map that maps each node in V_{n+i} to an integer
    for (const auto& i: D.R)
    {
        count = 0;
        for (const auto& w : G.V_i[n+i])
        {
            vec_map[i-1][w] = count;
            count++;
        }
    }
    
    // create arc map
    count = 0;
    for (const auto& a: G.A)
    {
        amap[a] = count;
        count ++;
    }
    G.acardinality = count; 

     // initialize rmap
    count = 0;
    for (const auto & i: D.R)
    {
        rmap[i] = count;
        count++;
    }
    
}

template<int Q>
void RollingHorizon<Q>::update_maps(const std::vector<int>& new_requests, DARP& D, DARPGraph<Q>& G)
{
    uint64_t count = G.vcardinality;
    uint64_t count_vin = G.vincardinality; // index is at vincardinality -1
    uint64_t count_vout = G.voutcardinality; // index is at voutcardinality -1

    for (const auto& v: G.V_in_new)
    {
        vmap[v] = count;
        count++;
        vinmap[v] = count_vin;
        count_vin++;

        G.V_i_new[v[0]].push_back(v);
    }
    for (const auto& v: G.V_out_new)
    {
        vmap[v] = count;
        count++;
        voutmap[v] = count_vout;
        count_vout++;

        G.V_i_new[v[0]].push_back(v);
    }

    // Test if vmap assigned correctly
    if (G.vcardinality + G.num_new_nodes != count)
        report_error("%s: Error in node count and vmap\n",__FUNCTION__);
    
    // No error --> update cardinalities
    G.vcardinality = count;
    G.vincardinality = count_vin;
    G.voutcardinality = count_vout;


    // for the new requests create an unordered map that maps each node in V_{n+i} to an integer
    for (const auto & i : new_requests)
    {
        count = 0;
        for (const auto& w : G.V_i_new[n+i])
        {
            vec_map[i-1].insert({w,count});
            count++;
        }
    }
    
    // for each old request add the new drop-off nodes to vec_map
    for (const auto& i : D.R)
    {
        count = vec_map[i-1].size(); // last element has index count-1
        for (const auto& w: G.V_i_new[n+i])
        {    
            vec_map[i-1].insert({w,count});
            count++;
        }
    }

    // update arc maps
    count = G.acardinality;
    for (const auto& a: G.A_new)
    {
        amap[a] = count;
        count ++;
    }
    // Test if amap assigned correctly
    if (G.acardinality + G.num_new_arcs != count)
        report_error("%s: Error in new arc count and amap\n",__FUNCTION__);
    G.acardinality = count; 
}



template class RollingHorizon<3>;
template class RollingHorizon<6>;