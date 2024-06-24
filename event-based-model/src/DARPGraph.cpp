#include "DARPH.h"

template <int Q>
DARPGraph<Q>::DARPGraph(int num_requests) : n{num_requests} { 
    // all other attributes are assigned in the course of the algorithm and when the graph is created, see create_graph()
}

template <int Q>
DARPGraph<Q>::~DARPGraph() {

}




template <int Q>
void DARPGraph<Q>::create_nodes(DARP& D, int*** f)
{

    if constexpr (Q==3)
    {
        depot = {0,0,0};
    }
    else
        depot = {0,0,0,0,0,0};
    
    for (const auto& i: D.R)
    {
        if constexpr (Q==3)
        {
            V_in.push_back({i,0,0});
            V_out.push_back({n+i,0,0});  
        }
        else
        {
            V_in.push_back({i,0,0,0,0,0});
            V_out.push_back({n+i,0,0,0,0,0});  
        }
        
        for (const auto& j: D.R)
        {
            if (j != i)
            {
                if (f[i][j][0] || f[i][j][1])
                {
                    if constexpr (Q==3)
                        V_in.push_back({i,j,0});
                    else
                        V_in.push_back({i,j,0,0,0,0});

                    for (const auto& k: D.R)
                    {
                        if (k != i && k<=j-1)
                        {
                            if (f[i][k][0] || f[i][k][1])
                            {
                                if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand <= D.veh_capacity)
                                {
                                    if constexpr (Q==3)
                                        V_in.push_back({i,j,k});
                                    else
                                        V_in.push_back({i,j,k,0,0,0});
                                    
                                    if constexpr (Q == 6)
                                    {
                                        // go on with nodes that are available for Q = 6
                                        for (const auto& a: D.R)
                                        {
                                            if (a != i && a <= k-1)
                                            {
                                                if (f[i][a][0] || f[i][a][1])
                                                {
                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand <= D.veh_capacity)
                                                    {
                                                        V_in.push_back({i,j,k,a,0,0});

                                                        for (const auto& b: D.R)
                                                        {
                                                            if (b != i && b <= a-1)
                                                            {
                                                                if (f[i][b][0] || f[i][b][1])
                                                                {
                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand <= D.veh_capacity)
                                                                    {
                                                                        V_in.push_back({i,j,k,a,b,0});

                                                                        for (const auto& c: D.R)
                                                                        {
                                                                            if (c != i && c <= b-1)
                                                                            {
                                                                                if (f[i][c][0] || f[i][c][1])
                                                                                {
                                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand + D.nodes[c].demand <= D.veh_capacity)
                                                                                        V_in.push_back({i,j,k,a,b,c});
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                if (f[i][j][1] || f[j][i][0])
                {
                    if constexpr (Q==3)
                        V_out.push_back({n+i,j,0});
                    else
                        V_out.push_back({n+i,j,0,0,0,0});
                    for (const auto& k: D.R)
                    {
                        if (k != i && k<=j-1)
                        {
                            if (f[i][k][1] || f[k][i][0])
                            {
                                if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand <= D.veh_capacity)
                                {
                                    if constexpr (Q==3)
                                        V_out.push_back({n+i,j,k});
                                    else
                                        V_out.push_back({n+i,j,k,0,0,0});
                                    
                                    if constexpr (Q==6)
                                    {
                                        for (const auto& a: D.R)
                                        {
                                            if (a != i && a <= k-1)
                                            {
                                                if (f[i][a][1] || f[a][i][0])
                                                {
                                                    if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand <= D.veh_capacity)
                                                    {
                                                        V_out.push_back({n+i,j,k,a,0,0});

                                                        for (const auto& b: D.R)
                                                        {
                                                            if (b != i && b <= a-1)
                                                            {
                                                                if (f[i][b][1] || f[b][i][0])
                                                                {
                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand <= D.veh_capacity)
                                                                    {
                                                                        V_out.push_back({n+i,j,k,a,b,0});

                                                                        for (const auto& c: D.R)
                                                                        {
                                                                            if (c != i && c <= b-1)
                                                                            {
                                                                                if (f[i][c][1] || f[c][i][0])
                                                                                {
                                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand + D.nodes[c].demand <= D.veh_capacity)
                                                                                    {
                                                                                        V_out.push_back({n+i,j,k,a,b,c});
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }

                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}


template <int Q>
void DARPGraph<Q>::create_new_nodes(DARP& D, int*** f, const std::vector<int> &new_requests)
{
    std::vector<int> Rplus(D.R);
    for (const auto & j: new_requests)
    {
        Rplus.push_back(j);
    }

    num_new_nodes = 0;
 
    for (const auto & i : new_requests)
    {
        if constexpr (Q==3)
        {
            V_in_new.push_back({i,0,0});
            num_new_nodes++;
            V_out_new.push_back({n+i,0,0});
            num_new_nodes++;
        }
        else
        {
            V_in_new.push_back({i,0,0,0,0,0});
            num_new_nodes++;
            V_out_new.push_back({n+i,0,0,0,0,0});
            num_new_nodes++;
        }   
    }
   

    for (const auto& i : Rplus)
    {   
        for (const auto& j: Rplus)
        {
            if (j != i)
            {
                if (f[i][j][0] || f[i][j][1])
                {
                    if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                    ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end())
                    {
                        if constexpr (Q==3)
                        {
                            V_in_new.push_back({i,j,0});
                            num_new_nodes++;
                        }
                        else
                        {
                            V_in_new.push_back({i,j,0,0,0,0});
                            num_new_nodes++;
                        }
                    }
                        
                    for (const auto& k: Rplus)
                    {
                        if (k != i && k <= j-1)
                        {
                            if (f[i][k][0] || f[i][k][1])
                            {
                                if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand <= D.veh_capacity)
                                {
                                    if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                    ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                    ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end())
                                    {    
                                        if constexpr (Q==3)
                                        {
                                            V_in_new.push_back({i,j,k});
                                            num_new_nodes++;
                                        }
                                        else
                                        {
                                            V_in_new.push_back({i,j,k,0,0,0});
                                            num_new_nodes++;
                                        }
                                    }
                                    if constexpr (Q==6)
                                    {
                                        for (const auto& a: Rplus)
                                        {
                                            if (a!= i && a <= k-1)
                                            {
                                                if (f[i][a][0] || f[i][a][1])
                                                {
                                                    if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand <= D.veh_capacity)
                                                    {
                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end())
                                                        {    
                                                            V_in_new.push_back({i,j,k,a,0,0});
                                                            num_new_nodes++;
                                                        }
                                                        for (const auto& b: Rplus)
                                                        {
                                                            if (b != i && b <= a-1)
                                                            {
                                                                if (f[i][b][0] || f[i][b][1])
                                                                {
                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand <= D.veh_capacity)
                                                                    {
                                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), b) != new_requests.end())
                                                                        { 
                                                                            V_in_new.push_back({i,j,k,a,b,0});
                                                                            num_new_nodes++;
                                                                        }
                                                                        for (const auto& c : Rplus)
                                                                        {
                                                                            if (c != i && c <= b-1)
                                                                            {
                                                                                if (f[i][c][0] || f[i][c][1])
                                                                                {
                                                                                    if (D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand + D.nodes[c].demand <= D.veh_capacity)
                                                                                    {
                                                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), b) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), c) != new_requests.end())
                                                                                        {
                                                                                            V_in_new.push_back({i,j,k,a,b,c});
                                                                                            num_new_nodes++;
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
            if (f[i][j][1] || f[j][i][0])
            {
                if (j != i)
                {
                    if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                    ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end())
                    {
                        if constexpr (Q==3)
                        {
                            V_out_new.push_back({n+i,j,0});
                            num_new_nodes++;
                        }
                        else
                        {
                            V_out_new.push_back({n+i,j,0,0,0,0});
                            num_new_nodes++;
                        }
                    }
                    
                    for (const auto& k: Rplus)
                    {
                        if (k != i && k <= j-1)
                        {
                            if (f[i][k][1] || f[k][i][0])
                            {
                                if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand <= D.veh_capacity)
                                {
                                    if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                    ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                    ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end())
                                    {  
                                        if constexpr (Q==3)
                                        {
                                            V_out_new.push_back({n+i,j,k});
                                            num_new_nodes++;
                                        }
                                        else
                                        {
                                            V_out_new.push_back({n+i,j,k,0,0,0});
                                            num_new_nodes++;
                                        }
                                    }

                                    if constexpr (Q==6)
                                    {
                                        for (const auto& a: Rplus)
                                        {
                                            if (a != i && a <= k-1)
                                            {
                                                if (f[i][a][1] || f[a][i][0])
                                                {
                                                    if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand <= D.veh_capacity)
                                                    {
                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end())
                                                        { 
                                                            V_out_new.push_back({n+i,j,k,a,0,0});
                                                            num_new_nodes++;
                                                        }

                                                        for (const auto& b: Rplus)
                                                        {
                                                            if (b != i && b <= a-1)
                                                            {
                                                                if (f[i][b][1] || f[b][i][0])
                                                                {
                                                                    if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand <= D.veh_capacity)
                                                                    {
                                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end() \
                                                                        ||std::find(new_requests.begin(), new_requests.end(), b) != new_requests.end())
                                                                        { 
                                                                            V_out_new.push_back({n+i,j,k,a,b,0});
                                                                            num_new_nodes++;
                                                                        }

                                                                        for (const auto& c: Rplus)
                                                                        {
                                                                            if (c != i && c <= b-1)
                                                                            {
                                                                                if (f[i][c][1] || f[c][i][0])
                                                                                {
                                                                                    if(D.nodes[i].demand + D.nodes[j].demand + D.nodes[k].demand + D.nodes[a].demand + D.nodes[b].demand + D.nodes[c].demand <= D.veh_capacity)
                                                                                    {
                                                                                        if (std::find(new_requests.begin(), new_requests.end(), i) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), j) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), k) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), a) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), b) != new_requests.end() \
                                                                                        ||std::find(new_requests.begin(), new_requests.end(), c) != new_requests.end())
                                                                                        {
                                                                                            V_out_new.push_back({n+i,j,k,a,b,c});
                                                                                            num_new_nodes++;
                                                                                        }
                                                                                    }
                                                                                }
                                                                            }
                                                                        }
                                                                    }
                                                                }
                                                            }
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}    





template <>
void DARPGraph<3>::create_arcs(DARP& D, int*** f)
{
    NODE u, w;
    ARC a;
    
    for (const auto& i: D.R)
    {
        // (0,0,0) --> (i,0,0)
        u = {0,0,0}; 
        w = {i,0,0};
        a = {u,w};
        A.push_back(a);
        c[a] = D.d[0][i];
        t[a] = D.tt[0][i];

        // (n+i,0,0) --> (0,0,0)
        w = {n+i,0,0};
        a = {w,u};
        A.push_back(a);
        c[a] = D.d[n+i][0];
        t[a] = D.tt[n+i][0];
    }

    for (const auto& v: V_in)
    {
        // transit from v[0]'s pick-up location to another user's drop-off location (including v[0])
        
        // (i,...,j,...) --> (n+i,...,j,...)
        // if j -- i -- n+i -- n+j is feasible for j = v[1], v[2]
        if (f[v[0]][v[1]][1] && f[v[0]][v[2]][1])
        {
            w = {n + v[0], v[1], v[2]};
            a = {v,w};
            A.push_back(a);
            c[a] = D.d[v[0]][n + v[0]];
            t[a] = D.tt[v[0]][n + v[0]];
        }

        // (i,...,j,...) --> (n+j,...,i,...)
        if (v[1] != 0)
        {
            // if v[1] -- v[0] -- n+v[1] -- n+v[0] is feasible 
            // (and v[1] -- v[2] -- n+v[1] -- n+v[2] OR  v[2] -- v[1] -- n+v[1] -- n+v[2] is feasible) : i = v[1] j = v[2] f[i][j][1]||f[j][i][0]
            if (f[v[0]][v[1]][0] && (f[v[1]][v[2]][1]||f[v[2]][v[1]][0]))
            {
                if (v[0] > v[2])
                    w = {n + v[1], v[0], v[2]};
                else
                    w = {n + v[1], v[2], v[0]};
                a = {v,w};
                A.push_back(a);
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }
                
            if (v[2] != 0) 
            {
                if (f[v[0]][v[2]][0] && (f[v[2]][v[1]][1]||f[v[1]][v[2]][0]))
                {
                    if (v[0] > v[1])
                        w = {n + v[2], v[0], v[1]};
                    else
                        w = {n + v[2], v[1], v[0]};
                    a = {v,w};
                    A.push_back(a);
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }
            }
        }

        // transit from v[0]'s pick-up location to another user's pick-up location
        if (v[2] == 0)
        {
            for (const auto& i: D.R)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[1],...) exists (if v[1] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1]};
                        else
                            w = {i, v[1], v[0]};
                        a = {v,w};
                        A.push_back(a);
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }
    for (const auto& v: V_out)
    {
        // transit from v[0]-n's drop-off location to another user i's pick-up location
        for (const auto& i: D.R)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...) and (i,...,v[2],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2]};
                            a = {v,w};
                            A.push_back(a);
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                } 
            }
        }

        // transit from v[0]'s drop-off location to another user's drop-off location
        if (v[1] != 0)
        {
            // check if node (n + v[1], ..., v[2], ...) exists 
            // i = v[1], j = v[2]
            if (f[v[1]][v[2]][1] || f[v[2]][v[1]][0])
            {
                w = {n + v[1], v[2], 0};
                a = {v,w};
                A.push_back(a);
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }

            if (v[2] != 0)
            {
                // check if node (n + v[2], ..., v[1], ...) exists 
                // i = v[2], j = v[1]
                if (f[v[2]][v[1]][1]||f[v[1]][v[2]][0])
                {
                    w = {n + v[2], v[1], 0};
                    a = {v,w};
                    A.push_back(a);
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }
            }
        }
    }
    
    // for each v create a vector of all arcs that start/ end in node v 
    for (const auto& a: A)
    {
        delta_out[a[0]].push_back(a);
        delta_in[a[1]].push_back(a);  
    }
}




template <>
void DARPGraph<6>::create_arcs(DARP& D, int*** f)
{
    NODE u, w;
    ARC a;
    
    for (const auto& i: D.R)
    {
        // (0,0,0,0,0,0) --> (i,0,0,0,0,0)
        u = {0,0,0,0,0,0};
        w = {i,0,0,0,0,0};
        a = {u,w};
        A.push_back(a);
        c[a] = D.d[0][i];
        t[a] = D.tt[0][i];

        // (n+i,0,0,0,0,0) --> (0,0,0,0,0,0)
        w = {n+i,0,0,0,0,0};
        a = {w,u};
        A.push_back(a);
        c[a] = D.d[n+i][0];
        t[a] = D.tt[n+i][0];
    }

    for (const auto& v: V_in)
    {
        // transit from v[0]'s pick-up location to another user's drop-off location (including v[0])
        
        // (i,...,j,...) --> (n+i,...,j,...)
        // if j -- i -- n+i -- n+j is feasible for j = v[1], v[2], v[3], v[4], v[5]
        if (f[v[0]][v[1]][1] && f[v[0]][v[2]][1] && f[v[0]][v[3]][1] && f[v[0]][v[4]][1] && f[v[0]][v[5]][1])
        {
            w = {n + v[0], v[1], v[2], v[3], v[4], v[5]};
            a = {v,w};
            A.push_back(a);
            c[a] = D.d[v[0]][n + v[0]];
            t[a] = D.tt[v[0]][n + v[0]];
        }

        // (i,...,j,...) --> (n+j,...,i,...)
        if (v[1] != 0)
        {
            // if v[1] -- v[0] -- n+v[1] -- n+v[0] is feasible 
            // (and v[1] -- v[2] -- n+v[1] -- n+v[2] OR  v[2] -- v[1] -- n+v[1] -- n+v[2] is feasible) : i = v[1] j = v[2] f[i][j][1]||f[j][i][0]
            // and so on for v[3],...,v[5]
            if (f[v[0]][v[1]][0] && (f[v[1]][v[2]][1]||f[v[2]][v[1]][0]) && (f[v[1]][v[3]][1]||f[v[3]][v[1]][0]) && (f[v[1]][v[4]][1]||f[v[4]][v[1]][0]) && (f[v[1]][v[5]][1]||f[v[5]][v[1]][0]))
            {
                if (v[0] > v[2])
                    w = {n + v[1], v[0], v[2], v[3], v[4], v[5]};
                else if (v[2] > v[0] && v[0] > v[3])
                    w = {n + v[1], v[2], v[0], v[3], v[4], v[5]};
                else if (v[3] > v[0] && v[0] > v[4])
                    w = {n + v[1], v[2], v[3], v[0], v[4], v[5]};
                else if (v[4] > v[0] && v[0] > v[5])
                    w = {n + v[1], v[2], v[3], v[4], v[0], v[5]};
                else
                    w = {n + v[1], v[2], v[3], v[4], v[5], v[0]};
                a = {v,w};
                A.push_back(a);
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }

            if (v[2] != 0)
            {
                if (f[v[0]][v[2]][0] && (f[v[2]][v[1]][1]||f[v[1]][v[2]][0]) && (f[v[2]][v[3]][1]||f[v[3]][v[2]][0]) && (f[v[2]][v[4]][1]||f[v[4]][v[2]][0]) && (f[v[2]][v[5]][1]||f[v[5]][v[2]][0]))
                {
                    if (v[0] > v[1])
                        w = {n + v[2], v[0], v[1], v[3], v[4], v[5]};
                    else if (v[1] > v[0] && v[0] > v[3])
                        w = {n + v[2], v[1], v[0], v[3], v[4], v[5]};
                    else if (v[3] > v[0] && v[0] > v[4])
                        w = {n + v[2], v[1], v[3], v[0], v[4], v[5]};
                    else if (v[4] > v[0] && v[0] > v[5])
                        w = {n + v[2], v[1], v[3], v[4], v[0], v[5]};
                    else
                        w = {n + v[2], v[1], v[3], v[4], v[5], v[0]};
                    a = {v,w};
                    A.push_back(a);
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }

                if (v[3] != 0)
                {
                    if (f[v[0]][v[3]][0] && (f[v[3]][v[1]][1]||f[v[1]][v[3]][0]) && (f[v[3]][v[2]][1]||f[v[2]][v[3]][0]) && (f[v[3]][v[4]][1]||f[v[4]][v[3]][0]) && (f[v[3]][v[5]][1]||f[v[5]][v[3]][0]))
                    {
                        if (v[0] > v[1])
                            w = {n + v[3], v[0], v[1], v[2], v[4], v[5]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {n + v[3], v[1], v[0], v[2], v[4], v[5]};
                        else if (v[2] > v[0] && v[0] > v[4])
                            w = {n + v[3], v[1], v[2], v[0], v[4], v[5]};
                        else if (v[4] > v[0] && v[0] > v[5])
                            w = {n + v[3], v[1], v[2], v[4], v[0], v[5]};
                        else
                            w = {n + v[3], v[1], v[2], v[4], v[5], v[0]};
                        a = {v,w};
                        A.push_back(a);
                        c[a] = D.d[v[0]][n + v[3]];
                        t[a] = D.tt[v[0]][n + v[3]];
                    }

                    if (v[4] != 0)
                    {
                        if (f[v[0]][v[4]][0] && (f[v[4]][v[1]][1]||f[v[1]][v[4]][0]) && (f[v[4]][v[2]][1]||f[v[2]][v[4]][0]) && (f[v[4]][v[3]][1]||f[v[3]][v[4]][0]) && (f[v[4]][v[5]][1]||f[v[5]][v[4]][0]))
                        {
                            if (v[0] > v[1])
                                w = {n + v[4], v[0], v[1], v[2], v[3], v[5]};
                            else if (v[1] > v[0] && v[0] > v[2])
                                w = {n + v[4], v[1], v[0], v[2], v[3], v[5]};
                            else if (v[2] > v[0] && v[0] > v[3])
                                w = {n + v[4], v[1], v[2], v[0], v[3], v[5]};
                            else if (v[3] > v[0] && v[0] > v[5])
                                w = {n + v[4], v[1], v[2], v[3], v[0], v[5]};
                            else
                                w = {n + v[4], v[1], v[2], v[3], v[5], v[0]};
                            a = {v,w};
                            A.push_back(a);
                            c[a] = D.d[v[0]][n + v[4]];
                            t[a] = D.tt[v[0]][n + v[4]];
                        }

                        if (v[5] != 0)
                        {
                            if (f[v[0]][v[5]][0] && (f[v[5]][v[1]][1]||f[v[1]][v[5]][0]) && (f[v[5]][v[2]][1]||f[v[2]][v[5]][0]) && (f[v[5]][v[3]][1]||f[v[3]][v[5]][0]) && (f[v[5]][v[4]][1]||f[v[4]][v[5]][0]))
                            {
                                if (v[0] > v[1])
                                    w = {n + v[5], v[0], v[1], v[2], v[3], v[4]};
                                else if (v[1] > v[0] && v[0] > v[2])
                                    w = {n + v[5], v[1], v[0], v[2], v[3], v[4]};
                                else if (v[2] > v[0] && v[0] > v[3])
                                    w = {n + v[5], v[1], v[2], v[0], v[3], v[4]};
                                else if (v[3] > v[0] && v[0] > v[4])
                                    w = {n + v[5], v[1], v[2], v[3], v[0], v[4]};
                                else
                                    w = {n + v[5], v[1], v[2], v[3], v[4], v[0]};
                                a = {v,w};
                                A.push_back(a);
                                c[a] = D.d[v[0]][n + v[5]];
                                t[a] = D.tt[v[0]][n + v[5]];
                            }
                        }
                    }
                }
            }
        }

        // transit from v[0]'s pick-up location to another user's pick-up location
        if (v[5] == 0)
        {
            for (const auto& i: D.R)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[k],...) k = 1,...,4 exists (if v[k] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1], v[2], v[3], v[4]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {i, v[1], v[0], v[2], v[3], v[4]};
                        else if (v[2] > v[0] && v[0] > v[3])
                            w = {i, v[1], v[2], v[0], v[3], v[4]};
                        else if (v[3] > v[0] && v[0] > v[4])
                            w = {i, v[1], v[2], v[3], v[0], v[4]};
                        else 
                            w = {i, v[1], v[2], v[3], v[4], v[0]};
                        a = {v,w};
                        A.push_back(a);
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }
    for (const auto& v: V_out)
    {
        // transit from v[0]'s drop-off location to another user i's pick-up location
        for (const auto& i: D.R)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (i != v[5]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...), (i,...,v[2],...), (i,...,v[3],...), (i,...,v[4],...) and (i,...,v[5],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]) && (f[i][v[5]][0] || f[i][v[5]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand + D.nodes[v[5]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2], v[3], v[4], v[5]};
                            a = {v,w};
                            A.push_back(a);
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                }
            }
        }

        // transit from v[0]'s drop-off location to another user's drop-off location
        if (v[1] != 0)
        {
            // check if nodes (n + v[1], ..., v[2], ...), (n + v[1], ..., v[3], ...), (n + v[1], ..., v[4], ...) and (n + v[1], ..., v[5], ...) exist 
            // i = v[1], j = v[2], v[3], v[4], v[5]
            if ((f[v[1]][v[2]][1]||f[v[2]][v[1]][0]) && (f[v[1]][v[3]][1]||f[v[3]][v[1]][0]) && (f[v[1]][v[4]][1]||f[v[4]][v[1]][0]) && (f[v[1]][v[5]][1]||f[v[5]][v[1]][0]))
            {
                w = {n + v[1], v[2], v[3], v[4], v[5] ,0};
                a = {v,w};
                A.push_back(a);
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }

            if (v[2] != 0)
            {
                // check if nodes (n + v[2], ..., v[1], ...), (n + v[2], ..., v[3], ...), (n + v[2], ..., v[4], ...) and (n + v[2], ..., v[5], ...) exist
                // i = v[2], j = v[1], v[3], v[4], v[5]
                if ((f[v[2]][v[1]][1]||f[v[1]][v[2]][0]) && (f[v[2]][v[3]][1]||f[v[3]][v[2]][0]) && (f[v[2]][v[4]][1]||f[v[4]][v[2]][0]) && (f[v[2]][v[5]][1]||f[v[5]][v[2]][0]))
                {
                    w = {n + v[2], v[1], v[3], v[4], v[5], 0};
                    a = {v,w};
                    A.push_back(a);
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }

                if (v[3] != 0)
                {
                    // check if nodes (n + v[3], ..., v[1], ...), (n + v[3], ..., v[2], ...), (n + v[3], ..., v[4], ...) and (n + v[3], ..., v[5], ...) exist
                    // i = v[3], j = v[1], v[2], v[4], v[5]
                    if ((f[v[3]][v[1]][1]||f[v[1]][v[3]][0]) && (f[v[3]][v[2]][1]||f[v[2]][v[3]][0]) && (f[v[3]][v[4]][1]||f[v[4]][v[3]][0]) && (f[v[3]][v[5]][1]||f[v[5]][v[3]][0]))
                    {
                        w = {n + v[3], v[1], v[2], v[4], v[5], 0};
                        a = {v,w};
                        A.push_back(a);
                        c[a] = D.d[v[0]][n + v[3]];
                        t[a] = D.tt[v[0]][n + v[3]];
                    }

                    if (v[4] != 0)
                    {
                        // check if nodes (n + v[4], ..., v[1], ...), (n + v[4], ..., v[2], ...), (n + v[4], ..., v[3], ...) and (n + v[4], ..., v[5], ...) exist
                        // i = v[4], j = v[1], v[2], v[3], v[5]
                        if ((f[v[4]][v[1]][1]||f[v[1]][v[4]][0]) && (f[v[4]][v[2]][1]||f[v[2]][v[4]][0]) && (f[v[4]][v[3]][1]||f[v[3]][v[4]][0]) && (f[v[4]][v[5]][1]||f[v[5]][v[4]][0]))
                        {
                            w = {n + v[4], v[1], v[2], v[3], v[5], 0};
                            a = {v,w};
                            A.push_back(a);
                            c[a] = D.d[v[0]][n + v[4]];
                            t[a] = D.tt[v[0]][n + v[4]];
                        }

                        if (v[5] != 0)
                        {
                            // check if nodes (n + v[5], ..., v[1], ...), (n + v[5], ..., v[2], ...), (n + v[5], ..., v[3], ...) and (n + v[5], ..., v[4], ...) exist
                            // i = v[5], j = v[1], v[2], v[3], v[4]
                            if ((f[v[5]][v[1]][1]||f[v[1]][v[5]][0]) && (f[v[5]][v[2]][1]||f[v[2]][v[5]][0]) && (f[v[5]][v[3]][1]||f[v[3]][v[5]][0]) && (f[v[5]][v[4]][1]||f[v[4]][v[5]][0]))
                            {
                                w = {n + v[5], v[1], v[2], v[3], v[4], 0};
                                a = {v,w};
                                A.push_back(a);
                                c[a] = D.d[v[0]][n + v[5]];
                                t[a] = D.tt[v[0]][n + v[5]];
                            }
                        }
                    }
                }
            }
        }
    }
    
    // for each v create a vector of all arcs that start/ end in node v 
    for (const auto& a: A)
    {
        delta_out[a[0]].push_back(a);
        delta_in[a[1]].push_back(a);  
    }
}



template <>
void DARPGraph<3>::create_new_arcs(DARP& D, int*** f, const std::vector<int>& new_requests, const std::vector<int>& all_seekers)
{
    NODE u,w;
    ARC a;

    num_new_arcs = 0;
    // acardinality is updated at the end
    
    u = {0,0,0};
    for (const auto & i : new_requests)
    {
        w = {i,0,0};
        a = {u,w};
        A_new.push_back(a);
        num_new_arcs++;
        c[a] = D.d[0][i];
        t[a] = D.tt[0][i];

        w = {n + i,0,0};
        a = {w,u};
        A_new.push_back(a);
        num_new_arcs++;
        c[a] = D.d[n+i][0];
        t[a] = D.tt[n+i][0];
    }
    

    
    for (const auto& v: V_in_new)
    {
        // new V_in nodes -> all possible drop-off nodes
        // transit from v[0]'s pick-up location to another user's drop-off location (including v[0])

        // (i,...,j,...) --> (n+i,...,j,...)
        // if j -- i -- n+i -- n+j is feasible for j = v[1], v[2]
        if (f[v[0]][v[1]][1] && f[v[0]][v[2]][1])
        {
            w = {n + v[0], v[1], v[2]};
            a = {v,w};
            A_new.push_back(a);
            num_new_arcs++;
            c[a] = D.d[v[0]][n + v[0]];
            t[a] = D.tt[v[0]][n + v[0]];
        }

        // (i,...,j,...) --> (n+j,...,i,...)
        if (v[1] != 0)
        {
            // if v[1] -- v[0] -- n+v[1] -- n+v[0] is feasible 
            // (and v[1] -- v[2] -- n+v[1] -- n+v[2] OR  v[2] -- v[1] -- n+v[1] -- n+v[2] is feasible) : i = v[1] j = v[2] f[i][j][1]||f[j][i][0]
            if (f[v[0]][v[1]][0] && (f[v[1]][v[2]][1]||f[v[2]][v[1]][0]))
            {
                if (v[0] > v[2])
                    w = {n + v[1], v[0], v[2]};
                else
                    w = {n + v[1], v[2], v[0]};
                a = {v,w};
                A_new.push_back(a);
                num_new_arcs++;
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }
                
            if (v[2] != 0)
            {
                if (f[v[0]][v[2]][0] && (f[v[2]][v[1]][1]||f[v[1]][v[2]][0]))
                {
                    if (v[0] > v[1])
                        w = {n + v[2], v[0], v[1]};
                    else
                        w = {n + v[2], v[1], v[0]};
                    a = {v,w};
                    A_new.push_back(a);
                    num_new_arcs++;
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }
            }
        }

        // new V_in nodes -> all possible other pick-up nodes of all_seekers and other new_requests
        // transit from v[0]'s pick-up location to another user's pick-up location
        if (v[2] == 0)
        {
            for (const auto& i: all_seekers)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[1],...) exists (if v[1] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1]};
                        else
                            w = {i, v[1], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
            for (const auto& i: new_requests)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[1],...) exists (if v[1] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1]};
                        else
                            w = {i, v[1], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }
    
    // old V_in nodes -> pick-up of new_request
    for (const auto& v: V_in)
    {
        // transit from v[0]'s pick-up location to another user's (a new_request's) pick-up location
        if (v[2] == 0)
        { 
            for (const auto & i : new_requests)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[1],...) exists (if v[1] == 0 too this will be feasible anyway)
                if ((f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1]};
                        else
                            w = {i, v[1], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }

    
    for (const auto& v: V_out_new)
    {
        // new V_out nodes -> all possible pick-ups of all_seekers and new_requests
        // transit from v[0]'s drop-off location to another user i's pick-up location
        for (const auto& i: all_seekers)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...) and (i,...,v[2],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2]};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                }
            }
        }
        for (const auto& i: new_requests)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...) and (i,...,v[2],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2]};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                }
            }
        }
        
        // * ingoing arc to old V_out node if first entry = n+i, where i new request
        // * number of outgoing arcs depends on the number of drop-off nodes of the new request 
        // * number of V_out nodes depends on with how many requests v[1], v[2] the drop-off of i is compatible  
        // new V_out nodes -> drop-offs outgoing from v 
        // transit from v[0]'s drop-off location to another user's drop-off location
        if (v[1] != 0)
        {
            // check if node (n + v[1], ..., v[2], ...) exists 
            // i = v[1], j = v[2]
            if (f[v[1]][v[2]][1] || f[v[2]][v[1]][0])
            {
                w = {n + v[1], v[2], 0};
                a = {v,w};
                A_new.push_back(a);
                num_new_arcs++;
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }

            if (v[2] != 0)
            {
                // check if node (n + v[2], ..., v[1], ...) exists 
                // i = v[2], j = v[1]
                if (f[v[2]][v[1]][1]||f[v[1]][v[2]][0])
                {
                    w = {n + v[2], v[1], 0};
                    a = {v,w};
                    A_new.push_back(a);
                    num_new_arcs++;
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }
            }
        } 
    }
    
    // * number of V_out nodes depends on with how many v[1], v[2] the new request is compatible
    // old V_out nodes -> all pick-ups with new_request
    for (const auto& v: V_out)
    {  
        for (const auto & i : new_requests)
        {
            // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
            if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
            {
                // check if nodes (i,...,v[1],...) and (i,...,v[2],...) exist
                if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand <= D.veh_capacity)
                    {    
                        w = {i, v[1], v[2]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }

    // add new arcs to delta_in, delta_out
    for (const auto& a: A_new)
    {
        delta_out_new[a[0]].push_back(a);
        delta_in_new[a[1]].push_back(a);  
    }


}

template <>
void DARPGraph<6>::create_new_arcs(DARP& D, int*** f, const std::vector<int> &new_requests,  const std::vector<int>& all_seekers)
{
    NODE u,w;
    ARC a;

    num_new_arcs = 0;
    // acardinality is updated at the end
    
    u = {0,0,0,0,0,0};
    for (const auto & i : new_requests)
    {
        w = {i,0,0,0,0,0};
        a = {u,w};
        A_new.push_back(a);
        num_new_arcs++;
        c[a] = D.d[0][i];
        t[a] = D.tt[0][i];

        w = {n + i,0,0,0,0,0};
        a = {w,u};
        A_new.push_back(a);
        num_new_arcs++;
        c[a] = D.d[n+i][0];
        t[a] = D.tt[n+i][0];
    }
    

    
    for (const auto& v: V_in_new)
    {
        // new V_in nodes -> all possible drop-off nodes
        // transit from v[0]'s pick-up location to another user's drop-off location (including v[0])

        // (i,...,j,...) --> (n+i,...,j,...)
        // if j -- i -- n+i -- n+j is feasible for j = v[1], v[2], v[3], v[4], v[5]
        if (f[v[0]][v[1]][1] && f[v[0]][v[2]][1] && f[v[0]][v[3]][1] && f[v[0]][v[4]][1] && f[v[0]][v[5]][1])
        {
            w = {n + v[0], v[1], v[2], v[3], v[4], v[5]};
            a = {v,w};
            A_new.push_back(a);
            num_new_arcs++;
            c[a] = D.d[v[0]][n + v[0]];
            t[a] = D.tt[v[0]][n + v[0]];
        }

        // (i,...,j,...) --> (n+j,...,i,...)
        if (v[1] != 0)
        {
            // if v[1] -- v[0] -- n+v[1] -- n+v[0] is feasible 
            // (and v[1] -- v[2] -- n+v[1] -- n+v[2] OR  v[2] -- v[1] -- n+v[1] -- n+v[2] is feasible) : i = v[1] j = v[2] f[i][j][1]||f[j][i][0]
            // and so on for v[3],...,v[5]
            if (f[v[0]][v[1]][0] && (f[v[1]][v[2]][1]||f[v[2]][v[1]][0]) && (f[v[1]][v[3]][1]||f[v[3]][v[1]][0]) && (f[v[1]][v[4]][1]||f[v[4]][v[1]][0]) && (f[v[1]][v[5]][1]||f[v[5]][v[1]][0]))
            {
                if (v[0] > v[2])
                    w = {n + v[1], v[0], v[2], v[3], v[4], v[5]};
                else if (v[2] > v[0] && v[0] > v[3])
                    w = {n + v[1], v[2], v[0], v[3], v[4], v[5]};
                else if (v[3] > v[0] && v[0] > v[4])
                    w = {n + v[1], v[2], v[3], v[0], v[4], v[5]};
                else if (v[4] > v[0] && v[0] > v[5])
                    w = {n + v[1], v[2], v[3], v[4], v[0], v[5]};
                else
                    w = {n + v[1], v[2], v[3], v[4], v[5], v[0]};
                a = {v,w};
                A_new.push_back(a);
                num_new_arcs++;
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }
            
            if (v[2] != 0)
            {
                if (f[v[0]][v[2]][0] && (f[v[2]][v[1]][1]||f[v[1]][v[2]][0]) && (f[v[2]][v[3]][1]||f[v[3]][v[2]][0]) && (f[v[2]][v[4]][1]||f[v[4]][v[2]][0]) && (f[v[2]][v[5]][1]||f[v[5]][v[2]][0]))
                {
                    if (v[0] > v[1])
                        w = {n + v[2], v[0], v[1], v[3], v[4], v[5]};
                    else if (v[1] > v[0] && v[0] > v[3])
                        w = {n + v[2], v[1], v[0], v[3], v[4], v[5]};
                    else if (v[3] > v[0] && v[0] > v[4])
                        w = {n + v[2], v[1], v[3], v[0], v[4], v[5]};
                    else if (v[4] > v[0] && v[0] > v[5])
                        w = {n + v[2], v[1], v[3], v[4], v[0], v[5]};
                    else
                        w = {n + v[2], v[1], v[3], v[4], v[5], v[0]};
                    a = {v,w};
                    A_new.push_back(a);
                    num_new_arcs++;
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }

                if (v[3] != 0)
                {
                    if (f[v[0]][v[3]][0] && (f[v[3]][v[1]][1]||f[v[1]][v[3]][0]) && (f[v[3]][v[2]][1]||f[v[2]][v[3]][0]) && (f[v[3]][v[4]][1]||f[v[4]][v[3]][0]) && (f[v[3]][v[5]][1]||f[v[5]][v[3]][0]))
                    {
                        if (v[0] > v[1])
                            w = {n + v[3], v[0], v[1], v[2], v[4], v[5]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {n + v[3], v[1], v[0], v[2], v[4], v[5]};
                        else if (v[2] > v[0] && v[0] > v[4])
                            w = {n + v[3], v[1], v[2], v[0], v[4], v[5]};
                        else if (v[4] > v[0] && v[0] > v[5])
                            w = {n + v[3], v[1], v[2], v[4], v[0], v[5]};
                        else
                            w = {n + v[3], v[1], v[2], v[4], v[5], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][n + v[3]];
                        t[a] = D.tt[v[0]][n + v[3]];
                    }

                    if (v[4] != 0)
                    {
                        if (f[v[0]][v[4]][0] && (f[v[4]][v[1]][1]||f[v[1]][v[4]][0]) && (f[v[4]][v[2]][1]||f[v[2]][v[4]][0]) && (f[v[4]][v[3]][1]||f[v[3]][v[4]][0]) && (f[v[4]][v[5]][1]||f[v[5]][v[4]][0]))
                        {
                            if (v[0] > v[1])
                                w = {n + v[4], v[0], v[1], v[2], v[3], v[5]};
                            else if (v[1] > v[0] && v[0] > v[2])
                                w = {n + v[4], v[1], v[0], v[2], v[3], v[5]};
                            else if (v[2] > v[0] && v[0] > v[3])
                                w = {n + v[4], v[1], v[2], v[0], v[3], v[5]};
                            else if (v[3] > v[0] && v[0] > v[5])
                                w = {n + v[4], v[1], v[2], v[3], v[0], v[5]};
                            else
                                w = {n + v[4], v[1], v[2], v[3], v[5], v[0]};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][n + v[4]];
                            t[a] = D.tt[v[0]][n + v[4]];
                        }

                        if (v[5] != 0)
                        {
                            if (f[v[0]][v[5]][0] && (f[v[5]][v[1]][1]||f[v[1]][v[5]][0]) && (f[v[5]][v[2]][1]||f[v[2]][v[5]][0]) && (f[v[5]][v[3]][1]||f[v[3]][v[5]][0]) && (f[v[5]][v[4]][1]||f[v[4]][v[5]][0]))
                            {
                                if (v[0] > v[1])
                                    w = {n + v[5], v[0], v[1], v[2], v[3], v[4]};
                                else if (v[1] > v[0] && v[0] > v[2])
                                    w = {n + v[5], v[1], v[0], v[2], v[3], v[4]};
                                else if (v[2] > v[0] && v[0] > v[3])
                                    w = {n + v[5], v[1], v[2], v[0], v[3], v[4]};
                                else if (v[3] > v[0] && v[0] > v[4])
                                    w = {n + v[5], v[1], v[2], v[3], v[0], v[4]};
                                else
                                    w = {n + v[5], v[1], v[2], v[3], v[4], v[0]};
                                a = {v,w};
                                A_new.push_back(a);
                                num_new_arcs++;
                                c[a] = D.d[v[0]][n + v[5]];
                                t[a] = D.tt[v[0]][n + v[5]];
                            }
                        }
                    }
                }
            }
        }

        // new V_in nodes -> all possible pick-up nodes of all_seekers and other new_requests
        // transit from v[0]'s pick-up location to another user's pick-up location
        if (v[5] == 0)
        {
            for (const auto& i: all_seekers)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[k],...) for k = 1,...,4 exists (if v[k] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1], v[2], v[3], v[4]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {i, v[1], v[0], v[2], v[3], v[4]};
                        else if (v[2] > v[0] && v[0] > v[3])
                            w = {i, v[1], v[2], v[0], v[3], v[4]};
                        else if (v[3] > v[0] && v[0] > v[4])
                            w = {i, v[1], v[2], v[3], v[0], v[4]};
                        else
                            w = {i, v[1], v[2], v[3], v[4], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
            for (const auto& i: new_requests)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[k],...) for k = 1,...,4 exists (if v[k] == 0 too this will be feasible anyway)
                if ((i != v[0]) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1], v[2], v[3], v[4]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {i, v[1], v[0], v[2], v[3], v[4]};
                        else if (v[2] > v[0] && v[0] > v[3])
                            w = {i, v[1], v[2], v[0], v[3], v[4]};
                        else if (v[3] > v[0] && v[0] > v[4])
                            w = {i, v[1], v[2], v[3], v[0], v[4]};
                        else
                            w = {i, v[1], v[2], v[3], v[4], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }
     
    // old V_in nodes -> pick-up of new_request
    for (const auto& v: V_in)
    {
        // transit from v[0]'s pick-up location to another user's (a new_request's) pick-up location
        if (v[5] == 0)
        { 
            for (const auto & i : new_requests)
            {
                // check if node (i,...,v[0],...) exists
                // and if (i,...,v[k],...) for k = 1,...,4 exists (if v[k] == 0 too this will be feasible anyway)
                if((f[i][v[0]][0] || f[i][v[0]][1]) && (f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[0]].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand <= D.veh_capacity)
                    {
                        if (v[0] > v[1])
                            w = {i, v[0], v[1], v[2], v[3], v[4]};
                        else if (v[1] > v[0] && v[0] > v[2])
                            w = {i, v[1], v[0], v[2], v[3], v[4]};
                        else if (v[2] > v[0] && v[0] > v[3])
                            w = {i, v[1], v[2], v[0], v[3], v[4]};
                        else if (v[3] > v[0] && v[0] > v[4])
                            w = {i, v[1], v[2], v[3], v[0], v[4]};
                        else
                            w = {i, v[1], v[2], v[3], v[4], v[0]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }

    
    for (const auto& v: V_out_new)
    {
        // new V_out nodes -> all possible pick-ups of all_seekers and new_requests
        // transit from v[0]'s drop-off location to another user i's pick-up location
        for (const auto& i: all_seekers)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (i != v[5]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...), (i,...,v[2],...), (i,...,v[3],...), (i,...,v[4],...) and (i,...,v[5],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]) && (f[i][v[5]][0] || f[i][v[5]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand + D.nodes[v[5]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2], v[3], v[4], v[5]};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                }
            }
        }
        for (const auto& i: new_requests)
        {
            if ((i != (v[0]-n)) && (i != v[1]) && (i != v[2]) && (i != v[3]) && (i != v[4]) && (i != v[5]))
            {
                // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
                if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
                {
                    // check if nodes (i,...,v[1],...), (i,...,v[2],...), (i,...,v[3],...), (i,...,v[4],...) and (i,...,v[5],...) exist
                    if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]) && (f[i][v[5]][0] || f[i][v[5]][1]))
                    {
                        if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand + D.nodes[v[5]].demand <= D.veh_capacity)
                        {    
                            w = {i, v[1], v[2], v[3], v[4], v[5]};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][i];
                            t[a] = D.tt[v[0]][i];
                        }
                    }
                }
            }
        }

        // new V_out nodes -> drop-offs out from v 
        // transit from v[0]'s drop-off location to another user's drop-off location
        if (v[1] != 0)
        {
            // check if nodes (n + v[1], ..., v[2], ...), (n + v[1], ..., v[3], ...), (n + v[1], ..., v[4], ...) and (n + v[1], ..., v[5], ...) exist 
            // i = v[1], j = v[2], v[3], v[4], v[5]
            if ((f[v[1]][v[2]][1]||f[v[2]][v[1]][0]) && (f[v[1]][v[3]][1]||f[v[3]][v[1]][0]) && (f[v[1]][v[4]][1]||f[v[4]][v[1]][0]) && (f[v[1]][v[5]][1]||f[v[5]][v[1]][0]))
            {
                w = {n + v[1], v[2], v[3], v[4], v[5] ,0};
                a = {v,w};
                A_new.push_back(a);
                num_new_arcs++;
                c[a] = D.d[v[0]][n + v[1]];
                t[a] = D.tt[v[0]][n + v[1]];
            }

            if (v[2] != 0)
            {
                // check if nodes (n + v[2], ..., v[1], ...), (n + v[2], ..., v[3], ...), (n + v[2], ..., v[4], ...) and (n + v[2], ..., v[5], ...) exist
                // i = v[2], j = v[1], v[3], v[4], v[5]
                if ((f[v[2]][v[1]][1]||f[v[1]][v[2]][0]) && (f[v[2]][v[3]][1]||f[v[3]][v[2]][0]) && (f[v[2]][v[4]][1]||f[v[4]][v[2]][0]) && (f[v[2]][v[5]][1]||f[v[5]][v[2]][0]))
                {
                    w = {n + v[2], v[1], v[3], v[4], v[5], 0};
                    a = {v,w};
                    A_new.push_back(a);
                    num_new_arcs++;
                    c[a] = D.d[v[0]][n + v[2]];
                    t[a] = D.tt[v[0]][n + v[2]];
                }

                if (v[3] != 0)
                {
                    // check if nodes (n + v[3], ..., v[1], ...), (n + v[3], ..., v[2], ...), (n + v[3], ..., v[4], ...) and (n + v[3], ..., v[5], ...) exist
                    // i = v[3], j = v[1], v[2], v[4], v[5]
                    if ((f[v[3]][v[1]][1]||f[v[1]][v[3]][0]) && (f[v[3]][v[2]][1]||f[v[2]][v[3]][0]) && (f[v[3]][v[4]][1]||f[v[4]][v[3]][0]) && (f[v[3]][v[5]][1]||f[v[5]][v[3]][0]))
                    {
                        w = {n + v[3], v[1], v[2], v[4], v[5], 0};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][n + v[3]];
                        t[a] = D.tt[v[0]][n + v[3]];
                    }

                    if (v[4] != 0)
                    {
                        // check if nodes (n + v[4], ..., v[1], ...), (n + v[4], ..., v[2], ...), (n + v[4], ..., v[3], ...) and (n + v[4], ..., v[5], ...) exist
                        // i = v[4], j = v[1], v[2], v[3], v[5]
                        if ((f[v[4]][v[1]][1]||f[v[1]][v[4]][0]) && (f[v[4]][v[2]][1]||f[v[2]][v[4]][0]) && (f[v[4]][v[3]][1]||f[v[3]][v[4]][0]) && (f[v[4]][v[5]][1]||f[v[5]][v[4]][0]))
                        {
                            w = {n + v[4], v[1], v[2], v[3], v[5], 0};
                            a = {v,w};
                            A_new.push_back(a);
                            num_new_arcs++;
                            c[a] = D.d[v[0]][n + v[4]];
                            t[a] = D.tt[v[0]][n + v[4]];
                        }

                        if (v[5] != 0)
                        {
                            // check if nodes (n + v[5], ..., v[1], ...), (n + v[5], ..., v[2], ...), (n + v[5], ..., v[3], ...) and (n + v[5], ..., v[4], ...) exist
                            // i = v[5], j = v[1], v[2], v[3], v[4]
                            if ((f[v[5]][v[1]][1]||f[v[1]][v[5]][0]) && (f[v[5]][v[2]][1]||f[v[2]][v[5]][0]) && (f[v[5]][v[3]][1]||f[v[3]][v[5]][0]) && (f[v[5]][v[4]][1]||f[v[4]][v[5]][0]))
                            {
                                w = {n + v[5], v[1], v[2], v[3], v[4], 0};
                                a = {v,w};
                                A_new.push_back(a);
                                num_new_arcs++;
                                c[a] = D.d[v[0]][n + v[5]];
                                t[a] = D.tt[v[0]][n + v[5]];
                            }
                        }
                    }
                }
            }
        } 
    }
    
    // old V_out nodes -> all pick-ups with new_request
    for (const auto& v: V_out)
    {  
        for (const auto & i : new_requests)
        {
            // check if pick-up after drop-off is feasible e_{n+j} + s_j + t_{n+j,i} < l_i
            if (D.nodes[v[0]].start_tw + D.nodes[v[0]].service_time + D.tt[v[0]][i] <= D.nodes[i].end_tw)
            {
                // check if nodes (i,...,v[1],...), (i,...,v[2],...), (i,...,v[3],...), (i,...,v[4],...) and (i,...,v[5],...) exist
                if ((f[i][v[1]][0] || f[i][v[1]][1]) && (f[i][v[2]][0] || f[i][v[2]][1]) && (f[i][v[3]][0] || f[i][v[3]][1]) && (f[i][v[4]][0] || f[i][v[4]][1]) && (f[i][v[5]][0] || f[i][v[5]][1]))
                {
                    if (D.nodes[i].demand + D.nodes[v[1]].demand + D.nodes[v[2]].demand + D.nodes[v[3]].demand + D.nodes[v[4]].demand + D.nodes[v[5]].demand <= D.veh_capacity)
                    {    
                        w = {i, v[1], v[2], v[3], v[4], v[5]};
                        a = {v,w};
                        A_new.push_back(a);
                        num_new_arcs++;
                        c[a] = D.d[v[0]][i];
                        t[a] = D.tt[v[0]][i];
                    }
                }
            }
        }
    }
    // add new arcs to delta_in, delta_out
    for (const auto& a: A_new)
    {
        delta_out_new[a[0]].push_back(a);
        delta_in_new[a[1]].push_back(a);  
    }  
}

template <int Q>
void DARPGraph<Q>::create_graph(DARP& D, int*** f)
{
    create_nodes(D, f);
    create_arcs(D, f);
}


template class DARPGraph<3>;
template class DARPGraph<6>;