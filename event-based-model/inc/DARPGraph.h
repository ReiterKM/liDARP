#ifndef _DARP_GRAPH_H
#define _DARP_GRAPH_H


template <int Q>
class DARPGraph {
    
private:
    typedef std::array<int,Q> NODE;
    typedef std::array<std::array<int,Q>,2> ARC;
    
    int n; // num_requests
    // Nodes
    NODE depot; 
    std::vector <NODE> V_in;
    std::vector <NODE> V_out;
    std::vector <NODE> V_in_new;
    std::vector <NODE> V_out_new;

    std::unordered_map <int,std::vector<NODE> > V_i;
    std::unordered_map <int,std::vector<NODE> > V_i_new; // new drop-off nodes corresponding to n+i
        
    // Arcs
    std::vector <ARC> A; // arc set
    std::vector <ARC> A_new;

    // costs and travel time arcs
    std::unordered_map <ARC,double,HashFunction<Q> > c; // cost == distance
    std::unordered_map <ARC,double,HashFunction<Q> > t;

    int num_new_arcs;
    int num_new_nodes;
    uint64_t vincardinality;
    uint64_t voutcardinality;
    uint64_t vcardinality;
    uint64_t acardinality;

    // for each v create a vector of all arcs that start/ end in node v 
    std::unordered_map <NODE,std::vector<ARC>,HashFunction<Q>> delta_in;
    std::unordered_map <NODE,std::vector<ARC>,HashFunction<Q>> delta_out;
    std::unordered_map <NODE,std::vector<ARC>,HashFunction<Q>> delta_in_new;
    std::unordered_map <NODE,std::vector<ARC>,HashFunction<Q>> delta_out_new;

public:
    DARPGraph(int);
    ~DARPGraph();
    // no copy/ move constructor or assignment/ move operator needed so far since not more than one instance created
    
    
    // create event-based graph
    void create_nodes(DARP&, int***);
    void create_arcs(DARP&, int***);
    void create_graph(DARP&, int***);

    void create_new_nodes(DARP&, int***, const std::vector<int> &);
    void create_new_arcs(DARP&, int***, const std::vector<int> &, const std::vector<int>&);
    
    template<int S>
    friend class RollingHorizon;
    friend class DARPSolver;

};

template <typename T, std::size_t N>
std::ostream& operator<<(std::ostream& os, std::array<T, N> const& v)
{
    os << "(";
    for (unsigned int i=0; i<N-1; ++i)
    {
        os << v[i] << ",";
    } 
    os << v[N-1] << ")";

    //std::for_each(begin(v1), end(v1), [&os](int val) {os << val << " "; });
    return os;
}

template <std::size_t N>
std::ostream& operator<<(std::ostream& os, std::array<std::array<int,N>,2> const& a)
{
    os << "((";
    for (unsigned int i=0; i<N-1; ++i)
    {
        os << a[0][i] << ",";
    } 
    os << a[0][N-1] << "),(";
    for (unsigned int i=0; i<N-1; ++i)
    {
        os << a[1][i] << ",";
    } 
    os << a[1][N-1] << "))";
    return os;
}

#endif