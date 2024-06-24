#ifndef _HASH_FUNCTION_H
#define _HASH_FUNCTION_H



// mapping nodes and arcs with variables
template<int Q>
class HashFunction
{
    public:
        typedef std::array<int,Q> NODE;
        typedef std::array<std::array<int,Q>,2> ARC;
        size_t operator()(const NODE& p) const
        {
            std::hash<int> hasher;
            size_t h = 0;
            for (size_t i = 0; i < Q; ++i)
            {
                h = h * 31 + hasher(p[i]);
            }
            return h;
        }
        
        size_t operator()(const ARC& p) const
        {
            std::hash<int> hasher;
            size_t h = 0;
            for (size_t i = 0; i < Q; ++i)
            {
                h = h * 31 + hasher(p[0][i]);
            }
            for (size_t i = 0; i < Q; ++i)
            {
                h = h * 31 + hasher(p[1][i]);
            }
            return h;
        }
}; 










#endif