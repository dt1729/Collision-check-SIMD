#include "AABB.h"
#include "polygon.h"
#include <algorithm>
#include <parallel/algorithm>
#include <vector>

struct Environment{

    std::pair<float, float> _x_range, _y_range, _z_range;
    std::vector<polygon*> _objects;
    Environment(std::pair<float, float> xRange, std::pair<float, float> yRange, std::pair<float, float> zRange) : 
    _x_range(xRange), _y_range(yRange), _z_range(zRange){}

};

struct node{
    std::vector<polygon*> _objects;
    std::vector<node*> children;
    std::vector<polygon*> _x_sorted_objects, _y_sorted_objects, _z_sorted_objects;
    float _volume;
    AABB  _region;
    bool isLeaf;

    node(std::vector<polygon*> objects): _objects(objects)
    {
        _x_sorted_objects = _objects;
        _y_sorted_objects = _objects;
        _z_sorted_objects = _objects;

        __gnu_parallel::sort(_x_sorted_objects.begin(),
                             _x_sorted_objects.end(),
                             [](polygon* p1, polygon* p2){ return p1->_x_centroid > p2->_x_centroid;});

        __gnu_parallel::sort(_y_sorted_objects.begin(),
                             _y_sorted_objects.end(),
                             [](polygon* p1, polygon* p2){ return p1->_y_centroid > p2->_y_centroid;});

        __gnu_parallel::sort(_z_sorted_objects.begin(),
                             _z_sorted_objects.end(),
                             [](polygon* p1, polygon* p2){ return p1->_z_centroid > p2->_z_centroid;});
    }
};

class BVH{
    private: 
        int n; 
        __m256 endResult; 
        Environment env;
        std::vector<node> treeNodes; 
        enum SplitAxis {X, Y, Z} _split_axis;  

    public:
        void create_BVH(){

        }


    protected:
        void 
};
