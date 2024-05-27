#include "AABB.h"
#include "polygon.h"
#include <algorithm>
#include <execution>
#include <vector>

struct Environment{

    std::pair<float, float> _x_range, _y_range, _z_range;
    std::vector<polygon> _objects;

    std::vector<polygon> _x_sorted_objects = _objects;
    std::vector<polygon> _y_sorted_objects = _objects;
    std::vector<polygon> _z_sorted_objects = _objects;

    Environment(){
        std::sort(std::execution::par_unseq,
                  _x_sorted_objects.begin(),
                  _x_sorted_objects.end(),
                  );

        std::sort(std::execution::par_unseq,
                  _y_sorted_objects.begin(),
                  _y_sorted_objects.end());

        std::sort(std::execution::par_unseq,
                  _z_sorted_objects.begin(),
                  _z_sorted_objects.end());
    }

};

struct node{
    std::vector<polygon> _shapes;
    std::vector<node*> children;
    float _volume;
    AABB  _region;
    bool isLeaf;
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
