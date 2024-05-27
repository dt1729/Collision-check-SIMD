#include "AABB.h"
#include "polygon.h"
#include <algorithm>
#include <parallel/algorithm>
#include <vector>

struct Environment{
    public:
        std::vector<polygon*> _objects;
        
        std::pair<float, float> _x_range, _y_range, _z_range;

        Environment(std::pair<float, float> xRange, std::pair<float, float> yRange, std::pair<float, float> zRange) : 
        _x_range(xRange), _y_range(yRange), _z_range(zRange){}
};

struct node{
    std::vector<polygon*> _objects;
    std::vector<polygon*> _sorted_objects;

    std::vector<node*> children;
    
    enum SplitAxis {X, Y, Z} _split_axis;

    float _volume;
    AABB  _region;
    bool isLeaf;

    node(std::vector<polygon*> objects, SplitAxis ax): _objects(objects)
    {
        _sorted_objects = _objects;

        __gnu_parallel::sort(_sorted_objects.begin(),
                             _sorted_objects.end(),
                             [this, ax](polygon* p1, polygon* p2){ return _return_value(ax, p1) > _return_value(ax, p2);});
    }

    float _return_value(SplitAxis ax, polygon* p){
        switch (ax)
        {
            case X:
                return p->_x_centroid;
            case Y:
                return p->_y_centroid;
            case Z:
                return p->_z_centroid;
        }
    }
};

class BVH{
    private:
        int n; 
        __m256 endResult; 
        Environment env;
        std::vector<node> treeNodes;
        int branching = 4; 
        enum SplitAxis {X, Y, Z} _split_axis;  

    public:
        void create_BVH(){

            if(env._x_range > env._y_range && env._x_range > env._z_range){
                _split_axis = X;
            }
            else if(env._y_range > env._x_range && env._y_range > env._z_range){
                _split_axis = Y;
            }
            else if(env._z_range > env._x_range && env._z_range > env._y_range){
                _split_axis = Z;
            }

            node* BVHNode = new node(env._objects, _split_axis);
        }


    protected:
        void 
};
