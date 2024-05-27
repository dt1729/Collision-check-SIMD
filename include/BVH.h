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
    float _x_range, _y_range, _z_range;

    float _volume;
    AABB  _region;
    bool isLeaf;

    node(std::vector<polygon*> objects, AABB bbox): 
    _objects(objects), _region(bbox)
    {
        _sorted_objects = _objects;

        if(_x_range > _y_range && _x_range > _z_range){
            _split_axis = X;
        }
        else if(_y_range > _x_range && _y_range > _z_range){
            _split_axis = Y;
        }
        else if(_z_range > _x_range && _z_range > _y_range){
            _split_axis = Z;
        }

        __gnu_parallel::sort(_sorted_objects.begin(),
                             _sorted_objects.end(),
                             [this](polygon* p1, polygon* p2){ return _return_value(_split_axis, p1) > _return_value(_split_axis, p2);});
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
        node* initialise_BVH(){

            node* BVHNode = new node(env._objects, _split_axis);
            return BVHNode
        }

        void create_BVH(node* root){

            /**
             * @brief Add branching nodes and call create_BVH recursively on them.
             * 
             */
            for(int i = 0; i < branching; i++){
                /*Create sub list of Objects
                  Create Bounding box for these objects {min(AABBs), max(AABBs)}
                 */
                node* childNode = new node(/*Objects in split axis range take objects from sorted array in root node*/, /*AABB bounding box for the complete set of objects, so min(AABBs), max(AABBs)*/);
                create_BVH(childNode);
                root->children.push_back(childNode);
            }
        }
};
