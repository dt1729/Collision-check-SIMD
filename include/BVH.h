#include <algorithm>
#include <parallel/algorithm>
#include <vector>
#include <unordered_map>

#include "AABB.h"
#include "polygon.h"

struct Environment{
    public:
        std::unordered_map<polygon*, bool> _objects;        
        std::pair<float, float> _x_range, _y_range, _z_range;

        Environment(std::pair<float, float> xRange, std::pair<float, float> yRange, std::pair<float, float> zRange) : 
        _x_range(xRange), _y_range(yRange), _z_range(zRange){}
};

struct node{
    std::unordered_map<polygon*, bool> _objects;
    std::vector<node*> children;

    enum SplitAxis {X, Y, Z} _split_axis;
    float _x_range, _y_range, _z_range;

    float _volume;
    AABB  _region;
    bool isLeaf;

    node(std::vector<polygon*> objects, AABB bbox): 
    _objects(objects), _region(bbox)
    {
        if(_x_range > _y_range && _x_range > _z_range){
            _split_axis = X;
        }
        else if(_y_range > _x_range && _y_range > _z_range){
            _split_axis = Y;
        }
        else if(_z_range > _x_range && _z_range > _y_range){
            _split_axis = Z;
        }

        __gnu_parallel::sort(_objects.begin(),
                             _objects.end(),
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
        float _threshold;
        enum SplitAxis {X, Y, Z} _split_axis;  

    public:
        BVH(Environment _env): env(_env){
            point p1 = point(env._x_range.first, env._y_range.first, env._z_range.first);
            point p2 = point(env._x_range.second, env._y_range.second, env._z_range.second);
            treeNodes.push_back(node(env._objects, AABB(p1, p2)));
        }

        void create_BVH(node* root, float volume){
            /**
             * @brief Add branching nodes and call create_BVH recursively on them.
             * 
             */
            
            if(volume < this->_threshold){
                return;
            }

            for(int i = 0; i < branching; i++){
                /*Create sub list of Objects
                  Create Bounding box for these objects {min(AABBs), max(AABBs)}
                 */
                

                node* childNode = new node( /*Objects in split axis range take objects from sorted array in root node*/, std::min/*AABB bounding box for the complete set of objects, so min(AABBs), max(AABBs)*/);
                create_BVH(childNode);
                root->children.push_back(childNode);
            }
        }
};
