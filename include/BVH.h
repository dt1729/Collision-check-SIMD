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

    node(std::vector<std::pair<polygon*, bool>> objects, AABB bbox): 
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
        float _threshold;
        enum SplitAxis {X, Y, Z} _split_axis;  

    public:
        BVH(Environment _env): env(_env){
            point p1 = point(env._x_range.first, env._y_range.first, env._z_range.first);
            point p2 = point(env._x_range.second, env._y_range.second, env._z_range.second);
            treeNodes.push_back(node(env._objects, AABB(p1, p2)));
        }

        void create_BVH(node* root){
            /**
             * @brief Add branching nodes and call create_BVH recursively on them.
             * 
             */
            
            if(root->_objects.size() < 1){
                root->isLeaf = true;
                return;
            }

            for(int i = 0; i < 2; i++){
                /*Create sub list of Objects
                  Create Bounding box for these objects {min(AABBs), max(AABBs)}
                 */
                std::vector<std::pair<polygon*, bool>> objects;
                float _running_x_min = INFINITY, _running_y_min = INFINITY, _running_z_min = INFINITY;
                float _running_x_max = -INFINITY, _running_y_max = -INFINITY, _running_z_max = -INFINITY;
                point min_pt(root->_region._x_min, root->_region._y_min, root->_region._z_min),\
                      max_pt(root->_region._x_max, root->_region._y_max, root->_region._z_max);

                for(auto k : root->_objects){
                    if (i ==0 && k.second && (root->_return_value(root->_split_axis, k.first) < root->_return_value(root->_split_axis, &root->_region))){
                        objects.push_back(std::make_pair(k.first, true));
                        k.second = false;

                        if(k.first->_x_min < _running_x_min && k.first->_y_min < _running_y_min && k.first->_z_min < _running_z_min){
                            min_pt = point(k.first->_x_min, k.first->_y_min, k.first->_z_min);
                            _running_x_min = k.first->_x_min; _running_y_min = k.first->_y_min; _running_z_min = k.first->_z_min;
                        }
                        else if(k.first->_x_max >= _running_x_max && k.first->_y_max >= _running_y_max && k.first->_z_max >= _running_z_max){
                            max_pt = point(k.first->_x_max, k.first->_y_max, k.first->_z_max);
                            _running_x_max = k.first->_x_max; _running_y_max = k.first->_y_max; _running_z_max = k.first->_z_max;
                        }
                    }
                    else if (i == 1 && k.second && (root->_return_value(root->_split_axis, k.first) > root->_return_value(root->_split_axis, &root->_region))){
                        objects.push_back(std::make_pair(k.first, true));
                        k.second = false;

                        if(k.first->_x_min < _running_x_min && k.first->_y_min < _running_y_min && k.first->_z_min < _running_z_min){
                            min_pt = point(k.first->_x_min, k.first->_y_min, k.first->_z_min);
                            _running_x_min = k.first->_x_min; _running_y_min = k.first->_y_min; _running_z_min = k.first->_z_min;
                        }
                        else if(k.first->_x_max >= _running_x_max && k.first->_y_max >= _running_y_max && k.first->_z_max >= _running_z_max){
                            max_pt = point(k.first->_x_max, k.first->_y_max, k.first->_z_max);
                            _running_x_max = k.first->_x_max; _running_y_max = k.first->_y_max; _running_z_max = k.first->_z_max;
                        }
                    }
                }

                node* childNode = new node(objects /*Objects in split axis range take objects from sorted array in root node*/,\
                                           AABB(min_pt, max_pt) /*AABB bounding box for the complete set of objects, so min(AABBs), max(AABBs)*/
                                           );

                create_BVH(childNode);
                root->children.push_back(childNode);
            }
            root->isLeaf = false;
        }
};
