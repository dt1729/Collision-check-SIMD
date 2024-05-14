#include <iostream>
#include <vector>
#include <x86intrin.h>
#include <thread>

struct point{
    float _x, _y, _z;
    public:
        point(float x, float y, float z): _x(x), _y(y), _z(z){};
};

class abstract_polygon{
    protected:
        int num_edges;
        std::vector<point> points;

    public:
        virtual void create_object() = 0;
        virtual void update_properties(point p, int idx) = 0;
        virtual abstract_polygon* return_object_pointer() = 0;
        virtual abstract_polygon& operator+(abstract_polygon& p2) = 0;
        virtual bool contains(point *p) = 0;
        virtual float volume() = 0;
};


class AABB : public abstract_polygon{
    private:
        float _x_min, _y_min, _z_min;
        float _x_max, _y_max, _z_max;

    public:

        AABB(point p_min, point p_max)
            : _x_min(p_min._x), _y_min(p_min._y), _z_min(p_min._z),
            _x_max(p_max._x), _y_max(p_max._y), _z_max(p_max._z) {

                /* Serialised CW Storage of mesh's points starting from the bottom most layer
                   in the front view for the global coordinate system.
                */
                for(auto i : std::vector<float>{_z_min, _z_max}){
                    points.push_back(point(_x_min, _y_min, i));
                    points.push_back(point(_x_max, _y_min, i));
                    points.push_back(point(_x_max, _y_max, i));
                    points.push_back(point(_x_min, _y_max, i));
                }
            }

        void update_properties(point p, int idx) override{
            this->points[idx] = std::move(p);
        }

        abstract_polygon* return_object_pointer() override{
            return this;
        }

        float volume() override{
            return (_x_max-_x_min)*(_y_max - _y_min)*(_z_max - _z_min);
        }
        
        bool contains(point *p) override{ 
            return (p->_x < this->_x_max && p->_x > this->_x_min)&&
                   (p->_y < this->_y_max && p->_y > this->_y_min)&&
                   (p->_z < this->_z_max && p->_z > this->_z_min);
        }
};