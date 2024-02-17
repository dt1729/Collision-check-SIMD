#include <iostream>
#include <x86intrin.h>
#include <thread>

class abstract_polygon{
    private:
        int num_edges;

    public:
        virtual void create_object() = 0;
        virtual void update_properties() = 0;
        virtual abstract_polygon* return_object_pointer() = 0;
        virtual abstract_polygon& operator+(abstract_polygon& p2) = 0;
};


class AABB : public abstract_polygon{
    
};