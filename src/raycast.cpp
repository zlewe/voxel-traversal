#include "voxel_traversal.h"

using namespace voxel_traversal;

// types for vectors and indices

using grid_type = Grid3DSpatialDef<double>;
using V3 = typename grid_type::Vector3d;
using C3 = typename grid_type::Index3d;
using S3 = typename grid_type::Size3d;
using R = Ray<double>;

struct DataBuf{
    S3 delta{};
    S3 tmax{};
    C3 step_index{};
    C3 current_index{};
    C3 final_index{};
    C3 overflow_index{};
};

class RayTracingObject{
    public:
        DataBuf buff;

        RayTracingObject(){
            grid = grid_type();
        };
        RayTracingObject(V3 bound_min, V3 bound_max, C3 voxel_count){
            grid = grid_type(bound_min, bound_max, voxel_count);
        }
        bool setup(V3 origin, V3 destination){
            R ray = R::fromOriginDir(origin, destination);
            const auto intersect = detail::setupTraversal<grid_type>(ray, grid, 0.0, 1.0, buff.delta, buff.tmax, buff.step_index, buff.current_index, buff.final_index);
            if (!intersect) return false;
            buff.overflow_index = buff.final_index + buff.step_index;
            return true;
        }
        bool traverseOnce(V3& answer){
            if (!detail::traverseSingle<double>(buff.tmax, buff.current_index, buff.overflow_index, buff.step_index, buff.delta)) return false;
            answer[0] = double(buff.current_index[0]);
            answer[1] = double(buff.current_index[1]);
            answer[2] = double(buff.current_index[2]);
            return true;
        }
        void reset(){
            buff = {};
        }

    private:
        grid_type grid;

};


// int main(void){
//     //test
//     V3 bound_min(0.0,0.0,0.0);
//     V3 bound_max(5.0,5.0,1.0);
//     C3 voxel_count(5,5,5);
//     RayTracingObject rt(bound_min,bound_max,voxel_count);

//     V3 origin(2.0,1.0,0.0);
//     V3 destination(4.9,3.0,0.5);
//     bool set = rt.setup(origin, destination);
//     if (!set) return -1;

//     while(rt.traverseOnce()){
//         C3 ind = rt.buff.current_index;
//         printf("x: %d, y: %d, z: %d", ind[0], ind[1], ind[2]);
//     }
// }


