#ifndef VOXEL_TRAVERSAL_H
#define VOXEL_TRAVERSAL_H

#include "grid.h"
#include "ray.h"

namespace voxel_traversal {

namespace detail {
    template <typename Grid>
    bool setupTraversal(const Ray<typename Grid::float_t>& ray, const Grid& grid,
                        typename Grid::float_t t0, typename Grid::float_t t1,
                        typename Grid::Size3d& delta, typename Grid::Size3d& tmax,
                        typename Grid::Index3d& step_index,
                        typename Grid::Index3d& current_index,
                        typename Grid::Index3d& final_index);
                        
    template <typename float_type>
    bool traverseSingle(
        typename Grid3DSpatialDef<float_type>::Size3d& tmax,
        typename Grid3DSpatialDef<float_type>::Index3d& current_index,
        const typename Grid3DSpatialDef<float_type>::Index3d& overflow_index,
        const typename Grid3DSpatialDef<float_type>::Index3d& step_index,
        const typename Grid3DSpatialDef<float_type>::Size3d& delta);                  
}

//! Type to record the indices of traversed voxels
template <typename float_type>
using TraversedVoxels =
    std::vector<typename Grid3DSpatialDef<float_type>::Index3d>;

/*!
 * Traverse a voxel grid along a ray and record all traversed voxels.
 *
 * Implements the algorithm presented in Amanatides & Woo's "A Fast Voxel
 * Traversal Algorithm for Ray Tracing."
 * If the ray origin is outside the voxel grid, uses a safer version of Smit's
 * ray box intersection algorithm to determine intersection.
 * The voxel indices in traversed_voxels are in order of traversal from
 * ray origin to ray end.
 *
 * @tparam float_type precision
 * @param ray Start at ray origin and end traversal at ray end.
 * @param grid Defines voxel grid.
 * @param traversed_voxels Output for the recorded voxels.
 * @param t0 ray start bound. 0 equals the ray origin.
 * @param t1 ray end bound. 1 equals the ray end.
 * @return true if the ray intersects the voxel grid
 */
template <typename float_type = double>
bool traverseVoxelGrid(const Ray<float_type>& ray,
                       const Grid3DSpatialDef<float_type>& grid,
                       TraversedVoxels<float_type>& traversed_voxels,
                       float_type t0 = float_type{0.0},
                       float_type t1 = float_type{1.0}) noexcept;

/*!
 * Traverse a voxel grid along a ray and increase grid counter for traversed
 * voxels.
 *
 * Implements the algorithm presented in Amanatides & Woo's "A Fast Voxel
 * Traversal Algorithm for Ray Tracing."
 * If the ray origin is outside the voxel grid, uses a safer version of Smit's
 * ray box intersection algorithm to determine intersection.
 *
 * @tparam float_type precision
 * @param ray Start at ray origin and end traversal at ray end.
 * @param grid Defines voxel grid and holds the permanent voxel counter.
 * @param t0 ray start bound. 0 equals the ray origin.
 * @param t1 ray end bound. 1 equals the ray end.
 * @return true if the ray intersects the voxel grid
 */
template <typename float_type = double>
bool traverseVoxelGrid(const Ray<float_type>& ray,
                       Grid3DTraversalCounter<float_type>& grid,
                       float_type t0 = float_type{0.0},
                       float_type t1 = float_type{1.0}) noexcept;

/*!
 * Get the positions t_min and t_max along a ray where it enters and exists a
 * voxel grid.
 *
 * @tparam float_type precision
 * @param ray Start at ray origin and end traversal at ray end.
 * @param grid Define voxel grid
 * @param tmin smallest position along ray that falls into the grid
 * @param tmax largest position along ray that falls into the grid
 * @param t0 ray start bound. 0 equals the ray origin.
 * @param t1 ray end bound. 1 equals the ray end.
 * @return true if the ray intersects the voxel grid
 */
template <typename float_type = double>
[[nodiscard]] bool rayBoxIntersection(const Ray<float_type>& ray,
                                      const Grid3DSpatialDef<float_type>& grid,
                                      float_type& tmin, float_type& tmax,
                                      float_type t0 = 0.0,
                                      float_type t1 = 1.0) noexcept;
}  // namespace voxel_traversal

#endif  // VOXEL_TRAVERSAL_H
