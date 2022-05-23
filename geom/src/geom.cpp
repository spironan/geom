#include "geom.h"

#include <iostream>     // std::cout
#include <algorithm>    // std::min/std::max, std::abs

// standard geometry function implementation
namespace geom
{
    bool epsilon_test(value_type value)
    {
        return std::abs(value) < epsilon;
    }

    value_type dot(vector ptA, vector ptB)
    {
        value_type res{0};
        for(size_type i = 0; i < dim; ++i)
        {
            res += ptA[i] * ptB[i];
        }
        return res;
    }

    value_type distance_sqaured(point ptA, point ptB)
    {
        vector res = ptB - ptA;
        return dot(res, res);
    }

    value_type distance_sqaured(point pt, aabb const& aabb)
    {
        value_type sqDist = 0.f;
        for (size_type i = 0; i < dim; ++i)
        {
            value_type v = pt[i];
            if (v < aabb.min[i]) sqDist += (aabb.min[i] - v) * (aabb.min[i] - v);
            if (v > aabb.max[i]) sqDist += (v - aabb.max[i]) * (v - aabb.max[i]);
        }
        return sqDist;
    }

    value_type length_squared(point pt)
    {
        return distance_sqaured(point{ 0 }, pt);
    }

    value_type length(point pt)
    {
        return std::sqrt(length_squared(pt));
    }

    vector normalize(vector vec)
    {
        value_type d = length(vec);
        return vec / d;
    }

    vector cross(vector vecA, vector vecB)
    {
        vector result;
        
        for(size_type i = 0; i < dim; ++i)
        {
            auto idxA = (i+1) % dim;
            auto idxB = (i+2) % dim;
            result[i] = (vecA[idxA] * vecB[idxB]) - (vecA[idxB] * vecB[idxA]);
        }
        
        return result;
    }

    bool same_side(point p1, point p2, point a, point b)
    {
        auto cp1 = cross(b-a, p1-a);
        auto cp2 = cross(b-a, p2-a);
        
        if(dot(cp1, cp2) >= 0)
            return true;
        
        return false;
    }
}

// intersection implementation
namespace geom
{
    namespace intersection
    {
        bool sphere_sphere(sphere const& sphereA, sphere const& sphereB)
        {
            return (sphereA.radius + sphereB.radius) * (sphereA.radius + sphereB.radius) 
            >= distance_sqaured(sphereA.center, sphereB.center);
        }

        bool aabb_sphere(aabb const& aabb, sphere const& sphere)
        {
            // Compute squared distance between sphere center and AABB
            value_type sqDist = distance_sqaured(sphere.center, aabb);
            // Sphere and AABB intersect if the (squared) distance
            // between them is less than the (squared) sphere radius
            return sqDist <= sphere.radius * sphere.radius;
        }

        bool aabb_aabb(aabb const& aabbA, aabb const& aabbB)
        {
            for(size_t i = 0; i < dim; ++i)
            {
                if(!(aabbA.min[i] <= aabbB.max[i] && aabbA.max[i] >= aabbB.min[i]))
                {
                    return false;
                }
            }

            return true;
        }



        bool point_sphere(point const& point, sphere const& sphere)
        {
            return sphere.radius * sphere.radius >= distance_sqaured(point, sphere.center);
        }

        bool point_aabb(point const& point, aabb const& aabb)
        {
            for(size_t i = 0; i < dim; ++i)
            {
                if(!(point[i] <= aabb.max[i] && point[i] >= aabb.min[i]))
                {
                    return false;
                }
            }

            return true;
        }

        bool point_triangle(point const& point, triangle const& triangle)
        {
            vector p = point;
            vector a = triangle.a;
            vector b = triangle.b;
            vector c = triangle.c;

            // Move the triangle so that the point becomes the 
            // triangles origin
            a -= p;
            b -= p;
            c -= p;

            // The point should be moved too, so they are both
            // relative, but because we don't use p in the
            // equation anymore, we don't need it!
            // p -= p;

            // Compute the normal vectors for triangles:
            // u = normal of PBC
            // v = normal of PCA
            // w = normal of PAB

            vector u = cross(b, c); 
            vector v = cross(c, a); 
            vector w = cross(a, b);

            // Test to see if the normals are facing 
            // the same direction, return false if not
            if (dot(u, v) < 0.0f) 
            {
                return false;
            }

            if (dot(u, w) < 0.0f) 
            {
                return false;
            }

            // All normals facing the same way, return true
            return true;
        }

        bool point_plane(point const& point, plane const& plane)
        {
            //std::cout << dot(point, plane.normal) << " " << plane.dist << std::endl;
            return epsilon_test(dot(point, plane.normal) - plane.dist);
        }



        RaycastResult ray_plane(ray const& ray, plane const& plane)
        {
            value_type dir_dot_normal = dot(ray.dir, plane.normal);
            value_type ray_dot_normal = dot(ray.point, plane.normal);

            // positive distance means ray start position is in-front of plane
            // negative distance means ray start position is behind the plane.
            value_type dist_ray_to_plane = ray_dot_normal - plane.dist;

            if (epsilon_test(ray_dot_normal))
            {
                if (epsilon_test(dist_ray_to_plane))
                {
                    // true if ray is collinear to plane and always intersecting
                    return { true, 0.0, maximum, ray.point, max_point };
                }
                else
                {
                    // false if ray is parallel to plane and never hit
                    return { };
                }
            }

            time_type entry_time = dir_dot_normal / dist_ray_to_plane;
            if (epsilon_test(dist_ray_to_plane))
            {
                // ray is on the plane and point
                entry_time = 0.0;
            }
            point final_pos = ray.point + entry_time * ray.dir;
            
            // only one point when hitting the plane therefore entry == exit
            // if time is <= 0 the plane is behind the ray
            return { entry_time >= 0.0, entry_time, entry_time, final_pos, final_pos };
        }

        RaycastResult ray_aabb(ray const& ray, aabb const& aabb)
        {
            vector ray_normal_inversed = 1.0f / ray.dir;

            value_type tmin = std::numeric_limits<value_type>::min();
            value_type tmax = std::numeric_limits<value_type>::max();

            for(size_t i = 0; i < dim; ++i)
            {
                value_type t1 = (aabb.min[i] - ray.point[i]) * ray_normal_inversed[i];
                value_type t2 = (aabb.max[i] - ray.point[i]) * ray_normal_inversed[i];

                tmin = std::max(std::min(t1, t2), tmin);
                tmax = std::min(std::max(t1, t2), tmax);
            }
            
            // value_type tx1 = (aabb.min[0] - ray.point[0]) * ray_normal_inversed[0];
            // value_type tx2 = (aabb.max[0] - ray.point[0]) * ray_normal_inversed[0];

            // value_type tmin = std::min(tx1, tx2);
            // value_type tmax = std::max(tx1, tx2);

            // value_type ty1 = (aabb.min[1] - ray.point[1]) * ray_normal_inversed[1];
            // value_type ty2 = (aabb.max[1] - ray.point[1]) * ray_normal_inversed[1];

            // tmin = std::max(tmin, std::min(ty1, ty2));
            // tmax = std::min(tmax, std::max(ty1, ty2));

            // value_type tz1 = (aabb.min[2] - ray.point[2]) * ray_normal_inversed[2];
            // value_type tz2 = (aabb.max[2] - ray.point[2]) * ray_normal_inversed[2];

            // tmin = std::max(tmin, std::min(tz1, tz2));
            // tmax = std::min(tmax, std::max(tz1, tz2));

            point entry_pos = ray.point + tmin * ray.dir;
            point exit_pos = ray.point + tmax * ray.dir;

            return { tmax >= std::max(0.0f, tmin) /* && tmin < t; */ , tmin, tmax, entry_pos, exit_pos };
        }

        bool ray_sphere(ray const& ray, sphere const& sphere)
        {
            auto ca = ray.point - sphere.center;
            auto radius_squared = sphere.radius * sphere.radius;

            // check if point is in circle
            if(length_squared(ca) <= radius_squared)
                return true;

            auto dir = ray.dir;
            
            // don't assumes ray direction is normalized.
            if (epsilon_test(length_squared(dir) - value_type{ 1 }))
                dir = normalize(dir);

            // early rejection test.
            value_type proj = dot(ca, dir);
            if(proj >= 0)
                return false;

            auto rejection_vector = static_cast<point>(ca + (dir  * proj));
            if(length_squared(rejection_vector) <= radius_squared)
                return true;    // can solve for time here.

            return false;
        }

        bool ray_triangle(ray const& ray, triangle const& triangle)
        {
            //ray triangle using ray-plane + point-aabb.
            vector plane_normal = cross(triangle.c - triangle.a, triangle.b - triangle.a);
            plane plane_formed_by_triangle = { plane_normal, dot(triangle.a, plane_normal) };
            RaycastResult res = ray_plane(ray, plane_formed_by_triangle);
            
            if (!res.intersect)
                return false;
            
            return point_triangle(res.p_entry, triangle);
        }


        bool plane_aabb(plane const& plane, aabb const& aabb)
        {
            // these two lines not necessary wit ha (center, extents) AABB representation
            vector center = (aabb.max + aabb.min) * 0.5f;   // compute AABB center
            vector extents = aabb.max - center;   // compute positive extents

            // Compute the projection interval radius of aabb onto L(t) = aabb.c + t * plane2d.n
            value_type r = extents[0] * std::abs(plane.normal[0]) + extents[1] * std::abs(plane.normal[1]) + extents[2] * std::abs(plane.normal[2]);
            // Compute distance of box center from plane
            value_type s = dot(plane.normal, center) - plane.dist;

            // Intersection occurs when distance s falls within [-r, +r] interval
            return std::abs(s) <= r;
        }

        bool plane_sphere(plane const& plane, sphere const& sphere)
        {
            // For a normalized plane ( plane2d.normal = 1 ), evaluating the plane equation
            value_type dist = dot(sphere.center, plane.normal) - plane.dist;
            // If sphere center within +/-radius from plane, plane intersects sphere
            return std::abs(dist) <= sphere.radius;
        }
    }
}
