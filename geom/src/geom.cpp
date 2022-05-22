#include "geom.h"

#include <iostream>     // std::cout
#include <algorithm>    // std::min/std::max, std::abs

// standard geometry function implementation
namespace geom
{
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
    
    value_type length(point pt)
    {
        return std::sqrt(distance_sqaured(pt, point{ 0 }));
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
            // function PointInTriangle(p, a, b, c)
            //     if SameSide(p,a, b,c) and SameSide(p,b, a,c)
            //         and SameSide(p,c, a,b) then return true
            //     else return false

            /*if( same_side(point, triangle.a, triangle.b, triangle.c) &&
                same_side(point, triangle.b, triangle.a, triangle.c) &&
                same_side(point, triangle.c, triangle.a, triangle.b) )
            {
                //requires extra coplaner check for 3D
                return true;
            }

            return false;*/

            {
                // using barycentric
                vector p = point;
                vector a = triangle.a;
                vector b = triangle.b;
                vector c = triangle.c;
                
                // Prepare our barycentric variables
                vector u = b - a;
                vector v = c - a;
                vector w = p - a;
                
                vector vCrossW = cross(v, w);
                vector vCrossU = cross(v, u);

                // Test sign of r

                if (dot(vCrossW, vCrossU) < 0)
                    return false;

                vector uCrossW = cross(u, w);
                vector uCrossV = cross(u, v);

                // Test sign of t
                if (dot(uCrossW, uCrossV) < 0)
                    return false;

                // At this point, we know that r and t and both > 0.
                // Therefore, as long as their sum is <= 1, each must be less <= 1
                value_type denom = length(uCrossV);
                value_type r = length(vCrossW) / denom;
                value_type t = length(uCrossW) / denom;

                return (r + t <= value_type{ 1.0 });
            }

            //vector p = point;
            //vector a = triangle.a;
            //vector b = triangle.b;
            //vector c = triangle.c;

            //// Move the triangle so that the point becomes the 
            //// triangles origin
            //a -= p;
            //b -= p;
            //c -= p;

            //// The point should be moved too, so they are both
            //// relative, but because we don't use p in the
            //// equation anymore, we don't need it!
            //// p -= p;

            //// Compute the normal vectors for triangles:
            //// u = normal of PBC
            //// v = normal of PCA
            //// w = normal of PAB

            //// u = a, v = b, w = c

            //vector u = cross(b, c); 
            //vector v = cross(c, a); 
            //vector w = cross(a, b);

            //// Test to see if the normals are facing 
            //// the same direction, return false if not
            //if (dot(u, v) < 0.0f) 
            //{
            //    return false;
            //}

            //if (dot(u, w) < 0.0f) 
            //{
            //    return false;
            //}

            //// All normals facing the same way, return true
            //return true;

        }

        bool point_plane(point const& point, plane const& plane)
        {
            //std::cout << dot(point, plane.normal) << " " << plane.dist << std::endl;
            return dot(point, plane.normal) == plane.dist;
        }



        bool ray_plane(ray const& ray, plane const& plane)
        {
            value_type dir_dot_normal = dot(ray.dir, plane.normal);
            value_type ray_dot_normal = dot(ray.point, plane.normal);
            //std::cout << "dir dot normal : " << dir_dot_normal << "\n";
            //std::cout << "ray dot normal : " << ray_dot_normal << "\n";
            // plane and ray are parallel
            if(dir_dot_normal == value_type{0})
            {
                //std::cout << ray_dot_normal << " " << plane.dist << std::endl;
                return ray_dot_normal == plane.dist; // ray either has infinite solns or none.
            }

            // minusing off the distance than multiplying by the normal would not be positive
            // draw it out and test(should work on all quadrants)
            return ( (plane.dist - ray_dot_normal) * dir_dot_normal ) >= value_type{0};
        }

        bool ray_aabb(ray const& ray, aabb const& aabb)
        {
            vector ray_normal_inversed = 1.0f / ray.dir;

            value_type tmin = std::numeric_limits<value_type>::min();
            value_type tmax = std::numeric_limits<value_type>::max();

            for(size_t i = 0; i < dim; ++i)
            {
                value_type t1 = (std::abs(aabb.min[i]) - std::abs(ray.point[i])) * ray_normal_inversed[i];
                value_type t2 = (std::abs(aabb.max[i]) - std::abs(ray.point[i])) * ray_normal_inversed[i];

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

            return tmax >= std::max(0.0f, tmin); //&& tmin < t;
        }

        bool ray_sphere(ray const& ray, sphere const& sphere)
        {
            auto ca = ray.point - sphere.center;
            auto radius_squared = sphere.radius * sphere.radius;

            // effectively distance(ray.point, sphere.center) <= sphere. radius
            if(dot(ca, ca) <= radius_squared)
                return true;

            // assumes ray direction is normalized.
            value_type proj = dot(ca, ray.dir);
            if(proj >= 0)
                return false;

            // ray will eventually hit circle
            auto newVec = static_cast<point>(ca + (ray.dir * proj));
            if(distance_sqaured(newVec, newVec) <= radius_squared)
                return true;

            return false;
        }

        bool ray_triangle(ray const& ray, triangle const& triangle)
        {
            vector ab = triangle.b - triangle.a;
            vector ac = triangle.c - triangle.a;
            vector qp = ray.dir; //p - q;
            // Compute triangle normal. Can be precalculated or cached if
            // intersecting multiple segments against the same triangle
            vector n = cross(ab, ac);
            //std::cout << "n:[" << n[0] << "," << n[1] << "," << n[2] << "]\n";
            // Compute denominator d. If d <= 0, segment is parallel to or points
            // away from triangle, so exit early
            value_type d = dot(qp, n);
            //std::cout << "d:(" << d << ")\n";
            if (d <= value_type{0})
                return false;
            // Compute intersection t value of pq with plane of triangle. A ray
            // intersects iff 0 <= t. Segment intersects iff 0 <= t <= 1. Delay
            // dividing by d until intersection has been found to pierce triangle
            vector ap = ray.point - triangle.a; //p - a;
            auto t = dot(ap, n);
            //std::cout << "t" << t << "\n";
            if (t < value_type{0}) 
                return false;
            // if (t > d) 
            //     return false; // For segment; exclude this code line for a ray test

            // Compute barycentric coordinate components and test if within bounds
            vector e = cross(qp, ap);
            auto v = dot(ac, e);
            if (v < value_type{0} /*|| v > d*/) 
                return false;
            auto w = -dot(ab, e);
            if (w < value_type{0} /*|| v + w > d*/) 
                return false;
            // Segment/ray intersects triangle. Perform delayed division and
            // compute the last barycentric coordinate component
            value_type ood = 1.0f / d;
            t *= ood;
            v *= ood;
            w *= ood;
            auto u = 1.0f - v - w;
            (void)u;
            
            return true;

            // // do a point triangle test first.
            // if(point_triangle(ray.point, triangle))
            //     return true;
            
            // return false;
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
