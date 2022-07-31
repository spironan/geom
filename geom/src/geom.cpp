#include "geom.h"

#include <iostream>     // std::cout
#include <algorithm>    // std::min/std::max, std::abs
#include <map>          // std::map


// standard geometry function implementation
namespace geom
{
    bool epsilon_test(value_type value)
    {
        return std::abs(value) <= epsilon;
    }

    value_type min(value_type lhs, value_type rhs)
    {
        return lhs < rhs ? lhs : rhs;
    }
    
    value_type max(value_type lhs, value_type rhs)
    {
        return lhs > rhs ? lhs : rhs;
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
            
            value_type margin = 100 * epsilon;
            // ensure point is on the plane formed by triangle
            if (dot(w, vCrossU) > margin)
                return false;

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

            //vector u = cross(b, c); 
            //vector v = cross(c, a); 
            //vector w = cross(a, b);

            //vector originalArea = cross(b - a, c - a);

            //if ( length_squared(u + v + w - originalArea) < epsilon )
            //    return true;

            ////// Test to see if the normals are facing 
            ////// the same direction, return false if not
            ////if (dot(u, v) < 0.0f) 
            ////{
            ////    return false;
            ////}

            ////if (dot(u, w) < 0.0f)
            ////{
            ////    return false;
            ////}

            //// All normals facing the same way, return true
            ////return true;
            //
            //return false;
        }

        bool point_plane(point const& point, plane const& plane)
        {
            //std::cout << dot(point, plane.normal) << " " << plane.dist << std::endl;
            return epsilon_test(dot(point, plane.normal) - plane.dist);
        }


        RaycastResult ray_plane(ray const& ray, plane const& plane)
        {
            auto ray_dir = normalize(ray.dir);
            auto plane_normal = normalize(plane.normal);
            value_type dir_dot_normal = dot(ray_dir, plane_normal);
            value_type ray_dot_normal = dot(ray.point, plane_normal);

            if (epsilon_test(dir_dot_normal))
            {
                if (epsilon_test(ray_dot_normal))
                {
                    // ray is collinear to plane and always intersecting
                    return { true, 0.0, maximum, ray.point, max_point };
                }
                else
                {
                    // ray is parallel to plane and never hit
                    return { false };
                }
            }

            // positive time means ray start position is in-front of plane
            // negative time means ray start position is behind the plane.
            time_type entry_time = (plane.dist - ray_dot_normal) / dir_dot_normal;
            point entry_pos = ray.point + entry_time * ray_dir;
            
            // only one point when hitting the plane therefore entry == exit
            // if time is <= 0 the plane is behind the ray
            bool will_collide = entry_time >= 0.0;
            return { will_collide, entry_time, entry_time, entry_pos, entry_pos };
        }

        RaycastResult ray_aabb(ray const& ray, aabb const& aabb)
        {
            vector ray_normal_inversed = value_type{ 1.0 } / ray.dir;

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

            return { tmax >= std::max(value_type{0.0}, tmin) /* && tmin < t; */ , tmin, tmax, entry_pos, exit_pos };
        }

        RaycastResult ray_sphere(ray const& ray, sphere const& sphere)
        {
            auto point_to_sphere = sphere.center - ray.point;
            auto radius_squared = sphere.radius * sphere.radius;

            auto dir = ray.dir;

            // don't assumes ray direction is normalized.
            if (epsilon_test(length_squared(dir) - value_type{ 1 }))
                dir = normalize(dir);

            // check if point is in circle
            auto point_sphere_len_sqr = length_squared(point_to_sphere);
            if (point_sphere_len_sqr <= radius_squared)
            {
                auto exit_dist = std::sqrt(radius_squared - point_sphere_len_sqr);
                auto exit_time = exit_dist / dot(point_to_sphere, dir);
                return { true, 0.0, exit_time, ray.point, ray.point + dir * exit_time };
            }

            // early rejection test.
            value_type proj = dot(point_to_sphere, dir);
            if (proj < 0)
                return { false };

            auto rejection_vector = static_cast<point>(-point_to_sphere + (dir * proj));
            auto rej_vec_len_sqr = length_squared(rejection_vector);
            if (rej_vec_len_sqr <= radius_squared)
            {
                auto s = std::sqrt(radius_squared - rej_vec_len_sqr);
                auto entry_time = proj - s;
                auto exit_time = proj + s;
                auto entry_point = ray.point + dir * entry_time;
                auto exit_point = ray.point + dir * exit_time;

                return { true, entry_time, exit_time, entry_point, exit_point };
            }

            return { false };
        }

        RaycastResult ray_triangle(ray const& ray, triangle const& triangle)
        {
            //ray triangle using ray-plane + point-aabb.
            vector plane_normal = cross(triangle.b - triangle.a, triangle.c - triangle.a);  // ab x ac
            plane_normal = normalize(plane_normal);
            plane plane_formed_by_triangle = { plane_normal, dot(triangle.a, plane_normal) };
            RaycastResult res = ray_plane(ray, plane_formed_by_triangle);
            
            if (res.intersect == false)
                return res;

            // There is an extreme edge case where the point is on the plane and dir is parallel to the plane
            // and its currently not handled.
            
            //// if ray is on 2d plane.
            //if (res.t_entry == 0.0 && res.t_exit == maximum)
            //{
            //    return ray_triangle_2d(ray, triangle);
            //}
            
            return { point_triangle(res.p_entry, triangle), res.t_entry, res.t_exit, res.p_entry, res.p_exit };
        }


        bool ray_sphere_bool(ray const& ray, sphere const& sphere)
        {
            auto point_to_sphere = sphere.center - ray.point;
            auto radius_squared = sphere.radius * sphere.radius;

            // check if point is in circle
            if (length_squared(point_to_sphere) <= radius_squared)
                return true;

            auto dir = ray.dir;

            // don't assumes ray direction is normalized.
            if (epsilon_test(length_squared(dir) - value_type{ 1 }))
                dir = normalize(dir);

            // early rejection test.
            value_type proj = dot(point_to_sphere, dir);
            if (proj < 0)
                return false;

            auto rejection_vector = static_cast<point>(ray.point + (dir * proj) - sphere.center);
            //auto rejection_vector = static_cast<point>(-point_to_sphere + (dir * proj));
            auto rej_vec_len_sqr = length_squared(rejection_vector);
            if (rej_vec_len_sqr <= radius_squared)
                return true;

            return false;
        }



        bool plane_aabb(plane const& plane, aabb const& aabb)
        {
            // these two lines not necessary wit ha (center, extents) AABB representation
            vector center = (aabb.max + aabb.min) * value_type{ 0.5 };   // compute AABB center
            vector extents = aabb.max - center;   // compute positive extents

            // Compute the projection interval radius of aabb onto L(t) = aabb.c + t * plane2d.n
            value_type r = 0;
            for (size_type i = 0; i < dim; ++i)
            {
                r += extents[i] * std::abs(plane.normal[i]);
            }
            
            //extents[0] * std::abs(plane.normal[0]) + extents[1] * std::abs(plane.normal[1]) + extents[2] * std::abs(plane.normal[2]);
             
            // Compute distance of box center from plane
            value_type s = dot(plane.normal, center) - plane.dist;

            // Intersection occurs when distance s falls within [-r, +r] interval
            return std::abs(s) <= r;
        }

        bool plane_sphere(plane const& plane, sphere const& sphere)
        {
            // For a normalized plane ( plane2d.normal = 1 ), evaluating the plane equation
            value_type dist = dot(sphere.center, plane.normal) - plane.dist;
            // If sphere center within +/- radius from plane, plane intersects sphere
            return std::abs(dist) <= sphere.radius;
        }
    }
}

// bounding volume hierarchies
namespace geom
{
    namespace bvh
    {
        aabb make_fitting_aabb(std::vector<point> const& vertices)
        {
            if (vertices.size() == 0)
                return aabb{};

            point smallest = vertices.front(), largest = vertices.front();

            for (auto& vertex : vertices)
            {
                for (int i = 0; i < dim; ++i)
                {
                    smallest[i] = min(vertex[i], smallest[i]);
                    largest[i] = max(vertex[i], largest[i]);
                }
            }

            return { smallest, largest };
        }

        std::pair<point, point> most_separated_points_on_axis(std::vector<point> const& vertices, vector const& normal)
        {
            // First retrieve all the normals we will be dotting against
            std::pair<point, point> extrema_points;
            int min = 0, max = 0;
            for (int i = 0; i < vertices.size(); ++i)
            {
                if (dot(vertices[i], normal) < dot(vertices[min], normal)) min = i;
                if (dot(vertices[i], normal) > dot(vertices[max], normal)) max = i;
            }
            return { vertices[min], vertices[max] };
        }

        std::pair<point, point> most_separated_points_on_aabb(std::vector<point> const& vertices)
        {
            // First lets find the most extreme points along the principal axes
            int minx = 0, miny = 0, minz = 0, maxx = 0, maxy = 0, maxz = 0;
            for (int i = 1; i < vertices.size(); ++i)
            {
                if (vertices[i].x < vertices[minx].x) minx = i;
                if (vertices[i].y < vertices[miny].y) miny = i;
                if (vertices[i].z < vertices[minz].z) minz = i;

                if (vertices[i].x > vertices[maxx].x) maxx = i;
                if (vertices[i].y > vertices[maxy].y) maxy = i;
                if (vertices[i].z > vertices[maxz].z) maxz = i;
            }

            // Compute the squared distances for the three pairs of points
            value_type distance_squared_x = distance_sqaured(vertices[maxx], vertices[minx]);
            value_type distance_squared_y = distance_sqaured(vertices[maxy], vertices[miny]);
            value_type distance_squared_z = distance_sqaured(vertices[maxz], vertices[minz]);

            // the 2 furthest points
            point min, max;

            min = vertices[minx];
            max = vertices[maxx];

            if (distance_squared_y > distance_squared_x && distance_squared_y > distance_squared_z)
            {
                max = vertices[maxy];
                min = vertices[miny];
            }

            if (distance_squared_z > distance_squared_x && distance_squared_z > distance_squared_y)
            {
                max = vertices[maxz];
                min = vertices[minz];
            }

            return { min, max };
        }

        sphere make_sphere_from_distant_points(std::vector<point> const& vertices)
        {
            auto [min, max] = most_separated_points_on_aabb(vertices);

            point center = (max + min) * 0.5f;
            value_type radius = length(max - center);
            return { center, radius };
        }

        void grow_sphere(sphere& sphere, point const vertex)
        {
            // compute squared distance between point and sphere center
            value_type dist_squared = distance_sqaured(vertex, sphere.center);
            assert(dist_squared != std::numeric_limits<value_type>::infinity());
            // only update if sphere if point is outside it
            if (sphere.radius * sphere.radius < dist_squared)
            {
                value_type new_dist = std::sqrt(dist_squared);
                value_type new_radius = (sphere.radius + new_dist) * 0.5f;
                value_type k = (new_radius - sphere.radius) / new_dist;
                sphere.radius = new_radius;
                sphere.center += (vertex - sphere.center) * k;
            }
        }

        sphere make_ritters_sphere(std::vector<point> const& vertices)
        {
            sphere sphere = make_sphere_from_distant_points(vertices);

            // Grow sphere to include all points
            for (auto& vertex : vertices)
            {
                grow_sphere(sphere, vertex);
            }

            return sphere;
        }

        // covariance matrix is always symmetric
        matrix covariance_matrix(std::vector<point> const& vertices)
        {
            assert(vertices.size() != 0, "size of vector should not be zero!");

            // one over size
            value_type oon = 1.f / vertices.size();
            point center_of_mass{ 0 };
            value_type e00, e11, e22, e01, e02, e12;

            // compute the center of mass (centroid) of the points
            for (auto& vertex : vertices)
                center_of_mass += vertex;
            center_of_mass *= oon;

            // compute covariance elements;
            e00 = e11 = e22 = e01 = e02 = e12 = 0.f;
            for (auto& vertex : vertices)
            {
                // translate points so center of mass is at origin
                point p = vertex - center_of_mass;

                // compute covariance of translated points
                e00 += p.x * p.x;
                e11 += p.y * p.y;
                e22 += p.z * p.z;

                // only need to compute one side of the matrix triangle because result is symmetric matrix
                e01 += p.x * p.y;
                e02 += p.x * p.z;
                e12 += p.y * p.z;
            }

            matrix result;
            // Fill in the covariance matrix elements
            result[0][0] = e00 * oon;
            result[1][1] = e11 * oon;
            result[2][2] = e22 * oon;
            result[0][1] = result[1][0] = e01 * oon;
            result[0][2] = result[2][0] = e02 * oon;
            result[1][2] = result[2][1] = e12 * oon;

            return result;
        }

        sin_cos_pair symschur2(matrix const& a, int p, int q)
        {
            value_type sin, cos;
            if (!epsilon_test(a[p][q]))
            {
                value_type r = (a[q][q] - a[p][p]) / (2.0f * a[p][q]);
                value_type t;
                if (r >= 0.f)
                    t = 1.f / (r + sqrt(1.f + r * r));
                else
                    t = -1.f / (-r + sqrt(1.f + r * r));
                cos = 1.f / sqrt(1.f + t * t);
                sin = t * cos;
            }
            else
            {
                cos = 1.f;
                sin = 0.f;
            }

            return { sin, cos };
        }

        // Computes the eigenvectors and eigenvalues of the symmetric matrix A using
        // the classic Jacobi method of iteratively updating A as A = J^T * A * J,
        // where J = J(p, q, theta) is the Jacobi rotation matrix
        //
        // On exit, v will contain the eigenvectors, and the diagonal elements
        // of a are the corresponding eigenvalues.
        //
        // See Golub, Van Loan, Matrix Computations, 3rd edition, pg 428
        void jacobi(matrix& a, matrix& v)
        {
            int i, j, n, p, q;
            float prevoff{}, c, s;
            matrix J; // , b, t;
            // Initialize v to identify matrix
            for (i = 0; i < dim; i++)
            {
                for (j = 0; j < dim; ++j)
                {
                    v[i][j] = 0.f;
                }
                //v[i][0] = v[i][1] = v[i][2] = 0.0f;
                v[i][i] = 1.0f;
            }
            // Repeat for some maximum number of iterations
            const int MAX_ITERATIONS = 50;
            for (n = 0; n < MAX_ITERATIONS; n++)
            {
                // Find largest off-diagonal absolute element a[p][q]
                p = 0; q = 1;
                for (i = 0; i < 3; i++)
                {
                    for (j = 0; j < 3; j++)
                    {
                        if (i == j) continue;
                        if (abs(a[i][j]) > abs(a[p][q]))
                        {
                            p = i;
                            q = j;
                        }
                    }
                }
                // Compute the Jacobi rotation matrix J(p, q, theta)
                // (This code can be optimized for the three different cases of rotation)
                std::tie(s, c) = symschur2(a, p, q);

                // remove all values in j except for one side of the symmetry
                for (i = 0; i < 3; i++)
                {
                    J[i][0] = J[i][1] = J[i][2] = 0.0f;
                    J[i][i] = 1.0f;
                }

                J[p][p] = c; J[p][q] = s;
                J[q][p] = -s; J[q][q] = c;
                // Cumulate rotations into what will contain the eigenvectors
                v = v * J;
                // Make ’a’ more diagonal, until just eigenvalues remain on diagonal
                a = (J.Transpose() * a) * J;
                // Compute "norm" of off-diagonal elements
                float off = 0.0f;
                for (i = 0; i < dim; i++)
                {
                    for (j = 0; j < dim; j++)
                    {
                        if (i == j) continue;
                        off += a[i][j] * a[i][j];
                    }
                }
                /* off = sqrt(off); not needed for norm comparison */
                // Stop when norm no longer decreasing
                if (n > 2 && off >= prevoff)
                    return;
                prevoff = off;
            }
        }

        sphere make_eigen_sphere(std::vector<point> const& vertices)
        {
            assert(vertices.size() != 0, "size of vector should not be zero!");

            matrix m, v;

            // Compute the covariance matrix m
            m = covariance_matrix(vertices);
            // Decompose it into eigen vectors (in v) and eigenvalues (in m)
            jacobi(m, v);

            // Find the component with largest magnitude and eigen value(largest spread)
            vector e;
            int maxc = 0;
            value_type maxf, maxe = abs(m[0][0]);
            if (( (maxf = abs(m[1][1])) > maxe )) maxc = 1, maxe = maxf;
            if (( (maxf = abs(m[2][2])) > maxe )) maxc = 2, maxe = maxf;
            e[0] = v[0][maxc];
            e[1] = v[1][maxc];
            e[2] = v[2][maxc];

            // Find the most extreme points along direction 'e'
            auto [min, max] = most_separated_points_on_axis(vertices, e);
            value_type dist = length(max - min);
            value_type radius = dist * 0.5f;
            point center = (min + max) * 0.5f;
            return { center, radius };
        }

        sphere make_ritter_eigen_sphere(std::vector<point> const& vertices)
        {
            // start with sphere from maximum spread
            sphere ritter_eigen_sphere = make_eigen_sphere(vertices);

            // Grow sphere to include all points
            for (auto& vertex : vertices)
                grow_sphere(ritter_eigen_sphere, vertex);

            return ritter_eigen_sphere;
        }

        static const std::map<larsons_index, std::vector<point>> larsons_normals =
        {
            {
                // index
                larsons_index::EPOS_6,  
                // normals
                {
                    { 1, 0, 0},
                    { 0, 1, 0},
                    { 0, 0, 1},
                }
            },
            {
                // index
                larsons_index::EPOS_14,
                // normals
                {
                    { 1, 0, 0},
                    { 0, 1, 0},
                    { 0, 0, 1},

                    { 1, 1, 1},
                    { 1, 1,-1},
                    { 1,-1, 1},
                    { 1,-1,-1},
                }
            },
            {
                // index
                larsons_index::EPOS_26,
                // normals
                {
                    { 1, 0, 0},
                    { 0, 1, 0},
                    { 0, 0, 1},

                    { 1, 1, 1},
                    { 1, 1,-1},
                    { 1,-1, 1},
                    { 1,-1,-1},

                    { 1, 1, 0},
                    { 1,-1, 0},
                    { 1, 0, 1},
                    { 1, 0,-1},
                    { 0, 1, 1},
                    { 0, 1,-1},
                }
            },
            {
                // index
                larsons_index::EPOS_98,
                // normals
                {
                    { 1, 0, 0},
                    { 0, 1, 0},
                    { 0, 0, 1},

                    { 1, 1, 1},
                    { 1, 1,-1},
                    { 1,-1, 1},
                    { 1,-1,-1},

                    { 1, 1, 0},
                    { 1,-1, 0},
                    { 1, 0, 1},
                    { 1, 0,-1},
                    { 0, 1, 1},
                    { 0, 1,-1},

                    // 1st set of 12
                    { 0, 1, 2},
                    { 0, 2, 1},
                    { 1, 0, 2},
                    { 2, 0, 1},
                    { 1, 2, 0},
                    { 2, 1, 0},
                    
                    { 0, 1,-2},
                    { 0, 2,-1},
                    { 1, 0,-2},
                    { 2, 0,-1},
                    { 1,-2, 0},
                    { 2,-1, 0},

                    // 2nd set of 12
                    { 1, 1, 2},
                    { 2, 1, 1},
                    { 1, 2, 1},
                    { 1,-1, 2},
                    { 1, 1,-2},
                    { 1,-1,-2},

                    { 2,-1, 1},
                    { 2, 1,-1},
                    { 2,-1,-1},
                    { 1,-2, 1},
                    { 1, 2,-1},
                    { 1,-2,-1},

                    // 3rd set of 12
                    { 2, 2, 1},
                    { 1, 2, 2},
                    { 2, 1, 2},
                    { 2,-2, 1},
                    { 2, 2,-1},
                    { 2,-2,-1},

                    { 1,-2, 2},
                    { 1, 2,-2},
                    { 1,-2,-2},
                    { 2,-1, 2},
                    { 2, 1,-2},
                    { 2,-1,-2},
                }
            },
        };
        
        std::pair<point, point> most_separated_points_on_larsons_normals(std::vector<point> const& vertices, larsons_index index)
        {
            // First retrieve all the normals we will be dotting against
            std::vector<point> normals = larsons_normals.at(index);
            std::vector<std::pair<point, point>> extrema_points;
            for (auto& normal : normals)
            {
                int min = 0, max = 0;
                for (int i = 1; i < vertices.size(); ++i)
                {
                    if(dot(vertices[i], normal) < dot(vertices[min], normal)) min = i;
                    if(dot(vertices[i], normal) > dot(vertices[max], normal)) max = i;
                }
                extrema_points.emplace_back(vertices[min], vertices[max]);
            }

            std::pair<point, point> furthest_pair;
            value_type current_max = minimum;
            for (auto& [ptA, ptB] : extrema_points)
            {
                value_type new_dist = distance_sqaured(ptA, ptB);
                if (new_dist > current_max)
                {
                    furthest_pair = { ptA, ptB };
                    current_max = new_dist;
                }
            }

            return furthest_pair;
        }

        sphere make_sphere_from_larsons_normals(std::vector<point> const& vertices, larsons_index index)
        {
            auto [min, max] = most_separated_points_on_larsons_normals(vertices, index);

            point center = (max + min) * 0.5f;
            value_type radius = length(max - center);
            return { center, radius };
        }

        sphere make_larson_sphere(std::vector<point> const& vertices, larsons_index index)
        {
            sphere sphere = make_sphere_from_larsons_normals(vertices, index);

            // Grow sphere to include all points
            for (auto& vertex : vertices)
            {
                grow_sphere(sphere, vertex);
            }

            return sphere;
        }
    }

}

namespace geom
{
    namespace bsp
    {
        BSPNode* BuildLeafStoringBSPTree(std::vector<polygon*>& polygons, std::size_t depth)
        {
            // Return NULL tree if there are no polygons
            if (polygons.empty())
                return nullptr;

            // Get number of polygons in the input vector
            std::size_t numPolygons = polygons.size();

            // If criterion for a leaf is matched, create a leaf node from remaining polygons
            if (depth >= max_depth || numPolygons <= min_leaf_size)
                return new BSPNode(polygons);

            // Select best possible partitioning plane based on the input geometry
            Plane splitPlane = PickSplittingPlane(polygons);
            std::vector<polygon*> frontList, backList;

            // Test each polygon against the dividing plane, adding them
            // to the front list, back list, or both, as appropriate
            for (polygon* poly : polygons)
            {
                polygon* frontPart, * backPart;
                switch (ClassifyPolygonToPlane(poly, splitPlane))
                {
                case polygon_to_plane::polygon_coplanar_with_plane:
                    // What’s done in this case depends on what type of tree is being
                    // built. For a node-storing tree, the polygon is stored inside
                    // the node at this level (along with all other polygons coplanar
                    // with the plane). Here, for a leaf-storing tree, coplanar polygons
                    // are sent to either side of the plane. In this case, to the front
                    // side, by falling through to the next case
                case polygon_to_plane::polygon_in_front_of_plane:
                    frontList.emplace_back(poly);
                    break;
                case polygon_to_plane::polygon_behind_plane:
                    backList.emplace_back(poly);
                    break;
                case polygon_to_plane::polygon_straddling_plane:
                    // Split polygon to plane and send a part to each side of the plane
                    SplitPolygon(*poly, splitPlane, &frontPart, &backPart);
                    frontList.emplace_back(frontPart);
                    backList.emplace_back(backPart);
                    break;
                }
            }
            // Recursively build child subtrees and return new tree root combining them
            BSPNode* frontTree = BuildLeafStoringBSPTree(frontList, depth + 1);
            BSPNode* backTree = BuildLeafStoringBSPTree(backList, depth + 1);
            return new BSPNode(frontTree, backTree);
        }

        plane GetPlaneFromPolygon(polygon& polygon)
        {
            point vert1 = polygon.vertex_at(0);
            point vert2 = polygon.vertex_at(1);

            vector normal = normalize(cross(vert1, vert2));
            value_type dist = dot(vert1, normal);

            return plane{ normal, dist };
        }

        // Given a vector of polygons, attempts to compute a good splitting plane
        plane PickSplittingPlane(std::vector<polygon*>& polygons)
        {
            // Blend factor for optimizing for balance or splits (should be tweaked)
            const value_type K = 0.8f;
            // Variables for tracking best splitting plane seen so far
            plane bestPlane;
            value_type bestScore = std::numeric_limits<value_type>::max();
            // Try the plane of each polygon as a dividing plane
            for (std::size_t i = 0; i < polygons.size(); i++)
            {
                std::int32_t numInFront = 0, numBehind = 0, numStraddling = 0;
                plane plane = GetPlaneFromPolygon(*polygons[i]);
                // Test against all other polygons
                for (std::size_t j = 0; j < polygons.size(); j++)
                {
                    // Ignore testing against self
                    if (i == j)
                        continue;

                    // Keep standing count of the various poly-plane relationships
                    switch (ClassifyPolygonToPlane(polygons[j], plane))
                    {
                    case polygon_to_plane::polygon_coplanar_with_plane:
                        /* Coplanar polygons treated as being in front of plane */
                    case polygon_to_plane::polygon_in_front_of_plane:
                        numInFront++;
                        break;
                    case polygon_to_plane::polygon_behind_plane:
                        numBehind++;
                        break;
                    case polygon_to_plane::polygon_straddling_plane:
                        numStraddling++;
                        break;
                    }
                }

                // Compute score as a weighted combination (based on K, with K in range
                // 0..1) between balance and splits (lower score is better)
                value_type score = K * numStraddling + (1.0f - K) * std::abs(numInFront - numBehind);
                if (score < bestScore)
                {
                    bestScore = score;
                    bestPlane = plane;
                }
            }
            return bestPlane;
        }

        // Classify point p to a plane thickened by a given thickness epsilon
        point_to_plane ClassifyPointToPlane(point p, plane plane)
        {
            // Compute signed distance of point from plane
            value_type dist = dot(plane.normal, p) - plane.dist;
            // Classify p based on the signed distance
            if (dist > plane_thickness_epsilon)
                return point_to_plane::point_in_front_of_plane;
            if (dist < -plane_thickness_epsilon)
                return point_to_plane::point_behind_plane;
            return point_to_plane::point_on_plane;
        }

        // Return value specifying whether the polygon ‘poly’ lies in front of,
        // behind of, on, or straddles the plane ‘plane’
        polygon_to_plane ClassifyPolygonToPlane(polygon const* poly, plane plane)
        {
            // Loop over all polygon vertices and count how many vertices
            // lie in front of and how many lie behind of the thickened plane
            std::int32_t numInFront = 0, numBehind = 0;
            std::int32_t numVerts = poly->size();
            for (std::size_t i = 0; i < numVerts; i++)
            {
                point p = poly->vertex_at(i);
                switch (ClassifyPointToPlane(p, plane))
                {
                case point_to_plane::point_in_front_of_plane:
                    numInFront++;
                    break;
                case point_to_plane::point_behind_plane:
                    numBehind++;
                    break;
                }
            }
            // If vertices on both sides of the plane, the polygon is straddling
            if (numBehind != 0 && numInFront != 0)
                return polygon_to_plane::polygon_straddling_plane;
            // If one or more vertices in front of the plane and no vertices behind
            // the plane, the polygon lies in front of the plane
            if (numInFront != 0)
                return polygon_to_plane::polygon_in_front_of_plane;
            // Ditto, the polygon lies behind the plane if no vertices in front of
            // the plane, and one or more vertices behind the plane
            if (numBehind != 0)
                return polygon_to_plane::polygon_behind_plane;
            // All vertices lie on the plane so the polygon is coplanar with the plane
            return polygon_to_plane::polygon_coplanar_with_plane;
        }

        void SplitPolygon(polygon& poly, plane plane, polygon** frontPoly, polygon** backPoly)
        {
            static constexpr std::size_t MAX_POINTS = 30; //poly.size() * 3;

            std::size_t numFront = 0, numBack = 0;
            point frontVerts[MAX_POINTS], backVerts[MAX_POINTS];
            // Test all edges (a, b) starting with edge from last to first vertex
            std::size_t numVerts = poly.size();
            point a = poly.vertex_at(numVerts - 1);
            point_to_plane aSide = ClassifyPointToPlane(a, plane);
            // Loop over all edges given by vertex pair (n - 1, n)
            for (std::size_t n = 0; n < numVerts; n++)
            {
                point b = poly.vertex_at(n);
                point_to_plane bSide = ClassifyPointToPlane(b, plane);
                if (bSide == point_to_plane::point_in_front_of_plane)
                {
                    if (aSide == point_to_plane::point_behind_plane)
                    {
                        // Edge (a, b) straddles, output intersection point to both sides
                        // Consistently clip edge as ordered going from in front -> behind
                        point i = IntersectEdgeAgainstPlane(b, a, plane);
                        assert(ClassifyPointToPlane(i, plane) == point_to_plane::point_on_plane);
                        frontVerts[numFront++] = backVerts[numBack++] = i;
                    }
                    // In all three cases, output b to the front side
                    frontVerts[numFront++] = b;
                }
                else if (bSide == point_to_plane::point_behind_plane)
                {
                    if (aSide == point_to_plane::point_in_front_of_plane)
                    {
                        // Edge (a, b) straddles plane, output intersection point
                        point i = IntersectEdgeAgainstPlane(a, b, plane);
                        assert(ClassifyPointToPlane(i, plane) == point_to_plane::point_on_plane);
                        frontVerts[numFront++] = backVerts[numBack++] = i;
                    }
                    else if (aSide == point_to_plane::point_on_plane)
                    {
                        // Output a when edge (a, b) goes from ‘on’ to ‘behind’ plane
                        backVerts[numBack++] = a;
                    }
                    // In all three cases, output b to the back side
                    backVerts[numBack++] = b;
                }
                else
                {
                    // b is on the plane. In all three cases output b to the front side
                    frontVerts[numFront++] = b;
                    // In one case, also output b to back side
                    if (aSide == point_to_plane::point_behind_plane)
                        backVerts[numBack++] = b;
                }

                // Keep b as the starting point of the next edge
                a = b;
                aSide = bSide;
            }

            // Create (and return) two new polygons from the two vertex lists
            *frontPoly = new polygon(frontVerts, numFront);
            *backPoly = new polygon(backVerts, numBack);
        }
    }


}
