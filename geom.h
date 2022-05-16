#pragma once
#include <cstddef>

namespace geom
{
    using size_type = size_t;
    using value_type = float;

    template <size_type size>
    struct Point;

    template<typename T, size_type size>
    struct Vector
    {
        union 
        {
            T data[size];
            T x, y, z, w;
            // include the type you want here.
        };
        

        Vector<T, size> operator-(Vector<T, size> const& other) const
        {
            Vector<T, size> result{};

            for(size_type i = 0; i < size; ++i)
                result[i] = data[i] - other[i];

            return result;
        }
        
        Vector<T, size> operator+(Vector<T, size> const& other) const
        {
            Vector<T, size> result{};

            for(size_type i = 0; i < size; ++i)
                result[i] = data[i] + other[i];

            return result;
        }

        Vector<T, size> operator*(value_type scalar) const
        {
            Vector<T, size> result{};

            for(size_type i = 0; i < size; ++i)
                result[i] = data[i] * scalar;

            return result;
        }

        value_type& operator[](size_type index)
        {
            return data[index];
        }
        
        value_type const& operator[](size_type index) const
        {
            return data[index];
        }

        friend Vector<value_type, size> operator/(value_type scalar, Vector<value_type, size> const& vector)
        {
            Vector<value_type, size> result;
            
            for(size_type i = 0; i < size; ++i)
            {
                result[i] = scalar / vector[i];
            }

            return result;
        }

        explicit operator Point<size>() const
        {
            return Point<size>(*this);
        }
    };

    template <size_type size>
    struct Point
    {
        using vector = Vector<value_type, size>;
        vector data;

        value_type& operator[](size_type index)
        {
            return data[index];
        }
        
        value_type const& operator[](size_type index) const
        {
            return data[index];
        }

        vector operator-(Point<size> const& other) const
        {
            return data - other.data;
        }

        vector operator-(vector const& other) const
        {
            return data - other;
        }

        vector operator+(Point<size> const& other) const
        {
            return data + other.data;
        }

        vector operator*(value_type scalar) const
        {
            return data * scalar;
        }
        
        operator vector() const
        {
            return data;
        }
    };

    template<size_type dim>
    struct Sphere
    {
        Point<dim> center;
        value_type radius;
    };

    template<size_type dim>
    struct Triangle
    {
        Point<dim> a, b, c;
    };

    template<size_type dim>
    struct AABB
    {
        Point<dim> min, max;
    };

    template<size_type dim>
    struct Plane
    {
        Vector<value_type, dim> normal;
        value_type dist;
    };

    template<size_type dim>
    struct Ray
    {
        Point<dim> point;
        Vector<value_type, dim> dir;
    };

    
    // Functionality
    static constexpr size_type dim = 3;
    using vector    = Vector<value_type, dim>;
    using point     = Point<dim>;
    using plane     = Plane<dim>;
    using triangle  = Triangle<dim>;
    using sphere    = Sphere<dim>;
    using aabb      = AABB<dim>;
    using ray       = Ray<dim>;
    
    //value_type abs(value_type val);
    value_type dot(vector ptA, vector ptB);
    value_type distance_sqaured(point ptA, point ptB);
    value_type squared_distance(point pt, aabb const& aabb);
    
    vector cross(vector vecA, vector vecB);
    bool same_side(point p1, point p2, point a, point b);

    // intersection between geometry support
    namespace intersection
    {
        bool sphere_sphere    (sphere const& sphereA, sphere const& sphereB);
        bool aabb_sphere      (aabb const& aabb, sphere const& sphere);
        bool aabb_aabb        (aabb const& aabbA, aabb const& aabbB);

        bool point_sphere     (point const& point, sphere const& sphere);
        bool point_aabb       (point const& point, aabb const& aabb);
        bool point_triangle   (point const& point, triangle const& triangle);
        bool point_plane      (point const& point, plane const& plane);         // assumes plane's normal is normalized

        bool ray_plane        (ray const& ray, plane const& plane);
        bool ray_aabb         (ray const& ray, aabb const& aabb);
        bool ray_sphere       (ray const& ray, sphere const& sphere);
        bool ray_triangle     (ray const& ray, triangle const& triangle);

        bool plane_aabb       (plane const& plane, aabb const& aabb);
        bool plane_sphere     (plane const& plane, sphere const& sphere);
    }

}
