#include <geom.h>
#include <iostream>

using namespace geom;

int main()
{
    using Vec3D         = Vector<float, 3>;
    using Point3D       = Point<3>;
    using Sphere3D      = Sphere<3>;
    using Triangle3D    = Triangle<3>;
    using AABB3D        = AABB<3>;
    using Plane3D       = Plane<3>;
    using Ray3D         = Ray<3>;

    Vec3D base_dir{ 0, 0, 1 };
    Point3D base_point{ 0.5f, 0.5f, 0.f };
    Sphere3D base_sphere{ base_point, 0.5f };
    Triangle3D base_triangle{ {0, 0, 0}, {1, 0, 0}, {0, 1, 0} }; //ccw
    AABB3D base_aabb{ {0, 0, 0}, {1, 1, 1} };
    Plane3D base_plane{ base_dir, 0 };
    Ray3D base_ray{ base_point, base_dir };

    {
        //Vec3D dir = base_dir;
        Point3D point       = base_point;
        Sphere3D sphere     = base_sphere;
        Triangle3D triangle = base_triangle;
        AABB3D aabb         = base_aabb;
        Plane3D plane       = base_plane;
        Ray3D ray           = base_ray;
        
        std::cout << "=========================================================================================\n";
        std::cout << "Top Left Quadrant Test\n";
        std::cout << "=========================================================================================\n";
        std::cout << "sphere vs sphere   :\t"     << intersection::sphere_sphere    (sphere, sphere)       << "\n";
        std::cout << "aabb   vs sphere   :\t"     << intersection::aabb_sphere      (aabb, sphere)         << "\n";
        std::cout << "aabb   vs aabb     :\t"     << intersection::aabb_aabb        (aabb, aabb)           << "\n";
        std::cout << "point  vs sphere   :\t"     << intersection::point_sphere     (point, sphere)        << "\n";
        std::cout << "point  vs aabb     :\t"     << intersection::point_aabb       (point, aabb)          << "\n";
        std::cout << "point  vs triangle :\t"     << intersection::point_triangle   (point, triangle)      << "\n";
        std::cout << "point  vs plane    :\t"     << intersection::point_plane      (point, plane)         << "\n";
        std::cout << "ray    vs plane    :\t"     << intersection::ray_plane        (ray, plane)           << "\n";
        std::cout << "ray    vs aabb     :\t"     << intersection::ray_aabb         (ray, aabb)            << "\n";
        std::cout << "ray    vs sphere   :\t"     << intersection::ray_sphere       (ray, sphere)          << "\n";
        std::cout << "ray    vs triangle :\t"     << intersection::ray_triangle     (ray, triangle)        << "\n";
        std::cout << "plane  vs aabb     :\t"     << intersection::plane_aabb       (plane, aabb)          << "\n";
        std::cout << "plane  vs sphere   :\t"     << intersection::plane_sphere     (plane, sphere)        << "\n";
        std::cout << "=========================================================================================\n";
    }

    {
        //Vec3D dir = base_dir;
        Point3D point       = { -0.5f, 0.5f, 0.f };
        Sphere3D sphere     = { point, 0.5f };  //radius always positive.
        Triangle3D triangle = { {0, 0, 0}, {0, 1, 0}, {-1, 0, 0} }; //ccw
        AABB3D aabb         = { {0, 0, 0}, {-1, 1, 1} };
        Plane3D plane       = { base_dir, 0 };
        Ray3D ray           = { point, base_dir };

        std::cout << "Top Right Quadrant Test\n";
        std::cout << "=========================================================================================\n";
        std::cout << "sphere vs sphere   :\t"     << intersection::sphere_sphere    (sphere, sphere)       << "\n";
        std::cout << "aabb   vs sphere   :\t"     << intersection::aabb_sphere      (aabb, sphere)         << "\n";
        std::cout << "aabb   vs aabb     :\t"     << intersection::aabb_aabb        (aabb, aabb)           << "\n";
        std::cout << "point  vs sphere   :\t"     << intersection::point_sphere     (point, sphere)        << "\n";
        std::cout << "point  vs aabb     :\t"     << intersection::point_aabb       (point, aabb)          << "\n";
        std::cout << "point  vs triangle :\t"     << intersection::point_triangle   (point, triangle)      << "\n";
        std::cout << "point  vs plane    :\t"     << intersection::point_plane      (point, plane)         << "\n";
        std::cout << "ray    vs plane    :\t"     << intersection::ray_plane        (ray, plane)           << "\n";
        std::cout << "ray    vs aabb     :\t"     << intersection::ray_aabb         (ray, aabb)            << "\n";
        std::cout << "ray    vs sphere   :\t"     << intersection::ray_sphere       (ray, sphere)          << "\n";
        std::cout << "ray    vs triangle :\t"     << intersection::ray_triangle     (ray, triangle)        << "\n";
        std::cout << "plane  vs aabb     :\t"     << intersection::plane_aabb       (plane, aabb)          << "\n";
        std::cout << "plane  vs sphere   :\t"     << intersection::plane_sphere     (plane, sphere)        << "\n";
        std::cout << "=========================================================================================\n";
    }

    {
        //Vec3D dir = base_dir;
        Point3D point       = { -0.5f, -0.5f, 0.f };
        Sphere3D sphere     = { point, 0.5f };  //radius always positive.
        Triangle3D triangle = { {0, 0, 0}, {-1, 0, 0}, {0, -1, 0} }; //ccw
        AABB3D aabb         = { {0, 0, 0}, {-1, -1, 1} };
        Plane3D plane       = { base_dir, 0 };
        Ray3D ray           = { point, base_dir };

        std::cout << "Bottom Left Quadrant Test\n";
        std::cout << "=========================================================================================\n";
        std::cout << "sphere vs sphere   :\t"     << intersection::sphere_sphere    (sphere, sphere)       << "\n";
        std::cout << "aabb   vs sphere   :\t"     << intersection::aabb_sphere      (aabb, sphere)         << "\n";
        std::cout << "aabb   vs aabb     :\t"     << intersection::aabb_aabb        (aabb, aabb)           << "\n";
        std::cout << "point  vs sphere   :\t"     << intersection::point_sphere     (point, sphere)        << "\n";
        std::cout << "point  vs aabb     :\t"     << intersection::point_aabb       (point, aabb)          << "\n";
        std::cout << "point  vs triangle :\t"     << intersection::point_triangle   (point, triangle)      << "\n";
        std::cout << "point  vs plane    :\t"     << intersection::point_plane      (point, plane)         << "\n";
        std::cout << "ray    vs plane    :\t"     << intersection::ray_plane        (ray, plane)           << "\n";
        std::cout << "ray    vs aabb     :\t"     << intersection::ray_aabb         (ray, aabb)            << "\n";
        std::cout << "ray    vs sphere   :\t"     << intersection::ray_sphere       (ray, sphere)          << "\n";
        std::cout << "ray    vs triangle :\t"     << intersection::ray_triangle     (ray, triangle)        << "\n";
        std::cout << "plane  vs aabb     :\t"     << intersection::plane_aabb       (plane, aabb)          << "\n";
        std::cout << "plane  vs sphere   :\t"     << intersection::plane_sphere     (plane, sphere)        << "\n";
        std::cout << "=========================================================================================\n";
    }

    {
        //Vec3D dir = base_dir;
        Point3D point       = { 0.5f, -0.5f, 0.f };
        Sphere3D sphere     = { point, 0.5f };  //radius always positive.
        Triangle3D triangle = { {0, 0, 0}, {0, -1, 0}, {1, 0, 0} }; //ccw
        AABB3D aabb         = { {0, 0, 0}, {1, -1, 1} };
        Plane3D plane       = { base_dir, 0 };
        Ray3D ray           = { point, base_dir };

        std::cout << "Bottom Right Quadrant Test\n";
        std::cout << "=========================================================================================\n";
        std::cout << "sphere vs sphere   :\t"     << intersection::sphere_sphere    (sphere, sphere)       << "\n";
        std::cout << "aabb   vs sphere   :\t"     << intersection::aabb_sphere      (aabb, sphere)         << "\n";
        std::cout << "aabb   vs aabb     :\t"     << intersection::aabb_aabb        (aabb, aabb)           << "\n";
        std::cout << "point  vs sphere   :\t"     << intersection::point_sphere     (point, sphere)        << "\n";
        std::cout << "point  vs aabb     :\t"     << intersection::point_aabb       (point, aabb)          << "\n";
        std::cout << "point  vs triangle :\t"     << intersection::point_triangle   (point, triangle)      << "\n";
        std::cout << "point  vs plane    :\t"     << intersection::point_plane      (point, plane)         << "\n";
        std::cout << "ray    vs plane    :\t"     << intersection::ray_plane        (ray, plane)           << "\n";
        std::cout << "ray    vs aabb     :\t"     << intersection::ray_aabb         (ray, aabb)            << "\n";
        std::cout << "ray    vs sphere   :\t"     << intersection::ray_sphere       (ray, sphere)          << "\n";
        std::cout << "ray    vs triangle :\t"     << intersection::ray_triangle     (ray, triangle)        << "\n";
        std::cout << "plane  vs aabb     :\t"     << intersection::plane_aabb       (plane, aabb)          << "\n";
        std::cout << "plane  vs sphere   :\t"     << intersection::plane_sphere     (plane, sphere)        << "\n";
        std::cout << "=========================================================================================\n";
    }
    
    return 0;
}