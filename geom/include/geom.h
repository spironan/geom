#pragma once
#include <cstddef>
#include <array>
#include <iostream>
#include <cassert>
#include <vector>       // std::vector
#include <algorithm>    // std::sort
namespace geom
{
    using size_type = size_t;
    using value_type = float;
    using index_type = int;

    template<typename T, size_type size>
    struct Vector;

    template <size_type size>
    using Point = Vector<value_type, size>;

    template<typename T, size_type size>
    struct Vector
    {
        union 
        {
            std::array<T, size> data;
            struct { T x, y, z, w; };
            // include the type you want here.
        };

        Vector<T, size> operator-() const
        {
            Vector<T, size> result{};

            for (size_type i = 0; i < size; ++i)
                result[i] = -data[i];

            return result;
        }

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
        
        Vector<T, size> operator+(value_type scalar) const
        {
            Vector<T, size> result{};

            for (size_type i = 0; i < size; ++i)
                result[i] = data[i] + scalar;

            return result;
        }

        Vector<T, size> operator*(value_type scalar) const
        {
            Vector<T, size> result{};

            for(size_type i = 0; i < size; ++i)
                result[i] = data[i] * scalar;

            return result;
        }

        Vector<T, size> operator/(value_type scalar) const
        {
            Vector<T, size> result{};

            for (size_type i = 0; i < size; ++i)
                result[i] = data[i] / scalar;

            return result;
        }

        Vector<T, size>& operator+=(Vector<T, size> const& other)
        {
            *this = *this + other;
            return *this;
        }

        Vector<T, size>& operator+=(value_type scalar)
        {
            *this = *this + scalar;
            return *this;
        }

        Vector<T, size>& operator-=(Vector<T, size> const& other)
        {
            *this = *this - other;
            return *this;
        }

        Vector<T, size>& operator*=(value_type scalar)
        {
            *this = *this * scalar;
            return *this;
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

        friend Vector<value_type, size> operator*(value_type scalar, Vector<value_type, size> const& vector)
        {
            return vector * scalar;
        }

        friend std::ostream& operator<<(std::ostream& oss, Vector<value_type, size> const& vector)
        {
            oss << vector.x << "," << vector.y << "," << vector.z << "\n";
            return oss;
        }
        
    };

    template<typename T, size_type dim>
    struct SquareMatrix
    {
        using type = Vector<T, dim>;
        std::array<type, dim> data;

        type& operator[](size_type index)
        {
            return data[index];
        }

        type const& operator[](size_type index) const
        {
            return data[index];
        }

        SquareMatrix Transpose() const
        {
            SquareMatrix<T, dim> transpose;

            for (int i = 0; i < dim; ++i)
            {
                for (int j = 0; j < dim; ++j)
                {
                    transpose[i][j] = data[j][i];
                }
            }

            return transpose;
        }

        SquareMatrix operator*(SquareMatrix const& other)
        {
            // need to do square matrix multiply!!
            SquareMatrix result = {};

            auto transpose = other.Transpose();
            for (int i = 0; i < dim; ++i)
            {
                for(int j = 0; j < dim; ++j)
                    result[i][j] = dot(operator[](i), transpose[j]);
            }

            return result;
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
    using matrix    = SquareMatrix<value_type, dim>;

    // shapes
    using plane     = Plane<dim>;
    using triangle  = Triangle<dim>;
    using sphere    = Sphere<dim>;
    using aabb      = AABB<dim>;
    using ray       = Ray<dim>;
    
    static constexpr value_type epsilon = std::numeric_limits<value_type>::epsilon();
    static constexpr value_type maximum = std::numeric_limits<value_type>::max();
    static constexpr value_type minimum = std::numeric_limits<value_type>::min();
    static constexpr point min_point = { minimum };
    static constexpr point max_point = { maximum };

    bool epsilon_test(value_type value);

    value_type min(value_type lhs, value_type rhs);
    value_type max(value_type lhs, value_type rhs);

    //value_type abs(value_type val);
    value_type dot(vector ptA, vector ptB);
    value_type distance_sqaured(point ptA, point ptB);
    value_type distance_sqaured(point pt, aabb const& aabb);
    value_type length_squared(point pt);
    value_type length(point pt);

    vector normalize(vector vec);
    vector cross(vector vecA, vector vecB);
    bool same_side(point p1, point p2, point a, point b);

    // intersection between geometry support
    namespace intersection
    {
        using time_type = value_type;

        struct RaycastResult
        {
            bool intersect = false;
            time_type t_entry = minimum, t_exit = maximum;
            point p_entry = min_point, p_exit = max_point;

            friend std::ostream& operator<<(std::ostream& oss, RaycastResult const& result)
            {
                oss << "intersect: " << (result.intersect ? "true" : "false") << "\n";
                oss << "time at entry: " << result.t_entry << ", time at exit: " << result.t_exit << "\n";
                oss << "point at entry: " << result.p_entry << ", point at exit: " << result.p_exit << "\n";
                return oss;
            }
        };

        bool sphere_sphere    (sphere const& sphereA, sphere const& sphereB);
        bool aabb_sphere      (aabb const& aabb, sphere const& sphere);
        bool aabb_aabb        (aabb const& aabbA, aabb const& aabbB);

        bool point_sphere     (point const& point, sphere const& sphere);
        bool point_aabb       (point const& point, aabb const& aabb);
        bool point_triangle   (point const& point, triangle const& triangle);
        bool point_plane      (point const& point, plane const& plane);         // assumes plane's normal is normalized

        RaycastResult ray_plane         (ray const& ray, plane const& plane);
        RaycastResult ray_aabb          (ray const& ray, aabb const& aabb);
        RaycastResult ray_sphere        (ray const& ray, sphere const& sphere);
        RaycastResult ray_triangle      (ray const& ray, triangle const& triangle);

        bool ray_sphere_bool       (ray const& ray, sphere const& sphere);

        bool plane_aabb       (plane const& plane, aabb const& aabb);
        bool plane_sphere     (plane const& plane, sphere const& sphere);
    }

    // bounding volume related functions and structure support
    namespace bvh
    {
        std::pair<point, point> most_separated_points_on_axis(std::vector<point> const& vertices, vector const& normal);

        aabb make_fitting_aabb(std::vector<point> const& vertices);
        sphere make_ritters_sphere(std::vector<point> const& vertices);
        
        // covariance matrix is always symmetric
        //matrix covariance_matrix(std::vector<point> const& vertices);;

        // 2-by-2 symmetric schur decomposition. Given an n-by-n symmetric matrix
        // and indices p, q such that 1 <= p < q <= n, computes a sine-cosine pair
        // (s,c) that will serve to forma Jacobi rotation matrix
        //
        // See Golub, Van Loan, Matrix Computations, 3rd edition, pg428
        using sin_cos_pair = std::pair<value_type, value_type>;
        //sin_cos_pair symschur2(matrix const& a, int p, int q);

        // Computes the eigenvectors and eigenvalues of the symmetric matrix A using
        // the classic Jacobi method of iteratively updating A as A = J^T * A * J,
        // where J = J(p, q, theta) is the Jacobi rotation matrix
        //
        // On exit, v will contain the eigenvectors, and the diagonal elements
        // of a are the corresponding eigenvalues.
        //
        // See Golub, Van Loan, Matrix Computations, 3rd edition, pg 428
        //void jacobi(matrix& a, matrix& v);

        sphere make_eigen_sphere(std::vector<point> const& vertices);

        sphere make_ritter_eigen_sphere(std::vector<point> const& vertices);

        enum class larsons_index
        {
            EPOS_6, // aka ritters
            EPOS_14,
            EPOS_26,
            EPOS_98,
        };
        sphere make_larson_sphere(std::vector<point> const& vertices, larsons_index index);

        template<typename volume>
        struct Node
        {
            // Leaf Node construction
            Node(volume volume)
                : Volume { volume }
                , Left { nullptr }
                , Right { nullptr }
                , Parent { nullptr }
                , Height { 0 }
            {
            }

            //Inner-Node Construction
            Node(Node* left, Node* right)
                : Left { left }
                , Right { right }
                , Parent { nullptr }
                , Height { left->Height - 1 }
                , Volume { }
            {
                Left->Parent = Right->Parent = this;
            }

            Node* Left;
            Node* Right;
            Node* Parent;
            std::size_t Height;

            volume Volume;
            bool Leaf;
        };

        using node_box = Node<aabb>;
        using node_sphere = Node<sphere>;

        /*enum class construct_type
        {
            bottom_up,
            top_down,
        };*/

        enum class Heuristic
        {
            // top-down
            median_of_centers,
            median_of_extents,
            k_even_split,
            
            // bottom-up
            nearest_neighbour,
            minimum_combined_volume,
            minimum_relative_increase,
        };

        enum class EndCondition
        {
            n_nodes_reached,
            max_depth,
        };
        
        struct Settings
        {
            Heuristic heuristics = Heuristic::median_of_centers;
            EndCondition end_condition = EndCondition::n_nodes_reached;
            std::size_t cutoff_amount = 1;
            std::size_t evenly_spaced_points = 10;  // minimally 2 points
        };

        class Hierarchy
        {
        public:
            template<typename volume>
            static Node<volume>* construct_top_down(std::vector<volume>& insertionData, Settings Settings)
            {
                if (ShouldTerminate(Settings, insertionData))
                    return new Node(insertionData.front());

                std::vector<volume> left, right;
                PartitionObjects(Settings, insertionData, left, right);
                return new Node(construct_top_down<volume>(left, Settings), construct_top_down<volume>(right, Settings));
            }
        
        private:
            static bool ShouldTerminate(Settings settings, std::vector<aabb>& insertionData) 
            {
                switch (settings.end_condition)
                {
                case EndCondition::n_nodes_reached:
                    {
                        return insertionData.size() <= settings.cutoff_amount;
                    }
                    break;
                case EndCondition::max_depth:
                    {
                        return false;
                        //return current_height <= settings.cutoff_amount;
                    }
                    break;
                }

                return true; 
            }

            static void PartitionObjects(Settings settings, std::vector<aabb>& insertionData, std::vector<aabb>& left, std::vector<aabb>& right)
            {
                switch (settings.heuristics)
                {
                case Heuristic::median_of_centers:
                    {
                        //sort data by axis
                        std::sort(insertionData.begin(), insertionData.end(), [](auto&& lhs, auto&& rhs) 
                            {
                                auto lhs_center = (lhs.min + lhs.max) * 0.5f;
                                auto rhs_center = (rhs.min + rhs.max) * 0.5f;
                                return lhs_center.x < rhs_center.x;
                            });
                        std::copy(insertionData.begin(), insertionData.begin() + insertionData.size() / 2, left.begin());
                        std::copy(insertionData.begin() + insertionData.size() / 2, insertionData.end(), right.begin());
                    }
                    break;
                case Heuristic::median_of_extents:
                    {
                        std::vector<vector> extents;
                        for (auto& bv : insertionData)
                        {
                            extents.emplace_back(bv.min);
                            extents.emplace_back(bv.max);
                        }
                        // sort extents based on y-axis
                        std::sort(extents.begin(), extents.end(), [](auto&& lhs, auto&& rhs)
                            {
                                return lhs.y < rhs.y;
                            });

                        // split point based on median of all extents projected along y-axis
                        vector split_point = extents[extents.size() / 2];

                        for (auto& bv : insertionData)
                        {
                            auto bv_center = (bv.min + bv.max) * 0.5f;
                            // we only care about their y values.
                            bv_center.x = bv_center.z = 0.0;
                            split_point.x = split_point.z = 0.0;
                            if (dot(bv_center, split_point) > 0.0)
                            {
                                left.emplace_back(bv);
                            }
                            else
                            {
                                right.emplace_back(bv);
                            }
                        }
                    }
                    break;
                case Heuristic::k_even_split:
                    {
                        std::vector<vector> extents;
                        for (auto& bv : insertionData)
                        {
                            extents.emplace_back(bv.min);
                            extents.emplace_back(bv.max);
                        }
                        vector axis = { 0,1,0 };
                        auto [lowest, highest] = most_separated_points_on_axis(extents, axis);
                        
                        auto increments = (highest - lowest) / settings.evenly_spaced_points;
                        vector split_point = lowest + settings.evenly_spaced_points / 2.0 * increments;

                        for (auto& bv : insertionData)
                        {
                            auto bv_center = (bv.min + bv.max) * 0.5f;
                            // we only care about their y values.
                            bv_center.x = bv_center.z = 0.0;
                            split_point.x = split_point.z = 0.0;
                            if (dot(bv_center, split_point) > 0.0)
                            {
                                left.emplace_back(bv);
                            }
                            else
                            {
                                right.emplace_back(bv);
                            }
                        }
                    }
                    break;

                    // Bottom-Up Approach
                case Heuristic::nearest_neighbour:
                    {

                    }
                    break;
                case Heuristic::minimum_combined_volume:
                    {

                    }
                    break;
                case Heuristic::minimum_relative_increase:
                    {

                    }
                    break;

                default:
                    break;
                }
            }

        };



    }
}
