#pragma once
#include <cstddef>
#include <array>
#include <iostream>
#include <cassert>
#include <vector>       // std::vector
#include <algorithm>    // std::sort
#include <memory>       
#include <limits>

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
        #pragma warning( push ) 
        #pragma warning( disable: 4201 ) // nonstandard extension used : nameless struct/union
        union 
        {
            std::array<T, size> data;
            struct { T x, y, z, w; };
            // include the type you want here.
        };
        #pragma warning( pop )

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
            SquareMatrix<T, dim> transpose{};

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

    template<size_type dim>
    struct Polygon
    {
        std::vector<Point<dim>> vertices;
        
        Polygon(Point<dim>* point, std::size_t size)
        {
            std::size_t iterations = size;
            while (iterations--)
            {
                vertices.emplace_back(*point);
                point++;
            }
        }

        std::size_t size() const { return vertices.size(); }
        Point<dim> vertex_at(std::size_t index) const { return vertices[index]; }
    };
    
    // Functionality
    static constexpr value_type pi = static_cast<value_type>(3.141592653589793238462643383279502884L);
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
    using polygon   = Polygon<dim>;
    
    static constexpr value_type epsilon = std::numeric_limits<value_type>::epsilon();
    static constexpr value_type maximum = std::numeric_limits<value_type>::max();
    static constexpr value_type minimum = std::numeric_limits<value_type>::lowest();
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

    template<typename T>
    value_type volume_of(T const& shape)
    {
        value_type vol = maximum;

        if constexpr (std::is_same_v<T, aabb>)
        {
            vol = dot(shape.max - shape.min, vector{ value_type{1.0} });
        }
        else if constexpr (std::is_same_v<T, sphere>)
        {
            vol = value_type{ 4.0 } / value_type{ 3.0 } * pi * shape.radius * shape.radius;
        }

        return vol;
    }

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
        
        using sin_cos_pair = std::pair<value_type, value_type>;
        //sin_cos_pair symschur2(matrix const& a, int p, int q);

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
            Node()
                : Volume{ volume{} }
                , Left { nullptr }
                , Right { nullptr }
                , Depth{ 0 }
            {
            }

            std::shared_ptr<Node> Left;
            std::shared_ptr<Node> Right;
            //Node* Parent;
            std::size_t Depth;

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
            // nearest neighbour with squared distance
            value_type nearest_neighbour_factor = 1.0f;
            // minimum combined volume
            value_type combined_volume_factor = 1.0f;
            // minimum combined volume
            value_type minimum_volume_relative_increase_factor = 1.0f;
        };

        class Hierarchy
        {
        public:
            template<typename volume>
            static std::shared_ptr<Node<volume>> construct_top_down(std::vector<volume>& insertionData, Settings Settings)
            {
                assert(insertionData.size() > 0);
                std::shared_ptr<Node<volume>> root = nullptr;
                std::size_t depth = 0;
                root = construct_top_down_recursion(&root, insertionData, Settings, depth);

                return root;
            }

            template<typename volume>
            static std::shared_ptr<Node<volume>> construct_bottom_up(std::vector<volume>& insertionData, Settings Settings)
            {
                assert(insertionData.size() > 0);

                std::vector<std::shared_ptr<Node<volume>>> pNodes;
                /*std::size_t numObjects = insertionData.size();
                std::shared_ptr<Node<volume>> pNodes = new Node<volume>(insertionData.size());*/
                for (auto& vol : insertionData)
                {
                    std::shared_ptr<Node<volume>> newNode = std::make_shared<Node<volume>>();
                    newNode->Leaf = true;
                    newNode->Left = newNode->Right = nullptr;
                    newNode->Volume = vol;
                    pNodes.emplace_back(newNode);
                }

                std::size_t i, j;
                // Merge pairs together until just the root object left
                while (pNodes.size() > 1)
                {
                    // Find indices of the two "nearest" nodes, based on some criterion
                    FindNodesToMerge<volume>(pNodes, i, j, Settings);
                    // Group nodes i and j together under a new internal node
                    std::shared_ptr<Node<volume>> pPair = std::make_shared<Node<volume>>();
                    //pPair->type = NODE;
                    pPair->Left = pNodes[i];
                    pPair->Right = pNodes[j];
                    pPair->Depth = std::max(pNodes[i]->Depth, pNodes[j]->Depth) + 1;
                    // Compute a bounding volume for the two nodes
                    pPair->Volume = ComputeBoundingVolume<volume>({ pNodes[i]->Volume, pNodes[j]->Volume });

                    // Remove the two nodes from the active set and add in the new node.
                    // Done by putting new node at index ’min’ and copying last entry to ’max’
                    std::size_t min = i, max = j;
                    if (i > j) min = j, max = i;
                    pNodes[min] = pPair;
                    std::swap(pNodes[max], pNodes.back());
                    
                    pNodes.pop_back();
                    //numObjects--;
                }
                // Free temporary storage and return root of tree
                std::shared_ptr<Node<volume>> root = pNodes.front();

                return root;
            }

        private:
            template<typename volume>
            static std::shared_ptr<Node<volume>> construct_top_down_recursion(std::shared_ptr<Node<volume>>* tree, std::vector<volume>& insertionData, Settings Settings, std::size_t depth)
            {
                assert(insertionData.size() > 0);

                const int MIN_OBJECTS_PER_LEAF = 1;
                std::shared_ptr<Node<volume>> pNode = std::make_shared<Node<volume>>();
                *tree = pNode;

                pNode->Depth = depth++;
                pNode->Volume = ComputeBoundingVolume<volume>(insertionData);
                if (insertionData.size() <= MIN_OBJECTS_PER_LEAF)
                {
                    pNode->Leaf = true;
                }
                else
                {
                    pNode->Leaf = false;
                    auto[leftSet, rightSet] = PartitionObjects<volume>(Settings, insertionData);
                    construct_top_down_recursion<volume>(&pNode->Left, leftSet, Settings, depth);
                    construct_top_down_recursion<volume>(&pNode->Right, rightSet, Settings, depth);
                }

                return pNode;
            }

            template<typename volume>
            static volume ComputeBoundingVolume(std::vector<volume> const& insertionData)
            {
                if constexpr (std::is_same_v<volume, sphere>)
                {
                    std::vector<point> vertices;

                    for (auto& sphere : insertionData)
                    {
                        vertices.emplace_back(sphere.center - point{sphere.radius, 0, 0});
                        vertices.emplace_back(sphere.center + point{sphere.radius, 0, 0});
                        vertices.emplace_back(sphere.center - point{0, sphere.radius, 0});
                        vertices.emplace_back(sphere.center + point{0, sphere.radius, 0});
                        vertices.emplace_back(sphere.center - point{0, 0, sphere.radius});
                        vertices.emplace_back(sphere.center + point{0, 0, sphere.radius});
                    }

                    return make_ritter_eigen_sphere(vertices);
                }
                else if constexpr (std::is_same_v<volume, aabb>)
                {
                    std::vector<point> vertices;
                    
                    for (auto& aabb : insertionData)
                    {
                        vertices.emplace_back(aabb.min);
                        vertices.emplace_back(aabb.max);
                    }

                    return make_fitting_aabb(vertices);
                }

                assert(true);
            }

            template<typename volume>
            static std::pair<std::vector<volume>, std::vector<volume>> PartitionObjects(Settings settings, std::vector<volume>& insertionData)
            {
                std::vector<volume> leftSet, rightSet;

                switch (settings.heuristics)
                {
                case Heuristic::median_of_centers:
                    {
                        //sort data by axis of our liking
                        vector axis = { 0, 1, 0 };

                        std::sort(insertionData.begin(), insertionData.end(), [&](auto&& lhs, auto&& rhs) 
                        {
                            if constexpr (std::is_same_v<volume, aabb>)
                            {
                                auto lhs_center = (lhs.min + lhs.max) * 0.5f;
                                auto rhs_center = (rhs.min + rhs.max) * 0.5f;

                                auto leftDot = dot(lhs_center, axis);
                                auto rightDot = dot(rhs_center, axis);

                                return leftDot < rightDot;
                            }
                            else if constexpr (std::is_same_v<volume, sphere>)
                            {
                                auto lhs_center = lhs.center;
                                auto rhs_center = rhs.center;

                                auto leftDot = dot(lhs_center, axis);
                                auto rightDot = dot(rhs_center, axis);

                                return leftDot < rightDot;
                            }
                        });

                        std::size_t leftSize = insertionData.size() / 2;
                        std::size_t rightSize = insertionData.end() - (insertionData.begin() + leftSize);
                        leftSet.resize(leftSize);
                        rightSet.resize(rightSize);
                        std::copy(insertionData.begin(), insertionData.begin() + insertionData.size() / 2, leftSet.begin());
                        std::copy(insertionData.begin() + insertionData.size() / 2, insertionData.end(), rightSet.begin());
                        
                    }
                    break;
                case Heuristic::median_of_extents:
                    {
                        std::vector<vector> extents;
                        //for (auto& bv : insertionData)
                        //{
                        //    extents.emplace_back(bv.min);
                        //    extents.emplace_back(bv.max);
                        //}
                        //// sort extents based on y-axis
                        //std::sort(extents.begin(), extents.end(), [](auto&& lhs, auto&& rhs)
                        //{
                        //    return lhs.y < rhs.y;
                        //});

                        //// split point based on median of all extents projected along y-axis
                        //vector split_point = extents[extents.size() / 2];

                        //for (auto& bv : insertionData)
                        //{
                        //    auto bv_center = (bv.min + bv.max) * 0.5f;
                        //    // we only care about their y values.
                        //    bv_center.x = bv_center.z = 0.0;
                        //    split_point.x = split_point.z = 0.0;
                        //    /*if (dot(bv_center, split_point) > 0.0)
                        //    {
                        //        left.emplace_back(bv);
                        //    }
                        //    else
                        //    {
                        //        right.emplace_back(bv);
                        //    }*/
                        //}
                    }
                    break;
                case Heuristic::k_even_split:
                    {
                        std::vector<vector> extents;
                        //for (auto& bv : insertionData)
                        //{
                        //    extents.emplace_back(bv.min);
                        //    extents.emplace_back(bv.max);
                        //}
                        //vector axis = { 0,1,0 };
                        //auto [lowest, highest] = most_separated_points_on_axis(extents, axis);
                        //
                        //auto increments = (highest - lowest) / settings.evenly_spaced_points;
                        //vector split_point = lowest + settings.evenly_spaced_points / 2.0 * increments;

                        //for (auto& bv : insertionData)
                        //{
                        //    auto bv_center = (bv.min + bv.max) * 0.5f;
                        //    // we only care about their y values.
                        //    bv_center.x = bv_center.z = 0.0;
                        //    split_point.x = split_point.z = 0.0;
                        //    /*if (dot(bv_center, split_point) > 0.0)
                        //    {
                        //        left.emplace_back(bv);
                        //    }
                        //    else
                        //    {
                        //        right.emplace_back(bv);
                        //    }*/
                        //}
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

                return { leftSet, rightSet };
            }

            template<typename volume>
            static void FindNodesToMerge(std::vector<std::shared_ptr<Node<volume>>> const& pNodes, std::size_t& i, std::size_t& j, Settings Settings)
            {
                // we're trying to find the min cost by looking at various heuristics combined.
                value_type lowest_cost = maximum;

                //value_type current_lowest = maximum;
                for (std::size_t x = 0; x < pNodes.size(); ++x)
                {
                    for (std::size_t y = x + 1; y < pNodes.size(); ++y)
                    {
                        value_type cost = CalculateCost<volume>(pNodes[x]->Volume, pNodes[y]->Volume, Settings);
                        if (cost < lowest_cost)
                        {
                            lowest_cost = cost;
                            i = x;
                            j = y;
                        }
                    }
                }
            }

            template<typename volume>
            static value_type CalculateCost(volume const& firstVol, volume const& secVol, Settings Settings)
            {
                value_type cost = maximum;

                // nearest neighbour with squared distance
                value_type nearest_neighbour_factor = Settings.nearest_neighbour_factor;

                // minimum combined volume
                value_type combined_volume_factor = Settings.combined_volume_factor;

                // minimum combined volume
                value_type minimum_volume_relative_increase_factor = Settings.minimum_volume_relative_increase_factor;

                if constexpr (std::is_same_v<volume, aabb>)
                {
                    auto lhs_center = firstVol.min + firstVol.max;
                    auto rhs_center = secVol.min + secVol.max;

                    auto sqDist = distance_sqaured(lhs_center, rhs_center);

                    auto aabb = ComputeBoundingVolume<volume>({ firstVol, secVol });
                    auto aabb_volume = volume_of(aabb);

                    auto aabb_relative_increase = aabb_volume / (volume_of(firstVol) + volume_of(secVol));

                    cost = sqDist * nearest_neighbour_factor 
                        + aabb_volume * combined_volume_factor
                        + aabb_relative_increase * minimum_volume_relative_increase_factor;
                }
                else if constexpr (std::is_same_v<volume, sphere>)
                {
                    auto lhs_center = firstVol.center;
                    auto rhs_center = secVol.center;

                    auto sqDist = distance_sqaured(lhs_center, rhs_center);

                    auto sphere = ComputeBoundingVolume<volume>({ firstVol, secVol });
                    auto sphere_volume = volume_of(sphere);

                    auto sphere_relative_increase = sphere_volume / (volume_of(firstVol) + volume_of(secVol));

                    cost = sqDist * nearest_neighbour_factor 
                        + sphere_volume * combined_volume_factor
                        + sphere_relative_increase * minimum_volume_relative_increase_factor;
                }

                return cost;
            }
        };



    }

    // octtree related functions and structures
    namespace octree
    {
        static constexpr std::size_t octree_size = 8;
        
        struct Object 
        {
            point center = {};           // Center point for object
            value_type radius = 0;      // Radius of object bounding sphere
            Object* pNextObject = nullptr;    // Pointer to next object when linked into list
        };

        struct Node
        {
            point center = {};
            value_type halfWidth = 0;
            Node* pChild[octree_size] = {};
            Object* pObjectList = nullptr;
        };

        // Preallocates an octree down to a specific depth
        Node* BuildOctree(point center, value_type halfWidth, std::int32_t stopDepth)
        {
            if (stopDepth < 0)
            {
                return nullptr;
            }
            else 
            {
                // Construct and fill in ’root’ of this subtree
                Node* pNode = new Node();
                pNode->center = center;
                pNode->halfWidth = halfWidth;
                pNode->pObjectList = nullptr;
                
                // Recursively construct the eight children of the subtree
                point offset{};
                value_type step = halfWidth * 0.5f;
                for (std::size_t i = 0; i < octree_size; i++)
                {
                    offset.x = ((i & 1) ? step : -step);
                    offset.y = ((i & 2) ? step : -step);
                    offset.z = ((i & 4) ? step : -step);
                    pNode->pChild[i] = BuildOctree(center + offset, step, stopDepth - 1);
                }

                return pNode;
            }
        }

        void InsertObject(Node* pTree, Object* pObject)
        {
            std::int32_t index = 0, straddle = 0;
            // Compute the octant number [0..7] the object sphere center is in
            // If straddling any of the dividing x, y, or z planes, exit directly
            for (std::int32_t i = 0; i < 3; i++) 
            {
                value_type delta = pObject->center[i] - pTree->center[i];
                
                if (std::abs(delta) < pTree->halfWidth + pObject->radius) 
                {
                    straddle = 1;
                    break;
                }

                if (delta > 0.0f) 
                    index |= (1 << i); // ZYX
            }

            if (!straddle) 
            {
                if (pTree->pChild[index] == nullptr) 
                {
                    pTree->pChild[index] = new Node();
                    //...initialize node contents here...
                }
                InsertObject(pTree->pChild[index], pObject);
            }
            else 
            {
                // Straddling, or no child node to descend into, so
                // link object into linked list at this node
                pObject->pNextObject = pTree->pObjectList;
                pTree->pObjectList = pObject;
            }
        }
    }

    //binary space partition related functions and structures
    namespace bsp
    {
        std::size_t max_depth = 20;
        std::size_t min_leaf_size = 20;
        
        value_type plane_thickness_epsilon = std::numeric_limits<value_type>::epsilon() * 1000;

        struct BSPNode
        {
            std::vector<polygon*> polygons = {};
            BSPNode* left = nullptr, *right = nullptr;
            BSPNode(std::vector<polygon*>& polygons)
                : polygons{ polygons }
                , left{nullptr}
                , right{nullptr}
            {
            }

            BSPNode(BSPNode* leftTree, BSPNode* rightTree)
                : polygons{ }
                , left{ leftTree }
                , right{ rightTree }
            {
            }
        };
        
        enum class point_to_plane
        {
            point_on_plane,
            point_in_front_of_plane,
            point_behind_plane,
        };

        enum class polygon_to_plane
        {
            polygon_coplanar_with_plane,
            polygon_in_front_of_plane,
            polygon_behind_plane,
            polygon_straddling_plane,
        };

        BSPNode* BuildLeafStoringBSPTree(std::vector<polygon*>& polygons, std::size_t depth);
        
        plane GetPlaneFromPolygon(polygon& polygon);

        point IntersectEdgeAgainstPlane(point const& a, point const& b, plane const& plane)
        {
            ray ray_from_points = { a, b - a };
            
            auto result = intersection::ray_plane(ray_from_points, plane);

            if(result.t_entry > 0.f && result.t_entry < 1.f)
                return result.p_entry;
            if (result.t_exit > 0.f && result.t_exit < 1.f)
                return result.p_exit;

            return point{ 0, 0 };
        }

        // Given a vector of polygons, attempts to compute a good splitting plane
        plane PickSplittingPlane(std::vector<polygon*>& polygons);

        // Classify point p to a plane thickened by a given thickness epsilon
        point_to_plane ClassifyPointToPlane(point p, plane plane);

        // Return value specifying whether the polygon ‘poly’ lies in front of,
        // behind of, on, or straddles the plane ‘plane’
        polygon_to_plane ClassifyPolygonToPlane(polygon const* poly, plane plane);

        void SplitPolygon(polygon& poly, plane plane, polygon** frontPoly, polygon** backPoly);
    }
}
