#include <gtest/gtest.h>

#include <ToLoG/Traits_fwd.hpp>

// Minimal Example for using a custom Vector Type. Must have:
/* Default Constructor
 * "Regular" Constructor
 * index [] operator
 * == operator
 * +, -, *, /
 * Traits
 * is_vector_type
 */

struct CustomVec3d
{
    CustomVec3d(double x, double y, double z) {
        data[0] = x;
        data[1] = y;
        data[2] = z;
    }
    CustomVec3d() : CustomVec3d(0,0,0) {}
    inline double operator[](int i) const {return data[i];}
    inline double& operator[](int i) {return data[i];}
    inline bool operator==(const CustomVec3d& v) const {
        return data[0]==v.data[0]&&data[1]==v.data[1]&&data[2]==v.data[2];
    }
    inline CustomVec3d operator+(const CustomVec3d& v) const {
        return CustomVec3d(
            data[0]+v.data[0],
            data[1]+v.data[1],
            data[2]+v.data[2]
            );
    }
    inline CustomVec3d operator-(const CustomVec3d& v) const {
        return CustomVec3d(
            data[0]-v.data[0],
            data[1]-v.data[1],
            data[2]-v.data[2]
            );
    }
    inline CustomVec3d operator/(const double& s) const {
        return CustomVec3d(
            data[0]/s,
            data[1]/s,
            data[2]/s
            );
    }
    double data[3];
};
inline std::ostream& operator<<(std::ostream& _os, const CustomVec3d& _v)
{
    return _os << _v[0] << " " << _v[1] << " " << _v[2];
}

namespace ToLoG
{
template<>
struct is_vector_type<CustomVec3d> : std::true_type {};

template<>
struct Traits<CustomVec3d>
{
    using value_type = double;
    using vector_type = CustomVec3d;
    constexpr static int dim = 3;
};
}

#include <ToLoG/Core.hpp>
#include <ToLoG/AABBTree.hpp>

TEST(GeometryCoreTest, CustomVec3dTest)
{
    auto p = CustomVec3d(1, 2, 3);
    auto q = CustomVec3d(3, 2, 1);
    auto r = CustomVec3d(1,0,0);
    auto s = CustomVec3d(0,0,1);

    // Traits
    EXPECT_EQ((ToLoG::Traits<CustomVec3d>::dim), 3);
    EXPECT_TRUE(ToLoG::is_vector_type<CustomVec3d>::value);

    // Vector Utils
    EXPECT_EQ(ToLoG::squared_norm(p), 14);
    EXPECT_EQ(ToLoG::norm(p), std::sqrt(14.0));
    EXPECT_EQ(ToLoG::dot(p,q), 10);
    EXPECT_EQ(ToLoG::cross(p,q), CustomVec3d(-4,8,-4));
    EXPECT_EQ(ToLoG::filled<CustomVec3d>(-7), CustomVec3d(-7,-7,-7));
    EXPECT_EQ(ToLoG::argmin(p), 0);
    EXPECT_EQ(ToLoG::argmax(p), 2);
    EXPECT_EQ(ToLoG::abs(CustomVec3d(-1,2,-3)), p);

    EXPECT_EQ(ToLoG::point_squared_distance(p, q), 8);

    // AABB
    auto aabb = ToLoG::AABB<CustomVec3d>();
    aabb.expand(p);
    EXPECT_EQ(ToLoG::aabb(p), aabb);

    // AABB Tree
    std::vector<CustomVec3d> pts = {p,q};
    auto tree = ToLoG::AABBTree<CustomVec3d>(pts);
    std::vector<uint32_t> knn;
    tree.k_nearest_neighbors(CustomVec3d(3,0,1),1,knn);
    EXPECT_EQ(knn[0], 1);

    // Other Primitives
    auto seg = ToLoG::Segment<CustomVec3d>(p,q);
    EXPECT_EQ(seg.end(), q);
    auto sphere = ToLoG::Sphere<CustomVec3d>(p, 1);
    EXPECT_EQ(sphere.center(), p);
    auto tri = ToLoG::Triangle<CustomVec3d>(p,q,r);
    auto tet = ToLoG::Tetrahedron<CustomVec3d>(p,q,r,s);

}
