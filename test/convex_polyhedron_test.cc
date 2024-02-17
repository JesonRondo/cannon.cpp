#include <gtest/gtest.h>

#include <cmath>
#include "shapes/Box.h"
#include "shapes/ConvexPolyhedron.h"
#include "math/Vec3.h"

using namespace Cannon;

Shapes::ConvexPolyhedron* createPolyBox(float sx, float sy, float sz) {
    std::unique_ptr<Shapes::Box> box(new Shapes::Box(new Math::Vec3(sx, sy, sz)));
    return box->convexPolyhedronRepresentation;
}

Shapes::ConvexPolyhedron* createBoxHull(float size) {
    std::unique_ptr<Shapes::Box> box(new Shapes::Box(new Math::Vec3(size, size, size)));
    return box->convexPolyhedronRepresentation;
}

Shapes::ConvexPolyhedron* createBoxHull() {
    return createBoxHull(0.5);
}

TEST(ConvexPolyhedron, CalculateWorldAABB) {
    // Shapes::ConvexPolyhedron* poly = createPolyBox(1, 1, 1);
    // std::unique_ptr<Math::Vec3> min(new Math::Vec3());
    // std::unique_ptr<Math::Vec3> max(new Math::Vec3());
    // poly->calculateWorldAABB(
    //     new Math::Vec3(1, 0, 0),
    //     new Math::Quaternion(0, 0, 0, 1),
    //     min.get(),
    //     max.get()
    // );

    // EXPECT_EQ(min->x, 0);
    // EXPECT_EQ(max->x, 8);
    // EXPECT_EQ(min->y, -1);
    // EXPECT_EQ(max->y, 1);
}

TEST(ConvexPolyhedron, CalculateWorldAABBAlwaysDecreasingVertsNoUndefined) {
    // std::unique_ptr<Shapes::ConvexPolyhedron> poly(new Shapes::ConvexPolyhedron(
    //     new std::vector<Math::Vec3>{
    //         Math::Vec3(4, 4, 4),
    //         Math::Vec3(3, 3, 3),
    //         Math::Vec3(2, 2, 2),
    //         Math::Vec3(1, 1, 1),
    //         Math::Vec3(0, 0, 0),
    //         Math::Vec3(-1, -1, -1),
    //         Math::Vec3(-2, -2, -2),
    //         Math::Vec3(-3, -3, -3),
    //     },
    //     new std::vector<std::vector<int>>{
    //         {3, 2, 1, 0},
    //         {4, 5, 6, 7},
    //         {5, 4, 0, 1},
    //         {2, 3, 7, 6},
    //         {0, 4, 7, 3},
    //         {1, 2, 6, 5}
    //     }
    // ));
    // std::unique_ptr<Math::Vec3> min(new Math::Vec3());
    // std::unique_ptr<Math::Vec3> max(new Math::Vec3());

    // poly->calculateWorldAABB(
    //     new Math::Vec3(0, 0, 0),
    //     new Math::Quaternion(0, 0, 0, 1),
    //     min.get(),
    //     max.get()
    // );

    // EXPECT_EQ(min->x, 0);
    // EXPECT_EQ(max->x, 8);
    // EXPECT_EQ(min->y, -1);
    // EXPECT_EQ(max->y, 1);
    // EXPECT_EQ(min->z, -1);
    // EXPECT_EQ(max->z, 1);
}
