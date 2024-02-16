#include <gtest/gtest.h>

#include <cmath>
#include "shapes/Box.h"
#include "math/Vec3.h"

using namespace Cannon;

TEST(Box, ForEachWOrldCorner) {
    std::unique_ptr<Shapes::Box> box(new Shapes::Box(new Math::Vec3(1, 1, 1)));
    std::unique_ptr<Math::Vec3> pos(new Math::Vec3());
    std::unique_ptr<Math::Quaternion> quat(new Math::Quaternion());

    quat->setFromAxisAngle(new Math::Vec3(0,0,1), M_PI * 0.25);

    int numCorners = 0;
    std::vector<Math::Vec3> unique;

    box->forEachWorldCorner(pos.get(), quat.get(), [&numCorners, &unique](float x, float y, float z) {
        std::unique_ptr<Math::Vec3> corner(new Math::Vec3(x, y, z));
        for (int i = 0; i < unique.size(); i++) {
            EXPECT_TRUE(!corner->almostEquals(&unique[i], 0.00001));
        }
        unique.push_back(*corner);
        numCorners++;
    });

    EXPECT_EQ(numCorners, 8);
}

TEST(Box, CalculateWorldAABB) {
    std::unique_ptr<Shapes::Box> box(new Shapes::Box(new Math::Vec3(1, 1, 1)));
    std::unique_ptr<Math::Vec3> min(new Math::Vec3());
    std::unique_ptr<Math::Vec3> max(new Math::Vec3());

    box->calculateWorldAABB(
        new Math::Vec3(3, 0, 0),
        new Math::Quaternion(),
        min.get(),
        max.get()
    );

    EXPECT_EQ(min->x, 2);
    EXPECT_EQ(max->x, 4);
    EXPECT_EQ(min->y, -1);
    EXPECT_EQ(max->y, 1);
}
