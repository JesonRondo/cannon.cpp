#include <gtest/gtest.h>

#include "math/Vec3.h"

using namespace Cannon;

TEST(Vec3, Creation) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    EXPECT_EQ(v->x, 1);
    EXPECT_EQ(v->y, 2);
    EXPECT_EQ(v->z, 3);
}
