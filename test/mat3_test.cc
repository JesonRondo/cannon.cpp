#include <gtest/gtest.h>

#include "math/Mat3.h"

using namespace Cannon;

TEST(Mat3, Creation) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());
    auto success = true;
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            success = success && (m->e(r, c) == 0);

    EXPECT_TRUE(success);
}
