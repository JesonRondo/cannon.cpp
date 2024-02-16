#include <gtest/gtest.h>

#include "shapes/Sphere.h"

using namespace Cannon;

TEST(Sphere, ThrowOnWrongRadius) {
    std::unique_ptr<Shapes::Sphere> s1(new Shapes::Sphere(1));
    std::unique_ptr<Shapes::Sphere> s2(new Shapes::Sphere(0));

    bool error = false;
    try {
        std::unique_ptr<Shapes::Sphere> s3(new Shapes::Sphere(-1));
    } catch (const std::exception& e) {
        error = true;
    }

    // should throw on negative radius
    EXPECT_TRUE(error);
}
