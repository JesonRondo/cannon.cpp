#include <gtest/gtest.h>

#include "math/Vec3.h"

using namespace Cannon;

TEST(Vec3, Creation) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    EXPECT_EQ(v->x, 1);
    EXPECT_EQ(v->y, 2);
    EXPECT_EQ(v->z, 3);
}

TEST(Vec3, Cross) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    std::unique_ptr<Math::Vec3> u(new Math::Vec3(4, 5, 6));
    std::unique_ptr<Math::Vec3> w(new Math::Vec3());

    v->cross(u.get(), w.get());

    EXPECT_EQ(w->x, -3);
    EXPECT_EQ(w->y, 6);
    EXPECT_EQ(w->z, -3);
}

TEST(Vec3, Dot) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    std::unique_ptr<Math::Vec3> u(new Math::Vec3(4, 5, 6));

    float dot = v->dot(u.get());
    EXPECT_EQ(dot, 4 + 10 + 18);

    v->set(3, 2, 1);
    u->set(4, 5, 6);
    dot = v->dot(u.get());
    EXPECT_EQ(dot, 12 + 10 + 6);
}

TEST(Vec3, Set) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    v->set(4, 5, 6);

    EXPECT_EQ(v->x, 4);
    EXPECT_EQ(v->y, 5);
    EXPECT_EQ(v->z, 6);
}

TEST(Vec3, Vadd) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 2, 3));
    std::unique_ptr<Math::Vec3> u(new Math::Vec3(4, 5, 6));
    std::unique_ptr<Math::Vec3> w(new Math::Vec3());
    v->vadd(u.get(), w.get());

    EXPECT_EQ(w->x, 5);
    EXPECT_EQ(w->y, 7);
    EXPECT_EQ(w->z, 9);
}

TEST(Vec3, IsAntiparallelTo) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 0, 0));
    std::unique_ptr<Math::Vec3> u(new Math::Vec3(-1, 0, 0));
    
    EXPECT_TRUE(v->isAntiparallelTo(u.get(), 0.00001));
}

TEST(Vec3, AlmostEquals) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1, 0, 0));
    std::unique_ptr<Math::Vec3> u(new Math::Vec3(1, 0, 0));
    
    EXPECT_TRUE(v->almostEquals(u.get(), 0.00001));
}
