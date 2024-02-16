#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>

#include "math/Mat3.h"
#include "math/Vec3.h"
#include "math/Quaternion.h"

using namespace Cannon;

TEST(Quaternion, Creation) {
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion(1, 2, 3, 4));

    EXPECT_EQ(q->x, 1);
    EXPECT_EQ(q->y, 2);
    EXPECT_EQ(q->z, 3);
    EXPECT_EQ(q->w, 4);
}

TEST(Quaternion, Conjugate) {
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion(1, 2, 3, 4));
    q->conjugate(q.get());

    EXPECT_EQ(q->x, -1);
    EXPECT_EQ(q->y, -2);
    EXPECT_EQ(q->z, -3);
    EXPECT_EQ(q->w, 4);
}

TEST(Quaternion, Inverse) {
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion(1, 2, 3, 4));
    
    float denominator = 1 + 2*2 + 3*3 + 4*4;
    q->inverse(q.get());

    EXPECT_NEAR(q->x, -1 / denominator, 0.00001);
    EXPECT_NEAR(q->y, -2 / denominator, 0.00001);
    EXPECT_NEAR(q->z, -3 / denominator, 0.00001);
    EXPECT_NEAR(q->w, 4 / denominator, 0.00001);
}

TEST(Quaternion, ToEuler) {
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion());
    std::unique_ptr<Math::Vec3> euler(new Math::Vec3());

    q->setFromAxisAngle(new Math::Vec3(0, 0, 1), M_PI / 4);
    q->toEuler(euler.get());

    EXPECT_EQ(euler->x, 0);
    EXPECT_EQ(euler->y, 0);
    EXPECT_TRUE(std::abs(euler->z - M_PI / 4) < 0.00001);
}

TEST(Quaternion, SetFromVectors) {
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion());
    q->setFromVectors(new Math::Vec3(1, 0, 0), new Math::Vec3(-1, 0, 0));
    EXPECT_TRUE(q->vmult(new Math::Vec3(1, 0, 0), new Math::Vec3())->almostEquals(new Math::Vec3(-1, 0, 0), 0.00001));

    q->setFromVectors(new Math::Vec3(0, 1, 0), new Math::Vec3(0, -1, 0));
    EXPECT_TRUE(q->vmult(new Math::Vec3(0, 1, 0), new Math::Vec3())->almostEquals(new Math::Vec3(0, -1, 0), 0.00001));

    q->setFromVectors(new Math::Vec3(0, 0, 1), new Math::Vec3(0, 0, -1));
    EXPECT_TRUE(q->vmult(new Math::Vec3(0, 0, 1), new Math::Vec3())->almostEquals(new Math::Vec3(0, 0, -1), 0.00001));
}

TEST(Quaternion, Slerp) {
    std::unique_ptr<Math::Quaternion> qa(new Math::Quaternion());
    std::unique_ptr<Math::Quaternion> qb(new Math::Quaternion());

    qa->slerp(qb.get(), 0.5, qb.get());
    EXPECT_EQ(qa->x, qb->x);
    EXPECT_EQ(qa->y, qb->y);
    EXPECT_EQ(qa->z, qb->z);
    EXPECT_EQ(qa->w, qb->w);

    qa->setFromAxisAngle(new Math::Vec3(0, 0, 1), M_PI / 4);
    qb->setFromAxisAngle(new Math::Vec3(0, 0, 1), -M_PI / 4);
    qa->slerp(qb.get(), 0.5, qb.get());
    EXPECT_EQ(qb->x, 0);
    EXPECT_EQ(qb->y, 0);
    EXPECT_EQ(qb->z, 0);
    EXPECT_EQ(qb->w, 1);
}
