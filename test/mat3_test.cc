#include <gtest/gtest.h>
#include <stdexcept>
#include <cmath>

#include "math/Mat3.h"
#include "math/Vec3.h"
#include "math/Quaternion.h"

using namespace Cannon;

TEST(Mat3, Creation) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());

    auto success = true;
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            success = success && (m->e(r, c) == 0);

    EXPECT_TRUE(success);
}

TEST(Mat3, E) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());

    m->setE(1, 2, 5);

    EXPECT_EQ(m->e(1, 2), 5);

    auto success = true;
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3 ; r ++)
            if (r != 1 || c != 2)
                success = success && (m->e(r, c) == 0);

    EXPECT_TRUE(success);
}

TEST(Mat3, Identity) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());

    m->identity();

    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3 ; r++)
            EXPECT_EQ(m->e(r, c), (r == c) ? 1 : 0);
}

TEST(Mat3, Vmult) {
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(2, 3, 7));
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());

    std::shared_ptr<Math::Vec3> t(new Math::Vec3());

    /*
      set the matrix to
      | 1 2 3 |
      | 4 5 6 |
      | 7 8 9 |
    */
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            m->setE(r, c, 1 + r * 3 + c);

    m->vmult(v.get(), t.get());
    EXPECT_TRUE(t->x == 29 && t->y == 65 && t->z == 101);
}

TEST(Mat3, Mmult) {
    std::unique_ptr<Math::Mat3> m1(new Math::Mat3());
    std::unique_ptr<Math::Mat3> m2(new Math::Mat3());
    std::unique_ptr<Math::Mat3> m3(new Math::Mat3());

    /* set the matrix to
        | 1 2 3 |
        | 4 5 6 |
        | 7 8 9 |
    */
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            m1->setE(r, c, 1 + r * 3 + c);

    /* set the matrix to
        | 5 2 4 |
        | 4 5 1 |
        | 1 8 0 |
    */
    m2->setE(0, 0, 5);
    m2->setE(0, 1, 2);
    m2->setE(0, 2, 4);
    m2->setE(1, 0, 4);
    m2->setE(1, 1, 5);
    m2->setE(1, 2, 1);
    m2->setE(2, 0, 1);
    m2->setE(2, 1, 8);
    m2->setE(2, 2, 0);

    m1->mmult(m2.get(), m3.get());

    EXPECT_TRUE(m3->e(0, 0) == 16
        &&  m3->e(0, 1) == 36
        &&  m3->e(0, 2) == 6
        &&  m3->e(1, 0) == 46
        &&  m3->e(1, 1) == 81
        &&  m3->e(1, 2) == 21
        &&  m3->e(2, 0) == 76
        &&  m3->e(2, 1) == 126
        &&  m3->e(2, 2) == 36);
}

TEST(Mat3, Solve) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(2, 3, 7));
    std::unique_ptr<Math::Vec3> t(new Math::Vec3());
    std::unique_ptr<Math::Vec3> vv(new Math::Vec3());

    /* set the matrix to
    | 5 2 4 |
    | 4 5 1 |
    | 1 8 0 |
    */
    m->setE(0, 0, 5);
    m->setE(0, 1, 2);
    m->setE(0, 2, 4);
    m->setE(1, 0, 4);
    m->setE(1, 1, 5);
    m->setE(1, 2, 1);
    m->setE(2, 0, 1);
    m->setE(2, 1, 8);
    m->setE(2, 2, 0);

    m->solve(v.get(), t.get());
    m->vmult(t.get(), vv.get());

    EXPECT_TRUE(vv->almostEquals(v.get(), 0.00001));

    std::unique_ptr<Math::Mat3> m1(new Math::Mat3());

    /* set the matrix to
        | 1 2 3 |
        | 4 5 6 |
        | 7 8 9 |
        */
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
        m1->setE(r, c, 1 + r * 3 + c);

    bool error = false;
    try {
        m1->solve(v.get(), t.get());
    } catch (const std::exception& e) {
        error = true;
    }

    // should rise an error if the system has no solutions
    EXPECT_TRUE(error);
}

TEST(Mat3, Reverse) {
    std::unique_ptr<Math::Mat3> m(new Math::Mat3());
    std::unique_ptr<Math::Mat3> m2(new Math::Mat3());
    std::unique_ptr<Math::Mat3> m3(new Math::Mat3());

    /* set the matrix to
    | 5 2 4 |
    | 4 5 1 |
    | 1 8 0 |
    */
    m->setE(0, 0, 5);
    m->setE(0, 1, 2);
    m->setE(0, 2, 4);
    m->setE(1, 0, 4);
    m->setE(1, 1, 5);
    m->setE(1, 2, 1);
    m->setE(2, 0, 1);
    m->setE(2, 1, 8);
    m->setE(2, 2, 0);

    m->reverse(m2.get());

    m2->mmult(m.get(), m3.get());

    bool success = true;
    for (int c = 0 ; c < 3 ; c++)
        for (int r = 0; r < 3; r++)
            success = success && (std::abs(m3->e(r, c) - (c == r ? 1.0f : 0.0f)) < 0.00001);

    // inversing
    EXPECT_TRUE(success);

    std::unique_ptr<Math::Mat3> m1(new Math::Mat3());

    /* set the matrix to
    | 1 2 3 |
    | 4 5 6 |
    | 7 8 9 |
    */
    for (int c = 0; c < 3; c++)
        for (int r = 0; r < 3; r++)
            m1->setE(r, c, 1 + r * 3 + c);

    bool error = false;
    
    try {
        m1->reverse(m2.get());
    } catch (const std::exception& e) {
        error = true;
    }

    // should rise an error if the matrix is not inersible
    EXPECT_TRUE(error);
}

TEST(Mat3, Transpose) {
    std::unique_ptr<Math::Mat3> M(new Math::Mat3(
        {
            1.0f, 2.0f, 3.0f,
            4.0f, 5.0f, 6.0f,
            7.0f, 8.0f, 9.0f
        }
    ));
    std::unique_ptr<Math::Mat3> MT(new Math::Mat3());
    M->transpose(MT.get());

    EXPECT_TRUE(
        MT->e(0, 0) == 1.0f && MT->e(0, 1) == 4.0f && MT->e(0, 2) == 7.0f &&
        MT->e(1, 0) == 2.0f && MT->e(1, 1) == 5.0f && MT->e(1, 2) == 8.0f &&
        MT->e(2, 0) == 3.0f && MT->e(2, 1) == 6.0f && MT->e(2, 2) == 9.0f);
}

TEST(Mat3, Scale) {
    std::unique_ptr<Math::Mat3> M(new Math::Mat3(
        {
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, 1.0f
        }
    ));
    std::unique_ptr<Math::Mat3> MT(new Math::Mat3());
    std::unique_ptr<Math::Vec3> v(new Math::Vec3(1.0f, 2.0f, 3.0f));

    M->scale(v.get(), MT.get());
    EXPECT_TRUE(
        MT->e(0, 0) == 1.0f && MT->e(0, 1) == 2.0f && MT->e(0, 2) == 3.0f &&
        MT->e(1, 0) == 1.0f && MT->e(1, 1) == 2.0f && MT->e(1, 2) == 3.0f &&
        MT->e(2, 0) == 1.0f && MT->e(2, 1) == 2.0f && MT->e(2, 2) == 3.0f);
}

TEST(Mat3, SetRotationFromQuaternion) {
    std::unique_ptr<Math::Mat3> M(new Math::Mat3());
    std::unique_ptr<Math::Quaternion> q(new Math::Quaternion());
    std::unique_ptr<Math::Vec3> original(new Math::Vec3(1.0f, 2.0f, 3.0f));
    
    std::unique_ptr<Math::Vec3> v(new Math::Vec3());
    std::unique_ptr<Math::Vec3> Mv(new Math::Vec3());
    std::unique_ptr<Math::Vec3> qv(new Math::Vec3());

    // Test zero rotation
    M->setRotationFromQuaternion(q.get());
    M->vmult(original.get(), v.get());

    EXPECT_TRUE(v->almostEquals(original.get(), 0.00001));

    // Test rotation along x axis
    q->setFromEuler(0.222, 0.123, 1.234);
    M->setRotationFromQuaternion(q.get());
    M->vmult(original.get(), Mv.get());
    q->vmult(original.get(), qv.get());

    EXPECT_TRUE(Mv->almostEquals(qv.get(), 0.00001));
}
