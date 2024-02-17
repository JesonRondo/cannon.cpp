#include <gtest/gtest.h>

#include <cmath>
#include "shapes/Box.h"
#include "shapes/ConvexPolyhedron.h"
#include "math/Vec3.h"
#include "math/Quaternion.h"

using namespace Cannon;

Shapes::ConvexPolyhedron* createPolyBox(float sx, float sy, float sz) {
    Shapes::Box* box = new Shapes::Box(new Math::Vec3(sx, sy, sz));
    return box->convexPolyhedronRepresentation;
}

Shapes::ConvexPolyhedron* createBoxHull(float size) {
    Shapes::Box* box = new Shapes::Box(new Math::Vec3(size, size, size));
    return box->convexPolyhedronRepresentation;
}

Shapes::ConvexPolyhedron* createBoxHull() {
    return createBoxHull(0.5);
}

TEST(ConvexPolyhedron, CalculateWorldAABB) {
    Shapes::ConvexPolyhedron* poly = createPolyBox(1, 1, 1);

    std::unique_ptr<Math::Vec3> min(new Math::Vec3());
    std::unique_ptr<Math::Vec3> max(new Math::Vec3());
    poly->calculateWorldAABB(
        new Math::Vec3(1, 0, 0),
        new Math::Quaternion(0, 0, 0, 1),
        min.get(),
        max.get()
    );

    EXPECT_EQ(min->x, 0);
    EXPECT_EQ(max->x, 2);
    EXPECT_EQ(min->y, -1);
    EXPECT_EQ(max->y, 1);
}

TEST(ConvexPolyhedron, CalculateWorldAABBAlwaysDecreasingVertsNoUndefined) {
    std::unique_ptr<Shapes::ConvexPolyhedron> poly(new Shapes::ConvexPolyhedron(
        new std::vector<Math::Vec3>{
            Math::Vec3(4, 4, 4),
            Math::Vec3(3, 3, 3),
            Math::Vec3(2, 2, 2),
            Math::Vec3(1, 1, 1),
            Math::Vec3(0, 0, 0),
            Math::Vec3(-1, -1, -1),
            Math::Vec3(-2, -2, -2),
            Math::Vec3(-3, -3, -3),
        },
        new std::vector<std::vector<int>>{
            {3, 2, 1, 0},
            {4, 5, 6, 7},
            {5, 4, 0, 1},
            {2, 3, 7, 6},
            {0, 4, 7, 3},
            {1, 2, 6, 5}
        }
    ));
    std::unique_ptr<Math::Vec3> min(new Math::Vec3());
    std::unique_ptr<Math::Vec3> max(new Math::Vec3());

    poly->calculateWorldAABB(
        new Math::Vec3(0, 0, 0),
        new Math::Quaternion(0, 0, 0, 1),
        min.get(),
        max.get()
    );

    EXPECT_EQ(min->x, -3);
    EXPECT_EQ(max->x, 4);
    EXPECT_EQ(min->y, -3);
    EXPECT_EQ(max->y, 4);
    EXPECT_EQ(min->z, -3);
    EXPECT_EQ(max->z, 4);
}

TEST(ConvexPolyhedron, ClipFaceAgainstPlane) {
    auto h = createBoxHull();

    // Four points 1 unit below the plane z=0 - we assume to get back 4
    std::vector<Math::Vec3>* inverts = new std::vector<Math::Vec3>{
        Math::Vec3(-0.2, -0.2, -1),
        Math::Vec3(-0.2, 0.2, -1),
        Math::Vec3(0.2, 0.2, -1),
        Math::Vec3(0.2, -0.2, -1)
    };
    std::vector<Math::Vec3>* outverts = new std::vector<Math::Vec3>{};
    h->clipFaceAgainstPlane(inverts, outverts, new Math::Vec3(0, 0, 1), 0.0);
    // did not get the assumed 4 vertices
    EXPECT_EQ(outverts->size(), 4);
    inverts = new std::vector<Math::Vec3>{};
    outverts = new std::vector<Math::Vec3>{};

    // Lower the plane to z=-2, we assume no points back
    h->clipFaceAgainstPlane(inverts, outverts, new Math::Vec3(0, 0, 1), 2);
    // got more than zero vertices left after clipping!
    EXPECT_EQ(outverts->size(), 0);

    // two points below, two over. We get four points back, though 2 of them are clipped to
    // the back of the  plane
    std::vector<Math::Vec3>* inverts2 = new std::vector<Math::Vec3>{
        Math::Vec3(-2, -2,  1),
        Math::Vec3(-2,  2,  1),
        Math::Vec3(2,  2, -1),
        Math::Vec3(2, -2, -1)
    };
    outverts = new std::vector<Math::Vec3>{};
    h->clipFaceAgainstPlane(inverts2, outverts, new Math::Vec3(0, 0, 1), 0.0);
    // Expected 4 points back from clipping a quad with plane, got outverts.length
    EXPECT_EQ(outverts->size(), 4);
}

// TODO
TEST(ConvexPolyhedron, ClipFaceAgainstHull) {
    // Create box
    auto hullA = createBoxHull(0.5);
    std::vector<Shapes::PointObject>* res = new std::vector<Shapes::PointObject>{};
    auto sepNormal = new Math::Vec3(0, 0, 1);

    // Move the box 0.45 units up - only 0.05 units of the box will be below plane z=0
    auto posA = new Math::Vec3(0, 0, 0.45);
    auto quatA = new Math::Quaternion();

    // All points from B is in the plane z=0
    std::vector<Math::Vec3> *worldVertsB = new std::vector<Math::Vec3>{
        Math::Vec3(-1.0, -1.0, 0),
        Math::Vec3(-1.0, 1.0, 0),
        Math::Vec3(1.0, 1.0, 0),
        Math::Vec3(1.0, -1.0, 0)
    };

    // We will now clip a face in hullA that is closest to the sepNormal
    // against the points in worldVertsB.
    // We can expect to get back the 4 corners of the box hullA penetrated 0.05 units
    // into the plane worldVertsB we constructed
    hullA->clipFaceAgainstHull(sepNormal, posA, quatA, worldVertsB, -100, 100, res);

    // EXPECT_EQ(res->size(), 4);
}

TEST(ConvexPolyhedron, ClipAgainstHull) {
    auto hullA = createBoxHull(0.6);
    auto posA = new Math::Vec3(-0.5, 0, 0);
    auto quatA = new Math::Quaternion();

    auto hullB = createBoxHull(0.5);
    auto posB = new Math::Vec3(0.5, 0, 0);
    auto quatB = new Math::Quaternion();

    auto sepaxis = new Math::Vec3();
    bool found = hullA->findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
    std::vector<Shapes::PointObject>* result = new std::vector<Shapes::PointObject>{};
    //hullA.clipAgainstHull(posA,quatA,hullB,posB,quatB,sepaxis,-100,100,result);
    quatB->setFromAxisAngle(new Math::Vec3(0, 0, 1), M_PI / 4);
    //console.log("clipping....");
    hullA->clipAgainstHull(posA, quatA, hullB, posB, quatB, sepaxis, -100, 100, result);
    //console.log("result:",result);
    //console.log("done....");
}

TEST(ConvexPolyhedron, TestSepAxis) {
    auto hullA = createBoxHull(0.5);
    auto posA = new Math::Vec3(-0.2, 0, 0);
    auto quatA = new Math::Quaternion();

    auto hullB = createBoxHull();
    auto posB = new Math::Vec3(0.2, 0, 0);
    auto quatB = new Math::Quaternion();

    auto sepAxis = new Math::Vec3(1, 0, 0);
    auto found1 = hullA->testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
    // didnt find sep axis depth"
    EXPECT_NEAR(found1.depth, 0.6, 0.00001);

    // Move away
    posA->x = -5;
    auto found2 = hullA->testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
    // found separating axis though there are none
    EXPECT_FALSE(found2.boolean);

    // Inclined 45 degrees, what happens then?
    posA->x = 1;
    quatB->setFromAxisAngle(new Math::Vec3(0, 0, 1), M_PI / 4);
    auto found3 = hullA->testSepAxis(sepAxis, hullB, posA, quatA, posB, quatB);
    // Did not fetch"
    EXPECT_TRUE(found3.boolean);
}

TEST(ConvexPolyhedron, FindSepAxis) {
    auto hullA = createBoxHull();
    auto posA = new Math::Vec3(-0.2, 0, 0);
    auto quatA = new Math::Quaternion();

    auto hullB = createBoxHull();
    auto posB = new Math::Vec3(0.2, 0, 0);
    auto quatB = new Math::Quaternion();

    auto sepaxis = new Math::Vec3();
    bool found = hullA->findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
    //console.log("SepAxis found:",found,", the axis:",sepaxis.toString());

    quatB->setFromAxisAngle(new Math::Vec3(0, 0, 1), M_PI / 4);
    bool found2 = hullA->findSeparatingAxis(hullB, posA, quatA, posB, quatB, sepaxis);
    //console.log("SepAxis found:",found2,", the axis:",sepaxis.toString());
}

TEST(ConvexPolyhedron, Project) {
    auto convex = createBoxHull(0.5);
    auto pos = new Math::Vec3(0, 0, 0);
    auto quat = new Math::Quaternion();

    auto axis = new Math::Vec3(1, 0, 0);
    std::array<float, 2>* result = new std::array<float, 2>();

    Shapes::ConvexPolyhedron::project(convex, axis, pos, quat, result);
    EXPECT_EQ(result->at(0), 0.5);
    EXPECT_EQ(result->at(1), -0.5);

    axis->set(-1, 0, 0);
    Shapes::ConvexPolyhedron::project(convex, axis, pos, quat, result);
    EXPECT_EQ(result->at(0), 0.5);
    EXPECT_EQ(result->at(1), -0.5);

    axis->set(0, 1, 0);
    Shapes::ConvexPolyhedron::project(convex, axis, pos, quat, result);
    EXPECT_EQ(result->at(0), 0.5);
    EXPECT_EQ(result->at(1), -0.5);

    pos->set(0, 1, 0);
    axis->set(0, 1, 0);
    Shapes::ConvexPolyhedron::project(convex, axis, pos, quat, result);
    EXPECT_EQ(result->at(0), 1.5);
    EXPECT_EQ(result->at(1), 0.5);

    // Test to rotate
    quat->setFromAxisAngle(new Math::Vec3(1, 0, 0), M_PI / 2);
    pos->set(0, 1, 0);
    axis->set(0, 1, 0);
    Shapes::ConvexPolyhedron::project(convex, axis, pos, quat, result);
    EXPECT_TRUE(std::abs(result->at(0) - 1.5) < 0.01);
    EXPECT_TRUE(std::abs(result->at(1) - 0.5) < 0.01);
}
