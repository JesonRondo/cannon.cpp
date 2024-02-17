#include "shapes/Box.h"

using namespace Cannon::Shapes;

Box::~Box() {
    delete this->halfExtents;
    delete this->convexPolyhedronRepresentation;
}

void Box::updateConvexPolyhedronRepresentation() {
    float sx = this->halfExtents->x;
    float sy = this->halfExtents->y;
    float sz = this->halfExtents->z;

    std::vector<Math::Vec3>* vertices = new std::vector<Math::Vec3>{
        Math::Vec3(-sx, -sy, -sz),
        Math::Vec3(sx, -sy, -sz),
        Math::Vec3(sx, sy, -sz),
        Math::Vec3(-sx, sy, -sz),
        Math::Vec3(-sx, -sy, sz),
        Math::Vec3(sx, -sy, sz),
        Math::Vec3(sx, sy, sz),
        Math::Vec3(-sx, sy, sz)
    };

    std::vector<std::vector<int>>* faces = new std::vector<std::vector<int>>{
        {3, 2, 1, 0},
        {4, 5, 6, 7},
        {5, 4, 0, 1},
        {2, 3, 7, 6},
        {0, 4, 7, 3},
        {1, 2, 6, 5}
    };

    std::vector<Math::Vec3>* uniqueAxes = new std::vector<Math::Vec3>{
        Math::Vec3(0, 0, 1),
        Math::Vec3(0, 1, 0),
        Math::Vec3(1, 0, 0)
    };

    this->convexPolyhedronRepresentation = new ConvexPolyhedron(vertices, faces, uniqueAxes);
    this->convexPolyhedronRepresentation->material = this->material;
}

void Box::calculateLocalInertia(float mass, Math::Vec3* target) {
    Box::calculateInertia(this->halfExtents, mass, target);
}

void Box::calculateInertia(Math::Vec3* halfExtents, float mass, Math::Vec3* target) {
    if (halfExtents->isZero()) {
        target->x = 2.0 / 12.0 * mass;
        target->y = 2.0 / 12.0 * mass;
        target->z = 2.0 / 12.0 * mass;
    } else {
        float ex = halfExtents->x;
        float ey = halfExtents->y;
        float ez = halfExtents->z;

        target->x = (1.0f / 12.0f) * mass * (2 * ey * 2 * ey + 2 * ez * 2 * ez);
        target->y = (1.0f / 12.0f) * mass * (2 * ex * 2 * ex + 2 * ez * 2 * ez);
        target->z = (1.0f / 12.0f) * mass * (2 * ex * 2 * ex + 2 * ey * 2 * ey);
    }
}

std::array<Cannon::Math::Vec3, 6>* Box::getSideNormals(
    std::array<Math::Vec3, 6>* sixTargetVectors,
    Math::Quaternion* quat) {
    auto sides = sixTargetVectors;
    auto ex = this->halfExtents;
    
    sides->at(0).set(ex->x, 0, 0);
    sides->at(1).set(0, ex->y, 0);
    sides->at(2).set(0, 0, ex->z);
    sides->at(3).set(-ex->x, 0, 0);
    sides->at(4).set(0, -ex->y, 0);
    sides->at(5).set(0, 0, -ex->z);

    if (quat != nullptr) {
        for (int i = 0; i < 6; i++) {
            quat->vmult(&sides->at(i), &sides->at(i));
        }
    }
    
    return sides;
}

double Box::volume() {
    return 8.0 * this->halfExtents->x * this->halfExtents->y * this->halfExtents->z;
}

void Box::updateBoundingSphereRadius() {
    this->boundingSphereRadius = this->halfExtents->length();
}

Cannon::Math::Vec3 worldCornerTempPos;
void Box::forEachWorldCorner(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    CornerCallback callback) {
    auto ex = this->halfExtents;

    std::array<Math::Vec3, 8> corners = {
        Math::Vec3(ex->x, ex->y, ex->z),
        Math::Vec3(-ex->x, ex->y, ex->z),
        Math::Vec3(-ex->x, -ex->y, ex->z),
        Math::Vec3(-ex->x, -ex->y, -ex->z),
        Math::Vec3(ex->x, -ex->y, -ex->z),
        Math::Vec3(ex->x, ex->y, -ex->z),
        Math::Vec3(-ex->x, ex->y, -ex->z),
        Math::Vec3(ex->x, -ex->y, ex->z)
    };

    for (int i = 0; i < 8; i++) {
        worldCornerTempPos.set(corners[i].x, corners[i].y, corners[i].z);
        quat->vmult(&worldCornerTempPos, &worldCornerTempPos);
        pos->vadd(&worldCornerTempPos, &worldCornerTempPos);
        callback(worldCornerTempPos.x, worldCornerTempPos.y, worldCornerTempPos.z);
    }
}

std::array<Cannon::Math::Vec3, 8> worldCornersTemp;
void Box::calculateWorldAABB(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    Math::Vec3* min,
    Math::Vec3* max) {
    auto ex = this->halfExtents;

    worldCornersTemp[0].set(ex->x, ex->y, ex->z);
    worldCornersTemp[1].set(-ex->x, ex->y, ex->z);
    worldCornersTemp[2].set(-ex->x, -ex->y, ex->z);
    worldCornersTemp[3].set(-ex->x, -ex->y, -ex->z);
    worldCornersTemp[4].set(ex->x, -ex->y, -ex->z);
    worldCornersTemp[5].set(ex->x, ex->y, -ex->z);
    worldCornersTemp[6].set(-ex->x, ex->y, -ex->z);
    worldCornersTemp[7].set(ex->x, -ex->y, ex->z);

    Math::Vec3* wc = &worldCornersTemp[0];
    quat->vmult(wc, wc);
    pos->vadd(wc, wc);
    min->copy(wc);
    max->copy(wc);

    for (int i = 1; i < 8; i++) {
        wc = &worldCornersTemp[i];
        quat->vmult(wc, wc);
        pos->vadd(wc, wc);

        float x = wc->x;
        float y = wc->y;
        float z = wc->z;

        max->x = std::max(max->x, x);
        max->y = std::max(max->y, y);
        max->z = std::max(max->z, z);

        min->x = std::min(min->x, x);
        min->y = std::min(min->y, y);
        min->z = std::min(min->z, z);
    }
}
