#include "shapes/ConvexPolyhedron.h"

// #include <iostream>
#include <cmath>
#include <stdexcept>
#include "math/Vec3.h"

using namespace Cannon::Shapes;

Cannon::Math::Vec3 cb;
Cannon::Math::Vec3 ab;
void ConvexPolyhedron::computeNormal(
    Cannon::Math::Vec3* va,
    Cannon::Math::Vec3* vb,
    Cannon::Math::Vec3* vc,
    Cannon::Math::Vec3* target) {
    vb->vsub(va, &ab);
    vc->vsub(vb, &cb);
    cb.cross(&ab, target);

    if (!target->isZero()) {
        target->normalize();
    }
}

ConvexPolyhedron::ConvexPolyhedron() : Shape(ShapeTypes::CONVEXPOLYHEDRON) {
    ConvexPolyhedron({}, {});
}

ConvexPolyhedron::ConvexPolyhedron(
    std::vector<Math::Vec3>* vertices,
    std::vector<std::vector<int>>* faces) : Shape(ShapeTypes::CONVEXPOLYHEDRON) {
    ConvexPolyhedron(vertices, faces, nullptr);
}

ConvexPolyhedron::ConvexPolyhedron(
    std::vector<Math::Vec3>* vertices,
    std::vector<std::vector<int>>* faces,
    std::vector<Math::Vec3>* uniqueAxes) : Shape(ShapeTypes::CONVEXPOLYHEDRON) {
    this->vertices = vertices;
    this->faces = faces;

    this->computeNormals();

    if (uniqueAxes != nullptr) {
        this->uniqueAxes = uniqueAxes;
    }

    this->computeEdges();
    this->updateBoundingSphereRadius(); 
}

ConvexPolyhedron::~ConvexPolyhedron() {
    delete this->vertices;
    delete this->faces;

    if (this->uniqueAxes != nullptr) {
        delete this->uniqueAxes;
    }
}

Cannon::Math::Vec3 computeEdges_tmpEdge;
void ConvexPolyhedron::computeEdges() {
    auto faces = *this->faces;
    auto vertices = *this->vertices;
    int nv = vertices.size();

    this->uniqueEdges.clear();

    Math::Vec3* edge = &computeEdges_tmpEdge;

    for (int i = 0; i != faces.size(); i++) {
        auto face = faces[i];
        int numVertices = face.size();

        for (int j = 0; j != numVertices; j++) {
            int k = (j + 1) % numVertices;
            vertices[face[j]].vsub(&vertices[face[k]], edge);
            edge->normalize();

            bool found = false;
            for (int p = 0; p != this->uniqueEdges.size(); p++) {
                if (this->uniqueEdges[p].almostEquals(edge, 0.00001)) {
                    found = true;
                    break;
                }
            }

            if (!found) {
                this->uniqueEdges.push_back(edge->clone());
            }
        }
    }
}

void ConvexPolyhedron::computeNormals() {
    this->faceNormals.resize(this->faces->size());

    auto faces = *this->faces;
    auto vertices = *this->vertices;

    // Generate normals
    for (int i = 0; i < faces.size(); i++) {
        // Check so all vertices exists for this face
        for (int j = 0; j < faces[i].size(); j++) {
            int idx = faces[i][j];
            if (idx >= vertices.size() || idx < 0) {
                throw std::runtime_error("Vertex " + std::to_string(faces[i][j]) + " not found!");
            }
        }

        auto n = &this->faceNormals[i];
        this->getFaceNormal(i, n);
        n->negate(n);

        // auto vertex = &vertices[faces[i][0]];
        // if (n->dot(vertex) < 0) {
            // std::cout << ".faceNormals[" +
            //     std::to_string(i) +
            //     "] = Vec3(" +
            //     n->toString() +
            //     ") looks like it points into the shape? The vertices follow. Make sure they are ordered CCW around the normal, using the right hand rule." << std::endl;

            // for (int j = 0; j < faces[i].size(); j++) {
                // std::cout << ".vertices[" +
                //     std::to_string(faces[i][j]) +
                //     "] = Vec3(" +
                //     vertices[faces[i][j]].toString() +
                //     ")" << std::endl;
            // }
        // }
    }
}

void ConvexPolyhedron::getFaceNormal(int i, Math::Vec3* target) {
    auto f = this->faces->at(i);
    auto va = &this->vertices->at(f[0]);
    auto vb = &this->vertices->at(f[1]);
    auto vc = &this->vertices->at(f[2]);
    return ConvexPolyhedron::computeNormal(va, vb, vc, target);
}

void ConvexPolyhedron::clipAgainstHull(
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    ConvexPolyhedron* hullB,
    Math::Vec3* posB,
    Math::Quaternion* quatB,
    Math::Vec3* separatingNormal,
    float minDist,
    float maxDist,
    std::vector<PointObject>* result) {
    // TODO
}

bool ConvexPolyhedron::findSeparatingAxis(
    ConvexPolyhedron* hullB,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    Math::Vec3* posB,
    Math::Quaternion* quatB,
    Math::Vec3* target,
    std::vector<int>* faceListA,
    std::vector<int>* faceListB) {
    // TODO
    return false;
}

float ConvexPolyhedron::testSepAxis(
    Math::Vec3* axis,
    ConvexPolyhedron* hullB,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    Math::Vec3* posB,
    Math::Quaternion* quatB) {
    // TODO
    return 0.0f;
}

void ConvexPolyhedron::calculateLocalInertia(float mass, Math::Vec3* target) {
    // TODO
}

float ConvexPolyhedron::getPlaneConstantOfFace(int face_i) {
    // TODO
    return 0.0f;
}

void ConvexPolyhedron::clipFaceAgainstHull(
    Math::Vec3* separatingNormal,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    std::vector<Math::Vec3>* worldVertsB1,
    float minDist,
    float maxDist,
    std::vector<PointObject>* result) {
    // TODO
}

void ConvexPolyhedron::clipFaceAgainstPlane(
    std::vector<Math::Vec3>* inVertices,
    std::vector<Math::Vec3>* outVertices,
    Math::Vec3* planeNormal,
    float planeConstant) {
    // TODO
}

void ConvexPolyhedron::computeWorldVertices(Math::Vec3* position, Math::Quaternion* quat) {
    // TODO
}

void ConvexPolyhedron::computeLocalAABB(Math::Vec3* aabbmin, Math::Vec3* aabbmax) {
    // TODO
}

void ConvexPolyhedron::computeWorldFaceNormals(Math::Quaternion* quat) {
    // TODO
}

void ConvexPolyhedron::updateBoundingSphereRadius() {
    float maxRadiusSq = 0.0f;
    for (int i = 0; i < this->vertices->size(); i++) {
        float l = this->vertices->at(i).lengthSquared();
        if (l > maxRadiusSq) {
            maxRadiusSq = l;
        }
    }
    this->boundingSphereRadius = std::sqrt(maxRadiusSq);
}

void ConvexPolyhedron::calculateWorldAABB(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    Math::Vec3* min,
    Math::Vec3* max) {
    // TODO
}

double ConvexPolyhedron::volume() {
    // TODO
    return 0.0;
}

Cannon::Math::Vec3* ConvexPolyhedron::getAveragePointLocal(Math::Vec3* target) {
    // TODO
    return nullptr;
}

void ConvexPolyhedron::transformAllPoints(Math::Vec3* offset, Math::Quaternion* quat) {
    // TODO
}

bool ConvexPolyhedron::pointIsInside(Math::Vec3* p) {
    // TODO
    return false;
}

void ConvexPolyhedron::project(
    ConvexPolyhedron* hull,
    Math::Vec3* axis,
    Math::Vec3* pos,
    Math::Quaternion* quat,
    std::array<float, 2>* result) {
    // TODO
}
