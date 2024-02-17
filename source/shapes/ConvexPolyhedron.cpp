#include "shapes/ConvexPolyhedron.h"

// #include <iostream>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include "math/Vec3.h"
#include "math/Transform.h"

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

Cannon::Math::Vec3 project_worldVertex;
Cannon::Math::Vec3 project_localAxis;
Cannon::Math::Vec3 project_localOrigin;
void ConvexPolyhedron::project(
    ConvexPolyhedron* hull,
    Math::Vec3* axis,
    Math::Vec3* pos,
    Math::Quaternion* quat,
    std::array<float, 2>* result) {
    int n = hull->vertices->size();
    Cannon::Math::Vec3* worldVertex = &project_worldVertex;
    Cannon::Math::Vec3* localAxis = &project_localAxis;
    float max = 0;
    float min = 0;
    Cannon::Math::Vec3* localOrigin = &project_localOrigin;
    std::vector<Cannon::Math::Vec3>* vs = hull->vertices;

    localOrigin->setZero();

    // Transform the axis to local
    Math::Transform::vectorToLocalFrame(pos, quat, axis, localAxis);
    Math::Transform::pointToLocalFrame(pos, quat, localOrigin, localOrigin);
    float add = localOrigin->dot(localAxis);

    min = max = vs->at(0).dot(localAxis);

    for (int i = 1; i < n; i++) {
        float val = vs->at(i).dot(localAxis);

        if (val > max) {
            max = val;
        }

        if (val < min) {
            min = val;
        }
    }

    min -= add;
    max -= add;

    if (min > max) {
        // Inconsistent - swap
        float temp = min;
        min = max;
        max = temp;
    }
    // Output
    result->at(0) = max;
    result->at(1) = min;
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

Cannon::Math::Vec3 cah_WorldNormal;
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
    Math::Vec3* WorldNormal = &cah_WorldNormal;
    ConvexPolyhedron* hullA = this;
    float curMaxDist = maxDist;
    int closestFaceB = -1;
    float dmax = -MAX_FLOAT;
    for (int face = 0; face < hullB->faces->size(); face++) {
        WorldNormal->copy(&hullB->faceNormals[face]);
        quatB->vmult(WorldNormal, WorldNormal);
        //posB.vadd(WorldNormal,WorldNormal);
        float d = WorldNormal->dot(separatingNormal);
        if (d > dmax){
            dmax = d;
            closestFaceB = face;
        }
    }
    std::vector<Math::Vec3> worldVertsB1;
    std::vector<int> polyB = hullB->faces->at(closestFaceB);
    int numVertices = polyB.size();
    for (int e0 = 0; e0 < numVertices; e0++) {
        Math::Vec3* b = &hullB->vertices->at(polyB[e0]);
        Math::Vec3 worldb;
        worldb.copy(b);
        quatB->vmult(&worldb, &worldb);
        posB->vadd(&worldb, &worldb);
        worldVertsB1.push_back(worldb);
    }

    if (closestFaceB >= 0) {
        this->clipFaceAgainstHull(
            separatingNormal,
            posA,
            quatA,
            &worldVertsB1,
            minDist,
            maxDist,
            result
        );
    }
}

Cannon::Math::Vec3 fsa_faceANormalWS3;
Cannon::Math::Vec3 fsa_Worldnormal1;
Cannon::Math::Vec3 fsa_deltaC;
Cannon::Math::Vec3 fsa_worldEdge0;
Cannon::Math::Vec3 fsa_worldEdge1;
Cannon::Math::Vec3 fsa_Cross;
bool ConvexPolyhedron::findSeparatingAxis(
    ConvexPolyhedron* hullB,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    Math::Vec3* posB,
    Math::Quaternion* quatB,
    Math::Vec3* target,
    std::vector<int>* faceListA,
    std::vector<int>* faceListB) {
    Cannon::Math::Vec3* faceANormalWS3 = &fsa_faceANormalWS3;
    Cannon::Math::Vec3* Worldnormal1 = &fsa_Worldnormal1;
    Cannon::Math::Vec3* deltaC = &fsa_deltaC;
    Cannon::Math::Vec3* worldEdge0 = &fsa_worldEdge0;
    Cannon::Math::Vec3* worldEdge1 = &fsa_worldEdge1;
    Cannon::Math::Vec3* Cross = &fsa_Cross;

    float dmin = MAX_FLOAT;
    ConvexPolyhedron* hullA = this;
    int curPlaneTests = 0;

    if (hullA->uniqueAxes == nullptr) {
        int numFacesA = faceListA != nullptr ? faceListA->size() : hullA->faces->size();

        // Test face normals from hullA
        for (int i = 0; i < numFacesA; i++) {
            int fi = faceListA != nullptr ? faceListA->at(i) : i;

            // Get world face normal
            faceANormalWS3->copy(&hullA->faceNormals[fi]);
            quatA->vmult(faceANormalWS3, faceANormalWS3);

            DepthOrBool d = hullA->testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
            if (!d.boolean){
                return false;
            }

            if (d.depth < dmin) {
                dmin = d.depth;
                target->copy(faceANormalWS3);
            }
        }
    } else {
        // Test unique axes
        for (int i = 0; i != hullA->uniqueAxes->size(); i++) {
            // Get world axis
            quatA->vmult(&hullA->uniqueAxes->at(i), faceANormalWS3);

            DepthOrBool d = hullA->testSepAxis(faceANormalWS3, hullB, posA, quatA, posB, quatB);
            if (!d.boolean){
                return false;
            }

            if (d.depth < dmin) {
                dmin = d.depth;
                target->copy(faceANormalWS3);
            }
        }
    }

    if (hullB->uniqueAxes == nullptr) {
        // Test face normals from hullB
        int numFacesB = faceListB != nullptr ? faceListB->size() : hullB->faces->size();
        for (int i = 0; i < numFacesB; i++) {
            int fi = faceListB != nullptr ? faceListB->at(i) : i;

            Worldnormal1->copy(&hullB->faceNormals[fi]);
            quatB->vmult(Worldnormal1, Worldnormal1);
            curPlaneTests++;
            DepthOrBool d = hullA->testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB);
            if (!d.boolean) {
                return false;
            }

            if (d.depth < dmin) {
                dmin = d.depth;
                target->copy(Worldnormal1);
            }
        }
    } else {
        // Test unique axes in B
        for (int i = 0; i != hullB->uniqueAxes->size(); i++) {
            quatB->vmult(&hullB->uniqueAxes->at(i), Worldnormal1);

            curPlaneTests++;
            DepthOrBool d = hullA->testSepAxis(Worldnormal1, hullB, posA, quatA, posB, quatB);
            if (!d.boolean) {
                return false;
            }

            if (d.depth < dmin) {
                dmin = d.depth;
                target->copy(Worldnormal1);
            }
        }
    }

    // Test edges
    for (int e0 = 0; e0 != hullA->uniqueEdges.size(); e0++) {
        // Get world edge
        quatA->vmult(&hullA->uniqueEdges[e0], worldEdge0);

        for (int e1 = 0; e1 != hullB->uniqueEdges.size(); e1++) {
            // Get world edge 2
            quatB->vmult(&hullB->uniqueEdges[e1], worldEdge1);
            worldEdge0->cross(worldEdge1, Cross);

            if (!Cross->almostZero(0.00001)) {
                Cross->normalize();
                DepthOrBool dist = hullA->testSepAxis(Cross, hullB, posA, quatA, posB, quatB);
                if (!dist.boolean) {
                    return false;
                }
                if (dist.depth < dmin) {
                    dmin = dist.depth;
                    target->copy(Cross);
                }
            }
        }
    }

    posB->vsub(posA, deltaC);
    if ((deltaC->dot(target)) > 0.0) {
        target->negate(target);
    }

    return true;
}

std::array<float, 2> maxminA;
std::array<float, 2> maxminB;
DepthOrBool ConvexPolyhedron::testSepAxis(
    Math::Vec3* axis,
    ConvexPolyhedron* hullB,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    Math::Vec3* posB,
    Math::Quaternion* quatB) {
    ConvexPolyhedron* hullA = this;
    ConvexPolyhedron::project(hullA, axis, posA, quatA, &maxminA);
    ConvexPolyhedron::project(hullB, axis, posB, quatB, &maxminB);
    float maxA = maxminA[0];
    float minA = maxminA[1];
    float maxB = maxminB[0];
    float minB = maxminB[1];
    if (maxA < minB || maxB < minA) {
        return {
            boolean: false,
        }; // Separated
    }
    float d0 = maxA - minB;
    float d1 = maxB - minA;
    float depth = d0 < d1 ? d0 : d1;
    return {
        depth,
        boolean: true,
    };
}

Cannon::Math::Vec3 cli_aabbmin;
Cannon::Math::Vec3 cli_aabbmax;
void ConvexPolyhedron::calculateLocalInertia(float mass, Math::Vec3* target) {
    // Approximate with box inertia
    // Exact inertia calculation is overkill, but see http://geometrictools.com/Documentation/PolyhedralMassProperties.pdf for the correct way to do it
    this->computeLocalAABB(&cli_aabbmin, &cli_aabbmax);
    float x = cli_aabbmax.x - cli_aabbmin.x;
    float y = cli_aabbmax.y - cli_aabbmin.y;
    float z = cli_aabbmax.z - cli_aabbmin.z;
    target->x = 1.0 / 12.0 * mass * (2 * y * 2 * y + 2 * z * 2 * z);
    target->y = 1.0 / 12.0 * mass * (2 * x * 2 * x + 2 * z * 2 * z);
    target->z = 1.0 / 12.0 * mass * (2 * y * 2 * y + 2 * x * 2 * x);
}

float ConvexPolyhedron::getPlaneConstantOfFace(int face_i) {
    std::vector<int> f = this->faces->at(face_i);
    Math::Vec3* n = &this->faceNormals[face_i];
    Math::Vec3* v = &this->vertices->at(f[0]);
    float c = -n->dot(v);
    return c;
}

Cannon::Math::Vec3 cfah_faceANormalWS;
Cannon::Math::Vec3 cfah_edge0;
Cannon::Math::Vec3 cfah_WorldEdge0;
Cannon::Math::Vec3 cfah_worldPlaneAnormal1;
Cannon::Math::Vec3 cfah_planeNormalWS1;
Cannon::Math::Vec3 cfah_worldA1;
Cannon::Math::Vec3 cfah_localPlaneNormal;
Cannon::Math::Vec3 cfah_planeNormalWS;
void ConvexPolyhedron::clipFaceAgainstHull(
    Math::Vec3* separatingNormal,
    Math::Vec3* posA,
    Math::Quaternion* quatA,
    std::vector<Math::Vec3>* worldVertsB1,
    float minDist,
    float maxDist,
    std::vector<PointObject>* result) {
    Cannon::Math::Vec3* faceANormalWS = &cfah_faceANormalWS;
    Cannon::Math::Vec3* edge0 = &cfah_edge0;
    Cannon::Math::Vec3* WorldEdge0 = &cfah_WorldEdge0;
    Cannon::Math::Vec3* worldPlaneAnormal1 = &cfah_worldPlaneAnormal1;
    Cannon::Math::Vec3* planeNormalWS1 = &cfah_planeNormalWS1;
    Cannon::Math::Vec3* worldA1 = &cfah_worldA1;
    Cannon::Math::Vec3* localPlaneNormal = &cfah_localPlaneNormal;
    Cannon::Math::Vec3* planeNormalWS = &cfah_planeNormalWS;

    ConvexPolyhedron* hullA = this;
    std::vector<Math::Vec3> worldVertsB2;
    std::vector<Math::Vec3>* pVtxIn = worldVertsB1;
    std::vector<Math::Vec3>* pVtxOut = &worldVertsB2;
    // Find the face with normal closest to the separating axis
    int closestFaceA = -1;
    float dmin = MAX_FLOAT;
    for (int face = 0; face < hullA->faces->size(); face++) {
        faceANormalWS->copy(&hullA->faceNormals[face]);
        quatA->vmult(faceANormalWS, faceANormalWS);
        //posA.vadd(faceANormalWS,faceANormalWS);
        float d = faceANormalWS->dot(separatingNormal);
        if (d < dmin) {
            dmin = d;
            closestFaceA = face;
        }
    }
    if (closestFaceA < 0) {
        // console.log("--- did not find any closest face... ---");
        return;
    }
    //console.log("closest A: ",closestFaceA);
    // Get the face and construct connected faces
    std::vector<int>* polyA = &hullA->faces->at(closestFaceA);
    std::vector<int> polyAConnectedFaces;
    for (int i = 0; i < hullA->faces->size(); i++) {
        for (int j = 0; j < hullA->faces->at(i).size(); j++) {
            if (std::find(polyA->begin(), polyA->end(), hullA->faces->at(i)[j]) != polyA->end() /* Sharing a vertex*/
                && i != closestFaceA /* Not the one we are looking for connections from */
                && std::find(polyAConnectedFaces.begin(), polyAConnectedFaces.end(), i) == polyAConnectedFaces.end() /* Not already added */) {
                polyAConnectedFaces.push_back(i);
            }
        }
    }
    // Clip the polygon to the back of the planes of all faces of hull A, that are adjacent to the witness face
    int numContacts = pVtxIn->size();
    int numVerticesA = polyA->size();
    for (int e0 = 0; e0 < numVerticesA; e0++) {
        Math::Vec3* a = &hullA->vertices->at(polyA->at(e0));
        Math::Vec3* b = &hullA->vertices->at(polyA->at((e0 + 1) % numVerticesA));
        a->vsub(b, edge0);
        WorldEdge0->copy(edge0);
        quatA->vmult(WorldEdge0, WorldEdge0);
        posA->vadd(WorldEdge0, WorldEdge0);
        worldPlaneAnormal1->copy(&this->faceNormals[closestFaceA]);//transA.getBasis()* btVector3(polyA.m_plane[0],polyA.m_plane[1],polyA.m_plane[2]);
        quatA->vmult(worldPlaneAnormal1, worldPlaneAnormal1);
        posA->vadd(worldPlaneAnormal1, worldPlaneAnormal1);
        WorldEdge0->cross(worldPlaneAnormal1, planeNormalWS1);
        planeNormalWS1->negate(planeNormalWS1);
        worldA1->copy(a);
        quatA->vmult(worldA1, worldA1);
        posA->vadd(worldA1, worldA1);
        float planeEqWS1 = -worldA1->dot(planeNormalWS1);
        float planeEqWS;
        if (true) {
            int otherFace = polyAConnectedFaces[e0];
            localPlaneNormal->copy(&this->faceNormals[otherFace]);
            float localPlaneEq = this->getPlaneConstantOfFace(otherFace);

            planeNormalWS->copy(localPlaneNormal);
            quatA->vmult(planeNormalWS, planeNormalWS);
            //posA.vadd(planeNormalWS,planeNormalWS);
            float planeEqWS = localPlaneEq - planeNormalWS->dot(posA);
        } else  {
            planeNormalWS->copy(planeNormalWS1);
            planeEqWS = planeEqWS1;
        }

        // Clip face against our constructed plane
        this->clipFaceAgainstPlane(pVtxIn, pVtxOut, planeNormalWS, planeEqWS);

        // Throw away all clipped points, but save the reamining until next clip
        while (!pVtxIn->empty()) {
            pVtxIn->erase(pVtxIn->begin());
        }
        while (!pVtxOut->empty()) {
            pVtxIn->push_back(pVtxOut->at(0));
            pVtxOut->erase(pVtxOut->begin());
        }
    }

    //console.log("Resulting points after clip:",pVtxIn);

    // only keep contact points that are behind the witness face
    localPlaneNormal->copy(&this->faceNormals[closestFaceA]);

    float localPlaneEq = this->getPlaneConstantOfFace(closestFaceA);
    planeNormalWS->copy(localPlaneNormal);
    quatA->vmult(planeNormalWS, planeNormalWS);

    float planeEqWS = localPlaneEq - planeNormalWS->dot(posA);
    for (int i = 0; i < pVtxIn->size(); i++) {
        float depth = planeNormalWS->dot(&pVtxIn->at(i)) + planeEqWS; //???
        /*console.log("depth calc from normal=",planeNormalWS.toString()," and constant "+planeEqWS+" and vertex ",pVtxIn[i].toString()," gives "+depth);*/
        if (depth <= minDist) {
            // console.log("clamped: depth="+depth+" to minDist="+(minDist+""));
            depth = minDist;
        }

        if (depth <= maxDist) {
            Math::Vec3* point = &pVtxIn->at(i);
            if (depth <= 0) {
                /*console.log("Got contact point ",point.toString(),
                  ", depth=",depth,
                  "contact normal=",separatingNormal.toString(),
                  "plane",planeNormalWS.toString(),
                  "planeConstant",planeEqWS);*/
                PointObject p {
                    point: point->clone(),
                    normal: planeNormalWS->clone(),
                    depth: depth,
                };
                result->push_back(p);
            }
        }
    }
}

void ConvexPolyhedron::clipFaceAgainstPlane(
    std::vector<Math::Vec3>* inVertices,
    std::vector<Math::Vec3>* outVertices,
    Math::Vec3* planeNormal,
    float planeConstant) {
    float n_dot_first, n_dot_last;
    int numVerts = inVertices->size();

    if (numVerts < 2) {
        return;
    }

    Math::Vec3* firstVertex = &inVertices->at(inVertices->size() - 1);
    Math::Vec3* lastVertex = &inVertices->at(0);

    n_dot_first = planeNormal->dot(firstVertex) + planeConstant;

    for (int vi = 0; vi < numVerts; vi++) {
        lastVertex = &inVertices->at(vi);
        n_dot_last = planeNormal->dot(lastVertex) + planeConstant;
        if (n_dot_first < 0) {
            if (n_dot_last < 0) {
                // Start < 0, end < 0, so output lastVertex
                Math::Vec3 newv;
                newv.copy(lastVertex);
                outVertices->push_back(newv);
            } else {
                // Start < 0, end >= 0, so output intersection
                Math::Vec3 newv;
                firstVertex->lerp(lastVertex,
                                 n_dot_first / (n_dot_first - n_dot_last),
                                 &newv);
                outVertices->push_back(newv);
            }
        } else {
            if (n_dot_last < 0) {
                // Start >= 0, end < 0 so output intersection and end
                Math::Vec3 newv;
                firstVertex->lerp(lastVertex,
                                 n_dot_first / (n_dot_first - n_dot_last),
                                 &newv);
                outVertices->push_back(newv);
                outVertices->push_back(*lastVertex);
            }
        }
        firstVertex = lastVertex;
        n_dot_first = n_dot_last;
    }
    return;
}

void ConvexPolyhedron::computeWorldVertices(Math::Vec3* position, Math::Quaternion* quat) {
    int N = this->vertices->size();
    while (this->worldVertices.size() < N) {
        this->worldVertices.push_back(Math::Vec3());
    }

    std::vector<Math::Vec3>* verts = this->vertices;
    std::vector<Cannon::Math::Vec3>* worldVerts = &this->worldVertices;
    for (int i = 0; i != N; i++) {
        quat->vmult(&verts->at(i), &worldVerts->at(i));
        position->vadd(&worldVerts->at(i), &worldVerts->at(i));
    }

    this->worldVerticesNeedsUpdate = false;
}

Cannon::Math::Vec3 computeLocalAABB_worldVert;
void ConvexPolyhedron::computeLocalAABB(Math::Vec3* aabbmin, Math::Vec3* aabbmax) {
    int n = this->vertices->size();
    std::vector<Math::Vec3>* vertices = this->vertices;
    Math::Vec3* worldVert = &computeLocalAABB_worldVert;

    aabbmin->set(MAX_FLOAT, MAX_FLOAT, MAX_FLOAT);
    aabbmax->set(-MAX_FLOAT, -MAX_FLOAT, -MAX_FLOAT);

    for (int i = 0; i < n; i++) {
        Math::Vec3* v = &vertices->at(i);

        if (v->x < aabbmin->x) {
            aabbmin->x = v->x;
        } else if(v->x > aabbmax->x) {
            aabbmax->x = v->x;
        }

        if (v->y < aabbmin->y) {
            aabbmin->y = v->y;
        } else if(v->y > aabbmax->y) {
            aabbmax->y = v->y;
        }
        
        if (v->z < aabbmin->z) {
            aabbmin->z = v->z;
        } else if(v->z > aabbmax->z) {
            aabbmax->z = v->z;
        }
    }
}

void ConvexPolyhedron::computeWorldFaceNormals(Math::Quaternion* quat) {
    int N = this->faceNormals.size();
    while (this->worldFaceNormals.size() < N) {
        this->worldFaceNormals.push_back(Math::Vec3());
    }

    std::vector<Math::Vec3>* normals = &this->faceNormals;
    std::vector<Math::Vec3>* worldNormals = &this->worldFaceNormals;
    for (int i = 0; i !=N; i++) {
        quat->vmult(&normals->at(i), &worldNormals->at(i));
    }

    this->worldFaceNormalsNeedsUpdate = false;
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

Cannon::Math::Vec3 tempWorldVertex;
void ConvexPolyhedron::calculateWorldAABB(
    Math::Vec3* pos,
    Math::Quaternion* quat,
    Math::Vec3* min,
    Math::Vec3* max) {
    int n = this->vertices->size();
    std::vector<Math::Vec3>* verts = this->vertices;
    float minx = -MAX_FLOAT;
    float miny = -MAX_FLOAT;
    float minz = -MAX_FLOAT;
    float maxx = MAX_FLOAT;
    float maxy = MAX_FLOAT;
    float maxz = MAX_FLOAT;

    for (int i = 0; i < n; i++) {
        tempWorldVertex.copy(&verts->at(i));
        quat->vmult(&tempWorldVertex, &tempWorldVertex);
        pos->vadd(&tempWorldVertex, &tempWorldVertex);
        Math::Vec3* v = &tempWorldVertex;
        if (v->x < minx) {
            minx = v->x;
        } 
        
        if (v->x > maxx) {
            maxx = v->x;
        }

        if (v->y < miny){
            miny = v->y;
        } 
        
        if (v->y > maxy){
            maxy = v->y;
        }

        if (v->z < minz){
            minz = v->z;
        }  
        
        if (v->z > maxz){
            maxz = v->z;
        }
    }
    min->set(minx, miny, minz);
    max->set(maxx, maxy, maxz);
}

double ConvexPolyhedron::volume() {
    return 4.0 * M_PI * this->boundingSphereRadius / 3.0;
}

Cannon::Math::Vec3* ConvexPolyhedron::getAveragePointLocal(Math::Vec3* target) {
    int n = this->vertices->size();
    std::vector<Math::Vec3>* verts = this->vertices;
    for (int i= 0; i < n; i++) {
        target->vadd(&verts->at(i), target);
    }
    target->scale(1 / n, target);
    return target;
}

void ConvexPolyhedron::transformAllPoints(Math::Vec3* offset, Math::Quaternion* quat) {
    int n = this->vertices->size();
    std::vector<Math::Vec3>* verts = this->vertices;

    // Apply rotation
    if (quat != nullptr) {
        // Rotate vertices
        for (int i = 0; i < n; i++) {
            Math::Vec3* v = &verts->at(i);
            quat->vmult(v, v);
        }
        // Rotate face normals
        for (int i = 0; i < this->faceNormals.size(); i++) {
            Math::Vec3* v = &this->faceNormals[i];
            quat->vmult(v, v);
        }
        /*
        // Rotate edges
        for(var i=0; i<this.uniqueEdges.length; i++){
            var v = this.uniqueEdges[i];
            quat.vmult(v,v);
        }*/
    }

    // Apply offset
    if (offset != nullptr) {
        for (int i = 0; i < n; i++) {
            Math::Vec3* v = &verts->at(i);
            v->vadd(offset, v);
        }
    }
}

Cannon::Math::Vec3 ConvexPolyhedron_pointIsInside;
Cannon::Math::Vec3 ConvexPolyhedron_vToP;
Cannon::Math::Vec3 ConvexPolyhedron_vToPointInside;
bool ConvexPolyhedron::pointIsInside(Math::Vec3* p) {
    std::vector<Math::Vec3>* verts = this->vertices;
    std::vector<std::vector<int>>* faces = this->faces;
    std::vector<Math::Vec3>* normals = &this->faceNormals;
    // var positiveResult = null;
    int N = this->faces->size();
    Cannon::Math::Vec3* pointInside = &ConvexPolyhedron_pointIsInside;
    this->getAveragePointLocal(pointInside);
    for (int i = 0; i < N; i++) {
        int numVertices = this->faces->at(i).size();
        Math::Vec3* n = &normals->at(i);
        Math::Vec3* v = &verts->at(faces->at(i)[0]); // We only need one point in the face

        // This dot product determines which side of the edge the point is
        Cannon::Math::Vec3* vToP = &ConvexPolyhedron_vToP;
        p->vsub(v, vToP);
        float r1 = n->dot(vToP);

        Cannon::Math::Vec3* vToPointInside = &ConvexPolyhedron_vToPointInside;
        pointInside->vsub(v, vToPointInside);
        float r2 = n->dot(vToPointInside);

        if ((r1 < 0 && r2 > 0) || (r1 > 0 && r2 < 0)) {
            return false; // Encountered some other sign. Exit.
        }
    }

    // If we got here, all dot products were of the same sign.
    // return positiveResult ? 1 : -1;
    return true;
}
