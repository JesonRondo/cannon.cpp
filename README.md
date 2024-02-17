# Cannon Work in Progress

![Package Size](https://img.shields.io/badge/dynamic/json?label=Package%20Size&query=$.lib_size&url=https://raw.githubusercontent.com/JesonRondo/cannon.cpp/develop/badge.json)

## Progress

 - [ ] AABB
 - [ ] ArrayCollisionMatrix
 - [ ] Body
 - [x] Box
 - [ ] Broadphase
 - [ ] Constraint
 - [ ] ContactEquation
 - [ ] Narrowphase
 - [ ] ConeTwistConstraint
 - [ ] ContactMaterial
 - [ ] ConvexPolyhedron
 - [ ] Cylinder
 - [ ] DistanceConstraint
 - [ ] Equation
 - [x] EventTarget
 - [ ] FrictionEquation
 - [ ] GSSolver
 - [ ] GridBroadphase
 - [ ] Heightfield
 - [ ] HingeConstraint
 - [ ] LockConstraint
 - [x] Mat3
 - [ ] Material
 - [ ] NaiveBroadphase
 - [ ] ObjectCollisionMatrix
 - [x] Pool
 - [ ] Particle
 - [x] Plane
 - [ ] PointToPointConstraint
 - [x] Quaternion
 - [ ] Ray
 - [ ] RaycastVehicle
 - [ ] RaycastResult
 - [ ] RigidVehicle
 - [ ] RotationalEquation
 - [ ] RotationalMotorEquation
 - [ ] SAPBroadphase
 - [ ] SPHSystem
 - [x] Shape
 - [ ] Solver
 - [x] Sphere
 - [ ] SplitSolver
 - [ ] Spring
 - [x] Transform
 - [ ] Trimesh
 - [x] Vec3
 - [x] Vec3Pool
 - [ ] World
 - [ ] Octree

## Build

> Recommand to use cmake in Linux or Mac.
> In Windows, use wsl with Linux subsystem avoid build problem.

```shell
cmake -S . -B build
cmake --build build
```

Update badge.

```shell
bash ./script/badge.sh
```

## Run

> With test case

```shell
cd build && ctest
#or
./build/cannon_test
```
