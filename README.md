# Cannon Work in Progress

## Progress

 - [ ] AABB
 - [ ] ArrayCollisionMatrix
 - [ ] Body
 - [ ] Box
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
 - [ ] EventTarget
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
 - [ ] Pool
 - [ ] Particle
 - [ ] Plane
 - [ ] PointToPointConstraint
 - [ ] Quaternion
 - [ ] Ray
 - [ ] RaycastVehicle
 - [ ] RaycastResult
 - [ ] RigidVehicle
 - [ ] RotationalEquation
 - [ ] RotationalMotorEquation
 - [ ] SAPBroadphase
 - [ ] SPHSystem
 - [ ] Shape
 - [ ] Solver
 - [ ] Sphere
 - [ ] SplitSolver
 - [ ] Spring
 - [ ] Transform
 - [ ] Trimesh
 - [ ] Vec3
 - [ ] Vec3Pool
 - [ ] World
 - [ ] Octree

## Build

> Recommand to use cmake in Linux or Mac.
> In Windows, use wsl with Linux subsystem avoid build problem.

```shell
cmake -S . -B build
cmake --build build
```

## Run

> With test case

```shell
cd build && ctest
```
