# Cannon Work in Progress

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
