# Dragonfly Flight Control

## Build Prerequisites

- C++17 compiler
- CMake 3.16+
- [Eigen3](https://eigen.tuxfamily.org/)
- [HDF5](https://www.hdfgroup.org/solutions/hdf5/) (C and C++)
- [NLopt](https://github.com/stevengj/nlopt)

`HighFive` is fetched automatically by CMake (`FetchContent`).

## Build

```bash
mkdir -p build
cd build
cmake ..
make
```

## Run Tests

```bash
cd build
ctest
```

## Build Documentation

```bash
python -m pip install -r docs/requirements.txt
make -C docs html
```

Open `docs/_build/html/index.html` in a browser.
