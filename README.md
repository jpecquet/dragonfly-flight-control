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

## Plot Command Config

The `dragonfly plot` command reads a config file (for example `configs/plot.cfg`).
You can optionally choose which Python interpreter it uses:

```ini
input = output.h5
output = output.mp4
python_executable = /Users/jean/.pyenv/shims/python
```

Interpreter resolution order is:
1. `python_executable` in the config
2. `PYTHON` environment variable
3. fallback `python3`

## Build Documentation

```bash
python -m pip install -r docs/requirements.txt
make -C docs html
```

Open `docs/_build/html/index.html` in a browser.
