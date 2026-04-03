# C++
The core library is in c++. To compile it, we rely on cmake. We assume you have the necessary tools to compile c++ code on your machine

## compiling
to compile and link the c++, in the root of the repository create a build folder and configure/build there:

#### linux & mac:
```bash
mkdir build && cd build && cmake .. && make -j$(nproc)
```

#### windows:
```bash
mkdir build && cd build && cmake .. && cmake --build . --config Debug
```

after this you should see in the `./src/tests/pybinding_test` the necessary .so/.dylib/.dll (depending on platform) as well as the python binding

# python
to run the python sandbox (where I test the library for now), `uv` is preferred. [Install uv](https://docs.astral.sh/uv/getting-started/installation/), then run in the `./src/tests/pybinding_test`:

```bash
uv sync #this will create the environment that we have defined for you
uv run main.py
```
If everything goes to plan, it should work (at least as soon as my 2 fixes for compas_viewer are merged...)