# Box2D-Lite
Box2D-Lite is a small 2D physics engine. It was developed for the [2006 GDC Physics Tutorial](docs/GDC2006_Catto_Erin_PhysicsTutorial.pdf). This is the original version of the larger [Box2D](https://box2d.org) library. The Lite version is more suitable for learning about game physics.

# Cloning
The Box2D-Lite samples program uses [GLFW](https://www.glfw.org/) and [dear imgui](https://github.com/ocornut/imgui) as git submodules. I recommend cloning Box2D-Lite using the following command:
```bash
git clone --recursive https://github.com/erincatto/box2d-lite.git
```

# Building
- Install [CMake](https://cmake.org/)
- Ensure CMake is in the user `PATH`
- Visual Studio 2017: run `build.bat`
- Otherwise: run `build.sh` from a bash shell

# Build Status
[![Build Status](https://travis-ci.org/erincatto/box2d-lite.svg?branch=master)](https://travis-ci.org/erincatto/box2d-lite)
