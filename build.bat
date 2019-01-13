rem Use this batch file to build box2d-lite and samples for Visual Studio 2017
rmdir /s /q build
mkdir build
cd build
cmake ..
cmake --build .
start box2d-lite.sln
