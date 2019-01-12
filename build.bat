rem Use this batch file to build box2d-lite and samples for Visual Studio 2017
rmdir /s /q build
mkdir build
cd build
cmake .. -G "Visual Studio 15 2017" -A x64
cmake --build .
start box2d-lite.sln
