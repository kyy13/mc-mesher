cd ..
mkdir build
cd build
mkdir Debug
cd Debug

cmake ^
    -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++.exe ^
    -DCMAKE_BUILD_TYPE=Debug ^
    -G "MinGW Makefiles" ^
    ../../

mingw32-make

ctest

pause
