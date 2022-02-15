cd ..
mkdir build
cd build
mkdir Release
cd Release

cmake ^
    -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++.exe ^
    -DCMAKE_BUILD_TYPE=Release ^
    -G "MinGW Makefiles" ^
    ../../

mingw32-make

ctest

pause
