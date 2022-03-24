cd ..
mkdir build
cd build
mkdir godot_cpp
cd godot_cpp

cmake ^
    -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++.exe ^
    -DCMAKE_BUILD_TYPE=Release ^
    -G "MinGW Makefiles" ^
    ../../ext/godot-cpp

mingw32-make

pause
