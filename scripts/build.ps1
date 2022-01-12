Param(
    [Parameter()]
    [ValidateSet("Debug","Release")]
    [string]$Target="Debug"
);

$BuildDirectory = "..\build\"
$TargetBuildDirectory = "..\build\$Target"

Clear-Host

# Create Build Directory

if (-Not (Test-Path -Path $BuildDirectory))
{
    New-Item -Path ".." -Name "build" -ItemType "directory"
}

# Build source

if (-Not (Test-Path -Path $TargetBuildDirectory))
{
    New-Item -Path "..\build" -Name $Target -ItemType "directory"
}

if ($Target -eq "Debug")
{
    cd ../build/Debug/
    cmake -DCMAKE_PREFIX_PATH=c:/SDL2 -DCMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++.exe -DCMAKE_BUILD_TYPE=Debug -G "MinGW Makefiles" ../../
    mingw32-make
}
else
{
    cd ../build/Release/
    cmake -D CMAKE_CXX_COMPILER=x86_64-w64-mingw32-g++.exe -D CMAKE_BUILD_TYPE=Release -G "MinGW Makefiles" ../../
    mingw32-make
}

Pause

