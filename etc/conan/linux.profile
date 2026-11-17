# Toolchain the Viam C++ SDK publishes its Linux binaries with: the default
# profile of the ghcr.io/viamrobotics/cpp-sdk-conan-ubuntu:noble-2026-11.02
# image that CI builds in.
# bin/build.sh passes it with -pr:a so our dependency package IDs match those
# binaries. A `conan profile detect` profile often differs (e.g. cppstd=gnu17),
# which silently misses the cache and rebuilds everything from source.
#
# compiler.cppstd is 20, the standard the SDK publishes every binary with (its
# recipe rejects anything lower). gcc 13 is the project's compiler floor (see
# CMakeLists.txt). Keep in sync when bumping the SDK.
include(default)

[settings]
os=Linux
compiler=gcc
compiler.version=13
compiler.libcxx=libstdc++11
compiler.cppstd=20
build_type=Release
