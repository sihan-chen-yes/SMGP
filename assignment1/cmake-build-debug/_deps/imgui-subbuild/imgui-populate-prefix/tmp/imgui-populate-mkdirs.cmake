# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-build"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/tmp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imgui-subbuild/imgui-populate-prefix/src/imgui-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
