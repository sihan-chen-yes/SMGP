# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-build"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/tmp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/src/imguizmo-populate-stamp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/src/imguizmo-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/src/imguizmo-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/imguizmo-subbuild/imguizmo-populate-prefix/src/imguizmo-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
