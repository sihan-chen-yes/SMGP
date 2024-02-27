# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-build"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/tmp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/src"
  "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/sihanchen/Desktop/smgp/gp24-sihan-chen-yes/assignment1/cmake-build-debug/_deps/eigen-subbuild/eigen-populate-prefix/src/eigen-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
