# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shipengl/LRSMT/env_build/eigen-3.3.2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shipengl/LRSMT/build/Eigen3

# Include any dependencies generated for this target.
include doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/depend.make

# Include the progress variables for this target.
include doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/progress.make

# Include the compile flags for this target's objects.
include doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/flags.make

doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o: doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/flags.make
doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o: doc/snippets/compile_MatrixBase_asDiagonal.cpp
doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o: /home/shipengl/LRSMT/env_build/eigen-3.3.2/doc/snippets/MatrixBase_asDiagonal.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shipengl/LRSMT/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o"
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o -c /home/shipengl/LRSMT/build/Eigen3/doc/snippets/compile_MatrixBase_asDiagonal.cpp

doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.i"
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shipengl/LRSMT/build/Eigen3/doc/snippets/compile_MatrixBase_asDiagonal.cpp > CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.i

doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.s"
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shipengl/LRSMT/build/Eigen3/doc/snippets/compile_MatrixBase_asDiagonal.cpp -o CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.s

# Object files for target compile_MatrixBase_asDiagonal
compile_MatrixBase_asDiagonal_OBJECTS = \
"CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o"

# External object files for target compile_MatrixBase_asDiagonal
compile_MatrixBase_asDiagonal_EXTERNAL_OBJECTS =

doc/snippets/compile_MatrixBase_asDiagonal: doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/compile_MatrixBase_asDiagonal.cpp.o
doc/snippets/compile_MatrixBase_asDiagonal: doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/build.make
doc/snippets/compile_MatrixBase_asDiagonal: doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shipengl/LRSMT/build/Eigen3/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable compile_MatrixBase_asDiagonal"
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/compile_MatrixBase_asDiagonal.dir/link.txt --verbose=$(VERBOSE)
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && ./compile_MatrixBase_asDiagonal >/home/shipengl/LRSMT/build/Eigen3/doc/snippets/MatrixBase_asDiagonal.out

# Rule to build all files generated by this target.
doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/build: doc/snippets/compile_MatrixBase_asDiagonal

.PHONY : doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/build

doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/clean:
	cd /home/shipengl/LRSMT/build/Eigen3/doc/snippets && $(CMAKE_COMMAND) -P CMakeFiles/compile_MatrixBase_asDiagonal.dir/cmake_clean.cmake
.PHONY : doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/clean

doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/depend:
	cd /home/shipengl/LRSMT/build/Eigen3 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shipengl/LRSMT/env_build/eigen-3.3.2 /home/shipengl/LRSMT/env_build/eigen-3.3.2/doc/snippets /home/shipengl/LRSMT/build/Eigen3 /home/shipengl/LRSMT/build/Eigen3/doc/snippets /home/shipengl/LRSMT/build/Eigen3/doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : doc/snippets/CMakeFiles/compile_MatrixBase_asDiagonal.dir/depend

