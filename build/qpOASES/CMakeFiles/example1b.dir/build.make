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
CMAKE_SOURCE_DIR = /home/shipengl/LRSMT/env_build/qpOASES-3.2.1

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shipengl/LRSMT/build/qpOASES

# Include any dependencies generated for this target.
include CMakeFiles/example1b.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/example1b.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/example1b.dir/flags.make

CMakeFiles/example1b.dir/examples/example1b.cpp.o: CMakeFiles/example1b.dir/flags.make
CMakeFiles/example1b.dir/examples/example1b.cpp.o: /home/shipengl/LRSMT/env_build/qpOASES-3.2.1/examples/example1b.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shipengl/LRSMT/build/qpOASES/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/example1b.dir/examples/example1b.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/example1b.dir/examples/example1b.cpp.o -c /home/shipengl/LRSMT/env_build/qpOASES-3.2.1/examples/example1b.cpp

CMakeFiles/example1b.dir/examples/example1b.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/example1b.dir/examples/example1b.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shipengl/LRSMT/env_build/qpOASES-3.2.1/examples/example1b.cpp > CMakeFiles/example1b.dir/examples/example1b.cpp.i

CMakeFiles/example1b.dir/examples/example1b.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/example1b.dir/examples/example1b.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shipengl/LRSMT/env_build/qpOASES-3.2.1/examples/example1b.cpp -o CMakeFiles/example1b.dir/examples/example1b.cpp.s

# Object files for target example1b
example1b_OBJECTS = \
"CMakeFiles/example1b.dir/examples/example1b.cpp.o"

# External object files for target example1b
example1b_EXTERNAL_OBJECTS =

bin/example1b: CMakeFiles/example1b.dir/examples/example1b.cpp.o
bin/example1b: CMakeFiles/example1b.dir/build.make
bin/example1b: libs/libqpOASES.a
bin/example1b: CMakeFiles/example1b.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shipengl/LRSMT/build/qpOASES/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable bin/example1b"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/example1b.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/example1b.dir/build: bin/example1b

.PHONY : CMakeFiles/example1b.dir/build

CMakeFiles/example1b.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/example1b.dir/cmake_clean.cmake
.PHONY : CMakeFiles/example1b.dir/clean

CMakeFiles/example1b.dir/depend:
	cd /home/shipengl/LRSMT/build/qpOASES && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shipengl/LRSMT/env_build/qpOASES-3.2.1 /home/shipengl/LRSMT/env_build/qpOASES-3.2.1 /home/shipengl/LRSMT/build/qpOASES /home/shipengl/LRSMT/build/qpOASES /home/shipengl/LRSMT/build/qpOASES/CMakeFiles/example1b.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/example1b.dir/depend

