# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.28

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ubuntu/psd_ws/deps/libzmq-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/psd_ws/deps/libzmq-master/build

# Include any dependencies generated for this target.
include tests/CMakeFiles/test_req_relaxed.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include tests/CMakeFiles/test_req_relaxed.dir/compiler_depend.make

# Include the progress variables for this target.
include tests/CMakeFiles/test_req_relaxed.dir/progress.make

# Include the compile flags for this target's objects.
include tests/CMakeFiles/test_req_relaxed.dir/flags.make

tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o: tests/CMakeFiles/test_req_relaxed.dir/flags.make
tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o: /home/ubuntu/psd_ws/deps/libzmq-master/tests/test_req_relaxed.cpp
tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o: tests/CMakeFiles/test_req_relaxed.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/ubuntu/psd_ws/deps/libzmq-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o"
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o -MF CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o.d -o CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o -c /home/ubuntu/psd_ws/deps/libzmq-master/tests/test_req_relaxed.cpp

tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.i"
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ubuntu/psd_ws/deps/libzmq-master/tests/test_req_relaxed.cpp > CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.i

tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.s"
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build/tests && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ubuntu/psd_ws/deps/libzmq-master/tests/test_req_relaxed.cpp -o CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.s

# Object files for target test_req_relaxed
test_req_relaxed_OBJECTS = \
"CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o"

# External object files for target test_req_relaxed
test_req_relaxed_EXTERNAL_OBJECTS =

bin/test_req_relaxed: tests/CMakeFiles/test_req_relaxed.dir/test_req_relaxed.cpp.o
bin/test_req_relaxed: tests/CMakeFiles/test_req_relaxed.dir/build.make
bin/test_req_relaxed: lib/libtestutil.a
bin/test_req_relaxed: /usr/lib/x86_64-linux-gnu/librt.a
bin/test_req_relaxed: lib/libzmq.so.5.2.6
bin/test_req_relaxed: lib/libunity.a
bin/test_req_relaxed: tests/CMakeFiles/test_req_relaxed.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/ubuntu/psd_ws/deps/libzmq-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../bin/test_req_relaxed"
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build/tests && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_req_relaxed.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
tests/CMakeFiles/test_req_relaxed.dir/build: bin/test_req_relaxed
.PHONY : tests/CMakeFiles/test_req_relaxed.dir/build

tests/CMakeFiles/test_req_relaxed.dir/clean:
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build/tests && $(CMAKE_COMMAND) -P CMakeFiles/test_req_relaxed.dir/cmake_clean.cmake
.PHONY : tests/CMakeFiles/test_req_relaxed.dir/clean

tests/CMakeFiles/test_req_relaxed.dir/depend:
	cd /home/ubuntu/psd_ws/deps/libzmq-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/psd_ws/deps/libzmq-master /home/ubuntu/psd_ws/deps/libzmq-master/tests /home/ubuntu/psd_ws/deps/libzmq-master/build /home/ubuntu/psd_ws/deps/libzmq-master/build/tests /home/ubuntu/psd_ws/deps/libzmq-master/build/tests/CMakeFiles/test_req_relaxed.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : tests/CMakeFiles/test_req_relaxed.dir/depend
