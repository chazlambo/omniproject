# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /snap/cmake/775/bin/cmake

# The command to remove a file.
RM = /snap/cmake/775/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/charlie/Desktop/Testing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/charlie/Desktop/Testing/build

# Include any dependencies generated for this target.
include CMakeFiles/multimagnet.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/multimagnet.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/multimagnet.dir/flags.make

CMakeFiles/multimagnet.dir/multimagnet.cpp.o: CMakeFiles/multimagnet.dir/flags.make
CMakeFiles/multimagnet.dir/multimagnet.cpp.o: ../multimagnet.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/charlie/Desktop/Testing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/multimagnet.dir/multimagnet.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/multimagnet.dir/multimagnet.cpp.o -c /home/charlie/Desktop/Testing/multimagnet.cpp

CMakeFiles/multimagnet.dir/multimagnet.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/multimagnet.dir/multimagnet.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/charlie/Desktop/Testing/multimagnet.cpp > CMakeFiles/multimagnet.dir/multimagnet.cpp.i

CMakeFiles/multimagnet.dir/multimagnet.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/multimagnet.dir/multimagnet.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/charlie/Desktop/Testing/multimagnet.cpp -o CMakeFiles/multimagnet.dir/multimagnet.cpp.s

# Object files for target multimagnet
multimagnet_OBJECTS = \
"CMakeFiles/multimagnet.dir/multimagnet.cpp.o"

# External object files for target multimagnet
multimagnet_EXTERNAL_OBJECTS =

libmultimagnet.a: CMakeFiles/multimagnet.dir/multimagnet.cpp.o
libmultimagnet.a: CMakeFiles/multimagnet.dir/build.make
libmultimagnet.a: CMakeFiles/multimagnet.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/charlie/Desktop/Testing/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libmultimagnet.a"
	$(CMAKE_COMMAND) -P CMakeFiles/multimagnet.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/multimagnet.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/multimagnet.dir/build: libmultimagnet.a

.PHONY : CMakeFiles/multimagnet.dir/build

CMakeFiles/multimagnet.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/multimagnet.dir/cmake_clean.cmake
.PHONY : CMakeFiles/multimagnet.dir/clean

CMakeFiles/multimagnet.dir/depend:
	cd /home/charlie/Desktop/Testing/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/charlie/Desktop/Testing /home/charlie/Desktop/Testing /home/charlie/Desktop/Testing/build /home/charlie/Desktop/Testing/build /home/charlie/Desktop/Testing/build/CMakeFiles/multimagnet.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/multimagnet.dir/depend
