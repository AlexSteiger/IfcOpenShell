# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/alex/Documents/IfcOpenShell/cmake

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/alex/Documents/IfcOpenShell/build

# Include any dependencies generated for this target.
include examples/CMakeFiles/IfcAdvancedHouse.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/IfcAdvancedHouse.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/IfcAdvancedHouse.dir/flags.make

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o: examples/CMakeFiles/IfcAdvancedHouse.dir/flags.make
examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o: /home/alex/Documents/IfcOpenShell/src/examples/IfcAdvancedHouse.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Documents/IfcOpenShell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o -c /home/alex/Documents/IfcOpenShell/src/examples/IfcAdvancedHouse.cpp

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.i"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Documents/IfcOpenShell/src/examples/IfcAdvancedHouse.cpp > CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.i

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.s"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Documents/IfcOpenShell/src/examples/IfcAdvancedHouse.cpp -o CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.s

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.requires:

.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.requires

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.provides: examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/IfcAdvancedHouse.dir/build.make examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.provides.build
.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.provides

examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.provides.build: examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o


# Object files for target IfcAdvancedHouse
IfcAdvancedHouse_OBJECTS = \
"CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o"

# External object files for target IfcAdvancedHouse
IfcAdvancedHouse_EXTERNAL_OBJECTS =

examples/IfcAdvancedHouse: examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o
examples/IfcAdvancedHouse: examples/CMakeFiles/IfcAdvancedHouse.dir/build.make
examples/IfcAdvancedHouse: libIfcParse.a
examples/IfcAdvancedHouse: libIfcGeom.a
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKernel.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKMath.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKBRep.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKGeomBase.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKGeomAlgo.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKG3d.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKG2d.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKShHealing.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKTopAlgo.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKMesh.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKPrim.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKBool.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKBO.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKFillet.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKSTEP.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKSTEPBase.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKSTEPAttr.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKXSBase.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKSTEP209.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKIGES.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKOffset.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libTKSTL.so
examples/IfcAdvancedHouse: libIfcParse.a
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libicuuc.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libicudata.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libdl.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libicui18n.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_system.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_regex.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_thread.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
examples/IfcAdvancedHouse: /usr/lib/x86_64-linux-gnu/libpthread.so
examples/IfcAdvancedHouse: examples/CMakeFiles/IfcAdvancedHouse.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Documents/IfcOpenShell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable IfcAdvancedHouse"
	cd /home/alex/Documents/IfcOpenShell/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IfcAdvancedHouse.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/IfcAdvancedHouse.dir/build: examples/IfcAdvancedHouse

.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/build

examples/CMakeFiles/IfcAdvancedHouse.dir/requires: examples/CMakeFiles/IfcAdvancedHouse.dir/IfcAdvancedHouse.cpp.o.requires

.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/requires

examples/CMakeFiles/IfcAdvancedHouse.dir/clean:
	cd /home/alex/Documents/IfcOpenShell/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/IfcAdvancedHouse.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/clean

examples/CMakeFiles/IfcAdvancedHouse.dir/depend:
	cd /home/alex/Documents/IfcOpenShell/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Documents/IfcOpenShell/cmake /home/alex/Documents/IfcOpenShell/src/examples /home/alex/Documents/IfcOpenShell/build /home/alex/Documents/IfcOpenShell/build/examples /home/alex/Documents/IfcOpenShell/build/examples/CMakeFiles/IfcAdvancedHouse.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/IfcAdvancedHouse.dir/depend

