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
include examples/CMakeFiles/IfcParseExamples.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/IfcParseExamples.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/IfcParseExamples.dir/flags.make

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o: examples/CMakeFiles/IfcParseExamples.dir/flags.make
examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o: /home/alex/Documents/IfcOpenShell/src/examples/IfcParseExamples.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/alex/Documents/IfcOpenShell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o -c /home/alex/Documents/IfcOpenShell/src/examples/IfcParseExamples.cpp

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.i"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/alex/Documents/IfcOpenShell/src/examples/IfcParseExamples.cpp > CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.i

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.s"
	cd /home/alex/Documents/IfcOpenShell/build/examples && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/alex/Documents/IfcOpenShell/src/examples/IfcParseExamples.cpp -o CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.s

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.requires:

.PHONY : examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.requires

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.provides: examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/IfcParseExamples.dir/build.make examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.provides.build
.PHONY : examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.provides

examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.provides.build: examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o


# Object files for target IfcParseExamples
IfcParseExamples_OBJECTS = \
"CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o"

# External object files for target IfcParseExamples
IfcParseExamples_EXTERNAL_OBJECTS =

examples/IfcParseExamples: examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o
examples/IfcParseExamples: examples/CMakeFiles/IfcParseExamples.dir/build.make
examples/IfcParseExamples: libIfcParse.a
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libicuuc.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libicudata.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libdl.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libicui18n.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_system.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_regex.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_thread.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
examples/IfcParseExamples: /usr/lib/x86_64-linux-gnu/libpthread.so
examples/IfcParseExamples: examples/CMakeFiles/IfcParseExamples.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/alex/Documents/IfcOpenShell/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable IfcParseExamples"
	cd /home/alex/Documents/IfcOpenShell/build/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/IfcParseExamples.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/IfcParseExamples.dir/build: examples/IfcParseExamples

.PHONY : examples/CMakeFiles/IfcParseExamples.dir/build

examples/CMakeFiles/IfcParseExamples.dir/requires: examples/CMakeFiles/IfcParseExamples.dir/IfcParseExamples.cpp.o.requires

.PHONY : examples/CMakeFiles/IfcParseExamples.dir/requires

examples/CMakeFiles/IfcParseExamples.dir/clean:
	cd /home/alex/Documents/IfcOpenShell/build/examples && $(CMAKE_COMMAND) -P CMakeFiles/IfcParseExamples.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/IfcParseExamples.dir/clean

examples/CMakeFiles/IfcParseExamples.dir/depend:
	cd /home/alex/Documents/IfcOpenShell/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/alex/Documents/IfcOpenShell/cmake /home/alex/Documents/IfcOpenShell/src/examples /home/alex/Documents/IfcOpenShell/build /home/alex/Documents/IfcOpenShell/build/examples /home/alex/Documents/IfcOpenShell/build/examples/CMakeFiles/IfcParseExamples.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/IfcParseExamples.dir/depend

