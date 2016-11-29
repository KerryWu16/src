# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/ubuntu/catkin_ws/src/aruco-1.2.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ubuntu/catkin_ws/src/aruco-1.2.4/build

# Include any dependencies generated for this target.
include utils/CMakeFiles/aruco_create_marker.dir/depend.make

# Include the progress variables for this target.
include utils/CMakeFiles/aruco_create_marker.dir/progress.make

# Include the compile flags for this target's objects.
include utils/CMakeFiles/aruco_create_marker.dir/flags.make

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o: utils/CMakeFiles/aruco_create_marker.dir/flags.make
utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o: ../utils/aruco_create_marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o"
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o -c /home/ubuntu/catkin_ws/src/aruco-1.2.4/utils/aruco_create_marker.cpp

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i"
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/ubuntu/catkin_ws/src/aruco-1.2.4/utils/aruco_create_marker.cpp > CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.i

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s"
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/ubuntu/catkin_ws/src/aruco-1.2.4/utils/aruco_create_marker.cpp -o CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.s

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires:
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires
	$(MAKE) -f utils/CMakeFiles/aruco_create_marker.dir/build.make utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides.build
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides

utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.provides.build: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o

# Object files for target aruco_create_marker
aruco_create_marker_OBJECTS = \
"CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o"

# External object files for target aruco_create_marker
aruco_create_marker_EXTERNAL_OBJECTS =

utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o
utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/build.make
utils/aruco_create_marker: src/libaruco.so.1.2.4
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_videostab.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_ts.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_superres.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_stitching.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_ocl.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_gpu.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_contrib.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_photo.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_legacy.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_video.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_objdetect.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_ml.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_calib3d.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_features2d.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_highgui.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_imgproc.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_flann.so.2.4.8
utils/aruco_create_marker: /usr/lib/arm-linux-gnueabihf/libopencv_core.so.2.4.8
utils/aruco_create_marker: utils/CMakeFiles/aruco_create_marker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable aruco_create_marker"
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/aruco_create_marker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utils/CMakeFiles/aruco_create_marker.dir/build: utils/aruco_create_marker
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/build

utils/CMakeFiles/aruco_create_marker.dir/requires: utils/CMakeFiles/aruco_create_marker.dir/aruco_create_marker.cpp.o.requires
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/requires

utils/CMakeFiles/aruco_create_marker.dir/clean:
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils && $(CMAKE_COMMAND) -P CMakeFiles/aruco_create_marker.dir/cmake_clean.cmake
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/clean

utils/CMakeFiles/aruco_create_marker.dir/depend:
	cd /home/ubuntu/catkin_ws/src/aruco-1.2.4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ubuntu/catkin_ws/src/aruco-1.2.4 /home/ubuntu/catkin_ws/src/aruco-1.2.4/utils /home/ubuntu/catkin_ws/src/aruco-1.2.4/build /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils /home/ubuntu/catkin_ws/src/aruco-1.2.4/build/utils/CMakeFiles/aruco_create_marker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utils/CMakeFiles/aruco_create_marker.dir/depend

