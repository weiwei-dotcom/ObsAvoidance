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
CMAKE_SOURCE_DIR = /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/weiwei/Desktop/project/ObsAvoidance/build/orbslam

# Include any dependencies generated for this target.
include CMakeFiles/orbslam.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/orbslam.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/orbslam.dir/flags.make

CMakeFiles/orbslam.dir/src/main.cpp.o: CMakeFiles/orbslam.dir/flags.make
CMakeFiles/orbslam.dir/src/main.cpp.o: /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weiwei/Desktop/project/ObsAvoidance/build/orbslam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/orbslam.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/orbslam.dir/src/main.cpp.o -c /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/main.cpp

CMakeFiles/orbslam.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orbslam.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/main.cpp > CMakeFiles/orbslam.dir/src/main.cpp.i

CMakeFiles/orbslam.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orbslam.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/main.cpp -o CMakeFiles/orbslam.dir/src/main.cpp.s

CMakeFiles/orbslam.dir/src/slam.cpp.o: CMakeFiles/orbslam.dir/flags.make
CMakeFiles/orbslam.dir/src/slam.cpp.o: /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/slam.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/weiwei/Desktop/project/ObsAvoidance/build/orbslam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/orbslam.dir/src/slam.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/orbslam.dir/src/slam.cpp.o -c /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/slam.cpp

CMakeFiles/orbslam.dir/src/slam.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orbslam.dir/src/slam.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/slam.cpp > CMakeFiles/orbslam.dir/src/slam.cpp.i

CMakeFiles/orbslam.dir/src/slam.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orbslam.dir/src/slam.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam/src/slam.cpp -o CMakeFiles/orbslam.dir/src/slam.cpp.s

# Object files for target orbslam
orbslam_OBJECTS = \
"CMakeFiles/orbslam.dir/src/main.cpp.o" \
"CMakeFiles/orbslam.dir/src/slam.cpp.o"

# External object files for target orbslam
orbslam_EXTERNAL_OBJECTS =

orbslam: CMakeFiles/orbslam.dir/src/main.cpp.o
orbslam: CMakeFiles/orbslam.dir/src/slam.cpp.o
orbslam: CMakeFiles/orbslam.dir/build.make
orbslam: /home/weiwei/Desktop/project/ObsAvoidance/install/interface/lib/libinterface__rosidl_typesupport_introspection_c.so
orbslam: /home/weiwei/Desktop/project/ObsAvoidance/install/interface/lib/libinterface__rosidl_typesupport_c.so
orbslam: /home/weiwei/Desktop/project/ObsAvoidance/install/interface/lib/libinterface__rosidl_typesupport_introspection_cpp.so
orbslam: /home/weiwei/Desktop/project/ObsAvoidance/install/interface/lib/libinterface__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libmessage_filters.so
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
orbslam: /opt/ros/foxy/lib/libcv_bridge.so
orbslam: /usr/lib/libORB_SLAM3.so
orbslam: /home/weiwei/Desktop/thirdParty/ORB_SLAM3/Thirdparty/DBoW2/lib/libDBoW2.so
orbslam: /home/weiwei/Desktop/thirdParty/ORB_SLAM3/Thirdparty/g2o/lib/libg2o.so
orbslam: /usr/local/lib/libpango_glgeometry.so
orbslam: /usr/local/lib/libpango_python.so
orbslam: /usr/local/lib/libpango_scene.so
orbslam: /usr/local/lib/libpango_tools.so
orbslam: /usr/local/lib/libpango_video.so
orbslam: /opt/ros/foxy/lib/libmessage_filters.so
orbslam: /opt/ros/foxy/lib/librclcpp.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librcutils.so
orbslam: /opt/ros/foxy/lib/librcpputils.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librosidl_runtime_c.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libpcl_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librclcpp.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librcl.so
orbslam: /opt/ros/foxy/lib/librcl_yaml_param_parser.so
orbslam: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libtracetools.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
orbslam: /home/weiwei/Desktop/project/ObsAvoidance/install/interface/lib/libinterface__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/liblibstatistics_collector_test_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/librcl_interfaces__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/librcl_interfaces__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librmw_implementation.so
orbslam: /opt/ros/foxy/lib/librmw.so
orbslam: /opt/ros/foxy/lib/librcl_logging_spdlog.so
orbslam: /usr/lib/x86_64-linux-gnu/libspdlog.so.1.5.0
orbslam: /opt/ros/foxy/lib/libyaml.so
orbslam: /opt/ros/foxy/lib/librosgraph_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libstatistics_msgs__rosidl_generator_c.so
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
orbslam: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
orbslam: /usr/local/lib/libpango_geometry.so
orbslam: /usr/local/lib/libtinyobj.so
orbslam: /usr/local/lib/libpango_plot.so
orbslam: /usr/local/lib/libpango_display.so
orbslam: /usr/local/lib/libpango_vars.so
orbslam: /usr/local/lib/libpango_windowing.so
orbslam: /usr/local/lib/libpango_opengl.so
orbslam: /usr/lib/x86_64-linux-gnu/libGLEW.so
orbslam: /usr/lib/x86_64-linux-gnu/libOpenGL.so
orbslam: /usr/lib/x86_64-linux-gnu/libGLX.so
orbslam: /usr/lib/x86_64-linux-gnu/libGLU.so
orbslam: /usr/local/lib/libpango_image.so
orbslam: /usr/local/lib/libpango_packetstream.so
orbslam: /usr/local/lib/libpango_core.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libsensor_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libgeometry_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/libstd_msgs__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_generator_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_introspection_cpp.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_introspection_c.so
orbslam: /opt/ros/foxy/lib/libbuiltin_interfaces__rosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_cpp.so
orbslam: /opt/ros/foxy/lib/librosidl_typesupport_c.so
orbslam: /opt/ros/foxy/lib/librcpputils.so
orbslam: /opt/ros/foxy/lib/librosidl_runtime_c.so
orbslam: /opt/ros/foxy/lib/librcutils.so
orbslam: CMakeFiles/orbslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/weiwei/Desktop/project/ObsAvoidance/build/orbslam/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable orbslam"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/orbslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/orbslam.dir/build: orbslam

.PHONY : CMakeFiles/orbslam.dir/build

CMakeFiles/orbslam.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/orbslam.dir/cmake_clean.cmake
.PHONY : CMakeFiles/orbslam.dir/clean

CMakeFiles/orbslam.dir/depend:
	cd /home/weiwei/Desktop/project/ObsAvoidance/build/orbslam && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam /home/weiwei/Desktop/project/ObsAvoidance/src/orbslam /home/weiwei/Desktop/project/ObsAvoidance/build/orbslam /home/weiwei/Desktop/project/ObsAvoidance/build/orbslam /home/weiwei/Desktop/project/ObsAvoidance/build/orbslam/CMakeFiles/orbslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/orbslam.dir/depend
