# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake

# The command to remove a file.
RM = /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin

# Include any dependencies generated for this target.
include CMakeFiles/main.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/main.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/main.dir/flags.make

CMakeFiles/main.dir/src/main_generated_core.cu.o: CMakeFiles/main.dir/src/main_generated_core.cu.o.depend
CMakeFiles/main.dir/src/main_generated_core.cu.o: CMakeFiles/main.dir/src/main_generated_core.cu.o.cmake
CMakeFiles/main.dir/src/main_generated_core.cu.o: ../src/core.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building NVCC (Device) object CMakeFiles/main.dir/src/main_generated_core.cu.o"
	cd /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src && /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake -E make_directory /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/.
	cd /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src && /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake -D verbose:BOOL=$(VERBOSE) -D build_configuration:STRING= -D generated_file:STRING=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/./main_generated_core.cu.o -D generated_cubin_file:STRING=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/./main_generated_core.cu.o.cubin.txt -P /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/main_generated_core.cu.o.cmake

CMakeFiles/main.dir/src/main_generated_main.cu.o: CMakeFiles/main.dir/src/main_generated_main.cu.o.depend
CMakeFiles/main.dir/src/main_generated_main.cu.o: CMakeFiles/main.dir/src/main_generated_main.cu.o.cmake
CMakeFiles/main.dir/src/main_generated_main.cu.o: ../src/main.cu
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building NVCC (Device) object CMakeFiles/main.dir/src/main_generated_main.cu.o"
	cd /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src && /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake -E make_directory /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/.
	cd /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src && /GPUFS/app_GPU/application/cmake/3.14.3/bin/cmake -D verbose:BOOL=$(VERBOSE) -D build_configuration:STRING= -D generated_file:STRING=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/./main_generated_main.cu.o -D generated_cubin_file:STRING=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/./main_generated_main.cu.o.cubin.txt -P /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/main_generated_main.cu.o.cmake

CMakeFiles/main.dir/src/samples.cpp.o: CMakeFiles/main.dir/flags.make
CMakeFiles/main.dir/src/samples.cpp.o: ../src/samples.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/main.dir/src/samples.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/main.dir/src/samples.cpp.o -c /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/src/samples.cpp

CMakeFiles/main.dir/src/samples.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/main.dir/src/samples.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/src/samples.cpp > CMakeFiles/main.dir/src/samples.cpp.i

CMakeFiles/main.dir/src/samples.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/main.dir/src/samples.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/src/samples.cpp -o CMakeFiles/main.dir/src/samples.cpp.s

# Object files for target main
main_OBJECTS = \
"CMakeFiles/main.dir/src/samples.cpp.o"

# External object files for target main
main_EXTERNAL_OBJECTS = \
"/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/main_generated_core.cu.o" \
"/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/src/main_generated_main.cu.o"

main: CMakeFiles/main.dir/src/samples.cpp.o
main: CMakeFiles/main.dir/src/main_generated_core.cu.o
main: CMakeFiles/main.dir/src/main_generated_main.cu.o
main: CMakeFiles/main.dir/build.make
main: /usr/local/cuda/lib64/libcudart_static.a
main: /usr/lib64/librt.so
main: CMakeFiles/main.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable main"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/main.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/main.dir/build: main

.PHONY : CMakeFiles/main.dir/build

CMakeFiles/main.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/main.dir/cmake_clean.cmake
.PHONY : CMakeFiles/main.dir/clean

CMakeFiles/main.dir/depend: CMakeFiles/main.dir/src/main_generated_core.cu.o
CMakeFiles/main.dir/depend: CMakeFiles/main.dir/src/main_generated_main.cu.o
	cd /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin /GPUFS/sysu_hpcedu_302/luowle/tem/CUDA-learn/Calculate_2D_entropy/sources/bin/CMakeFiles/main.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/main.dir/depend

