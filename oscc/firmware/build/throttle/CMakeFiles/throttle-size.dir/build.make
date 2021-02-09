# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

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
CMAKE_SOURCE_DIR = /home/pi/Razor/oscc/firmware

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Razor/oscc/firmware/build

# Utility rule file for throttle-size.

# Include the progress variables for this target.
include throttle/CMakeFiles/throttle-size.dir/progress.make

throttle/CMakeFiles/throttle-size: throttle/throttle.elf
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Calculating throttle image size"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/cmake -DFIRMWARE_IMAGE=/home/pi/Razor/oscc/firmware/build/throttle/throttle.elf -DMCU=atmega328p -DEEPROM_IMAGE=/home/pi/Razor/oscc/firmware/build/throttle/throttle.eep -P /home/pi/Razor/oscc/firmware/build/CMakeFiles/FirmwareSize.cmake

throttle-size: throttle/CMakeFiles/throttle-size
throttle-size: throttle/CMakeFiles/throttle-size.dir/build.make

.PHONY : throttle-size

# Rule to build all files generated by this target.
throttle/CMakeFiles/throttle-size.dir/build: throttle-size

.PHONY : throttle/CMakeFiles/throttle-size.dir/build

throttle/CMakeFiles/throttle-size.dir/clean:
	cd /home/pi/Razor/oscc/firmware/build/throttle && $(CMAKE_COMMAND) -P CMakeFiles/throttle-size.dir/cmake_clean.cmake
.PHONY : throttle/CMakeFiles/throttle-size.dir/clean

throttle/CMakeFiles/throttle-size.dir/depend:
	cd /home/pi/Razor/oscc/firmware/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Razor/oscc/firmware /home/pi/Razor/oscc/firmware/throttle /home/pi/Razor/oscc/firmware/build /home/pi/Razor/oscc/firmware/build/throttle /home/pi/Razor/oscc/firmware/build/throttle/CMakeFiles/throttle-size.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : throttle/CMakeFiles/throttle-size.dir/depend

