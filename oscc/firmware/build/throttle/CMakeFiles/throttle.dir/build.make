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

# Include any dependencies generated for this target.
include throttle/CMakeFiles/throttle.dir/depend.make

# Include the progress variables for this target.
include throttle/CMakeFiles/throttle.dir/progress.make

# Include the compile flags for this target's objects.
include throttle/CMakeFiles/throttle.dir/flags.make

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj: ../common/libs/arduino_init/arduino_init.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/arduino_init/arduino_init.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/arduino_init/arduino_init.cpp > CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/arduino_init/arduino_init.cpp -o CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj: ../common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp > CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp -o CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj: ../common/libs/mcp_can/mcp_can.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/mcp_can/mcp_can.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/mcp_can/mcp_can.cpp > CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/mcp_can/mcp_can.cpp -o CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj: ../common/libs/serial/oscc_serial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/serial/oscc_serial.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/serial/oscc_serial.cpp > CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/serial/oscc_serial.cpp -o CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj: ../common/libs/can/oscc_can.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/can/oscc_can.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/can/oscc_can.cpp > CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/can/oscc_can.cpp -o CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj: ../common/libs/dac/oscc_dac.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/dac/oscc_dac.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/dac/oscc_dac.cpp > CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/dac/oscc_dac.cpp -o CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj


throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj: ../common/libs/timer/oscc_timer.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj -c /home/pi/Razor/oscc/firmware/common/libs/timer/oscc_timer.cpp

throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/common/libs/timer/oscc_timer.cpp > CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.i

throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/common/libs/timer/oscc_timer.cpp -o CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.s

throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj


throttle/CMakeFiles/throttle.dir/src/main.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/main.cpp.obj: ../throttle/src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object throttle/CMakeFiles/throttle.dir/src/main.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/main.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/main.cpp

throttle/CMakeFiles/throttle.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/main.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/main.cpp > CMakeFiles/throttle.dir/src/main.cpp.i

throttle/CMakeFiles/throttle.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/main.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/main.cpp -o CMakeFiles/throttle.dir/src/main.cpp.s

throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/main.cpp.obj


throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj: ../throttle/src/globals.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/globals.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/globals.cpp

throttle/CMakeFiles/throttle.dir/src/globals.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/globals.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/globals.cpp > CMakeFiles/throttle.dir/src/globals.cpp.i

throttle/CMakeFiles/throttle.dir/src/globals.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/globals.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/globals.cpp -o CMakeFiles/throttle.dir/src/globals.cpp.s

throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj


throttle/CMakeFiles/throttle.dir/src/init.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/init.cpp.obj: ../throttle/src/init.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object throttle/CMakeFiles/throttle.dir/src/init.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/init.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/init.cpp

throttle/CMakeFiles/throttle.dir/src/init.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/init.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/init.cpp > CMakeFiles/throttle.dir/src/init.cpp.i

throttle/CMakeFiles/throttle.dir/src/init.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/init.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/init.cpp -o CMakeFiles/throttle.dir/src/init.cpp.s

throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/init.cpp.obj


throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj: ../throttle/src/communications.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/communications.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/communications.cpp

throttle/CMakeFiles/throttle.dir/src/communications.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/communications.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/communications.cpp > CMakeFiles/throttle.dir/src/communications.cpp.i

throttle/CMakeFiles/throttle.dir/src/communications.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/communications.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/communications.cpp -o CMakeFiles/throttle.dir/src/communications.cpp.s

throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj


throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj: ../throttle/src/throttle_control.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/throttle_control.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/throttle_control.cpp

throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/throttle_control.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/throttle_control.cpp > CMakeFiles/throttle.dir/src/throttle_control.cpp.i

throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/throttle_control.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/throttle_control.cpp -o CMakeFiles/throttle.dir/src/throttle_control.cpp.s

throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj


throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj: throttle/CMakeFiles/throttle.dir/flags.make
throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj: ../throttle/src/timers.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/throttle.dir/src/timers.cpp.obj -c /home/pi/Razor/oscc/firmware/throttle/src/timers.cpp

throttle/CMakeFiles/throttle.dir/src/timers.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/throttle.dir/src/timers.cpp.i"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Razor/oscc/firmware/throttle/src/timers.cpp > CMakeFiles/throttle.dir/src/timers.cpp.i

throttle/CMakeFiles/throttle.dir/src/timers.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/throttle.dir/src/timers.cpp.s"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Razor/oscc/firmware/throttle/src/timers.cpp -o CMakeFiles/throttle.dir/src/timers.cpp.s

throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.requires:

.PHONY : throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.requires

throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.provides: throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.requires
	$(MAKE) -f throttle/CMakeFiles/throttle.dir/build.make throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.provides.build
.PHONY : throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.provides

throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.provides.build: throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj


# Object files for target throttle
throttle_OBJECTS = \
"CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj" \
"CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj" \
"CMakeFiles/throttle.dir/src/main.cpp.obj" \
"CMakeFiles/throttle.dir/src/globals.cpp.obj" \
"CMakeFiles/throttle.dir/src/init.cpp.obj" \
"CMakeFiles/throttle.dir/src/communications.cpp.obj" \
"CMakeFiles/throttle.dir/src/throttle_control.cpp.obj" \
"CMakeFiles/throttle.dir/src/timers.cpp.obj"

# External object files for target throttle
throttle_EXTERNAL_OBJECTS =

throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/main.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/init.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/build.make
throttle/throttle.elf: can_gateway/libuno_SPI.a
throttle/throttle.elf: can_gateway/libuno_CORE.a
throttle/throttle.elf: throttle/CMakeFiles/throttle.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Razor/oscc/firmware/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Linking CXX executable throttle.elf"
	cd /home/pi/Razor/oscc/firmware/build/throttle && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/throttle.dir/link.txt --verbose=$(VERBOSE)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating EEP image"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 /home/pi/Razor/oscc/firmware/build/throttle/throttle.elf /home/pi/Razor/oscc/firmware/build/throttle/throttle.eep
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating HEX image"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/avr-objcopy -O ihex -R .eeprom /home/pi/Razor/oscc/firmware/build/throttle/throttle.elf /home/pi/Razor/oscc/firmware/build/throttle/throttle.hex
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Calculating image size"
	cd /home/pi/Razor/oscc/firmware/build/throttle && /usr/bin/cmake -DFIRMWARE_IMAGE=/home/pi/Razor/oscc/firmware/build/throttle/throttle.elf -DMCU=atmega328p -DEEPROM_IMAGE=/home/pi/Razor/oscc/firmware/build/throttle/throttle.eep -P /home/pi/Razor/oscc/firmware/build/CMakeFiles/FirmwareSize.cmake

# Rule to build all files generated by this target.
throttle/CMakeFiles/throttle.dir/build: throttle/throttle.elf

.PHONY : throttle/CMakeFiles/throttle.dir/build

throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/arduino_init/arduino_init.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/DAC_MCP49xx/DAC_MCP49xx.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/mcp_can/mcp_can.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/serial/oscc_serial.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/can/oscc_can.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/dac/oscc_dac.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/__/common/libs/timer/oscc_timer.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/main.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/globals.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/init.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/communications.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/throttle_control.cpp.obj.requires
throttle/CMakeFiles/throttle.dir/requires: throttle/CMakeFiles/throttle.dir/src/timers.cpp.obj.requires

.PHONY : throttle/CMakeFiles/throttle.dir/requires

throttle/CMakeFiles/throttle.dir/clean:
	cd /home/pi/Razor/oscc/firmware/build/throttle && $(CMAKE_COMMAND) -P CMakeFiles/throttle.dir/cmake_clean.cmake
.PHONY : throttle/CMakeFiles/throttle.dir/clean

throttle/CMakeFiles/throttle.dir/depend:
	cd /home/pi/Razor/oscc/firmware/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Razor/oscc/firmware /home/pi/Razor/oscc/firmware/throttle /home/pi/Razor/oscc/firmware/build /home/pi/Razor/oscc/firmware/build/throttle /home/pi/Razor/oscc/firmware/build/throttle/CMakeFiles/throttle.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : throttle/CMakeFiles/throttle.dir/depend

