The pipeline is in locate_device.cpp.

1. How to compile:
cd build
cmake ..
make

2. How to run:
cd build
./locate_device <deviceType> <displayImage>

<deviceType> = V1 for spigot valve,
	           V2 for wheel valve,
	           V3 for shuttlecock valve,
	           A  for breaker box A,
	           B  for breaker box B.

<displayImage> = 1 if want to imshow stuff for debugging/visualization
               = 0 if turn off imshow

3. Output
The pipeline calculated the 3D coordinate, state, and configuration of each device.

0) The state/configuration constants are defined in util.hpp
1) The 3D point for valve is stored in valve_coord
2) The valve configuration (HORIZONTAL/VERTICAL) is stored in device_config
3) The shuttlecock state(OPEN/CLOSED) is stored in device_state
4) The breaker states (UP/DOWN for each breaker from left to right) is stored in the array breaker_state[]
5) The breaker 3D points are stored in the vector breaker_coords
7) When nothing is detected, error = ERROR_BAD_FRAME, pipeline continues.
7) On error, error > 0, pipeline exits. Else error = 0 or ERROR_BAD_FRAME.

For each frame, these values should be ready at the end of the while loop.
Check the error code for validity of this frame.

4. Some ideas
The pipeline might give out different state/config/3d coordinate for a device using different frames, even if the realsense is not moving. This is because of background noise, sensor noise, etc. For this reason, to determine device position and states, we can do a majority vote / averaging among the results coming from several frames.


