1. How to compile:
cd build
cmake ..
make

2. How to run:
cd build
./locate_device <deviceType>

<deviceType> = V1 for spigot,
	       V2 for wheel valve,
	       V3 for shuttlecock,
	       A  for breaker box A,
	       B  for breaker box B.

3. Output
The pipeline calculated the 3D coordinate, state, and configuration of each device.
1) The 3D point is stored in 
2) The valve configuration (HORIZONTAL/VERTICAL) is stored in
3) The valve state(OPEN/CLOSED) is stored in 
4) The breaker states (UP/DOWN for each breaker from left to right) is stored in 
5) When nothing is detected, invalid = 1.

For each frame, these values should be ready at the end of the while loop.
Check the invalid variable for validity of this frame.


