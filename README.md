# Astar_Code_Sample
This project is to use A* searching algorithm to do path planning in 2D grid map. The agent is only allowed to move vertically or horizontally.

# Source Files:
main.cpp --> read map file and run Astar 

Astar.h --> declare the variable and objects used for Astar.cpp

Astar.cpp --> define several functions for the A* algorithm

map.txt --> 2-D map, where 0 is the available vacant space and 1 represent for obstacles. 

# How to run this program:

1. git clone this repository

2. Jump to the src folder

3. Open main.cpp and modify line 17 into your absolute address of the map.txt. Save the chages.

4. Jump to the build file

5. Run cmake .. 

6. Run make and now you should see an exe file named Astar in the build folder

7. Run ./Astar and check the output result.

