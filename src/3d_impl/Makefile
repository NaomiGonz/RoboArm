# Makefile for 3‑D RRT implementation
# -----------------------------------------------------

CXX       := g++
CXX_ARM   := arm-linux-gnueabihf-g++
CXXFLAGS  := -I.. -std=c++14 -O2

# -----------------------------------------------------
# Native build (default)
# -----------------------------------------------------
rrt_3d: rrt_3d.o rrt_base.o fk.o
	$(CXX) $(CXXFLAGS) -o rrt_3d rrt_3d.o rrt_base.o fk.o

rrt_3d.o: rrt_3d.cpp rrt_3d.h ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c rrt_3d.cpp -o rrt_2d.o

rrt_base.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c ../rrt_base.cpp -o rrt_base.o

fk.o: forward_kinematics.cpp forward_kinematics_roarm_double.h
	$(CXX) $(CXXFLAGS) -c forward_kinematics.cpp -o fk.o 

# -----------------------------------------------------
# ARM cross‑compile build
# -----------------------------------------------------
rrt_3d_ARM: rrt_3d_ARM.o rrt_base_ARM.o
	$(CXX_ARM) $(CXXFLAGS) -static -o rrt_2d_ARM rrt_2d_ARM.o rrt_base_ARM.o fk_ARM.o 

rrt_3d_ARM.o: rrt_3d.cpp rrt_3d.h ../rrt_base.hpp
	$(CXX_ARM) $(CXXFLAGS) -static -c rrt_3d.cpp -o rrt_2d_ARM.o

rrt_base_ARM.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX_ARM) $(CXXFLAGS) -static -c ../rrt_base.cpp -o rrt_base_ARM.o

fk_ARM.o: forward_kinematics.cpp forward_kinematics_roarm_double.h
	$(CXX) $(CXXFLAGS) -static -c forward_kinematics.cpp -o fk_ARM.o 

# -----------------------------------------------------
# Clean up everything
# -----------------------------------------------------
clean:
	rm -f *.o rrt_2d rrt_2d_ARM