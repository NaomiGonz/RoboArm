# Makefile for 3‑D RRT implementation
# -----------------------------------------------------

CXX       := g++
CXX_ARM   := g++
CXXFLAGS  := -I.. -std=c++17 -O2 -I./forward_kinematics

# -----------------------------------------------------
# Native build (default)
# -----------------------------------------------------
rrt_3d: rrt_3d.o rrt_base.o fk.o main.o
	$(CXX) $(CXXFLAGS) -o rrt_3d rrt_3d.o rrt_base.o fk.o main.o

main.o: main.cpp rrt_3d.h
	$(CXX) $(CXXFLAGS) -c main.cpp -o main.o

rrt_3d.o: rrt_3d.cpp rrt_3d.h ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c rrt_3d.cpp -o rrt_3d.o

rrt_base.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c ../rrt_base.cpp -o rrt_base.o

fk.o: ./forward_kinematics/forward_kinematics.cpp ./forward_kinematics/forward_kinematics_roarm_double.h
	$(CXX) $(CXXFLAGS) -c ./forward_kinematics/forward_kinematics.cpp -o fk.o 

# -----------------------------------------------------
# ARM cross‑compile build
# -----------------------------------------------------
rrt_3d_ARM: rrt_3d_ARM.o rrt_base_ARM.o main_ARM.o
	$(CXX_ARM) $(CXXFLAGS) -static -o rrt_3d_ARM rrt_3d_ARM.o rrt_base_ARM.o main_ARM.o

# Compile main_ARM.o using the ARM cross-compiler
main_ARM.o: BB_main.cpp rrt_3d.h
	$(CXX_ARM) $(CXXFLAGS) -c BB_main.cpp -o main_ARM.o

rrt_3d_ARM.o: rrt_3d.cpp rrt_3d.h ../rrt_base.hpp ./forward_kinematics/forward_kinematics_roarm_double.h
	$(CXX_ARM) $(CXXFLAGS) -c rrt_3d.cpp -o rrt_3d_ARM.o

rrt_base_ARM.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX_ARM) $(CXXFLAGS) -c ../rrt_base.cpp -o rrt_base_ARM.o

# fk_ARM.o: ./forward_kinematics/forward_kinematics.cpp ./forward_kinematics/forward_kinematics_roarm_double.h
# 	$(CXX) $(CXXFLAGS) -static -c ./forward_kinematics/forward_kinematics.cpp -o fk_ARM.o 

# -----------------------------------------------------
# Clean up everything
# -----------------------------------------------------
clean:
	rm -f *.o rrt_3d rrt_3d_ARM