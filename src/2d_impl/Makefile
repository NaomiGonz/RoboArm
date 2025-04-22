# Makefile for 2‑D RRT implementation
# -----------------------------------------------------

CXX       := g++
CXX_ARM   := arm-linux-gnueabihf-g++
CXXFLAGS  := -I.. -std=c++14 -O2

# -----------------------------------------------------
# Native build (default)
# -----------------------------------------------------
rrt_2d: rrt_2d.o rrt_base.o
	$(CXX) $(CXXFLAGS) -o rrt_2d rrt_2d.o rrt_base.o

rrt_2d.o: rrt_2d.cpp rrt_2d.h ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c rrt_2d.cpp -o rrt_2d.o

rrt_base.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX) $(CXXFLAGS) -c ../rrt_base.cpp -o rrt_base.o

# -----------------------------------------------------
# ARM cross‑compile build
# -----------------------------------------------------
rrt_2d_ARM: rrt_2d_ARM.o rrt_base_ARM.o
	$(CXX_ARM) $(CXXFLAGS) -static -o rrt_2d_ARM rrt_2d_ARM.o rrt_base_ARM.o

rrt_2d_ARM.o: rrt_2d.cpp rrt_2d.h ../rrt_base.hpp
	$(CXX_ARM) $(CXXFLAGS) -static -c rrt_2d.cpp -o rrt_2d_ARM.o

rrt_base_ARM.o: ../rrt_base.cpp ../rrt_base.hpp
	$(CXX_ARM) $(CXXFLAGS) -static -c ../rrt_base.cpp -o rrt_base_ARM.o

# -----------------------------------------------------
# Clean up everything
# -----------------------------------------------------
clean:
	rm -f *.o rrt_2d rrt_2d_ARM
