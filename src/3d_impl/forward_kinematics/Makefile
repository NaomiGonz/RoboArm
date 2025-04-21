# Makefile for Forward Kinematics Testing Script 
# -----------------------------------------------------
# Author: Naomi

CXX       := g++
CXXFLAGS  := -std=c++17

fk_test: test.o fk.o
	$(CXX) $(CXXFLAGS) -o fk_test test.o fk.o

test.o: fk_test.cpp forward_kinematics_roarm_double.h
	$(CXX) $(CXXFLAGS) -c fk_test.cpp -o test.o

fk.o: forward_kinematics.cpp forward_kinematics_roarm_double.h
	$(CXX) $(CXXFLAGS) -c forward_kinematics.cpp -o fk.o 

clean:
	rm -f *.o fk_test

