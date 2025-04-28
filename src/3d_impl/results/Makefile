# Makefile for plotting_csv.cpp + forward kinematics

CXX = g++
CXXFLAGS = -O2 -std=c++17 -I../forward_kinematics
TARGET = plotting_csv
SRC = plotting_csv.cpp ../forward_kinematics/forward_kinematics.cpp

all: $(TARGET)

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
