# Makefile for C++ Serial Communication App
# -----------------------------------------------------

CXX       := g++
CXXFLAGS  := -std=c++17 -Wall -O2

LDFLAGS   := -lboost_system -lboost_program_options -pthread
TARGET    := serial_app

SOURCES   := serial_app.cpp
OBJECTS   := $(SOURCES:.cpp=.o)

# -----------------------------------------------------
# Default build rule (native)
# -----------------------------------------------------
all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS) $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# -----------------------------------------------------
# Clean up everything
# -----------------------------------------------------
clean:
	rm -f *.o $(TARGET)

