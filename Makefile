#
# Makefile for color SOM
# 

###############################################################################
# MAKEFILE VARIABLES
###############################################################################

# CXX is the name of the compiler we are using (clang++)

CXX = clang++


# CXXFLAGS are the flags we will be passing each compile

CXXFLAGS       = -std=c++11 -O3 -Wall -Wextra -pedantic -pthread 


# TARGETS is the list of all programs created when we do "make all"
#   (and which should be deleted when we do "make clean")

TARGETS = color_som

###############################################################################
# "PHONY" (BUT USEFUL) MAKEFILE TARGETS
###############################################################################


# "make all" brings all programs up-to-date (recursively)
#     and then runs no commands.

all: $(TARGETS)

# "make clean" brings nothing up to date, but always
#      runs commands to delete all created files

clean:
	rm -f $(TARGETS)
	rm -rf *.o

###############################################################################
# CREATING PROGRAMS
###############################################################################

color_som: color_som.cpp SelfOrganizingMap.o
	$(CXX) $(CXXFLAGS) -lpng16 SelfOrganizingMap.o -o color_som color_som.cpp

###############################################################################
# GENERATING OBJECT FILES
###############################################################################

# The .o files depend on the corresponding .cpp file and the .hpp
#      files that C++ code includes.
#      In each case, the command to generate the .o file uses
#      our C++ compiler to compile the .cpp file, with the -c flag.

SelfOrganizingMap.o: SelfOrganizingMap.cpp SelfOrganizingMap.hpp LoadingBar.hpp

# When/if you need more .o files, their dependencies and the
#    compilation commands would go here.


