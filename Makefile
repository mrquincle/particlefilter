#!/bin/make

# Author: Anne C. van Rossum
# Date: Apr. 16, 2012

all: 
	@mkdir -p build
	cd build && cmake $(CMAKE_FLAGS) .. $(FLAGS)
	cd build && make

clean:
	cd build && make clean

distclean: clean

.PHONY:	all clean distclean
