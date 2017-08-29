#!/bin/bash

cd ../external
if [ -d eigen ]; then
	cd eigen
	git pull	
	cd ..	
else
	git clone https://github.com/RLovelett/eigen.git	
fi

if [ -d pybind11 ]; then
	cd pybind11
	git pull
	cd ..
else
	git clone https://github.com/pybind/pybind11.git
fi

if [ -d Catch ]; then
	cd Catch
	git pull
	cd ..
else
	git clone https://github.com/philsquared/Catch.git
fi

cd ../scripts
