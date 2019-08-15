#!/bin/sh

# Install CppAD
git clone https://github.com/coin-or/CppAD.git cppad
cd cppad
mkdir build
cd build/
cmake ../
sudo make install
cd ../../
rm -rf cppad/