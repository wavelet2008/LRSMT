#!/bin/bash
wget https://github.com/QianLabUSC/LRSMT/releases/download/Dependencies/qpOASES-3.2.1.zip
unzip qpOASES-3.2.1.zip
cd qpOASES-3.2.1
mkdir build
cd build
cmake ..
sudo make 
sudo make install

