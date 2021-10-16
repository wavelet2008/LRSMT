#!/bin/bash
wget #!/bin/bash
wget https://github.com/QianLabUSC/LRSMT/releases/download/Dependencies/eigen-3.3.2.zip
unzip eigen-3.3.2.zip
cd eigen-3.3.2
mkdir build
cd build
cmake ..
sudo make
sudo make install

