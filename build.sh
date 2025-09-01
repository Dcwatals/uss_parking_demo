rm -rf build
mkdir build
cd build
cmake ..
make 
# xvfb-run ./uss_demo
./uss_demo