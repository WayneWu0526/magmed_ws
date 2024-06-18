g++ -I../../include -L../../lib -L/usr/lib/x86_64-linux-gnu -o linear_actuator linear_actuator.cpp -lch9326 -lusb -lpthread
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:../../lib
./linear_actuator