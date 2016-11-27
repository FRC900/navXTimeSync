#include <iostream>
#include <TimeStamp.h>
#include <stdio.h>
#include <stdlib.h>
#include <AHRS.h>
#include <chrono>
#include <thread>
#include <iomanip>

int main(int argc, char *argv[]) {
    std::cout << "Program Executing\n";

    AHRS com = AHRS("/dev/ttyACM0");

    printf("Initializing\n");

   

    printf("Probably Initialized. %d\n\n", com.IsCalibrating());


    std::cout << "Pitch  |  Roll  |  Yaw  |  X-Accel  | Y-Accel  |  Z-Accel  |  Time  |" << std::endl;

    while( 1 == 1){
    std::cout << std::fixed << std::setprecision(2) << com.GetPitch() << "      " << com.GetRoll() << "   " << com.GetYaw() << "     " <<com.GetWorldLinearAccelX() << "     " << com.GetWorldLinearAccelY() << "       " << com.GetWorldLinearAccelZ() << "      " << com.GetLastSensorTimestamp() << "      " << '\r' << std::flush;
    std::this_thread::sleep_for(std::chrono::milliseconds(125));
    }
    printf("Program Execution Complete.\n");

    return 0;
}
