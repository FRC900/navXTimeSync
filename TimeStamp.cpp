#include <iostream>
#include <TimeStamp.h>
#include <stdio.h>
#include <stdlib.h>
#include <AHRS.h>
#include <chrono>
#include <thread>

int main(int argc, char *argv[]) {
    std::cout << "Program Executing\n";

    AHRS com = AHRS("/dev/ttyACM0");

    printf("Initializing\n");

    while(1 == 1) std::this_thread::sleep_for(std::chrono::seconds(8));

    printf("Probably Initialized. %d\n\n", com.IsCalibrating());


    std::cout << "Firmware Version: " << com.GetFirmwareVersion() << std::endl;
    printf("Pitch: %f\n", com.GetPitch());
    printf("Roll: %f\n", com.GetRoll());
    printf("Velocity-X: %f\n", com.GetVelocityX());
    printf("Raw Gyro X: %f\n", com.GetRawGyroX());
    printf("Accel X: %f\n", com.GetWorldLinearAccelX());
    printf("Temp (C): %f\n\n", com.GetTempC());

    printf("Program Execution Complete.\n");

    return 0;
}
