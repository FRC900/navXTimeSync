#include <iostream>
#include <thread>
#include <iomanip>
#include <signal.h>

#include "AHRS.h"
#include "TimeStamp.h"

volatile sig_atomic_t sflag = 0;

void handle_sig(int sig)
{
	(void)sig;
    sflag = 1;
}


int main(int argc, char *argv[]) {
	(void)argc;
	(void)argv;
    std::cout << "Program Executing\n";
    signal(SIGINT, handle_sig);

    AHRS com("/dev/ttyACM0", AHRS::SerialDataType::kProcessedData, 200);

    printf("Initializing\n\n");

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    std::cout << "Pitch  |  Roll  |  Yaw  |  X-Accel  | Y-Accel  |  Z-Accel  |  Time  |" << std::endl;

    while( 1 == 1){
        //std::cout << std::fixed << std::setprecision(2) << std::setw(6) << com.GetPitch() << "  " << std::setw(6) << com.GetRoll() << "   " << std::setw(7) << com.GetYaw() << "   " << std::setw(6) << com.GetWorldLinearAccelX() << "    " << std::setw(6) << com.GetWorldLinearAccelY() << "      " << std::setw(6) << com.GetWorldLinearAccelZ() << "      " << com.GetLastSensorTimestamp() << "      " << '\r' << std::flush;

		float r,p,y;
		float qx,qy,qz,qw;
		float ax, ay, az;
		long stamp;
		com.GetRPYQAccel(r, p, y, qx, qy, qz, qw, ax, ay, az, stamp);
        std::cout << std::fixed << std::setprecision(2) << std::setw(6) << p << "  " << std::setw(6) << r << "   " << std::setw(7) << y << "   " << std::setw(6) << ax << "    " << std::setw(6) << ay << "      " << std::setw(6) << az << "      " << stamp << "      " << '\r' << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(4));
        if(sflag){
            sflag = 0;
            com.Close();
			printf("\nExit Caught... Closing device.\n");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            break;
        }
    }

    return 0;
}
