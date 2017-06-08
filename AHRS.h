/*
 * AHRS.h
 *
 *  Created on: Jul 30, 2015
 *      Author: Scott
 */

#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include <memory>
#include <mutex>
#include <string>
#include <pthread.h>

#include "ITimestampedDataSubscriber.h"

class IIOProvider;
class ContinuousAngleTracker;
class InertialDataIntegrator;
class OffsetTracker;
class AHRSInternal;

class AHRS{
public:

    enum BoardAxis {
        kBoardAxisX = 0,
        kBoardAxisY = 1,
        kBoardAxisZ = 2,
    };

    struct BoardYawAxis
    {
        /* Identifies one of the board axes */
        BoardAxis board_axis;
        /* true if axis is pointing up (with respect to gravity); false if pointing down. */
        bool up;
    };

    enum SerialDataType {
    /**
     * (default):  6 and 9-axis processed data
     */
    kProcessedData = 0,
    /**
     * unprocessed data from each individual sensor
     */
    kRawData = 1
    };

private:
    friend class AHRSInternal;
    AHRSInternal *ahrs_internal;

    float      yaw;
    float      pitch;
    float      roll;
    float      compass_heading;
    float      world_linear_accel_x;
    float      world_linear_accel_y;
    float      world_linear_accel_z;
    float      mpu_temp_c;
    float      fused_heading;
    float      altitude;
    float      baro_pressure;
    bool       is_moving;
    bool       is_rotating;
    float      baro_sensor_temp_c;
    bool       altitude_valid;
    bool       is_magnetometer_calibrated;
    bool       magnetic_disturbance;
    float      quaternionW;
    float      quaternionX;
    float      quaternionY;
    float      quaternionZ;

    /* Integrated Data */
    float velocity[3];
    float displacement[3];


    /* Raw Data */
    int16_t    raw_gyro_x;
    int16_t    raw_gyro_y;
    int16_t    raw_gyro_z;
    int16_t    raw_accel_x;
    int16_t    raw_accel_y;
    int16_t    raw_accel_z;
    int16_t    cal_mag_x;
    int16_t    cal_mag_y;
    int16_t    cal_mag_z;

    /* Configuration/Status */
    uint8_t    update_rate_hz;
    int16_t    accel_fsr_g;
    int16_t    gyro_fsr_dps;
    int16_t    capability_flags;
    uint8_t    op_status;
    int16_t    sensor_status;
    uint8_t    cal_status;
    uint8_t    selftest_status;

    /* Board ID */
    uint8_t    board_type;
    uint8_t    hw_rev;
    uint8_t    fw_ver_major;
    uint8_t    fw_ver_minor;

    long       last_sensor_timestamp;
    double     last_update_time;

	pthread_t  trd;
	std::mutex mutex;

    InertialDataIntegrator *integrator;
    ContinuousAngleTracker *yaw_angle_tracker;
    OffsetTracker *         yaw_offset_tracker;
    IIOProvider *           io;


#define MAX_NUM_CALLBACKS 3
    ITimestampedDataSubscriber *callbacks[MAX_NUM_CALLBACKS];
    void *callback_contexts[MAX_NUM_CALLBACKS];

public:
    AHRS(const std::string &serial_port_id);

    AHRS(const std::string &serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);

    float  GetPitch(void);
    float  GetRoll(void);
    float  GetYaw(void);
    float  GetCompassHeading(void);
    void   ZeroYaw(void);
    bool   IsCalibrating(void);
    bool   IsConnected(void) const;
    double GetByteCount(void) const;
    double GetUpdateCount(void) const;
    long   GetLastSensorTimestamp(void);
    float  GetWorldLinearAccelX(void);
    float  GetWorldLinearAccelY(void);
    float  GetWorldLinearAccelZ(void);
    bool   IsMoving(void);
    bool   IsRotating(void);
    float  GetBarometricPressure(void);
    float  GetAltitude(void);
    bool   IsAltitudeValid(void);
    float  GetFusedHeading(void);
    bool   IsMagneticDisturbance(void);
    bool   IsMagnetometerCalibrated(void);
    float  GetQuaternionW(void);
    float  GetQuaternionX(void);
    float  GetQuaternionY(void);
    float  GetQuaternionZ(void);
    void   ResetDisplacement(void);
    void   UpdateDisplacement( float accel_x_g, float accel_y_g,
                               int update_rate_hz, bool is_moving );
    float  GetVelocityX(void);
    float  GetVelocityY(void);
    float  GetVelocityZ(void);
    float  GetDisplacementX(void);
    float  GetDisplacementY(void);
    float  GetDisplacementZ(void);
    double GetAngle(void);
    double GetRate(void);
    void   Reset(void);
    float  GetRawGyroX(void);
    float  GetRawGyroY(void);
    float  GetRawGyroZ(void);
    float  GetRawAccelX(void);
    float  GetRawAccelY(void);
    float  GetRawAccelZ(void);
    float  GetRawMagX(void);
    float  GetRawMagY(void);
    float  GetRawMagZ(void);
    float  GetPressure(void);
    float  GetTempC(void);
    AHRS::BoardYawAxis GetBoardYawAxis(void);
    std::string GetFirmwareVersion(void);

	void GetRPYQAccel(float &r, float &p, float &y, float &qx, float &qy, float &qz, float &qw, float &ax, float &ay, float &az, long &stamp);

    bool RegisterCallback( ITimestampedDataSubscriber *callback, void *callback_context);
    bool DeregisterCallback( ITimestampedDataSubscriber *callback );

    int GetActualUpdateRate(void);
    int GetRequestedUpdateRate(void);

    void Close(void);

private:
    void SerialInit(const std::string &serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
    void commonInit( uint8_t update_rate_hz );
    static void *ThreadFunc(void *threadarg);

    /* LiveWindowSendable implementation */
    std::string GetSmartDashboardType(void) const;
    void StartLiveWindowMode(void);
    void StopLiveWindowMode(void);

    /* PIDSource implementation */
    double PIDGet(void);

    uint8_t GetActualUpdateRateInternal(uint8_t update_rate);
};

#endif /* SRC_AHRS_H_ */
