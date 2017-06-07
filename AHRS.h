/*
 * AHRS.h
 *
 *  Created on: Jul 30, 2015
 *      Author: Scott
 */

#ifndef SRC_AHRS_H_
#define SRC_AHRS_H_

#include "ITimestampedDataSubscriber.h"
#include <memory>
#include <string>

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
    AHRSInternal *      ahrs_internal;

    volatile float      yaw;
    volatile float      pitch;
    volatile float      roll;
    volatile float      compass_heading;
    volatile float      world_linear_accel_x;
    volatile float      world_linear_accel_y;
    volatile float      world_linear_accel_z;
    volatile float      mpu_temp_c;
    volatile float      fused_heading;
    volatile float      altitude;
    volatile float      baro_pressure;
    volatile bool       is_moving;
    volatile bool       is_rotating;
    volatile float      baro_sensor_temp_c;
    volatile bool       altitude_valid;
    volatile bool       is_magnetometer_calibrated;
    volatile bool       magnetic_disturbance;
    volatile float    	quaternionW;
    volatile float    	quaternionX;
    volatile float    	quaternionY;
    volatile float    	quaternionZ;

    /* Integrated Data */
    float velocity[3];
    float displacement[3];


    /* Raw Data */
    volatile int16_t    raw_gyro_x;
    volatile int16_t    raw_gyro_y;
    volatile int16_t    raw_gyro_z;
    volatile int16_t    raw_accel_x;
    volatile int16_t    raw_accel_y;
    volatile int16_t    raw_accel_z;
    volatile int16_t    cal_mag_x;
    volatile int16_t    cal_mag_y;
    volatile int16_t    cal_mag_z;

    /* Configuration/Status */
    volatile uint8_t    update_rate_hz;
    volatile int16_t    accel_fsr_g;
    volatile int16_t    gyro_fsr_dps;
    volatile int16_t    capability_flags;
    volatile uint8_t    op_status;
    volatile int16_t    sensor_status;
    volatile uint8_t    cal_status;
    volatile uint8_t    selftest_status;

    /* Board ID */
    volatile uint8_t    board_type;
    volatile uint8_t    hw_rev;
    volatile uint8_t    fw_ver_major;
    volatile uint8_t    fw_ver_minor;

    long                last_sensor_timestamp;
    double              last_update_time;


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

    float  GetPitch() const;
    float  GetRoll() const;
    float  GetYaw() const;
    float  GetCompassHeading() const;
    void   ZeroYaw() const;
    bool   IsCalibrating() const;
    bool   IsConnected() const;
    double GetByteCount() const;
    double GetUpdateCount() const;
    long   GetLastSensorTimestamp() const;
    float  GetWorldLinearAccelX() const;
    float  GetWorldLinearAccelY() const;
    float  GetWorldLinearAccelZ() const;
    bool   IsMoving() const;
    bool   IsRotating() const;
    float  GetBarometricPressure() const;
    float  GetAltitude() const;
    bool   IsAltitudeValid() const;
    float  GetFusedHeading() const;
    bool   IsMagneticDisturbance() const;
    bool   IsMagnetometerCalibrated() const;
    float  GetQuaternionW() const;
    float  GetQuaternionX() const;
    float  GetQuaternionY() const;
    float  GetQuaternionZ() const;
    void   ResetDisplacement();
    void   UpdateDisplacement( float accel_x_g, float accel_y_g,
                               int update_rate_hz, bool is_moving );
    float  GetVelocityX() const;
    float  GetVelocityY() const;
    float  GetVelocityZ() const;
    float  GetDisplacementX() const;
    float  GetDisplacementY() const;
    float  GetDisplacementZ() const;
    double GetAngle() const;
    double GetRate() const;
    void   Reset();
    float  GetRawGyroX() const;
    float  GetRawGyroY() const;
    float  GetRawGyroZ() const;
    float  GetRawAccelX() const;
    float  GetRawAccelY() const;
    float  GetRawAccelZ() const;
    float  GetRawMagX() const;
    float  GetRawMagY() const;
    float  GetRawMagZ() const;
    float  GetPressure() const;
    float  GetTempC() const;
    AHRS::BoardYawAxis GetBoardYawAxis() const;
    std::string GetFirmwareVersion() const;

    bool RegisterCallback( ITimestampedDataSubscriber *callback, void *callback_context);
    bool DeregisterCallback( ITimestampedDataSubscriber *callback );

    int GetActualUpdateRate() const;
    int GetRequestedUpdateRate() const;

    void Close();

private:
    void SerialInit(const std::string &serial_port_id, AHRS::SerialDataType data_type, uint8_t update_rate_hz);
    void commonInit( uint8_t update_rate_hz );
    static void *ThreadFunc(void *threadarg);

    /* LiveWindowSendable implementation */
    std::string GetSmartDashboardType() const;
    void StartLiveWindowMode();
    void StopLiveWindowMode();

    /* PIDSource implementation */
    double PIDGet() const;

    uint8_t GetActualUpdateRateInternal(uint8_t update_rate) const;
};

#endif /* SRC_AHRS_H_ */
