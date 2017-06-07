/*
 * IIOCompleteNotification.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#ifndef SRC_IIOCOMPLETENOTIFICATION_H_
#define SRC_IIOCOMPLETENOTIFICATION_H_

#include <stdint.h>

#include "IMUProtocol.h"
#include "AHRSProtocol.h"

class IIOCompleteNotification {
public:
    IIOCompleteNotification(void) {}
    struct BoardState {
        uint8_t op_status;
        int16_t sensor_status;
        uint8_t cal_status;
        uint8_t selftest_status;
        int16_t capability_flags;
        uint8_t update_rate_hz;
        int16_t accel_fsr_g;
        int16_t gyro_fsr_dps;
    };
    virtual void SetYawPitchRoll(const IMUProtocol::YPRUpdate& ypr_update, long sensor_timestamp) = 0;
    virtual void SetAHRSData(const AHRSProtocol::AHRSUpdate& ahrs_update, long sensor_timestamp) = 0;
    virtual void SetAHRSPosData(const AHRSProtocol::AHRSPosUpdate& ahrs_update, long sensor_timestamp) = 0;
    virtual void SetRawData(const IMUProtocol::GyroUpdate& raw_data_update, long sensor_timestamp) = 0;
    virtual void SetBoardID(const AHRSProtocol::BoardID& board_id) = 0;
    virtual void SetBoardState(const BoardState& board_state) = 0;
};

#endif /* SRC_IIOCOMPLETENOTIFICATION_H_ */
