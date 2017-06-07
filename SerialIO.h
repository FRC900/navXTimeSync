/*
 * SerialIO.h
 *
 *  Created on: Jul 31, 2015
 *      Author: Scott
 */

#ifndef SRC_SERIALIO_H_
#define SRC_SERIALIO_H_

//include "WPILIb.h"

#include <cstdint>

#include "AHRSProtocol.h"
#include "IBoardCapabilities.h"
#include "IIOCompleteNotification.h"
#include "IIOProvider.h"
#include "IMUProtocol.h"
#include "SerialPort.h"

class SerialIO : public IIOProvider {


    std::string serial_port_id;
    SerialPort *serial_port;
    uint8_t next_integration_control_action;
    bool signal_transmit_integration_control;
    bool signal_retransmit_stream_config;
    bool stop;
    uint8_t update_type; //IMUProtocol.MSGID_XXX
    uint8_t update_rate_hz;
    int byte_count;
    int update_count;
    IMUProtocol::YPRUpdate ypr_update_data;
    IMUProtocol::GyroUpdate gyro_update_data;
    AHRSProtocol::AHRSUpdate ahrs_update_data;
    AHRSProtocol::AHRSPosUpdate ahrspos_update_data;
    AHRSProtocol::AHRSPosTSUpdate ahrspos_ts_update_data;
    AHRSProtocol::BoardID board_id;
    IIOCompleteNotification *notify_sink;
    IIOCompleteNotification::BoardState board_state;
    IBoardCapabilities *board_capabilities;
    double last_valid_packet_time;

public:
    SerialIO( std::string port_id,
              uint8_t update_rate_hz,
              bool processed_data,
              IIOCompleteNotification *notify_sink,
              IBoardCapabilities *board_capabilities );
    bool IsConnected(void) const;
    double GetByteCount(void) const;
    double GetUpdateCount(void) const;
    void SetUpdateRateHz(uint8_t update_rate);
    void ZeroYaw(void);
    void ZeroDisplacement(void);
    void Run(void);
    void Stop(void);
private:

    SerialPort *ResetSerialPort(void);
    SerialPort *GetMaybeCreateSerialPort(void);
    void EnqueueIntegrationControlMessage(uint8_t action);
    void DispatchStreamResponse(const IMUProtocol::StreamResponse& response);
    int DecodePacketHandler(const char * received_data, int bytes_remaining);
};

#endif /* SRC_SERIALIO_H_ */
