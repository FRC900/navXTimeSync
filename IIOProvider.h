/*
 * IIOProvider.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#ifndef SRC_IIOPROVIDER_H_
#define SRC_IIOPROVIDER_H_

#include <cstdint>

class IIOProvider {
public:
    IIOProvider(void) {}
    virtual bool   IsConnected(void) const = 0;
    virtual double GetByteCount(void) const = 0;
    virtual double GetUpdateCount(void) const = 0;
    virtual void   SetUpdateRateHz(uint8_t update_rate) = 0;
    virtual void   ZeroYaw(void) = 0;
    virtual void   ZeroDisplacement(void) = 0;
    virtual void   Run(void) = 0;
    virtual void   Stop(void) = 0;
};

#endif /* SRC_IIOPROVIDER_H_ */
