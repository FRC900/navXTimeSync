/*
 * IBoardCapabilities.h
 *
 *  Created on: Jul 29, 2015
 *      Author: Scott
 */

#ifndef SRC_IBOARDCAPABILITIES_H_
#define SRC_IBOARDCAPABILITIES_H_

class IBoardCapabilities {
public:
    IBoardCapabilities(void) {}
    virtual bool IsOmniMountSupported(void) const = 0;
    virtual bool IsBoardYawResetSupported(void) const = 0;
    virtual bool IsDisplacementSupported(void) const = 0;
    virtual bool IsAHRSPosTimestampSupported(void) const = 0;
};

#endif /* SRC_IBOARDCAPABILITIES_H_ */
