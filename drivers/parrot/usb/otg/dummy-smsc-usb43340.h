/**
********************************************************************************
* @file dummy-smsc-usb43340.h
* @brief smsc USB43340 transceiver 
*
* Copyright (C) 2010 Parrot S.A.
*
* @author     Christian ROSALIE <christian.rosalie@parrot.com>
* @date       2010-09-03
********************************************************************************
*/

#ifndef __DUMMY_SMSC43340_USB_H_
#define __DUMMY_SMSC43340_USB_H_

struct smsc43340_usb_data {
        /* pins must be set before being used */
	int			gpio_reset;
	int			gpio_overcurrent;
};

#endif
