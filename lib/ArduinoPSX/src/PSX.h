/*
    Name:       PSX.h
    Descr:      Playstation 2 controller library for Arduino definition
    Version:    1.0.1
    Author:     Christoph Honal
    Date:       2016-04-11
    Update:     2020-04-25
*/

#ifndef Psx_h
#define Psx_h

#include "Arduino.h"


// Button bitfield constants

#define PSXBTN_LEFT         0x8000
#define PSXBTN_DOWN         0x4000
#define PSXBTN_RIGHT        0x2000
#define PSXBTN_UP           0x1000
#define PSXBTN_START        0x0800
#define PSBTN_ACT_RIGHT     0x0400
#define PSBTN_ACT_LEFT      0x0200
#define PSXBTN_SELECT       0x0100
#define PSXBTN_SQUARE       0x0080
#define PSXBTN_CROSS        0x0040
#define PSXBTN_CIRCLE       0x0020
#define PSXBTN_TRIANGLE     0x0010
#define PSXBTN_R1           0x0008
#define PSXBTN_L1           0x0004
#define PSXBTN_R2           0x0002
#define PSXBTN_L2           0x0001

// Protocol commands and constants

#define PSXPROT_IDLE        0x00
#define PSXPROT_NONZERO     0xFF
#define PSXPROT_ZERO        0x00
#define PSXPROT_HANDSHAKE   0x01
#define PSXPROT_GETDATA     0x42
#define PSXPROT_STARTDATA   0x5A
#define PSXPROT_CONFIG      0x43
#define PSXPROT_CONFIGMOTOR 0x4D
#define PSXPROT_MOTORMAP    0x01
#define PSXPROT_ENTERCONFIG 0x01
#define PSXPROT_EXITCONFIG  0x00
#define PSXPROT_EXITCFGCNT  0x5A
#define PSXPROT_CONFIGMODE  0x44
#define PSXPROT_MODELOCK    0x03
#define PSXMODE_ANALOG      0x01
#define PSXMODE_DIGITAL     0x00

// Read function return codes

#define PSXERROR_SUCCESS    0
#define PSXERROR_NODATA     1


class PSX
{
    public:

        struct PSXDATA
        {
            unsigned int buttons;
            byte JoyLeftX;
            byte JoyLeftY;
            byte JoyRightX;
            byte JoyRightY;
        };

        PSX();

        void setupPins(byte dataPin, byte cmdPin, byte attPin, byte clockPin, byte delay);
        int read(PSXDATA &psxdata);
        void config(byte mode);

        /** Small motor: 0=off, 0xFF=on. Large motor: 0=off, 0x40-0xFF=intensity. */
        void setRumble(byte smallMotor, byte largeMotor);

        virtual void pinMode(byte pin, byte mode) { ::pinMode(pin, mode); };
        virtual void digitalWrite(byte pin, byte value){ ::digitalWrite(pin, value); };
        virtual byte digitalRead(byte pin){ return ::digitalRead(pin);};
        
    private:
        void sendCommand(byte command, byte &response);

        byte _dataPin, _cmdPin, _attPin, _clockPin, _delay;
        byte _rumbleSmall = 0;
        byte _rumbleLarge = 0;
};

#endif
