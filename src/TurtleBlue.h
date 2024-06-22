/*
Author: Cesare M. Casirati
*/

#ifndef TurtleBlue_h
#define TurtleBlue_h


#ifndef MicroBlue
#define MicroBlue
#endif

#include "FunctionType.h"
#include <Arduino.h>

// start of MicroBlue App command definitions
#ifdef MicroBlue
#define START_OF_HEADING     0x01
#define START_OF_TEXT        0x02
#define END_OF_TEXT          0x03

#define PAD                 'd'
#define DIGITAL_PAD         0
#define ANALOG_PAD          1
#define BUTTON              'b'
#define SWITCH1             's'
#define SWITCH2             'w'
#define TEXT                't'
#define SLIDER1             's'
#define SLIDER2             'l'
#endif
// end of MicroBlue App command definitions

// start of ArduinoBlue App command definitions
#ifdef ArduinoBlue
// Denotes start of path transmission from mobile app.
#define PATH_TRANSMISSION               0xF4
// Denotes that the path transmission was received successfully by the Arduino. (Sent from Arduino).
#define PATH_TRANSMISSION_CONFIRMATION  0xF5

// Denotes start of location transmission from Arduino
#define LOCATION_TRANSMISSION_START     0xF6
#define DELIMETER                       0xF7
#define TEXT_SEND_TRANSMISSION          0xF8
#define CONNECTION_CHECK                0xF9
#define DRIVE_TRANSMISSION              0xFB

// Denotes start of button transmission from mobile app.
#define BUTTON_TRANSMISSION             0xFC

// Denotes start of slider transmission from mobile app.
#define SLIDER_TRANSMISSION             0xFD

// Denotes start of text transmission from mobile app.
#define TEXT_TRANSMISSION               0xFE
#define DISPLAY_SEND_TRANSMISSION       0xF3

// Denotes that the text, button, or slider transmission has been received successfully by the Arduino. (Sent from Arduino).
#define TRANSMISSION_END                0xFA

// Default value for signal array elements.
#define DEFAULT_VALUE                   0xFF

#define PATH_OVERFLOW_VALUE             1000000
#endif
// end of ArduinoBlue App command definitions

// Default value for signal array elements.
#define DEFAULT_VALUE                   0xFF

#define TEXT_TRANSMISSION_TIMEOUT       5000    // ms
#define SHORT_TRANSMISSION_TIMEOUT      500
#define PATH_TRANSMISSION_TIMEOUT       10000

const uint8_t DEFAULT_STEERING = 49;
const uint8_t DEFAULT_THROTTLE = 49;
const uint8_t MAX_SHORT_SIGNAL_LENGTH = 3;


class TurtleBlue {
public:
    TurtleBlue( Stream &output );
    int getButton();
    int getSliderId();
    int getSliderVal();
    int getThrottle();
    int getSteering();
    float * getPathArrayX();
    float * getPathArrayY();
    float getPathY( float );
    int getPathLength();
    bool checkBluetooth();
    bool isConnected();
    bool isPathAvailable();
    void sendText( String msg );
    void sendMessage( String msg );
    void sendDisplayData( uint8_t id, String data );
    void sendLocation( float, float, float, float, float );
    static float bytesToFloat( uint8_t u1, uint8_t u2, uint8_t u3, uint8_t u4 );
    String getText();
    void setInterruptToggle( functiontype attach, functiontype detach );
private:
    Stream & _bluetooth;
    uint8_t _signal[MAX_SHORT_SIGNAL_LENGTH];
    uint8_t _signalLength = 0;
    uint8_t _throttle = DEFAULT_STEERING;
    uint8_t _steering = DEFAULT_THROTTLE;
    uint8_t _sliderVal = DEFAULT_VALUE;
    uint8_t _sliderId = DEFAULT_VALUE;
    uint8_t _button = DEFAULT_VALUE;
    bool _pathAvailable = false;
    String _text;
    String _MicroBlueCommand;
    float * _pathX;
    float * _pathY;
    float _prevReturnXx;
    int _pathLength;
    void clearSignalArray();
    void pushToSignalArray( uint8_t item );
    void storeShortTransmission();
    bool storePathTransmission();
    void processDriveTransmission();
    void processButtonTransmission();
    void processSliderTransmission();
    void processTextTransmission();
    void processPathTransmission();
    void sendFloatAsBytes( float );
    void processMicroBlueCommand();
    void processMicroBlueDrive();
    void processMicroBlueDigitalPad();
    void processMicroBlueAnalogPad();
    void processMicroBlueButton();
    void processMicroBlueSwitch();
    void processMicroBlueSlider();
    void processMicroBlueText();
    void attachInterrupts();
    void detachInterrupts();
    String readString();
    functiontype _attachInterrupts = nullptr;
    functiontype _detachInterrupts = nullptr;
    };

#endif
