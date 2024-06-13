/*
Author: Cesare M. Casirati
*/

#include "TurtleBlue.h"
#include <Arduino.h>

typedef union {
	float number;
	uint8_t bytes[4];
    } FLOATUNION_t;

TurtleBlue::TurtleBlue( Stream &output ) : 
        _bluetooth( output ) { }

// Checks for incoming bluetooth signal
bool TurtleBlue::checkBluetooth() {

    bool isDataRead = _bluetooth.available() > 0;

    while ( _bluetooth.available() > 0 ) {
        uint8_t intRead = _bluetooth.read();

        // Check for transmission starting
        // If a new transmission starts process the transmission
        // according MicroBlue app syntax
#ifdef MicroBlue
        if( intRead == START_OF_HEADING ) {
            processMicroBlueCommand();
            }
#endif
        // otherwise 
        // according ArduinoBlue app syntax
#ifdef ArduinoBlue
        if( intRead == DRIVE_TRANSMISSION ) {
            processDriveTransmission();
            }
        else if( intRead == BUTTON_TRANSMISSION ) {
            processButtonTransmission();
            }
        else if( intRead == SLIDER_TRANSMISSION ) {
            processSliderTransmission();
            }
        else if( intRead == TEXT_TRANSMISSION ) {
            processTextTransmission();
            }
        else if( intRead == PATH_TRANSMISSION ) {
            processPathTransmission();
            }
        else if( intRead == CONNECTION_CHECK ) {
            _bluetooth.print(CONNECTION_CHECK);
            }
#endif
        }

    return isDataRead;
    }

// MicroBlue app command process
// Process incoming command Id.
void TurtleBlue::processMicroBlueCommand() {
    uint8_t intRead;
    unsigned long prevTime = millis();

    _MicroBlueCommand = "";
    // Read until newline character or timeout is reached
    prevTime = millis();
    while( millis() - prevTime < TEXT_TRANSMISSION_TIMEOUT ) {
        if( _bluetooth.available() ) {
            intRead = _bluetooth.read();
            // break the loop if end of transmission
            if( intRead == END_OF_TEXT ) break;
            _MicroBlueCommand.concat( (char) intRead );
            }
        }
    
    if( _MicroBlueCommand.indexOf( START_OF_TEXT ) > 0 ) {
        if( _MicroBlueCommand[0] == PAD ) {
            processMicroBlueDrive();
            }
        else if( _MicroBlueCommand[0] == BUTTON ) {
            processMicroBlueButton();
            }
        else if( _MicroBlueCommand[0] == SWITCH1 && 
                 _MicroBlueCommand[1] == SWITCH2 ) { 
            processMicroBlueSwitch();
            }
        else if( _MicroBlueCommand[0] == TEXT ) {
            processMicroBlueText();
            }
        else if( _MicroBlueCommand[0] == SLIDER1 &&
                 _MicroBlueCommand[1] == SLIDER2 ) {
            processMicroBlueSlider();
            }
        }
    }

void TurtleBlue::processMicroBlueDrive() {
    // Read the incoming heading increments
    String h = _MicroBlueCommand.substring( _MicroBlueCommand.indexOf( START_OF_TEXT ) + 1, _MicroBlueCommand.indexOf( ',' ) );
    String v = _MicroBlueCommand.substring( _MicroBlueCommand.indexOf( ',' ) + 1, _MicroBlueCommand.length() );

    // Set throttle and steering values
    // throttle values:
    //     99 = max forward throttle
    //     49 = no throttle
    //      0 = max reverse throttle
    // steering values:
    //     99 = max right
    //     49 = straight
    //      0 = max left
    _throttle = map( v.toInt(), 0, 1023, 0, 99 );
    _steering = map( h.toInt(), 0, 1023, 0, 99 );
    }

// TO DO
void TurtleBlue::processMicroBlueButton() {}
void TurtleBlue::processMicroBlueSwitch() {}
void TurtleBlue::processMicroBlueSlider() {}
void TurtleBlue::processMicroBlueText() {}

#ifdef ArduinoBlue
void TurtleBlue::processDriveTransmission() {
    // Store the incoming throttle and steering values in the signal array.
    storeShortTransmission();

    // Set the values from the signal array.
    _throttle = _signal[0];
    _steering = _signal[1];

    clearSignalArray();
    }

// Process incoming button data.
void TurtleBlue::processButtonTransmission() {
    // Store the button transmission in the signal array.
    storeShortTransmission();

    // Set the values from the signal array.
    _button = _signal[0];
    clearSignalArray();
    }

// Process incoming slider data.
void TurtleBlue::processSliderTransmission() {
    // Store the slider transmission in the signal array.
    storeShortTransmission();

    // Set the values from the signal array.
    _sliderId = _signal[0];
    _sliderVal = _signal[1];

    clearSignalArray();
    }

// Process incoming text data.
void TurtleBlue::processTextTransmission() {
    // Get the text transmission.
    _text = readString();
    clearSignalArray();
    }

// TODO: Implement this.
void TurtleBlue::processPathTransmission() {
    // 
    detachInterrupts();
	_pathAvailable = storePathTransmission();
    _bluetooth.print( (char) PATH_TRANSMISSION_CONFIRMATION );
    _bluetooth.flush();

    clearSignalArray();
    attachInterrupts();
    }

// Send location data back to TurtleBlue
void TurtleBlue::sendLocation( float xPos, float yPos, float headingAngle, float xGoal, float yGoal ) {
	_bluetooth.print(  (char) LOCATION_TRANSMISSION_START );
	sendFloatAsBytes( (float) xPos );
	sendFloatAsBytes( (float) yPos );
	sendFloatAsBytes( (float) headingAngle );
	sendFloatAsBytes( (float) xGoal );
	sendFloatAsBytes( (float) yGoal );
    }

// TODO: May not be needed.
void TurtleBlue::sendFloatAsBytes( float num ) {
	FLOATUNION_t floatUnion;
	floatUnion.number = num;
	_bluetooth.print( (char) floatUnion.bytes[0] );
	_bluetooth.print( (char) floatUnion.bytes[1] );
	_bluetooth.print( (char) floatUnion.bytes[2] );
	_bluetooth.print( (char) floatUnion.bytes[3] );
    }

// Convert four bytes to float value.
float TurtleBlue::bytesToFloat( uint8_t u1, uint8_t u2, uint8_t u3, uint8_t u4 ) {
	FLOATUNION_t floatUntion;
	floatUntion.bytes[0] = u1;
	floatUntion.bytes[1] = u2;
	floatUntion.bytes[2] = u3;
	floatUntion.bytes[3] = u4;
	return floatUntion.number;
    }

// Read and store the path transmission.
bool TurtleBlue::storePathTransmission() {
	const int BYTES_PER_FLOAT = 4;

	// Delete the previously stored path if available.
	delete[] _pathX;
	delete[] _pathY;

	unsigned long prevMillis = millis();
	uint8_t intRead;
	int bytesReadIteration = 0;
	int numbersRead = 0;
	uint8_t ary[BYTES_PER_FLOAT];
	_pathLength = 9999; // Get in the loop

	// Read the float values corresponding to the path data.
	while( numbersRead <= _pathLength * 2 ) {
		if( _bluetooth.available() ) {

			intRead = _bluetooth.read();

			ary[bytesReadIteration] = intRead;
			bytesReadIteration++;

			if( bytesReadIteration == BYTES_PER_FLOAT ) {
				// Convert the byte array to float array.
				float number = bytesToFloat( ary[0], ary[1], ary[2], ary[3] );
				numbersRead++;

				// The first path data transmitted is the length.
				if ( numbersRead == 1 ) {
					_pathLength = (int)number;
					_pathX = new float[_pathLength];
					_pathY = new float[_pathLength];
#ifdef DEBUG
					Serial.print( "Path length: " ); 
                    Serial.println( _pathLength );
#endif
				    }
				// Subsequent path data transmission are the coordinates.
				else {
					int index = numbersRead / 2 - 1;
					if( fabs( number ) > PATH_OVERFLOW_VALUE ) {
						// If any value has overflowed and is invalid use the previous coordinate.
                        // Overflowed means that it's junk data. Throw it away.
						// TODO: Consider the case when the index is 0.
						_pathX[index] = _pathX[index - 1];
						_pathY[index] = _pathY[index - 1];
    					}
					else {
					    if( numbersRead % 2 == 0 ) {
						    _pathX[index] = number;
#ifdef DEBUG
						    Serial.print( number );
#endif
					        }
						else {
						    _pathY[index] = number;
#ifdef DEBUG
						    Serial.print( "\t" ); 
                            Serial.println( number );
#endif
						    }
					    }
#ifdef DEBUG
					Serial.print( "Number Read: " ); 
                    Serial.println( numberReadCount );
#endif
				    }
				bytesReadIteration = 0;
			    }
		    }
		if (millis() - prevMillis > PATH_TRANSMISSION_TIMEOUT) {
#ifdef DEBUG
			Serial.println("Error: path transmission took too long.");
#endif
			return false;
		    }
	    }
	return true;
    }

// Stores short transmission into the signal array
void TurtleBlue::storeShortTransmission() {
	unsigned long prevMillis = millis();
	uint8_t intRead;
	while( millis() - prevMillis < SHORT_TRANSMISSION_TIMEOUT ) {
		if( _bluetooth.available() ) {
			intRead = _bluetooth.read();
			if( intRead == TRANSMISSION_END ) break;
			pushToSignalArray( intRead );
		    }
	    }
    }

// Read and store incoming characters until TRANMISSION_END character is read or until TEXT_TRANMISSION_TIMEOUT.
// Return the characters read.
String TurtleBlue::readString() {
    String s;
    uint8_t intRead;
    unsigned long prevTime = millis();

    // Read until newline character or timeout is reached
    prevTime = millis();
    while( millis() - prevTime < TEXT_TRANSMISSION_TIMEOUT ) {
        if( _bluetooth.available() ) {
            intRead = _bluetooth.read();
            // break the loop if end of transmission
            if( intRead == TRANSMISSION_END ) break;
            s.concat( (char) intRead );
            }
        }
    return s;
    }

void TurtleBlue::pushToSignalArray( uint8_t item ) {
    if( item < 0 ) {
        // Serial.print("neg");
        }
    if( !( _signalLength + 1 == MAX_SHORT_SIGNAL_LENGTH ) ) {
        _signal[_signalLength] = item;
        _signalLength++;
        }
    else {
        // Serial.println( "TurtleBlue: Transmission error..." );
		// Serial.print( "item: " ); Serial.println( item );
        }
    }

void TurtleBlue::clearSignalArray() {
    for( uint8_t i = 0; i < _signalLength; i++ ) {
        _signal[i] = DEFAULT_VALUE;
        }
    _signalLength = 0;
    }
#endif

// Returns true if path data is available for use.
bool TurtleBlue::isPathAvailable() {
	bool toReturn = _pathAvailable;
	_pathAvailable = false;
	return toReturn;
    }

// Returns the ID of the button pressed.
// Returns -1 if a button has not been pressed.
int TurtleBlue::getButton() {
    checkBluetooth();
    uint8_t btn = _button;
    _button = DEFAULT_VALUE;
    if( btn == DEFAULT_VALUE ) return -1;
    return btn;
    }

// Returns the slider ID of the slider touched.
// Returns -1 if no slider was touched.
int TurtleBlue::getSliderId() {
    checkBluetooth();
    uint8_t id = _sliderId;
    _sliderId = DEFAULT_VALUE;
    if( id == DEFAULT_VALUE ) return -1;
    return id;
    }

// Returns the value of the latest slider touched.
int TurtleBlue::getSliderVal() {
    uint8_t val = _sliderVal;
    _sliderVal = DEFAULT_VALUE;
    if( val == DEFAULT_VALUE ) return -1;
    return val;
    }

// Returns the throttle value of the joystick or tilt.
int TurtleBlue::getThrottle() {
    checkBluetooth();
    return _throttle;
    }

// Returns the steering value of the joystick or tilt.
int TurtleBlue::getSteering() {
    checkBluetooth();
    return _steering;
    }

// Returns the pointer to the x coordinates of path data.
float * TurtleBlue::getPathArrayX() {
	return _pathX;
    }

// Returns the pointer to the y coordinates of path data.
float * TurtleBlue::getPathArrayY() {
	return _pathY;
    }

// Returns the length of the path data.
int TurtleBlue::getPathLength() {
	return _pathLength;
	//return tempPathLength;
    }

#ifdef ArduinoBlue
// Send text to TurtleBlue to show in the terminal component
void TurtleBlue::sendText( String msg ) {
    _bluetooth.print( ( (char) TEXT_SEND_TRANSMISSION) + msg + ( (char) TRANSMISSION_END) );
    _bluetooth.flush();
    }

// Send text to TurtleBlue to show in a display component
void TurtleBlue::sendDisplayData( uint8_t id, String msg ) {
    _bluetooth.print( ( (char) DISPLAY_SEND_TRANSMISSION ) + ( ( (char) id ) + msg ) + ( (char) TRANSMISSION_END) );
    _bluetooth.flush();
    }
#endif

// for backwards compatibility
void TurtleBlue::sendMessage( String msg ) {
    sendText(msg);
    }

#ifdef ArduinoBlue
bool TurtleBlue::isConnected() {
    _bluetooth.print( CONNECTION_CHECK );
    // wait for 500 ms
    delay( 500 );
    if( _bluetooth.available() ) {
        return _bluetooth.read() == CONNECTION_CHECK;
        }
    return false;
    }
#endif

// Returns the text that the user sent through the app.
String TurtleBlue::getText() {
    checkBluetooth();
    String ret = _text;
    _text = "";
    return ret;
    }

// Sets pointer to the functions to detach and attach interrupts.
// This is done so that interrupts will be disabled when path data is being read.
// See storePathTransmission().
void TurtleBlue::setInterruptToggle( functiontype attach, functiontype detach ) {
    _attachInterrupts = attach;
    _detachInterrupts = detach;
    }

// Attach interrupts
// See setInterruptToggle()
void TurtleBlue::attachInterrupts() {
    if( _attachInterrupts != nullptr ) {
        _attachInterrupts();
        }
    }

// Detach interrupts
// See setInterruptToggle()
void TurtleBlue::detachInterrupts() {
    if( _detachInterrupts != nullptr ) {
        _detachInterrupts();
        }
    }
