#include <TurtleBlue.h>
#include <SoftwareSerial.h>

unsigned long BAUD_RATE = 9600;

const int BLUETOOTH_TX = 8;
const int BLUETOOTH_RX = 7;

const int READ_TIME = 500; //ms

unsigned long prevMillis;

SoftwareSerial bluetooth( BLUETOOTH_TX, BLUETOOTH_RX );

void setup() {
  // Start serial communications.
  // The baud rate must be the same for both the serial and the bluetooth.
  Serial.begin( BAUD_RATE );
  bluetooth.begin( BAUD_RATE );
  Serial.println( "\"HM10 to Serial\" setup complete" );
  }

void loop() {
  // put your main code here, to run repeatedly:
  if( Serial.available() ) {
    String str = "";
    Serial.print( "Input: " );
    
    prevMillis = millis();
    while( millis() - prevMillis < READ_TIME ) {
      if( Serial.available() ) {
        char c = Serial.read();
        if( c != 10 && c != 13 ) { // Don't send line end characters to HM10.
          str += c;
          }
        }
      }
    
    bluetooth.print(str);
    Serial.println(str);
    }

  if( bluetooth.available() ) {
    String str = "";
    Serial.print( "HM10: " );
    
    prevMillis = millis();
    while( millis() - prevMillis < READ_TIME ) {
      if( bluetooth.available() ) {
        char c = bluetooth.read();
        if( isPrintable( c ) ) {
          str += c ;
          }
        else {
          str += '<';
          str += (char)( c + '0' );
          str += '>';
          }
        }
      }
    Serial.println( str );
    }
  }