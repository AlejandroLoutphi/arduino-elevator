#include <Arduino.h>
#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.
#include <AFMotor.h>
#include <IRremote.hpp>

#if defined(DECODE_BEO)
#define RECORD_GAP_MICROS 16000 // always get the complete frame in the receive buffer, but this prevents decoding of SONY!
#endif

#if !defined(RAW_BUFFER_LENGTH)
// For air condition remotes it requires 750. Default is 200.
#  if !((defined(RAMEND) && RAMEND <= 0x4FF) || (defined(RAMSIZE) && RAMSIZE < 0x4FF))
#define RAW_BUFFER_LENGTH  750
#  endif
#endif


#if defined(APPLICATION_PIN)
#define DEBUG_BUTTON_PIN    APPLICATION_PIN // if low, print timing for each received data set
#else
#define DEBUG_BUTTON_PIN   6
#endif

AF_DCMotor motor(4);
int8_t currentFloor;
bool receivedSignal;

void moveElevator();
void motorRun();
void irSteup();

void setup() {
  irSetup();

  // turn on motor
  motor.setSpeed(20);
 
  motor.run(3);
  receivedSignal = false;
  currentFloor = 1;
}

void loop() {
    /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */
    if (IrReceiver.decode()) {
        Serial.println();

      // Print Pressed button id (aka command)
      if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) && IrReceiver.decodedIRData.protocol != UNKNOWN) {
        Serial.println(IrReceiver.decodedIRData.command);
      }

      /*
        * !!!Important!!! Enable receiving of the next value, because receiving
        * has stopped after the end of the current received data packet.
        * Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        */
      IrReceiver.resume();

      // When we receive a good signal
      if (IrReceiver.decodedIRData.address == 0 && !receivedSignal) {
        moveElevator();
        delay(200);
      }

      receivedSignal = false;
    }
}

void moveElevator() {
  receivedSignal = true;
  int8_t floorDelta = 0;
  switch(IrReceiver.decodedIRData.command) {
    case(82): // 8 Key
      motorRun(150, true);
      motorRun(150, false);
      break;
    case(12): // 1 Key
      floorDelta = 1 - currentFloor;
      currentFloor = 1;
      break;
    case(24): // 2 Key
      floorDelta = 2 - currentFloor;
      currentFloor = 2;
      break;
    case(94): // 3 Key
      floorDelta = 3 - currentFloor;
      currentFloor = 3;
      break;
  }

  Serial.println(currentFloor);
  Serial.println(floorDelta);
  if (floorDelta == 0) {
    return;
  }
  bool up = floorDelta > 0;
  floorDelta = abs(floorDelta);
  motorRun(floorDelta == 1 ? 500 : 1000, up);
}

void motorRun(uint8_t maxSpeed, bool up) {
  uint8_t i;
    
  Serial.print("tick");
 
  AF_DCMotor motor(3);

  motor.run(up ? FORWARD : BACKWARD);
  /*for (i=0; i < maxSpeed; i++) {
    motor.setSpeed(i);  
    delay(10);
 }
 
  for (i = maxSpeed; i!=0; i--) {
    motor.setSpeed(i);  
    delay(10);
 }*/

 motor.setSpeed(up ? 140 : 120);
 delay(maxSpeed);
 motor.setSpeed(0);
}

void irSetup() {
    pinMode(DEBUG_BUTTON_PIN, INPUT_PULLUP);

    Serial.begin(115200);
    while (!Serial); // Wait for Serial to become available. Is optimized away for some cores.

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
}