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

AF_DCMotor motor(3);
int8_t pos;
uint8_t stopTimer;
// These act as a stack
int8_t posTargets[2];
int8_t posTargetsTop;

int8_t addInputToFloorTargets(
    int8_t pos,
    int8_t *posTargets,
    int8_t posTargetsTop);
void irSetup();


void setup() {
  // Setup for the IR receiver
  irSetup();

  // turn on motor
  motor.setSpeed(20);
  motor.run(3);

  // initialize variables
  pos = 1;
  posTargetsTop = 0;
  stopTimer = 0;
  pos = 0;
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
    // Print Pressed button id (aka command)
    if (!(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) && IrReceiver.decodedIRData.protocol != UNKNOWN) {
      Serial.println(IrReceiver.decodedIRData.command);
    }

    IrReceiver.resume();

    // When we receive a good signal
    if (IrReceiver.decodedIRData.address == 0) {
      posTargetsTop = addInputToFloorTargets(pos, posTargets, posTargetsTop);
    }
  }

  if (stopTimer) {
      stopTimer--;
  } else {
    if (posTargetsTop) {
      if (pos != posTargets[posTargetsTop - 1]) {
        // If we haven't gotten to destination floor, keep moving
        AF_DCMotor motor(3);
        motor.run(posTargets[posTargetsTop - 1] > pos ? FORWARD : BACKWARD);
        motor.setSpeed(posTargets[posTargetsTop - 1] > pos ? 140 : 120);
      } else {
        // If we've gotten to our destination, stop
        motor.setSpeed(0);
        stopTimer = 1000;
      }
    }
  }
}

// Returns new floorTargetsTop
int8_t addInputToFloorTargets(
    int8_t pos,
    int8_t *posTargets,
    int8_t posTargetsTop) {
  int8_t tmp;

#define FLOOR_HEIGHT 100
  // Add a new floor target to the stack based on input
  if (posTargetsTop < 2) {
    switch(IrReceiver.decodedIRData.command) {
      case(12): // 1 Key
        posTargets[posTargetsTop] = 0;
        break;
      case(24): // 2 Key
        posTargets[posTargetsTop] = FLOOR_HEIGHT;
        break;
      case(94): // 3 Key
        posTargets[posTargetsTop] = FLOOR_HEIGHT * 2;
        break;
    }

    // If the new floor target is the current floor, discard it
    // If both floor targets are the same, discard one
    if (posTargets[posTargetsTop] == pos ||
        (posTargetsTop == 1 && posTargets[1] == posTargets[0])) {
      return posTargetsTop;
    }

    // Sort stack with top being highest priority (smallest delta with currentFloor)
    // We only have to do this if both slots are populated
    if (posTargetsTop == 1 &&
        abs(pos - posTargets[1])
        > abs(pos - posTargets[0])) {
      //Swap
      tmp = posTargets[1];
      posTargets[1] = posTargets[0];
      posTargets[0] = tmp;
    }

    return posTargetsTop + 1;
  }
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