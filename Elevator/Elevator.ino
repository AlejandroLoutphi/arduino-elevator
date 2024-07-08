#include <Arduino.h>

#include "PinDefinitionsAndMore.h" // Define macros for input and output pin etc.

#include <AFMotor.h>

AF_DCMotor motor(4);
uint8_t currentFloor;
bool receivedSignal;

#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604. Code does not fit in program memory of ATtiny85 etc.
#endif
#if defined(DECODE_BEO)
#define RECORD_GAP_MICROS 16000 // always get the complete frame in the receive buffer, but this prevents decoding of SONY!
#endif

#if !defined(RAW_BUFFER_LENGTH)
// For air condition remotes it requires 750. Default is 200.
#  if !((defined(RAMEND) && RAMEND <= 0x4FF) || (defined(RAMSIZE) && RAMSIZE < 0x4FF))
#define RAW_BUFFER_LENGTH  750
#  endif
#endif

#include <IRremote.hpp>

#if defined(APPLICATION_PIN)
#define DEBUG_BUTTON_PIN    APPLICATION_PIN // if low, print timing for each received data set
#else
#define DEBUG_BUTTON_PIN   6
#endif

void generateTone();
void handleOverflow();
bool detectLongPress(uint16_t aLongPressDurationMillis);


void irSetup() {
#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604. Code does not fit in program memory of ATtiny85 etc.
    pinMode(DEBUG_BUTTON_PIN, INPUT_PULLUP);
#endif

    Serial.begin(115200);
    while (!Serial)
        ; // Wait for Serial to become available. Is optimized away for some cores.

#if defined(__AVR_ATmega32U4__) || defined(SERIAL_PORT_USBVIRTUAL) || defined(SERIAL_USB) /*stm32duino*/|| defined(USBCON) /*STM32_stm32*/ \
    || defined(SERIALUSB_PID)  || defined(ARDUINO_ARCH_RP2040) || defined(ARDUINO_attiny3217)
    delay(4000); // To be able to connect Serial monitor after reset or power up and before first print out. Do not wait for an attached Serial Monitor!
#endif
// Just to know which program is running on my Arduino
    Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

// In case the interrupt driver crashes on setup, give a clue
// to the user what's going on.
    Serial.println(F("Enabling IRin..."));

    // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    Serial.print(F("Ready to receive IR signals of protocols: "));
    printActiveIRProtocols(&Serial);
#if defined(IR_RECEIVE_PIN_STRING)
    Serial.println(F("at pin " IR_RECEIVE_PIN_STRING));
#else
    Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
#endif

#if FLASHEND >= 0x3FFF  // For 16k flash or more, like ATtiny1604. Code does not fit in program memory of ATtiny85 etc.
    Serial.println();
    if (digitalRead(DEBUG_BUTTON_PIN) != LOW) {
        Serial.print(F("If you connect debug pin "));
#  if defined(APPLICATION_PIN_STRING)
        Serial.print(APPLICATION_PIN_STRING);
#  else
        Serial.print(DEBUG_BUTTON_PIN);
#  endif
        Serial.print(F(" to ground, "));
    }
    Serial.println(F("raw data is always printed"));

    // infos for receive
    Serial.print(RECORD_GAP_MICROS);
    Serial.println(F(" us is the (minimum) gap, after which the start of a new IR packet is assumed"));
    Serial.print(MARK_EXCESS_MICROS);
    Serial.println(F(" us are subtracted from all marks and added to all spaces for decoding"));
#endif
}

void setup() {
  irSetup();
  Serial.println("Motor test!");

  // turn on motor
  motor.setSpeed(20);
 
  motor.run(3);
  receivedSignal = false;
  currentFloor = 1;
}

#define MAX_SPEED 80

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
#if FLASHEND < 0x3FFF  //
        // For less than 16k flash, only print a minimal summary of received data
        IrReceiver.printIRResultMinimal(&Serial);
#else

        /*
         *
         */
        if (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_WAS_OVERFLOW) {
            handleOverflow();
        } else {
            /*
             * No overflow here.
             * Stop receiver, generate a single beep, print short info and send usage and start receiver again
             *****************************************************************************************************/

            if ((IrReceiver.decodedIRData.protocol != SONY) && (IrReceiver.decodedIRData.protocol != PULSE_WIDTH)
                    && (IrReceiver.decodedIRData.protocol != PULSE_DISTANCE) && (IrReceiver.decodedIRData.protocol != UNKNOWN)
                    && digitalRead(DEBUG_BUTTON_PIN) != LOW) {
                /*
                 * For SONY the tone prevents the detection of a repeat after the 15 ms SONY gap.
                 * In debug mode and for unknown protocols, we need the time for extended output.
                 * Skipping tone will get exact gap time between transmissions and not running into repeat frames while wait for tone to end.
                 * This in turn enables the next CheckForRecordGapsMicros() call a chance to eventually propose a change of the current RECORD_GAP_MICROS value.
                 */
                generateTone();
            }

            /*
             * Print info
             */
            if (IrReceiver.decodedIRData.protocol == UNKNOWN || digitalRead(DEBUG_BUTTON_PIN) == LOW) {
                // We have debug enabled or an unknown protocol, print extended info
                if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
                    Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
                }
                IrReceiver.printIRResultRawFormatted(&Serial, true);
            }
            if (IrReceiver.decodedIRData.protocol != UNKNOWN) {
                /*
                 * The info output for a successful receive
                 */
                Serial.println(IrReceiver.decodedIRData.command);//.printIRResultShort(&Serial);

            }
        }
#endif // #if FLASHEND >= 0x3FFF

        /*
         * !!!Important!!! Enable receiving of the next value, because receiving
         * has stopped after the end of the current received data packet.
         * Do it here, to preserve raw data for printing with printIRResultRawFormatted()
         */
        IrReceiver.resume();

        /*
         * Finally check the received data and perform actions according to the received address and commands
         */
        if (IrReceiver.decodedIRData.address == 0 && !receivedSignal) {
          moveElevator();
          delay(200);
        }

        receivedSignal = false;

        // Check if repeats of the IR command was sent for more than 1000 ms
        if (detectLongPress(1000)) {
            Serial.print(F("Command 0x"));
            Serial.print(IrReceiver.decodedIRData.command, HEX);
            Serial.println(F(" was repeated for more than 2 seconds"));
        }
    } // if (IrReceiver.decode())

    /*
     * Your code here
     * For all users of the FastLed library, use this code for strip.show() to improve receiving performance (which is still not 100%):
     * if (IrReceiver.isIdle()) {
     *     strip.show();
     * }
     */

}

/*
 * Stop receiver, generate a single beep and start receiver again
 */
void generateTone() {
#if !defined(ESP8266) && !defined(NRF5) // tone on esp8266 works only once, then it disables IrReceiver.restartTimer() / timerConfigForReceive().
#  if defined(ESP32) // ESP32 uses another timer for tone()
    tone(TONE_PIN, 2200, 8);
#  else
    IrReceiver.stopTimer(); // ESP32 uses another timer for tone(), maybe other platforms (not tested yet) too.
    tone(TONE_PIN, 2200, 8);
    delay(8);
    IrReceiver.restartTimer(8000); // Restart IR timer. 8000 to compensate for 8 ms stop of receiver. This enables a correct gap measurement.
#  endif
#endif
}

void handleOverflow() {
    Serial.println(F("Overflow detected"));
    Serial.println(F("Try to increase the \"RAW_BUFFER_LENGTH\" value of " STR(RAW_BUFFER_LENGTH) " in " __FILE__));
    // see also https://github.com/Arduino-IRremote/Arduino-IRremote#compile-options--macros-for-this-library

#if !defined(ESP8266) && !defined(NRF5) // tone on esp8266 works once, then it disables IrReceiver.restartTimer() / timerConfigForReceive().
    /*
     * Stop timer, generate a double beep and start timer again
     */
#  if defined(ESP32) // ESP32 uses another timer for tone()
    tone(TONE_PIN, 1100, 10);
    delay(50);
    tone(TONE_PIN, 1100, 10);
#  else
    IrReceiver.stopTimer();
    tone(TONE_PIN, 1100, 10);
    delay(50);
    tone(TONE_PIN, 1100, 10);
    delay(50);
    IrReceiver.restartTimer(100000); // to compensate for 100 ms stop of receiver. This enables a correct gap measurement.
#  endif
#endif
}

unsigned long sMillisOfFirstReceive;
bool sLongPressJustDetected;
/**
 * True once we received the consecutive repeats for more than aLongPressDurationMillis milliseconds.
 * The first frame, which is no repeat, is NOT counted for the duration!
 * @return true once after the repeated IR command was received for longer than aLongPressDurationMillis milliseconds, false otherwise.
 */
bool detectLongPress(uint16_t aLongPressDurationMillis) {
    if (!sLongPressJustDetected && (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT)) {
        /*
         * Here the repeat flag is set (which implies, that command is the same as the previous one)
         */
        if (millis() - aLongPressDurationMillis > sMillisOfFirstReceive) {
            sLongPressJustDetected = true; // Long press here
        }
    } else {
        // No repeat here
        sMillisOfFirstReceive = millis();
        sLongPressJustDetected = false;
    }
    return sLongPressJustDetected; // No long press here
}

