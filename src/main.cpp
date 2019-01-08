#include <Arduino.h>
#include <Homie.h>
#include <NewPing.h>

const int PIN_RELAY = 14;
const int TRIG_PIN = 4;
const int ECHO_PIN = 5;
const int REED_PIN = 12;
const int MAX_PING_CM = 275; // Most garage doors are ~6-8ft, this should cover all of them

HomieNode garageDoorNode("garage", "shutter");
HomieNode garageDoorSettingsNode("garage-settings", "settings");

NewPing ultrasonic(TRIG_PIN, ECHO_PIN, MAX_PING_CM);
Bounce debouncer = Bounce();
int lastClosedSensor = -1;

unsigned long lastRangeReading = 0;
unsigned long lastRangeCheck = -1;
unsigned long rangeCheckIntervalNormal = 5000L; // msec, configurable property
unsigned long rangeCheckIntervalMoving = 500L; // msec, configurable property

unsigned long garageDoorMaxDistance = 30L; // cm, configurable property
unsigned long carRoofMaxDistance = 100L; // cm, configurable property

// How long to hold down the relay for a press
unsigned long switchPressTime = 500L; // msec, configurable property
// How long to wait for the door to clear the sensors when moving
unsigned long sensorClearTime = 5000L; // msec, configurable property
// How long we expect the door movement to take. Should overestimate by 3-5 seconds.
unsigned long doorMovementTimeout = 30000L; // msec, configurable property

unsigned long switchOnTime = -1;
bool switchOn = false;
bool forceClose = false;

/* state machines for door and range sensor */
enum DOORSTATE {
  DS_UNKNOWN = -1,
  DS_OPEN,
  DS_CLOSING,
  DS_CLOSED,
  DS_OPENING
};
DOORSTATE doorState = DS_UNKNOWN;
DOORSTATE lastDoorState = DS_UNKNOWN;
unsigned long doorStateChangeTime = -1;

enum RANGESTATE {
  RS_ERROR = -1,
  RS_UNKNOWN,
  RS_GARAGEDOOR,
  RS_NODOOR_CAR,
  RS_NODOOR
};
RANGESTATE rangeState = RS_UNKNOWN;
RANGESTATE lastRangeState = RS_UNKNOWN;

bool doorHandler(const HomieRange& range, const String& value) {
  Homie.getLogger() << "door received: " << value << endl;
  /* Check for valid values */
  if (value != "open" && value != "close" && value != "force") {
    return false;
  }

  /* ignore if currently pressed (too fast) OR
   * if close requested and currently closing
   * if open requested and currently opening 
   **/
  if ((switchOn && !forceClose) ||
      (value == "close" && doorState == DS_CLOSING) ||
      (value == "open" && doorState == DS_OPENING)) {
    return false;
  }

  /* if currently forceClose'ing and anything else requested, let go */
  if (forceClose) {
    digitalWrite(PIN_RELAY, LOW);
    switchOn = false;
    forceClose = false;
    return false;
  }

  /* press the switch and flag mainloop to turn off at approriate time */
  digitalWrite(PIN_RELAY, HIGH);
  switchOnTime = millis();
  switchOn = true;
  forceClose = (value == "force");
  Homie.getLogger() << "Switch is on." << endl;

  /* Change the doorstate */
  switch(doorState) {
    case DS_OPEN:
    case DS_OPENING:
      doorState = DS_CLOSING;
      break;
    case DS_CLOSED:
    case DS_CLOSING:
      doorState = DS_OPENING;
      break;
    case DS_UNKNOWN:
    default:
      doorState = DS_OPENING; // assume the worst (i.e. lower security)
      break;
  }
  doorStateChangeTime = millis();


  return true;
}

void loopHandler() {

  /**** Update Sensors ****/
  /* Update door sensor reading */
  int closedSensor = debouncer.read();
  if (closedSensor != lastClosedSensor) {
    Homie.getLogger() << "Door is now " << (closedSensor ? "open" : "closed") << endl;
    garageDoorNode.setProperty("closedSensor").send(closedSensor ? "open" : "close");
    lastClosedSensor = closedSensor;
  }

  /* Take a range sensor reading */
  // normally use the long interval, unless door is moving (or unknown)
  unsigned long rangeCheckInterval = rangeCheckIntervalNormal;
  if (doorState == DS_CLOSING || doorState == DS_OPENING || doorState == DS_UNKNOWN) {
    rangeCheckInterval = rangeCheckIntervalMoving;
  }
  if (millis() - lastRangeCheck >= rangeCheckInterval) {
    unsigned long range = ultrasonic.ping_cm();
    Homie.getLogger() << "Range is now " << range << endl;
    Homie.getLogger() << "closedSensor is reading: " << (closedSensor ? "open" : "closed") << endl;
    garageDoorNode.setProperty("rangeSensor").send(String(range));

    if (range == 0) {
      rangeState = RS_ERROR;
    } else if (range <= garageDoorMaxDistance) {
      rangeState = RS_GARAGEDOOR;
    } else if (range > garageDoorMaxDistance && range <= carRoofMaxDistance) {
      rangeState = RS_NODOOR_CAR;
    } else if (range > carRoofMaxDistance) {
      rangeState = RS_NODOOR;
    } else { // Should never come to this
      rangeState = RS_UNKNOWN;
    }

    lastRangeReading = range;
    lastRangeCheck = millis();
  }

  /**** Handle Relay Changes ****/
  /* Check for 'letting go' of switch */
  if (switchOn) {
    if (forceClose) {
      // Stop pressing either when the door is closed OR forceClose timed out
      if (closedSensor == LOW || millis() - switchOnTime >= doorMovementTimeout) {
        digitalWrite(PIN_RELAY, LOW);
        switchOn = false;
        forceClose = false;
        Homie.getLogger() << "Switch is off (end of forced)." << endl;
      }
    } else {
      // Turn off relay after switchPressTime
      if (millis() - switchOnTime >= switchPressTime) {
        digitalWrite(PIN_RELAY, LOW);
        switchOn = false;
        Homie.getLogger() << "Switch is off." << endl;
      }
    }
  }


  /**** State Transitions ****/
  /* Check for door state transitions */
  switch (doorState) {
    case DS_OPEN:
      /* If range sensor has detected a changed, assume we are closing */
      if (rangeState != RS_GARAGEDOOR && rangeState != RS_ERROR && rangeState != RS_UNKNOWN) {
        if (closedSensor == LOW) {
          doorState = DS_CLOSED;
          doorStateChangeTime = millis();
        } else {
          doorState = DS_CLOSING;
          doorStateChangeTime = millis();
        }
      }
      break;
    case DS_CLOSED:
      /* If door closed sensor detects not closed, then we are opening (or opened) */
      if (closedSensor == HIGH) {      
        if (rangeState == RS_GARAGEDOOR) {
          doorState = DS_OPEN;
          doorStateChangeTime = millis();
        } else {
          doorState = DS_OPENING;
          doorStateChangeTime = millis();
        }
      }
      break;
    case DS_CLOSING:
    case DS_OPENING:
      /* While in motion, wait for sensor clear timeout to avoid oscillating back and forth.
       * Otherwise, when the button is pressed, the range will detect garage door and it
       * will go back to "OPEN"; same with the closedSensor. */
      if (millis() - doorStateChangeTime >= sensorClearTime) {
        if (closedSensor == LOW) {
          doorState = DS_CLOSED;
          doorStateChangeTime = millis();
        } else if (rangeState == RS_GARAGEDOOR) {
          doorState = DS_OPEN;
          doorStateChangeTime = millis();
        }
      }
      if (millis() - doorStateChangeTime >= doorMovementTimeout) {
        doorState = DS_OPEN;
        doorStateChangeTime = millis();
      }
      break;
    case DS_UNKNOWN:
    default:
      /* On first run just set state by closed door sensor */
      if (closedSensor == LOW) {
        doorState = DS_CLOSED;
        doorStateChangeTime = millis();
      } else {
        doorState = DS_OPEN;
        doorStateChangeTime = millis();
      }
      break;      
  }

  /* Report changes in states */
  if (rangeState != lastRangeState) {
    String rangeStateString;
    String carString;
    switch (rangeState) {
      case RS_ERROR: rangeStateString = "error"; carString = "unknown"; break;
      case RS_GARAGEDOOR: rangeStateString = "door"; carString = "unknown"; break;
      case RS_NODOOR_CAR: rangeStateString = "car"; carString = "true"; break;
      case RS_NODOOR: rangeStateString = "nodoor"; carString = "false"; break;
      case RS_UNKNOWN: rangeStateString = "unknown"; carString = "unknown"; break;
    }
    garageDoorNode.setProperty("rangeState").send(rangeStateString);
    garageDoorNode.setProperty("car").send(carString);
    Homie.getLogger() << "rangeState is now: " << rangeStateString << endl;
    Homie.getLogger() << "car is now: " << carString << endl;
    lastRangeState = rangeState;
  }

  if (doorState != lastDoorState) {
    String doorStateString;
    String doorString;
    switch (doorState) {
      case DS_UNKNOWN: doorStateString = "unknown"; doorString = "open"; break;
      case DS_OPEN: doorStateString = "open"; doorString = "open"; break;
      case DS_OPENING: doorStateString = "opening"; doorString = "open"; break;
      case DS_CLOSING: doorStateString = "closing"; doorString = "open"; break;
      case DS_CLOSED: doorStateString = "closed"; doorString = "closed"; break;
    }
    garageDoorNode.setProperty("doorState").send(doorStateString);
    garageDoorNode.setProperty("door").send(doorString);
    Homie.getLogger() << "doorState is now: " << doorStateString << endl;
    Homie.getLogger() << "door is now: " << doorString << endl;
    lastDoorState = doorState;
  }
}

void setup() {
  Serial.begin(115200);
  Serial << endl << endl;

  /* Setup pins */
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);

  pinMode(REED_PIN, INPUT_PULLUP);
  debouncer.attach(REED_PIN);
  debouncer.interval(200);

  // Ensure we check the range right away
  lastRangeCheck = millis() - rangeCheckIntervalNormal - 100;

  Homie_setFirmware("garage-homie", "1.2.0");
  Homie.setLoopFunction(loopHandler);

  /* Main properties */
  garageDoorNode.advertise("door").settable(doorHandler);
  garageDoorNode.advertise("car");

  /* Individual sensors */
  garageDoorNode.advertise("closedSensor");
  garageDoorNode.advertise("rangeSensor");

  /* "States" - fusion of sensors */
  garageDoorNode.advertise("doorState");
  garageDoorNode.advertise("rangeState");

  /* Configurable properties */
  /* TODO:
  garageSettingsNode.advertise("garageDoorDistance").settable(setGarageDoorDistance);
  garageSettingsNode.advertise("carRoofDistance").settable(setCarRoofDistance);
  garageSettingsNode.advertise("rangeInterval").settable(setRangeInterval);
  garageSettingsNode.advertise("rangeIntervalMoving").settable(setRangeIntervalMoving);
  garageSettingsNode.advertise("switchPressTime").settable(setPressTime);
  garageSettingsNode.advertise("forceCloseTimeout").settable(setForceCloseTimeout);
  */
  Homie.setup();

  //when the door is moving; setIdle(false) will disable reset
}

void loop() {
  Homie.loop();
  debouncer.update();
}