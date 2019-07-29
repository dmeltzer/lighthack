// Copyright (c) 2019 Daniel Meltzer <dmeltzer.devel@gmail.com>
// Based on the box1_usb sketch:
// Copyright (c) 2017 Electronic Theatre Controls, Inc., http://www.etcconnect.com
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


/*******************************************************************************

     Electronic Theatre Controls

     lighthack - PS3 Controller

     (C) 2019 by Daniel Meltzer <dmeltzer.devel@gmail.com>


     This code implements basic control of Eos parameters using a PS3 Controller.
     Current Pan/Tilt and Next/Last are supported.  Future goals include Edge and 
     Shutters, with an ultimate purpose of simplifying ML focus updates.

 *******************************************************************************

    NOTE: UPDATE VERSION_STRING IN DEFINITIONS BELOW WHEN VERSION NUMBER CHANGES

    Revision History

    yyyy-mm-dd   Vxx      By_Who                 Comment

    2019-07-28   0.0.1    Daniel Meltzer         Initial Version.

 ******************************************************************************/

/*******************************************************************************
   Includes
 ******************************************************************************/
#include <OSCBoards.h>
#include <OSCBundle.h>
#include <OSCData.h>
#include <OSCMatch.h>
#include <OSCMessage.h>
#include <OSCTiming.h>
#ifdef BOARD_HAS_USB_SERIAL
#include <SLIPEncodedUSBSerial.h>
SLIPEncodedUSBSerial SLIPSerial(thisBoardsSerialUSB);
#else
#include <SLIPEncodedSerial.h>
SLIPEncodedSerial SLIPSerial(Serial);
#endif
#include <string.h>
#include <PS3USB.h>

/*******************************************************************************
   Macros and Constants
 ******************************************************************************/
#define SUBSCRIBE           ((int32_t)1)
#define UNSUBSCRIBE         ((int32_t)0)

#define EDGE_DOWN           ((int32_t)1)
#define EDGE_UP             ((int32_t)0)

#define FORWARD             0
#define REVERSE             1

// Change these values to switch which direction increase/decrease pan/tilt
#define PAN_DIR             FORWARD
#define TILT_DIR            FORWARD

// Use these values to make the encoder more coarse or fine.
// This controls the number of wheel "ticks" the device sends to the console
// for each tick of the encoder. 1 is the default and the most fine setting.
// Must be an integer.
#define PAN_SCALE           1
#define TILT_SCALE          1

#define OSC_BUF_MAX_SIZE    512

const String HANDSHAKE_QUERY = "ETCOSC?";
const String HANDSHAKE_REPLY = "OK";

//See displayScreen() below - limited to 10 chars (after 6 prefix chars)
#define VERSION_STRING      "0.0.1"

#define BOX_NAME_STRING     "PS3_CONTROL"

// Change these values to alter how long we wait before sending an OSC ping
// to see if Eos is still there, and then finally how long before we
// disconnect and show the splash screen
// Values are in milliseconds
#define PING_AFTER_IDLE_INTERVAL    2500
#define TIMEOUT_AFTER_IDLE_INTERVAL 5000

/*******************************************************************************
   Local Types
 ******************************************************************************/
enum WHEEL_TYPE { TILT, PAN, ZOOM, EDGE, THRUST, ANGLE };
enum WHEEL_MODE { COARSE, FINE };
enum SHUTTER { SHUTTER_A, SHUTTER_B, SHUTTER_C, SHUTTER_D };

enum ConsoleType
{
  ConsoleNone,
  ConsoleEos,
  ConsoleCobalt,
  ConsoleColorSource
};

enum ControllerMode
{
  PanTilt,
  FocusZoom,
  Shutter
};

/*******************************************************************************
   Global Variables
 ******************************************************************************/

ConsoleType connectedToConsole = ConsoleNone;
unsigned long lastMessageRxTime = 0;
bool timeoutPingSent = false;
USB Usb;
PS3USB PS3(&Usb); // TODO bluetoothify
ControllerMode activeMode;
SHUTTER activeShutter;
/*******************************************************************************
   Local Functions
 ******************************************************************************/

/*******************************************************************************
   Issues all our subscribes to Eos. When subscribed, Eos will keep us updated
   with the latest values for a given parameter.

   Parameters:  none

   Return Value: void

 ******************************************************************************/
void issueEosSubscribes()
{
  // Add a filter so we don't get spammed with unwanted OSC messages from Eos
  OSCMessage filter("/eos/filter/add");
  filter.add("/eos/out/param/*");
  filter.add("/eos/out/ping");
  filter.add("/eos/out/active/wheel/*");
  SLIPSerial.beginPacket();
  filter.send(SLIPSerial);
  SLIPSerial.endPacket();

  // subscribe to Eos pan & tilt updates
  OSCMessage subPan("/eos/subscribe/param/pan");
  subPan.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subPan.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subTilt("/eos/subscribe/param/tilt");
  subTilt.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subTilt.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subZoom("/eos/subscribe/param/zoom");
  subZoom.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subZoom.send(SLIPSerial);
  SLIPSerial.endPacket();

  OSCMessage subEdge("/eos/subscribe/param/edge");
  subEdge.add(SUBSCRIBE);
  SLIPSerial.beginPacket();
  subEdge.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void setControllerMode(ControllerMode mode)
{
  Serial.print(F("\r\nSetting To mode:"));
  Serial.print(mode);
   activeMode = mode;
   switch(mode)
   {
      case PanTilt:
        PS3.setLedOff();
        delay(50);
        PS3.setLedOn(LED1);
        break;
      case FocusZoom:
        PS3.setLedOff();
        delay(50);
        PS3.setLedOn(LED2);
        break;
      case Shutter:
        PS3.setLedOff();
        delay(50);
        PS3.setLedOn(LED1);
   }
}

/*******************************************************************************
   Given a valid OSCMessage (relevant to Pan/Tilt), we update our Encoder struct
   with the new position information.

   Parameters:
    msg - The OSC message we will use to update our internal data
    addressOffset - Unused (allows for multiple nested roots)

   Return Value: void

 ******************************************************************************/

void parseEos(OSCMessage& msg, int addressOffset)
{
  // If we don't think we're connected, reconnect and subscribe
  if (connectedToConsole != ConsoleEos)
  {
    issueEosSubscribes();
    connectedToConsole = ConsoleEos;
  }
}

/*******************************************************************************
   Given an unknown OSC message we check to see if it's a handshake message.
   If it's a handshake we issue a subscribe, otherwise we begin route the OSC
   message to the appropriate function.

   Parameters:
    msg - The OSC message of unknown importance

   Return Value: void

 ******************************************************************************/
void parseOSCMessage(String& msg)
{
  // check to see if this is the handshake string
  if (msg.indexOf(HANDSHAKE_QUERY) != -1)
  {
    // handshake string found!
    SLIPSerial.beginPacket();
    SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
    SLIPSerial.endPacket();

    // An Eos would do nothing until subscribed
    // Let Eos know we want updates on some things
    issueEosSubscribes();
  }
  else
  {
    // prepare the message for routing by filling an OSCMessage object with our message string
    OSCMessage oscmsg;
    oscmsg.fill((uint8_t*)msg.c_str(), (int)msg.length());
    // route pan/tilt messages to the relevant update function

    // Try the various OSC routes
    if (oscmsg.route("/eos", parseEos))
      return;
  }
}

/*******************************************************************************
   Sends a message to Eos informing them of a wheel movement.

   Parameters:
    type - the type of wheel that's moving (i.e. pan or tilt)
    ticks - the direction and intensity of the movement

   Return Value: void

 ******************************************************************************/

void sendOscMessage(const String &address, float value)
{
  OSCMessage msg(address.c_str());
  msg.add(value);
  SLIPSerial.beginPacket();
  msg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

String shutterString(SHUTTER shutter)
{
  switch (shutter) {
    case SHUTTER_A:
      return "A";
    case SHUTTER_B:
      return "B";
    case SHUTTER_C:
      return "C";
    case SHUTTER_D:
      return "D";
  }
}
void sendEosWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/eos/wheel");

  if (PS3.getButtonPress(R3))
    wheelMsg.concat("/fine");
  else
    wheelMsg.concat("/coarse");

  if (type == PAN)
    wheelMsg.concat("/pan");
  else if (type == TILT)
    wheelMsg.concat("/tilt");
  else if (type == ZOOM)
    wheelMsg.concat("/zoom");
  else if (type == EDGE)
    wheelMsg.concat("/edge");
  else if (type == THRUST)
    wheelMsg.concat("/frame_thrust_"+shutterString(activeShutter));
  else if (type == ANGLE)
    wheelMsg.concat("/frame_angle_"+shutterString(activeShutter));
  else
    // something has gone very wrong
    return;

  sendOscMessage(wheelMsg, ticks);
}

void sendCobaltWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/cobalt/param");

  if (type == PAN)
    wheelMsg.concat("/pan/wheel");
  else if (type == TILT)
    wheelMsg.concat("/tilt/wheel");
  else
    // something has gone very wrong
    return;


  sendOscMessage(wheelMsg, ticks);
}

void sendColorSourceWheelMove(WHEEL_TYPE type, float ticks)
{
  String wheelMsg("/cs/param");

  if (type == PAN)
    wheelMsg.concat("/pan/wheel");
  else if (type == TILT)
    wheelMsg.concat("/tilt/wheel");
  else
    // something has gone very wrong
    return;

  sendOscMessage(wheelMsg, ticks);
}

/******************************************************************************/

void sendWheelMove(WHEEL_TYPE type, float ticks)
{
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      sendEosWheelMove(type, ticks);
      break;
    case ConsoleCobalt:
      sendCobaltWheelMove(type, ticks);
      break;
    case ConsoleColorSource:
      sendColorSourceWheelMove(type, ticks);
      break;
  }
}

/*******************************************************************************
   Sends a message to the console informing them of a key press.

   Parameters:
    down - whether a key has been pushed down (true) or released (false)
    key - the OSC key name that has moved

   Return Value: void

 ******************************************************************************/
void sendKeyPress(bool down, const String &key)
{
  String keyAddress;
  switch (connectedToConsole)
  {
    default:
    case ConsoleEos:
      keyAddress = "/eos/key/" + key;
      break;
    case ConsoleCobalt:
      keyAddress = "/cobalt/key/" + key;
      break;
    case ConsoleColorSource:
      keyAddress = "/cs/key/" + key;
      break;
  }
  OSCMessage keyMsg(keyAddress.c_str());

  if (down)
    keyMsg.add(EDGE_DOWN);
  else
    keyMsg.add(EDGE_UP);

  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

void sendOscCommand(const String &command)
{
  OSCMessage keyMsg(command.c_str());
  SLIPSerial.beginPacket();
  keyMsg.send(SLIPSerial);
  SLIPSerial.endPacket();
}

/*******************************************************************************
   Checks the status of all the relevant buttons (i.e. Next & Last)

   NOTE: This does not check the shift key. The shift key is used in tandem with
   the encoder to determine coarse/fine mode and thus does not report directly.

   Parameters: none

   Return Value: void

 ******************************************************************************/

void checkButtons()
{
  if (PS3.getButtonClick(R1)) {// Next Channel
    sendKeyPress(true, "NEXT");
    delay(20);
    sendKeyPress(false, "NEXT");
  }
  if (PS3.getButtonClick(L1)) {// Prev Channel
    sendKeyPress(true, "LAST");
    delay(20);
    sendKeyPress(false, "LAST");
  }

  if (PS3.getButtonClick(R2)) {// Full
    sendOscCommand("/eos/at/full");
  }
  if (PS3.getButtonClick(L2)) {// Out
    sendOscCommand("/eos/at/out");
  }
  if (PS3.getButtonClick(START)) {// Out
    sendOscCommand("/eos/macro/802/fire");
  }
  if (PS3.getButtonClick(SELECT)) {// Out
    sendOscCommand("/eos/macro/801/fire");
  } 
  // If we're in shutter mode, allow assigning active shutter.
  if (activeMode == Shutter) {
      if(PS3.getButtonClick(UP))
        activeShutter = SHUTTER_A;
      if(PS3.getButtonClick(RIGHT))
        activeShutter = SHUTTER_B;
      if(PS3.getButtonClick(DOWN))
        activeShutter = SHUTTER_C;
      if(PS3.getButtonClick(LEFT))
        activeShutter = SHUTTER_D;
    }
  if (PS3.getButtonClick(PS)) { // Toggle Controller Mode
    switch(activeMode) {
      case PanTilt:
        setControllerMode(FocusZoom);
        break;
      case FocusZoom:
        setControllerMode(Shutter);
        break;
      case Shutter:
      default:
        setControllerMode(PanTilt);
        break;
    }

  }
}

/*******************************************************************************
   Initializes Connection to PS3 Controller and sets up the world.

   Return Value: void

 ******************************************************************************/

void initPS3Controller()
{
  if (Usb.Init() == -1) {
//    Serial.print(F("\r\nOSC Did not start"));
    while(1); // halt
  }

  setControllerMode(PanTilt);
//  Serial.print(F("\r\nUSB Lib started."));
}

/*******************************************************************************
   Generic function to respond to joystick actions.

   Parameters:
    - stickPos: Current Joystick position between 0-255
    - wheel: WHEEL_TYPE parameter.

   Return Value: void

 ******************************************************************************/
 
void handleParamMove(uint8_t stickPos, WHEEL_TYPE wheel)
{
  // Joystick position is in the range of 0-255, with 117-137 being "home"
  // Let's make this uniform (there's probably a better way.
  // If we subtract 127, we should get a value between -127 and 127.
  float normValue = stickPos - 127;
  // Let's convert this to a value between 0 and 10.
  sendWheelMove(wheel, normValue / 127);
}

/*******************************************************************************
   Here we setup our encoder, lcd, and various input devices. We also prepare
   to communicate OSC with Eos by setting up SLIPSerial. Once we are done with
   setup() we pass control over to loop() and never call setup() again.

   NOTE: This function is the entry function. This is where control over the
   Arduino is passed to us (the end user).

   Parameters: none

   Return Value: void

 ******************************************************************************/
void setup()
{
  SLIPSerial.begin(115200);
  // This is a hack around an Arduino bug. It was taken from the OSC library
  //examples
#ifdef BOARD_HAS_USB_SERIAL
  while (!SerialUSB);
#else
  while (!Serial);
#endif

  // This is necessary for reconnecting a device because it needs some time
  // for the serial port to open. The handshake message may have been sent
  // from the console before #lighthack was ready

  SLIPSerial.beginPacket();
  SLIPSerial.write((const uint8_t*)HANDSHAKE_REPLY.c_str(), (size_t)HANDSHAKE_REPLY.length());
  SLIPSerial.endPacket();

  // If it's an Eos, request updates on some things
  issueEosSubscribes();
  activeShutter = SHUTTER_A;
  initPS3Controller();
}

/*******************************************************************************
   Here we service, monitor, and otherwise control all our peripheral devices.
   First, we retrieve the status of our encoders and buttons and update Eos.
   Next, we check if there are any OSC messages for us.
   Finally, we update our display (if an update is necessary)

   NOTE: This function is our main loop and thus this function will be called
   repeatedly forever

   Parameters: none

   Return Value: void

 ******************************************************************************/
void loop()
{
  Usb.Task();
  if (!PS3.PS3Connected) {
    return; // No Controller Connected
  }

  uint8_t leftHatX = PS3.getAnalogHat(LeftHatX);
  uint8_t leftHatY = PS3.getAnalogHat(LeftHatY);
  uint8_t rightHatX = PS3.getAnalogHat(RightHatX);
  uint8_t rightHatY = PS3.getAnalogHat(RightHatY);

  WHEEL_TYPE leftStickParam;
  WHEEL_TYPE rightStickParam;
  switch(activeMode) {
    case FocusZoom:
      leftStickParam = EDGE;
      rightStickParam = ZOOM;
      break;
    case Shutter:
      leftStickParam = THRUST;
      rightStickParam = ANGLE;
      break;
    case PanTilt:
    default:
      leftStickParam = PAN;
      rightStickParam = TILT;
      break;
  }
  if (leftHatX > 137 
    || leftHatX < 117) {
      handleParamMove(leftHatX, leftStickParam);
    }

    if (rightHatY > 137 
    || rightHatY < 117) {
        handleParamMove(-rightHatY, rightStickParam);
    }
    
  static String curMsg;
  int size;

//  // check for next/last updates
  checkButtons();

  // Then we check to see if any OSC commands have come from Eos
  // and update the display accordingly.
  size = SLIPSerial.available();
  if (size > 0)
  {
    // Fill the msg with all of the available bytes
    while (size--)
      curMsg += (char)(SLIPSerial.read());
  }
  if (SLIPSerial.endofPacket())
  {
    parseOSCMessage(curMsg);
    lastMessageRxTime = millis();
    // We only care about the ping if we haven't heard recently
    // Clear flag when we get any traffic
    timeoutPingSent = false;
    curMsg = String();
  }

  if (lastMessageRxTime > 0)
  {
    unsigned long diff = millis() - lastMessageRxTime;
    //We first check if it's been too long and we need to time out
    if (diff > TIMEOUT_AFTER_IDLE_INTERVAL)
    {
      connectedToConsole = ConsoleNone;
      lastMessageRxTime = 0;
      timeoutPingSent = false;
    }

    //It could be the console is sitting idle. Send a ping once to
    // double check that it's still there, but only once after 2.5s have passed
    if (!timeoutPingSent && diff > PING_AFTER_IDLE_INTERVAL)
    {
      OSCMessage ping("/eos/ping");
      ping.add(BOX_NAME_STRING "_hello"); // This way we know who is sending the ping
      SLIPSerial.beginPacket();
      ping.send(SLIPSerial);
      SLIPSerial.endPacket();
      timeoutPingSent = true;
    }
  }

  delay(40);
}
