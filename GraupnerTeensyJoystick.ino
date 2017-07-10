// Using Teensy 3.0 and a Graupner Hott with SUMD to have a USB Joystick interface
// Solder servo cable black to GND, red to VC USB, yellow signal to Pin 0
// Bind and configure GR12 or other receiver to send SUMD, connect servo cable to SUMD port
// Compile the sketch using a HID interface with joystick

// Uses Cleanflight Sumd implementation

#define HWSERIAL Serial1

#define SUMD_SYNCBYTE 0xA8
#define SUMD_MAX_CHANNEL 16
#define SUMD_BUFFSIZE (SUMD_MAX_CHANNEL * 2 + 5) // 6 channels + 5 = 17 bytes for 6 channels
#define SUMD_OFFSET_CHANNEL_1_HIGH 3
#define SUMD_OFFSET_CHANNEL_1_LOW 4
#define SUMD_BYTES_PER_CHANNEL 2
#define SUMD_FRAME_STATE_OK 0x01
#define SUMD_FRAME_STATE_FAILSAFE 0x81
#define SUMD_BAUDRATE 115200

static bool sumdFrameDone = false;
static uint16_t sumdChannels[SUMD_MAX_CHANNEL];
static uint16_t crc;

typedef enum {
  RX_FRAME_PENDING = 0,
  RX_FRAME_COMPLETE = (1 << 0),
  RX_FRAME_FAILSAFE = (1 << 1)
} rxFrameState_e;

#define CRC_POLYNOME 0x1021

// CRC calculation, adds a 8 bit unsigned to 16 bit crc
static void CRC16(uint8_t value)
{
  uint8_t i;

  crc = crc ^ (int16_t)value << 8;
  for (i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ CRC_POLYNOME;
    else
      crc = (crc << 1);
  }
}

static uint8_t sumd[SUMD_BUFFSIZE] = { 0, };
static uint8_t sumdChannelCount;

void setup() {
  //Serial.begin(SUMD_BAUDRATE);
  HWSERIAL.begin(SUMD_BAUDRATE);
  Joystick.useManualSend(true);
}

static void sumdDataReceive(uint16_t c)
{
  uint32_t sumdTime;
  static uint32_t sumdTimeLast;
  static uint8_t sumdIndex;

  sumdTime = micros();
  if ((sumdTime - sumdTimeLast) > 4000)
    sumdIndex = 0;
  sumdTimeLast = sumdTime;

  if (sumdIndex == 0) {
    if (c != SUMD_SYNCBYTE) {
      return;
    }
    else
    {
      sumdFrameDone = false; // lazy main loop didnt fetch the stuff
      crc = 0;
    }
  }
  if (sumdIndex == 2)
    sumdChannelCount = (uint8_t)c;
  if (sumdIndex < SUMD_BUFFSIZE)
    sumd[sumdIndex] = (uint8_t)c;
  sumdIndex++;
  if (sumdIndex < sumdChannelCount * 2 + 4)
    CRC16((uint8_t)c);
  else if (sumdIndex == sumdChannelCount * 2 + 5) {
    sumdIndex = 0;
    sumdFrameDone = true;
  }
}

static uint8_t sumdFrameStatus(void)
{
  uint8_t channelIndex;

  uint8_t frameStatus = RX_FRAME_PENDING;

  if (!sumdFrameDone) {
    return frameStatus;
  }

  sumdFrameDone = false;

  // verify CRC
  if (crc != ((sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
              (sumd[SUMD_BYTES_PER_CHANNEL * sumdChannelCount + SUMD_OFFSET_CHANNEL_1_LOW])))
    return frameStatus;

  switch (sumd[1]) {
    case SUMD_FRAME_STATE_FAILSAFE:
      frameStatus = RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
      break;
    case SUMD_FRAME_STATE_OK:
      frameStatus = RX_FRAME_COMPLETE;
      break;
    default:
      return frameStatus;
  }

  if (sumdChannelCount > SUMD_MAX_CHANNEL)
    sumdChannelCount = SUMD_MAX_CHANNEL;

  for (channelIndex = 0; channelIndex < sumdChannelCount; channelIndex++) {
    sumdChannels[channelIndex] = (
                                   (sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_HIGH] << 8) |
                                   sumd[SUMD_BYTES_PER_CHANNEL * channelIndex + SUMD_OFFSET_CHANNEL_1_LOW]
                                 );
  }
  return frameStatus;
}



void loop() {
  if (HWSERIAL.available()) {
    sumdDataReceive(HWSERIAL.read());
    if (sumdFrameDone && sumdFrameStatus() == RX_FRAME_COMPLETE) {
      /*
        for (int i = 0; i < sumdChannelCount; i++) {
        Serial.print("Channel");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(sumdChannels[i] / 8);
        }
      */
      // "value" is from 0 to 1023
      // 512 is resting position

      Joystick.Y(map(sumdChannels[0] / 8, 900, 2100, 1023, 0));            // Throttle, inverted
      Joystick.Zrotate(map(sumdChannels[1] / 8, 900, 2100, 0, 1023));      // Roll
      Joystick.Z(map(sumdChannels[2] / 8, 900, 2100, 0, 1023));            // Pitch
      Joystick.X(map(sumdChannels[3] / 8, 900, 2100, 0, 1023));            // Yaw
      Joystick.sliderLeft(map(sumdChannels[4] / 8, 900, 2100, 0, 1023));   // Chan 5
      Joystick.sliderRight(map(sumdChannels[5] / 8, 900, 2100, 0, 1023));  // Chan 6
      Joystick.send_now();
    }
  }
}



