/**
 * @file JetsonESP32.ino
 * @brief Test program for ESP32 to turn off the WiFi and enable
 * the Bluetooth. The bluetooth will be used to communicate serial
 * commands from the ESP32's serial port.
 *
 * @version 0.2
 * @date 2023-08-02
 * @author Shamsullah Ahmadzai
 */

#include <BluetoothSerial.h>

BluetoothSerial SerialBT;
const byte PACKET_SIZE = 10;
byte packet[PACKET_SIZE];
byte packetIndex = 0;
bool isPacketStart = false;

void setup()
{
    SerialBT.begin("ESP32");

    Serial2.begin(115200); // Start serial communication on TX1 and RX1 pins

    Serial2.println(F("Bluetooth enabled."));
}

void loop()
{
    if (Serial2.available() >= 10) {
      byte byteRead = Serial2.read();

      if (isPacketStart) {
        packet[packetIndex++] = byteRead;
        if (packetIndex >= PACKET_SIZE) {
          // Serial2.println(packet[1]);
          SerialBT.write(packet, sizeof(packet));
          // for (int i=0; i < sizeof(packet); i++) {
          //   Serial.print(packet[i], HEX);
          //   Serial.print(" ");
          // }
          // interpretData(packet);
          isPacketStart = false;
          packetIndex = 0;
        }
      } else if (byteRead == 0xAA) {
        isPacketStart = true;
        packet[packetIndex++] = byteRead;
      }
    }

    if (SerialBT.available())
    {
        Serial2.write(SerialBT.read());
    }
    // Adjusted delay value
    delay(10);
    // Serial.println();
}

void interpretData(byte data[])
{
  float speedKMPH = 0.0;
  int temperature = 0;
  int odometer = 0;
  int maxSpeedKMPH = 0;
  int runTimeMinutes = 0;
  byte headlightStatus = 0x00;
  
  byte command = data[1];

  switch (command) {
    case 0xA1:
      // Interpret data for A1 command
      speedKMPH = ((data[4] * 0xFF + data[5]) / 10.0); // Current speed in km/h
      temperature = data[7]; // Temperature in Celsius

      Serial.print("A1 Command - Current Speed (KMPH): ");
      Serial.print(speedKMPH);
      Serial.print(", Temperature: ");
      Serial.println(temperature);
      break;

    case 0xA2:
      // Interpret data for A2 command
      odometer = data[4]; // Odometer value (unknown format)

      Serial.print("A2 Command - Odometer: ");
      Serial.println(odometer);
      break;

    case 0xA3:
      // Interpret data for A3 command
      maxSpeedKMPH = data[3]; // Max speed in km/h
      runTimeMinutes = data[7]; // Run time in minutes

      Serial.print("A3 Command - Max Speed (KMPH): ");
      Serial.print(maxSpeedKMPH);
      Serial.print(", Run Time (Minutes): ");
      Serial.println(runTimeMinutes);
      break;

    case 0xA4:
      // Interpret data for A4 command
      headlightStatus = data[7]; // 0x00 off, 0x10 on

      Serial.print("A4 Command - Headlight Status: ");
      if (headlightStatus == 0x00) {
        Serial.println("Off");
      } else if (headlightStatus == 0x10) {
        Serial.println("On");
      } else {
        Serial.println("Unknown");
      }
      break;

    default:
      // Handle unknown command or print raw data
      Serial.print("Received Unknown Command: ");
      for (int i = 0; i < PACKET_SIZE; i++) {
        Serial.print(data[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      break;
  }
}