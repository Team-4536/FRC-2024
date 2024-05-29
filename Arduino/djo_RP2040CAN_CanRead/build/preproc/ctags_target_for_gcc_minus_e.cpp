# 1 "C:\\Repos\\FRC-2024\\Arduino\\djo_RP2040CAN_CanRead\\djo_RP2040CAN_CanRead.ino"
/*

 * Adafruit MCP2515 FeatherWing CAN Receiver Example

 */
# 5 "C:\\Repos\\FRC-2024\\Arduino\\djo_RP2040CAN_CanRead\\djo_RP2040CAN_CanRead.ino"
# 6 "C:\\Repos\\FRC-2024\\Arduino\\djo_RP2040CAN_CanRead\\djo_RP2040CAN_CanRead.ino" 2
# 30 "C:\\Repos\\FRC-2024\\Arduino\\djo_RP2040CAN_CanRead\\djo_RP2040CAN_CanRead.ino"
// Set CAN bus baud rate


Adafruit_MCP2515 mcp((19u));

void setup() {
  delay(3000);
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Receiver test!");

  if (!mcp.begin((1000000))) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");
}

void loop() {
  // try to parse packet
  int packetSize = mcp.parsePacket();

  static long int lastTime = 0;
  if(millis() > lastTime + 1000) {
 Serial.print("no packet rx time: ");
 Serial.println(millis());
 lastTime = millis();
  }

  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (mcp.packetExtended()) {
      Serial.print("extended ");
    }

    if (mcp.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(mcp.packetId(), 16);

    if (mcp.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(mcp.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (mcp.available()) {
        Serial.print((char)mcp.read());
      }
      Serial.println();
    }

    Serial.println();
  }
}
