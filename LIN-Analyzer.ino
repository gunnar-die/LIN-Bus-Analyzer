#include <Arduino.h>

// Define the baud rate for the LIN bus you are sniffing.
// Common LIN baud rates are 9600 bps or 19200 bps for VAG cars.
// IMPORTANT: This must match the actual LIN bus baud rate!
const long LIN_BAUD_RATE = 19200; 

// A timeout to determine the end of a LIN frame or bus inactivity.
// If no new byte is received within this time, the state machine resets.
// A typical bit time for 19200 baud is ~52 microseconds. 20ms is a safe margin.
const unsigned long LIN_FRAME_TIMEOUT_MS = 20; 

// Maximum number of data bytes in a LIN frame (LIN 1.3 supports up to 8 bytes).
#define LIN_MAX_DATA_BYTES 8 

// State machine for parsing LIN frames.
enum LIN_ParseState {
    STATE_IDLE,             // Waiting for a new frame (break + sync)
    STATE_BREAK_DETECTED,   // Break detected, waiting for sync byte
    STATE_SYNC_RECEIVED,    // Sync byte received, waiting for PID
    STATE_PID_RECEIVED,     // PID received, collecting data bytes
    STATE_DATA_RECEIVED,    // All expected data bytes received, waiting for checksum
    STATE_CHECKSUM_RECEIVED // Checksum received, frame complete (transient state)
};

// Variables for LIN frame parsing
volatile LIN_ParseState currentState = STATE_IDLE;
volatile unsigned long lastActivityTime = 0; // Timestamp of the last received byte
volatile uint8_t currentPID;
volatile uint8_t dataBytes[LIN_MAX_DATA_BYTES];
volatile uint8_t dataIndex = 0;
volatile uint8_t expectedDataLength = 0; // Determined from PID based on LIN 1.3 spec

// --- LIN Protocol Helper Functions ---

/**
 * @brief Calculates the parity bits for a 6-bit LIN identifier.
 * P0 = ID0 XOR ID1 XOR ID2 XOR ID4
 * P1 = ID1 XOR ID3 XOR ID4 XOR ID5
 * @param id The 6-bit LIN identifier (0-63).
 * @return The 2-bit parity (P0 in bit 6, P1 in bit 7).
 */
uint8_t lin_calculate_parity(uint8_t id) {
    uint8_t p0 = ((id >> 0) ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 0x01;
    uint8_t p1 = ((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 0x01;
    return (p0 << 6) | (p1 << 7);
}

/**
 * @brief Checks the parity of a received 8-bit Protected Identifier (PID).
 * @param pid The received 8-bit PID byte.
 * @return true if parity is correct, false otherwise.
 */
bool lin_check_pid_parity(uint8_t pid) {
    uint8_t id_field = pid & 0x3F; // Extract 6-bit ID
    uint8_t calculated_parity = lin_calculate_parity(id_field);
    uint8_t received_parity = pid & 0xC0; // Extract P0 and P1 (bits 6 and 7)
    return (calculated_parity == received_parity);
}

/**
 * @brief Calculates the LIN Classic Checksum (LIN 1.x).
 * This is the inverted 8-bit sum of data bytes.
 * @param data Pointer to the array of data bytes.
 * @param len Number of data bytes.
 * @return The calculated classic checksum.
 */
uint8_t lin_calculate_classic_checksum(const uint8_t* data, uint8_t len) {
    uint16_t sum = 0;
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum); // Invert the sum
}

/**
 * @brief Calculates the LIN Enhanced Checksum (LIN 2.0+).
 * Inverted 8-bit sum of PID and data bytes.
 * @param pid The Protected Identifier byte.
 * @param data Pointer to the array of data bytes.
 * @param len Number of data bytes.
 * @return The calculated enhanced checksum.
 */
uint8_t lin_calculate_enhanced_checksum(uint8_t pid, const uint8_t* data, uint8_t len) {
    uint16_t sum = pid; // Start sum with PID
    for (uint8_t i = 0; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum); // Invert the sum
}


/**
 * @brief Determines the expected data length for a LIN frame based on its 6-bit ID.
 * This is specific to LIN 1.x / LIN 2.0 fixed data length schedules.
 * Common LIN 1.3/2.0 data length mapping:
 * IDs 0-31: 2 data bytes
 * IDs 32-47: 4 data bytes
 * IDs 48-63: 8 data bytes
 * @param id The 6-bit LIN identifier.
 * @return The expected number of data bytes, or 0 if ID is out of standard range.
 */
uint8_t getExpectedDataLength(uint8_t id) {
    if (id >= 0x00 && id <= 0x1F) { // IDs 0-31
        return 2;
    } else if (id >= 0x20 && id <= 0x2F) { // IDs 32-47
        return 4;
    } else if (id >= 0x30 && id <= 0x3F) { // IDs 48-63
        return 8;
    }
    return 0; // Invalid or unknown ID for fixed length
}

void setup() {
    // Initialize the hardware serial port (D0/D1) for LIN bus communication.
    // This connects to the TJA1028's RXD/TXD pins.
    // NOTE: This will conflict with USB Serial if both are active simultaneously.
    // Disconnect USB to sniff LIN, reconnect to view logs.
    Serial.begin(LIN_BAUD_RATE); 

    // Print welcome message
    // You will need to reconnect USB and set Serial Monitor to LIN_BAUD_RATE
    // to see this message.
    Serial.println("\n--- LIN Bus Sniffer (Arduino Nano + TJA1028) ---");
    Serial.print("Monitoring LIN bus at ");
    Serial.print(LIN_BAUD_RATE);
    Serial.println(" bps.");
    Serial.println("Waiting for LIN messages...");
    Serial.println("Note: D0/D1 (Hardware Serial) are used for LIN. ");
    Serial.println("Disconnect USB to sniff, reconnect USB (and set monitor baud rate to 19200) to view logs.");
    Serial.println("--------------------------------------------------");
}

void loop() {
    // Check for incoming data on the LIN bus (via Serial connected to TJA1028).
    while (Serial.available()) {
        uint8_t receivedByte = Serial.read();
        lastActivityTime = millis(); // Update last activity time

        switch (currentState) {
            case STATE_IDLE:
            case STATE_BREAK_DETECTED: // After break (or assumed break), expect Sync byte 0x55
                if (receivedByte == 0x55) {
                    currentState = STATE_SYNC_RECEIVED;
                    Serial.print("\n[LIN Sniffer] New Frame (Sync 0x55) -> ");
                }
                break;

            case STATE_SYNC_RECEIVED:
                currentPID = receivedByte;
                if (lin_check_pid_parity(currentPID)) {
                    currentState = STATE_PID_RECEIVED;
                    expectedDataLength = getExpectedDataLength(currentPID & 0x3F); // Use 6-bit ID for length
                    dataIndex = 0; // Reset data byte counter for new frame

                    Serial.print("PID: 0x"); Serial.print(currentPID, HEX);
                    Serial.print(" (ID: 0x"); Serial.print(currentPID & 0x3F, HEX);
                    Serial.print(") -> Data (Len "); Serial.print(expectedDataLength);
                    Serial.print("): ");
                } else {
                    Serial.print("[LIN Sniffer] Error: PID 0x"); Serial.print(currentPID, HEX);
                    Serial.println(" parity check failed. Resetting.");
                    currentState = STATE_IDLE;
                }
                break;

            case STATE_PID_RECEIVED:
            case STATE_DATA_RECEIVED: // We are collecting data bytes
                if (dataIndex < expectedDataLength) {
                    dataBytes[dataIndex++] = receivedByte;
                    Serial.print("0x"); Serial.print(receivedByte, HEX); Serial.print(" ");

                    if (dataIndex == expectedDataLength) {
                        currentState = STATE_DATA_RECEIVED; // All data bytes collected
                    }
                } else { // This means the current byte is the checksum
                    uint8_t receivedChecksum = receivedByte;
                    
                    uint8_t calculatedClassicChecksum = lin_calculate_classic_checksum(dataBytes, expectedDataLength);
                    uint8_t calculatedEnhancedChecksum = lin_calculate_enhanced_checksum(currentPID, dataBytes, expectedDataLength);

                    Serial.print("-> Checksum: 0x"); Serial.print(receivedChecksum, HEX);

                    if (receivedChecksum == calculatedEnhancedChecksum) {
                        Serial.println(" (ENHANCED CHECKSUM OK)");
                    } else if (receivedChecksum == calculatedClassicChecksum) {
                        Serial.println(" (CLASSIC CHECKSUM OK)");
                    } else {
                        Serial.print(" (CHECKSUM FAILED! Exp Enhanced 0x"); Serial.print(calculatedEnhancedChecksum, HEX);
                        Serial.print(", Exp Classic 0x"); Serial.print(calculatedClassicChecksum, HEX); Serial.println(")");
                    }
                    currentState = STATE_IDLE; // Frame complete, reset for next frame
                }
                break;

            case STATE_CHECKSUM_RECEIVED: // This state is transient, should not be lingered in
                currentState = STATE_IDLE;
                break;
        }
    }

    // Timeout mechanism: If no new byte for a while, reset state.
    // This helps in recovering from partial frames or noise.
    if (currentState != STATE_IDLE && (millis() - lastActivityTime > LIN_FRAME_TIMEOUT_MS)) {
        Serial.println("[LIN Sniffer] Frame Timeout / Error. Resetting State.");
        currentState = STATE_IDLE;
    }
}
