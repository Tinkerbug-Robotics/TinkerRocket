/*
 * BLE Multi-Notification Test
 *
 * This minimal sketch tests if multiple BLE notifications work correctly.
 * It sends 10 notifications with incrementing data to see if iOS receives them all.
 *
 * Expected behavior:
 *   - iOS should receive 10 chunks with different data
 *   - Each chunk should have offset=0,512,1024,... and different payload
 *
 * If iOS receives 0 bytes or the same data for all notifications,
 * it indicates a NimBLE or iOS CoreBluetooth bug.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Same UUIDs as TinkerRocket so you can use existing iOS app
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_CHAR_UUID   "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define FILE_OPS_UUID       "8d53dc1d-1db7-4cd3-868b-8a527460aa84"  // For file lists (JSON)
#define FILE_TRANSFER_UUID  "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"  // For file chunks (binary)

BLEServer* pServer = nullptr;
BLECharacteristic* pCommandChar = nullptr;
BLECharacteristic* pFileOpsChar = nullptr;         // Sends file lists
BLECharacteristic* pFileTransferChar = nullptr;    // Sends file chunks
bool deviceConnected = false;
bool startTest = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected!");
    }

    void onDisconnect(BLEServer* pServer) {
        deviceConnected = false;
        Serial.println("Device disconnected!");
        BLEDevice::startAdvertising();
    }
};

class CommandCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        uint8_t* data = pCharacteristic->getData();
        size_t len = pCharacteristic->getValue().length();

        if (len > 0) {
            uint8_t cmd = data[0];
            Serial.print("Received command: ");
            Serial.println(cmd);

            // Command 2 = request file list
            if (cmd == 2) {
                Serial.println("Sending fake file list...");
                // Send a fake file list with one test file (128KB)
                const char* fakeFileList = "[{\"name\":\"test.bin\",\"size\":128000}]";
                pFileOpsChar->setValue((uint8_t*)fakeFileList, strlen(fakeFileList));
                pFileOpsChar->notify();
            }
            // Command 4 = start download/test
            else if (cmd == 4) {
                Serial.println("Starting multi-notification test...");
                startTest = true;
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("BLE Multi-Notification Test");

    // Initialize BLE
    BLEDevice::init("TinkerRocket");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    // Create service
    BLEService *pService = pServer->createService(SERVICE_UUID);

    // Create command characteristic (for receiving download request)
    pCommandChar = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandChar->setCallbacks(new CommandCallbacks());

    // Create file ops characteristic (for sending file lists)
    pFileOpsChar = pService->createCharacteristic(
        FILE_OPS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pFileOpsChar->addDescriptor(new BLE2902());

    // Create file transfer characteristic (for sending file chunks)
    pFileTransferChar = pService->createCharacteristic(
        FILE_TRANSFER_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pFileTransferChar->addDescriptor(new BLE2902());

    // Start service
    pService->start();

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 7.5ms
    BLEDevice::startAdvertising();

    Serial.println("BLE server started, waiting for connection...");
}

void sendTestChunk(uint32_t offset, const uint8_t* data, size_t len, bool eof) {
    // Build chunk packet: [offset(4)][length(2)][flags(1)][data(N)]
    size_t packetSize = 7 + len;
    uint8_t* packet = new uint8_t[packetSize];

    // Offset (4 bytes, little-endian)
    packet[0] = (offset >> 0) & 0xFF;
    packet[1] = (offset >> 8) & 0xFF;
    packet[2] = (offset >> 16) & 0xFF;
    packet[3] = (offset >> 24) & 0xFF;

    // Length (2 bytes, little-endian)
    packet[4] = (len >> 0) & 0xFF;
    packet[5] = (len >> 8) & 0xFF;

    // Flags (1 byte)
    packet[6] = eof ? 0x01 : 0x00;

    // Data
    if (len > 0 && data != nullptr) {
        memcpy(packet + 7, data, len);
    }

    // Send via BLE
    pFileTransferChar->setValue(packet, packetSize);
    pFileTransferChar->notify();

    delete[] packet;
}

void runTest() {
    Serial.println("\n=== Starting Test ===");
    Serial.println("Sending 250 chunks of 512 bytes each (128KB total)...\n");

    // Create 512-byte test buffer with unique pattern for each chunk
    const size_t CHUNK_SIZE = 512;
    uint8_t chunk_buffer[CHUNK_SIZE];

    // Send 250 chunks of 512 bytes each (simulating a real 128KB file)
    for (int i = 0; i < 250; i++) {
        // Fill buffer with unique pattern for this chunk
        // Pattern: chunk number repeated, makes it easy to verify on iOS
        for (size_t j = 0; j < CHUNK_SIZE; j++) {
            chunk_buffer[j] = (i & 0xFF);  // Use chunk number as fill byte
        }

        // Add chunk number at start for easy identification
        chunk_buffer[0] = (i >> 0) & 0xFF;
        chunk_buffer[1] = (i >> 8) & 0xFF;

        uint32_t offset = i * 512;
        sendTestChunk(offset, chunk_buffer, CHUNK_SIZE, false);

        delay(15);  // Same delay as main code

        // Print progress every 50 chunks
        if ((i + 1) % 50 == 0) {
            Serial.print("Progress: ");
            Serial.print(i + 1);
            Serial.print(" chunks sent (");
            Serial.print((i + 1) * 512);
            Serial.println(" bytes)");
        }
    }

    // Send EOF chunk
    sendTestChunk(250 * 512, nullptr, 0, true);
    Serial.println("\nSent EOF chunk");

    Serial.println("=== Test Complete ===\n");
    Serial.print("Total sent: ");
    Serial.print(250 * 512);
    Serial.println(" bytes (128000 bytes)");
    Serial.println("Check iOS app - should show 128KB file!");
    Serial.println("If download fails or shows wrong size, we found the bug!\n");
}

void loop() {
    if (deviceConnected && startTest) {
        startTest = false;
        runTest();
    }

    delay(1000);
}
