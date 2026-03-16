/*
 * BLE MTU Finder Test
 *
 * This test finds the maximum chunk size that works for BLE notifications.
 * It tries different sizes and reports which ones succeed.
 */

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Same UUIDs as TinkerRocket
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define COMMAND_CHAR_UUID   "cba1d466-344c-4be3-ab3f-189f80dd7518"
#define FILE_OPS_UUID       "8d53dc1d-1db7-4cd3-868b-8a527460aa84"
#define FILE_TRANSFER_UUID  "1a2b3c4d-5e6f-7a8b-9c0d-1e2f3a4b5c6d"

BLEServer* pServer = nullptr;
BLECharacteristic* pCommandChar = nullptr;
BLECharacteristic* pFileOpsChar = nullptr;
BLECharacteristic* pFileTransferChar = nullptr;
bool deviceConnected = false;
bool startTest = false;

class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        deviceConnected = true;
        Serial.println("Device connected!");

        // Get MTU size
        uint16_t mtu = pServer->getPeerMTU(pServer->getConnId());
        Serial.print("Negotiated MTU: ");
        Serial.println(mtu);
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

            if (cmd == 2) {
                // Send fake file list
                const char* fakeFileList = "[{\"name\":\"mtu_test.bin\",\"size\":1000}]";
                pFileOpsChar->setValue((uint8_t*)fakeFileList, strlen(fakeFileList));
                pFileOpsChar->notify();
            }
            else if (cmd == 4) {
                Serial.println("Starting MTU test...");
                startTest = true;
            }
        }
    }
};

void setup() {
    Serial.begin(115200);
    Serial.println("BLE MTU Finder Test");

    BLEDevice::init("TinkerRocket");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);

    pCommandChar = pService->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
    );
    pCommandChar->setCallbacks(new CommandCallbacks());

    pFileOpsChar = pService->createCharacteristic(
        FILE_OPS_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pFileOpsChar->addDescriptor(new BLE2902());

    pFileTransferChar = pService->createCharacteristic(
        FILE_TRANSFER_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
    );
    pFileTransferChar->addDescriptor(new BLE2902());

    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);
    BLEDevice::startAdvertising();

    Serial.println("BLE server started, connect and download to start test...");
}

void sendTestChunk(size_t dataSize) {
    // Build packet: [offset(4)][length(2)][flags(1)][data(N)]
    size_t packetSize = 7 + dataSize;
    uint8_t* packet = new uint8_t[packetSize];

    // Offset
    packet[0] = 0x00;
    packet[1] = 0x00;
    packet[2] = 0x00;
    packet[3] = 0x00;

    // Length
    packet[4] = (dataSize >> 0) & 0xFF;
    packet[5] = (dataSize >> 8) & 0xFF;

    // Flags (EOF)
    packet[6] = 0x01;

    // Fill data with test pattern
    for (size_t i = 0; i < dataSize; i++) {
        packet[7 + i] = (i & 0xFF);
    }

    // Try to send
    pFileTransferChar->setValue(packet, packetSize);
    pFileTransferChar->notify();

    delete[] packet;
}

void runMTUTest() {
    Serial.println("\n=== MTU Size Test ===\n");

    // Test different sizes to find the limit
    size_t testSizes[] = {
        20,   // Known to work (from earlier tests)
        50,
        100,
        150,
        200,
        250,
        300,
        350,
        400,
        450,
        500
    };

    for (size_t i = 0; i < sizeof(testSizes) / sizeof(testSizes[0]); i++) {
        size_t dataSize = testSizes[i];
        size_t totalSize = 7 + dataSize;  // header + data

        Serial.print("Testing chunk size: ");
        Serial.print(dataSize);
        Serial.print(" bytes (total packet: ");
        Serial.print(totalSize);
        Serial.print(" bytes)... ");

        sendTestChunk(dataSize);
        delay(100);  // Give iOS time to process

        Serial.println("sent");
    }

    Serial.println("\n=== Test Complete ===");
    Serial.println("Check iOS logs to see which sizes were received!");
    Serial.println("The largest successful size is your MTU limit.\n");
}

void loop() {
    if (deviceConnected && startTest) {
        startTest = false;
        runMTUTest();
    }

    delay(1000);
}
