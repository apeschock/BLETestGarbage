#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <BLEClient.h>


byte flags = 0b00111110;
byte bpm;
byte heart[8] = { 0b00001110, 60, 0, 0, 0 , 0, 0, 0 };
byte hrmPos[1] = { 2 };

bool _BLEClientConnected = false;

#define heartRateService BLEUUID((uint16_t)0x180D)
BLECharacteristic heartRateMeasurementCharacteristics(BLEUUID((uint16_t)0x2A37), BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic sensorPositionCharacteristic(BLEUUID((uint16_t)0x2A38), BLECharacteristic::PROPERTY_READ);
BLEDescriptor heartRateDescriptor(BLEUUID((uint16_t)0x2901));
BLEDescriptor sensorPositionDescriptor(BLEUUID((uint16_t)0x2901));

BLEServer* pServer;

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
        _BLEClientConnected = true;
        param->connect.remote_bda
        param->gatts_read_evt_param.bda
    };

    void onDisconnect(BLEServer* pServer) {
        _BLEClientConnected = false;
    }
};

void InitBLE() {
    BLEDevice::init("FT7");
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    // Create the BLE Service
    BLEService* pHeart = pServer->createService(heartRateService);

    pHeart->addCharacteristic(&heartRateMeasurementCharacteristics);
    heartRateDescriptor.setValue("Rate from 0 to 200");
    heartRateMeasurementCharacteristics.addDescriptor(&heartRateDescriptor);
    heartRateMeasurementCharacteristics.addDescriptor(new BLE2902());

    pHeart->addCharacteristic(&sensorPositionCharacteristic);
    sensorPositionDescriptor.setValue("Position 0 - 6");
    sensorPositionCharacteristic.addDescriptor(&sensorPositionDescriptor);

    pServer->getAdvertising()->addServiceUUID(heartRateService);

    pHeart->start();
    // Start advertising
    pServer->getAdvertising()->start();

}


void setup() {
    Serial.begin(115200);
    Serial.println("Start");
    InitBLE();
    bpm = 1;
}

void loop() {
    // put your main code here, to run repeatedly:

    heart[1] = (byte)bpm;
    int energyUsed = 3000;
    heart[3] = energyUsed / 256;
    heart[2] = energyUsed - (heart[2] * 256);
    Serial.println(bpm);

    heartRateMeasurementCharacteristics.setValue(heart, 8);
    heartRateMeasurementCharacteristics.notify();

    sensorPositionCharacteristic.setValue(hrmPos, 1);
    bpm++;

    delay(2000);

    if (pServer->getConnectedCount() >= 1) {
        std::map<uint16_t, conn_status_t> map = pServer->getPeerDevices(false);
        std::vector<esp_bd_addr_t> keys;
        for (auto& it : map) {
            keys.push_back(it.first);
        }

        getRssi(keys[0].);
    }
    
}

float getRssi(esp_bd_addr_t remote_addr) {
    esp_err_t rc = ::esp_ble_gap_read_rssi(remote_addr);
    if (rc != ESP_OK) {
        ESP_LOGE(LOG_TAG, "<< getRssi: esp_ble_gap_read_rssi: rc=%d %s", rc, GeneralUtils::errorToString(rc));
        return 0;
    }
    int rssiValue = m_semaphoreRssiCmplEvt.wait("getRssi");
    return rssiValue;
}

static void gapEventHandler(esp_gap_ble_cb_event_t  event, esp_ble_gap_cb_param_t* param) {
    if(event == ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT){
        param->read_rssi_cmpl.status;
        param->read_rssi_cmpl.rssi;
        BLEAddress(param->read_rssi_cmpl.remote_addr).toString().c_str();
    }
}