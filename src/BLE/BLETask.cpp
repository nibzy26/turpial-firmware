/**
 * @file BLE.cpp
 * @author Locha Mesh project developers (locha.io)
 * @brief 
 * @version 0.1.1
 * @date 2019-08-15
 * 
 * @copyright Copyright (c) 2019 Locha Mesh project developers
 * @license Apache 2.0, see LICENSE file for details
 * 
 */
//#if defined(CONFIG_BT_ENABLED)
#ifndef ARDUINO_ARCH_ESP32
#define ARDUINO_ARCH_ESP32
#endif
#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/task.h>
#include <esp_err.h>
#include <nvs_flash.h>
#include <esp_bt.h>            // ESP32 BLE
#include <esp_bt_device.h>     // ESP32 BLE
#include <esp_bt_main.h>       // ESP32 BLE
#include <esp_gap_ble_api.h>   // ESP32 BLE
#include <esp_gatts_api.h>     // ESP32 BLE
#include <esp_gattc_api.h>     // ESP32 BLE
#include <esp_gatt_common_api.h>// ESP32 BLE
#include <esp_err.h>           // ESP32 ESP-IDF
//#include <map>                 // Part of C++ Standard library
//#include <sstream>             // Part of C++ Standard library
//#include <iomanip>             // Part of C++ Standard library


#include <BLEServer.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BLETask.h"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32-hal-bt.h"
#endif
/*
#if defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#define LOG_TAG ""
#else
#include "esp_log.h"
static const char* LOG_TAG = "BLEDevice";
#endif
*/
BLEServer *ble_server = NULL;
BLECharacteristic *tx_uart;
BLECharacteristic *rx_uart;

std::string txValue_uart;
std::string rxValue_uart;

uint16_t server_mtu = 255;

std::string server_name = "subid.locha.io";

std::string UUID_UART = "";
std::string TX_UART = "";
std::string RX_UART = "";
bool deviceConnected = false;
bool BLEinitialized     = false; 
class ServerCB : public BLEServerCallbacks
{
    void onConnect(BLEServer *ble_server)
    {
        deviceConnected = true;
        ESP_LOGD("BLE", "Client connected");
    }
    void onDisconnect(BLEServer *ble_server)
    {
        deviceConnected = false;
        ESP_LOGD("BLE", "Client disconnected");
        delay(100); // was 500
        ble_server->startAdvertising();
    }
};

class characteristicCB : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        // determinar *pCharacteristic
    }
    void onRead(BLECharacteristic *pCharacteristic)
    {
        // determinar *pCharacteristic
    }
};

void BLEModule::BLE_task(void *params)
{

    BLEDevice::setMTU(server_mtu);
    BLEDevice::init(server_name);

    ble_server = BLEDevice::createServer();
    ble_server->setCallbacks(new ServerCB());

    BLEService *server_service = ble_server->createService(UUID_UART);

    while (1)
    {
        if (deviceConnected)
        {
            if(txValue_uart.size() > 0) {
                // procesar tx_uart
            }
        }
        else
        {
            if(txValue_uart.size() > 0) {
                // aqui no ponemos nada, pues es temporal el uso de ble.
            }
        }
    }
}

esp_err_t BLEModule::init()
 {
    // iniciar servidor ble.
    esp_err_t errRc;
#ifdef ARDUINO_ARCH_ESP32
    if (!btStart()) {
        errRc = ESP_FAIL;
        return errRc;
    }
#else
    errRc = ::nvs_flash_init();
    if (errRc != ESP_OK) {
        ESP_LOGE(__func__, "nvs_flash_init: rc=%d %s", errRc, esp_err_to_name(errRc));
        return errRc;
    }

#ifndef CLASSIC_BT_ENABLED
    esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);  
#endif
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    errRc = esp_bt_controller_init(&bt_cfg);
    if (errRc != ESP_OK) {
        ESP_LOGE(__func__, "esp_bt_controller_init: rc=%d %s", errRc, esp_err_to_name(errRc));
        return errRc;
		}

#ifndef CLASSIC_BT_ENABLED
    errRc = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (errRc != ESP_OK) {
        ESP_LOGE(__func__, "esp_bt_controller_enable: rc=%d %s", errRc,esp_err_to_name(errRc));
        return errRc;
    }
#else
    errRc = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (errRc != ESP_OK) {
        ESP_LOGE(__func__, "esp_bt_controller_enable: rc=%d %s", errRc,errRc,esp_err_to_name(errRc));
        return errRc;
    }
#endif
#endif

    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
    if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        errRc = esp_bluedroid_init();
        if (errRc != ESP_OK) {
            ESP_LOGE(__func__, "esp_bluedroid_init: rc=%d %s", errRc, errRc,esp_err_to_name(errRc));
            return errRc;
        }
    }

    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
        errRc = esp_bluedroid_enable();
        if (errRc != ESP_OK) {
            ESP_LOGE(__func__, "esp_bluedroid_enable: rc=%d %s", errRc, errRc,esp_err_to_name(errRc));
            return errRc;
        }
    }

	BLEinitialized = true;	
    return ESP_OK;

    

}
//#endif // CONFIG_BT_ENABLED