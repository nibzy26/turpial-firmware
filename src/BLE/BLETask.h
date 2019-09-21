/**
 * @file BLE.h
 * @author Locha Mesh project developers (locha.io)
 * @brief 
 * @version 0.1.1
 * @date 2019-08-15
 * 
 * @copyright Copyright (c) 2019 Locha Mesh project developers
 * @license Apache 2.0, see LICENSE file for details
 * 
 */

#ifndef BLE_H
#define BLE_H

#include "Arduino.h"
#include <string>



class BLEModule 
{
    public:
        /**
         * @brief Construct a new BLE object
         * 
         */
        //BLE();

        /**
         * @brief Destroy the BLE object
         * 
         */
        //~BLE();

        /**
         * @Start the BLE functionality
         * 
         * @return esp_err_t
         */
        void BLE_task(void *params);
        /**
         * @Brief check of BLE at boot time
         * 
         * @return esp_err_t
         */
        esp_err_t init();

    private:
};   


#endif // BLE_H
