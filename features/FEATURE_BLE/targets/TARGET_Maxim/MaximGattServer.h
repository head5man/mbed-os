/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */

#ifndef _MAXIM_GATT_SERVER_H_
#define _MAXIM_GATT_SERVER_H_

#include <cstdint>
#include <stddef.h>

#include "ble/blecommon.h"
#include "ble/GattServer.h"
#include "wsf_types.h"
#include "att_api.h"

/*! Maximum count of characteristics that can be stored for authorisation purposes */
#define MAX_CHARACTERISTIC_AUTHORIZATION_CNT 20

/*! client characteristic configuration descriptors settings */
#define MAX_CCCD_CNT 20

class MaximGattServer : public ble::interface::GattServer<MaximGattServer>
{
    typedef ::ble::interface::GattServer<MaximGattServer> Base;

    struct alloc_block_t {
        alloc_block_t* next;
        uint8_t data[1];
    };
public:
    static MaximGattServer &getInstance();

    ::GattServer::EventHandler* getEventHandler() {
        return eventHandler;
    }

    /* Functions that must be implemented from GattServer */
    ble_error_t addService_(GattService &);

    ble_error_t read_(GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);
    ble_error_t read_(Gap::Handle_t connectionHandle, GattAttribute::Handle_t attributeHandle, uint8_t buffer[], uint16_t *lengthP);
    ble_error_t write_(GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);
    ble_error_t write_(Gap::Handle_t connectionHandle, GattAttribute::Handle_t, const uint8_t[], uint16_t, bool localOnly = false);

    ble_error_t areUpdatesEnabled_(const GattCharacteristic &characteristic, bool *enabledP);
    ble_error_t areUpdatesEnabled_(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabledP);

    bool isOnDataReadAvailable_() const { return true; }

        /**
     * @see ::GattServer::getPreferredConnectionParams
     */
    ::Gap::ConnectionParams_t getPreferredConnectionParams();

    /**
     * @see ::GattServer::setPreferredConnectionParams
     */
    void setPreferredConnectionParams(const ::Gap::ConnectionParams_t& params);

    /**
     * @see ::GattServer::setDeviceName
     */
    ble_error_t setDeviceName(const uint8_t *deviceName);

    /**
     * @see ::GattServer::getDeviceName
     */
    void getDeviceName(const uint8_t*& name, uint16_t& length);

    /**
     * @see ::GattServer::setAppearance
     */
    void setAppearance(GapAdvertisingData::Appearance appearance);

    /**
     * @see ::GattServer::getAppearance
     */
    GapAdvertisingData::Appearance getAppearance();

    /**
     * @see ::GattServer::reset
     */
    ble_error_t reset_(void);

private:
    static void cccCback(attsCccEvt_t *pEvt);
    static void attCback(attEvt_t *pEvt);
    static uint8_t attsReadCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset, attsAttr_t *pAttr);
    static uint8_t attsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr);
    static uint8_t attsAuthCback(dmConnId_t connId, uint8_t permit, uint16_t handle);

    void add_generic_access_service();
    void add_generic_attribute_service();

    void add_default_services();
    uint16_t compute_attributes_count(GattService &service);
    void insert_service_attribute(GattService& service, attsAttr_t *&attribute_it);
    void* alloc_block(size_t block_size);
    ble_error_t insert_characteristic(GattCharacteristic *characteristic, attsAttr_t *&attribute_it);
    ble_error_t insert_characteristic_value_attribute(GattCharacteristic *characteristic, attsAttr_t *&attribute_it);
    void insert_characteristic_declaration_attribute(GattCharacteristic *characteristic, attsAttr_t *&attribute_it);
    bool is_characteristic_valid(GattCharacteristic *characteristic);
    ble_error_t insert_descriptor(GattCharacteristic *characteristic, GattAttribute* descriptor, attsAttr_t *&attribute_it, bool& cccd_created);
    ble_error_t insert_cccd(GattCharacteristic *characteristic, attsAttr_t *&attribute_it);

    GattCharacteristic* get_auth_char(uint16_t value_handle);
    bool get_cccd_index_by_cccd_handle(GattAttribute::Handle_t cccd_handle, uint8_t& idx) const;
    bool get_cccd_index_by_value_handle(GattAttribute::Handle_t char_handle, uint8_t& idx) const;
    bool is_update_authorized(ble::connection_handle_t connection, GattAttribute::Handle_t value_handle);

    void* _signing_event_handler;

    /*! client characteristic configuration descriptors settings */
    #define MAX_CCC_CNT 20
    attsCccSet_t cccds[MAX_CCC_CNT];
    uint16_t cccd_values[MAX_CCC_CNT];
    uint16_t cccd_handles[MAX_CCC_CNT];
    uint8_t cccd_cnt;

    GattCharacteristic *_auth_char[MAX_CHARACTERISTIC_AUTHORIZATION_CNT];
    uint8_t _auth_char_count{0};
    bool default_services_added;
    uint16_t currentHandle;
    void* registered_service;
    alloc_block_t* allocated_blocks{NULL};

    struct {
        attsGroup_t service;
        attsAttr_t attributes[7];
        uint8_t device_name_declaration_value[5];
        uint16_t device_name_length;
        uint8_t appearance_declaration_value[5];
        uint16_t appearance;
        uint8_t ppcp_declaration_value[5];
        uint8_t ppcp[8];

        uint8_t*& device_name_value() {
            return attributes[2].pValue;
        }
    } generic_access_service;

    struct {
        attsGroup_t service;
        attsAttr_t attributes[4];
        uint8_t service_changed_declaration[5];
    } generic_attribute_service;

private:
  MaximGattServer();

  MaximGattServer(const MaximGattServer &);
  const MaximGattServer &operator=(const MaximGattServer &);
};

#endif /* _MAXIM_GATT_SERVER_H_ */
