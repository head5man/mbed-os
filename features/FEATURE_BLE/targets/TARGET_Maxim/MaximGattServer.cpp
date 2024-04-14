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

#include "MaximBLE.h"
#include "MaximGattServer.h"
#include "mbed.h"
#include "MaximGap.h"
#include "wsf_types.h"
#include "att_api.h"
#include <cstdint>

using att_security_requirement_t = ble::att_security_requirement_t;

static UUID CCCD_UUID(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG);
static const uint16_t CCCD_SIZE = sizeof(uint16_t);

static const unsigned int READ_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ;
static const unsigned int WRITE_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE;
static const unsigned int WRITE_WITHOUT_RESPONSE_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE_WITHOUT_RESPONSE;
static const unsigned int SIGNED_WRITE_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_AUTHENTICATED_SIGNED_WRITES;
static const unsigned int NOTIFY_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY;
static const unsigned int INDICATE_PROPERTY =
    GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE;

static const uint8_t WRITABLE_PROPERTIES =
    WRITE_PROPERTY |
    WRITE_WITHOUT_RESPONSE_PROPERTY |
    SIGNED_WRITE_PROPERTY;

static const uint8_t UPDATE_PROPERTIES =
    NOTIFY_PROPERTY |
    INDICATE_PROPERTY;

static const uint16_t CONNECTION_ID_LIMIT = 0x100;

typedef struct mxmChar_s {
    uint16_t descLen;
    mxmChar_s() {}
} mxmChar_t;

typedef struct mxmService_s mxmService_t;
struct mxmService_s {
    uint16_t uuidLen;
    mxmChar_t *chars;
    attsGroup_t attGroup;
    mxmService_t *next;
    mxmService_s() {}
};

static uint16_t currentHandle = 0x20;

static UUID cccUUID(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG);
static const uint16_t cccSize = sizeof(uint16_t);

MaximGattServer &MaximGattServer::getInstance() {
    static MaximGattServer m_instance;
    return m_instance;
}

void MaximGattServer::add_default_services()
{
    if (!default_services_added) {
        default_services_added = true;
        add_generic_access_service();
        add_generic_attribute_service();
    }
}

uint16_t MaximGattServer::compute_attributes_count(GattService& service)
{
    // start at 1, one attribute is required for the service itself
    uint16_t attributes_count = 1;
    for (int i = 0; i < service.getCharacteristicCount(); i++) {
        attributes_count += 2;
        GattCharacteristic *p_char = service.getCharacteristic(i);
        attributes_count += p_char->getDescriptorCount();
        if (p_char->getProperties() & UPDATE_PROPERTIES) {
            // add a CCCD
            ++attributes_count;

            // verify that it hasn't been counted twice
            for (size_t j = 0; j < p_char->getDescriptorCount(); ++j) {
                if (p_char->getDescriptor(j)->getUUID() ==
                    UUID(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)
                ) {
                    --attributes_count;
                    break;
                }
            }
        }
    }

    return attributes_count;
}

ble_error_t MaximGattServer::addService_(GattService &service)
{
    add_default_services();

    mxmService_t *mxmSvc = new mxmService_t;
    mxmSvc->attGroup.pNext = NULL;
    mxmSvc->attGroup.readCback = attsReadCback;
    mxmSvc->attGroup.writeCback = attsWriteCback;

    // Determine the attribute list length
    unsigned int attListLen = compute_attributes_count(service);

    // Create WiCentric attribute list
    
    mxmSvc->attGroup.pAttr = (attsAttr_t*)alloc_block(attListLen * sizeof(attsAttr_t));

    if (mxmSvc->attGroup.pAttr == NULL) {
        delete mxmSvc;
        return BLE_ERROR_BUFFER_OVERFLOW;
    }

    attsAttr_t *attribute_it = mxmSvc->attGroup.pAttr;

    /* service */
    insert_service_attribute(service, attribute_it);
    mxmSvc->attGroup.startHandle = currentHandle;
    service.setHandle(currentHandle);

    /* Add characteristics to the service */
    for (int i = 0; i < service.getCharacteristicCount(); i++) {
        ble_error_t err = insert_characteristic(
            service.getCharacteristic(i),
            attribute_it
        );
        if (err) {
            // FIXME: proper cleanup of data structure:
            //   - att_service->attGroup.pAttr
            //   - blocks allocated for characteristics value
            // NOTE: those are rightfully released when reset() is called.
            delete mxmSvc;
            return err;
        }
    }
    mxmSvc->attGroup.endHandle = currentHandle;

    mxmSvc->next = NULL;
    if (registered_service)
    {
        mxmSvc->next = (mxmService_t *)registered_service;
    }

    registered_service = mxmSvc;

    AttsAddGroup(&mxmSvc->attGroup);
    AttsCccRegister(cccd_cnt, (attsCccSet_t*)cccds, cccCback);
    return BLE_ERROR_NONE;

#if false
    // Create characteristics
    mxmSvc->chars = new mxmChar_t [service.getCharacteristicCount()];

    /* Add characteristics to the service */
    for (int i = 0; i < service.getCharacteristicCount(); i++) {
        GattCharacteristic *p_char = service.getCharacteristic(i);

        /* Skip any incompletely defined, read-only characteristics. */
        if ((p_char->getValueAttribute().getValuePtr() == NULL) &&
            (p_char->getValueAttribute().getLength() == 0) &&
            (p_char->getProperties() == GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)) {
            continue;
        }

        // Create Characteristic Attribute
        currentHandle += 2;
        currAtt->pUuid = attChUuid;

        p_char->getValueAttribute().setHandle(currentHandle);
        mxmSvc->chars[i].descLen = 1 + sizeof(currentHandle) + p_char->getValueAttribute().getUUID().getLen();
        currAtt->pValue = (uint8_t*)malloc(mxmSvc->chars[i].descLen);
        uint8_t *pValue = currAtt->pValue;
        *pValue++ = p_char->getProperties();
        memcpy(pValue, &currentHandle, sizeof(currentHandle));
        pValue += sizeof(currentHandle);
        memcpy(pValue, p_char->getValueAttribute().getUUID().getBaseUUID(), p_char->getValueAttribute().getUUID().getLen());

        currAtt->pLen = &mxmSvc->chars[i].descLen;
        currAtt->maxLen = mxmSvc->chars[i].descLen;
        currAtt->settings = 0;
        currAtt->permissions = ATTS_PERMIT_READ;
        currAtt++;

        // Create Value Attribute
        currAtt->pUuid = p_char->getValueAttribute().getUUID().getBaseUUID();
        currAtt->pValue = p_char->getValueAttribute().getValuePtr();
        currAtt->pLen = p_char->getValueAttribute().getLengthPtr();
        currAtt->maxLen = p_char->getValueAttribute().getMaxLength();
        currAtt->settings = ATTS_SET_WRITE_CBACK | ATTS_SET_READ_CBACK;

        if(p_char->getValueAttribute().hasVariableLength()) {
            currAtt->settings |= ATTS_SET_VARIABLE_LEN;
        }

        if (p_char->getValueAttribute().getUUID().shortOrLong() == UUID::UUID_TYPE_LONG) {
            currAtt->settings |= ATTS_SET_UUID_128;
        }
        currAtt->permissions = 0;
        if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ)  { currAtt->permissions |= ATTS_PERMIT_READ; }
        if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE) { currAtt->permissions |= ATTS_PERMIT_WRITE; }
        currAtt++;

        bool cccCreated = false;

        for (int i = 0; i < p_char->getDescriptorCount(); i++) {
            GattAttribute *p_att = p_char->getDescriptor(i);

            currentHandle++;

            currAtt->pUuid = p_att->getUUID().getBaseUUID();
            currAtt->pValue = p_att->getValuePtr();
            currAtt->pLen = p_att->getLengthPtr();
            currAtt->maxLen = p_att->getMaxLength();
            currAtt->settings = 0;
            currAtt->permissions = 0;
            if (p_att->getUUID().shortOrLong() == UUID::UUID_TYPE_LONG) {
                currAtt->settings |= ATTS_SET_UUID_128;
            }
            if (p_att->getUUID() == UUID(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)) {
                cccCreated = true;
                currAtt->settings |= ATTS_SET_CCC;
                currAtt->permissions |= ATTS_PERMIT_READ;
                currAtt->permissions |= ATTS_PERMIT_WRITE;

                if (cccCnt < MAX_CCC_CNT) {
                    cccSet[cccCnt].handle = currentHandle;
                    cccSet[cccCnt].valueRange = 0;
                    if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) {
                        cccSet[cccCnt].valueRange |= ATT_CLIENT_CFG_NOTIFY;
                    }
                    if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE) {
                        cccSet[cccCnt].valueRange |= ATT_CLIENT_CFG_INDICATE;
                    }
                    cccSet[cccCnt].secLevel = DM_SEC_LEVEL_NONE;
                    cccHandles[cccCnt] = p_char->getValueAttribute().getHandle();
                    cccCnt++;
                } else {
                    return BLE_ERROR_PARAM_OUT_OF_RANGE;
                }
            }
            currAtt++;
        }

        if (!cccCreated && (p_char->getProperties() & (GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY | GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE))) {
            /* There was not a CCCD included in the descriptors, but this
             * characteristic is notifiable and/or indicatable. A CCCD is
             * required so create one now.
             */
            if (cccCnt >= MAX_CCC_CNT) {
                return BLE_ERROR_PARAM_OUT_OF_RANGE;
            }

            currentHandle++;

            currAtt->pUuid = cccUUID.getBaseUUID();
            currAtt->pValue = (uint8_t*)&cccValues[cccCnt];
            currAtt->pLen = (uint16_t*)&cccSize;
            currAtt->maxLen = sizeof(uint16_t);
            currAtt->settings = ATTS_SET_CCC;
            currAtt->permissions = (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE);

            cccSet[cccCnt].handle = currentHandle;
            cccSet[cccCnt].valueRange = 0;
            if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY) {
                cccSet[cccCnt].valueRange |= ATT_CLIENT_CFG_NOTIFY;
            }
            if (p_char->getProperties() & GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE) {
                cccSet[cccCnt].valueRange |= ATT_CLIENT_CFG_INDICATE;
            }
            cccSet[cccCnt].secLevel = DM_SEC_LEVEL_NONE;
            cccHandles[cccCnt] = p_char->getValueAttribute().getHandle();

            cccCnt++;
            currAtt++;
        }
    }

    mxmSvc->attGroup->pNext = NULL;
    mxmSvc->attGroup->readCback = attsReadCback;
    mxmSvc->attGroup->writeCback = attsWriteCback;
    mxmSvc->attGroup->startHandle = startHandle;
    mxmSvc->attGroup->endHandle = currentHandle;
    AttsAddGroup(mxmSvc->attGroup);

    AttRegister(attCback);
    AttsCccRegister(cccCnt, (attsCccSet_t*)cccSet, cccCback);

    return BLE_ERROR_NONE;
#endif
}

void MaximGattServer::add_generic_access_service()
{
    ++currentHandle;
    generic_access_service.service.pNext = NULL;
    generic_access_service.service.startHandle = currentHandle;
    generic_access_service.service.readCback = attsReadCback;
    generic_access_service.service.writeCback = attsWriteCback;

    // bind attributes to the service
    generic_access_service.service.pAttr = generic_access_service.attributes;

    attsAttr_t* current_attribute = generic_access_service.attributes;

    // service attribute
    current_attribute->pUuid = attPrimSvcUuid;
    current_attribute->pValue = (uint8_t*) attGapSvcUuid;
    current_attribute->maxLen = sizeof(attGapSvcUuid);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // device name declaration
    currentHandle += 2; // note: incremented by two to get a pointer to the value handle
    ++current_attribute;

    // set properties
    generic_access_service.device_name_declaration_value[0] = ATT_PROP_READ;
    // set value handle
    memcpy(generic_access_service.device_name_declaration_value + 1, &currentHandle, sizeof(currentHandle));
    // set the characteristic UUID
    memcpy(generic_access_service.device_name_declaration_value + 3, attDnChUuid, sizeof(attDnChUuid));

    current_attribute->pUuid = attChUuid;
    current_attribute->pValue = generic_access_service.device_name_declaration_value;
    current_attribute->maxLen = sizeof(generic_access_service.device_name_declaration_value);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // device name value
    ++current_attribute;
    generic_access_service.device_name_length = 0;
    current_attribute->pUuid = attDnChUuid;
    current_attribute->maxLen = 248;
    current_attribute->pLen = &generic_access_service.device_name_length;
    current_attribute->pValue = NULL;
    current_attribute->settings = ATTS_SET_VARIABLE_LEN;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // appearance declaration
    currentHandle += 2; // note: incremented by two to get a pointer to the value handle
    ++current_attribute;

    // set properties
    generic_access_service.appearance_declaration_value[0] = ATT_PROP_READ;
    // set value handle
    memcpy(generic_access_service.appearance_declaration_value + 1, &currentHandle, sizeof(currentHandle));
    // set the characteristic UUID
    memcpy(generic_access_service.appearance_declaration_value + 3, attApChUuid, sizeof(attApChUuid));

    current_attribute->pUuid = attChUuid;
    current_attribute->pValue = generic_access_service.appearance_declaration_value;
    current_attribute->maxLen = sizeof(generic_access_service.appearance_declaration_value);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // appearance value
    ++current_attribute;
    generic_access_service.appearance = 0; // unknown appearance
    current_attribute->pUuid = attApChUuid;
    current_attribute->maxLen = sizeof(generic_access_service.appearance);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->pValue = (uint8_t*) &generic_access_service.appearance;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;


    // peripheral prefered connection parameters declaration
    currentHandle += 2; // note: incremented by two to get a pointer to the value handle
    ++current_attribute;

    // set properties
    generic_access_service.ppcp_declaration_value[0] = ATT_PROP_READ;
    // set value handle
    memcpy(generic_access_service.ppcp_declaration_value + 1, &currentHandle, sizeof(currentHandle));
    // set the characteristic UUID
    memcpy(generic_access_service.ppcp_declaration_value + 3, attPpcpChUuid, sizeof(attPpcpChUuid));

    current_attribute->pUuid = attChUuid;
    current_attribute->pValue = generic_access_service.ppcp_declaration_value;
    current_attribute->maxLen = sizeof(generic_access_service.ppcp_declaration_value);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // peripheral prefered connection parameters value
    ++current_attribute;
    const uint8_t default_ppcp_value[] = {
        0xFF, 0xFF, // no specific min connection interval
        0xFF, 0xFF, // no specific max connection interval
        0x00, 0x00, // no slave latency
        0xFF, 0xFF // no specific connection supervision timeout
    };
    memcpy(&generic_access_service.ppcp, default_ppcp_value, sizeof(default_ppcp_value));
    current_attribute->pUuid = attPpcpChUuid;
    current_attribute->maxLen = sizeof(generic_access_service.ppcp);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->pValue = generic_access_service.ppcp;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    generic_access_service.service.endHandle = currentHandle;
    AttsAddGroup(&generic_access_service.service);
}

void MaximGattServer::add_generic_attribute_service()
{
    ++currentHandle;
    generic_attribute_service.service.pNext = NULL;
    generic_attribute_service.service.startHandle = currentHandle;
    generic_attribute_service.service.readCback = attsReadCback;
    generic_attribute_service.service.writeCback = attsWriteCback;

    // bind attributes to the service
    generic_attribute_service.service.pAttr = generic_attribute_service.attributes;

    attsAttr_t* current_attribute = generic_attribute_service.attributes;

    // service attribute
    current_attribute->pUuid = attPrimSvcUuid;
    current_attribute->pValue = (uint8_t*) attGattSvcUuid;
    current_attribute->maxLen = sizeof(attGattSvcUuid);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // service changed declaration
    currentHandle += 2; // note: incremented by two to get a pointer to the value handle
    ++current_attribute;

    // set properties
    generic_attribute_service.service_changed_declaration[0] = ATT_PROP_INDICATE;
    // set value handle
    memcpy(generic_attribute_service.service_changed_declaration + 1, &currentHandle, sizeof(currentHandle));
    // set the characteristic UUID
    memcpy(generic_attribute_service.service_changed_declaration + 3, attScChUuid, sizeof(attScChUuid));

    current_attribute->pUuid = attChUuid;
    current_attribute->pValue = generic_attribute_service.service_changed_declaration;
    current_attribute->maxLen = sizeof(generic_attribute_service.service_changed_declaration);
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = 0;
    current_attribute->permissions = ATTS_PERMIT_READ;

    // service changed value
    ++current_attribute;
    current_attribute->pUuid = attScChUuid;
    current_attribute->maxLen = 0;
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->pValue = NULL;
    current_attribute->settings = 0;
    current_attribute->permissions = 0;

    // CCCD
    ++current_attribute;
    current_attribute->pUuid = attCliChCfgUuid;
    current_attribute->pValue = (uint8_t*)&cccd_values[cccd_cnt];
    current_attribute->maxLen = 2;
    current_attribute->pLen = &current_attribute->maxLen;
    current_attribute->settings = ATTS_SET_CCC;
    current_attribute->permissions = (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE);

    cccds[cccd_cnt].handle = currentHandle;
    cccds[cccd_cnt].valueRange = ATT_CLIENT_CFG_INDICATE;
    cccds[cccd_cnt].secLevel = DM_SEC_LEVEL_NONE;
    cccd_handles[cccd_cnt] = currentHandle - 1;
    cccd_cnt++;

    generic_attribute_service.service.endHandle = currentHandle;
    AttsAddGroup(&generic_attribute_service.service);
    AttsCccRegister(cccd_cnt, (attsCccSet_t*)cccds, cccCback);
}

GattCharacteristic* MaximGattServer::get_auth_char(uint16_t value_handle)
{
    for (size_t i = 0; i < _auth_char_count; ++i) {
        if (_auth_char[i]->getValueHandle() == value_handle) {
            return _auth_char[i];
        }
    }
    return NULL;
}

bool MaximGattServer::get_cccd_index_by_cccd_handle(GattAttribute::Handle_t cccd_handle, uint8_t& idx) const
{
    for (idx = 0; idx < cccd_cnt; idx++) {
        if (cccd_handle == cccds[idx].handle) {
            return true;
        }
    }
    return false;
}

bool MaximGattServer::get_cccd_index_by_value_handle(GattAttribute::Handle_t char_handle, uint8_t& idx) const
{
    for (idx = 0; idx < cccd_cnt; ++idx) {
        if (char_handle == cccd_handles[idx]) {
            return true;
        }
    }
    return false;
}

bool MaximGattServer::is_update_authorized(
    ble::connection_handle_t connection,
    GattAttribute::Handle_t value_handle
) {
    GattCharacteristic* auth_char = get_auth_char(value_handle);
    if (!auth_char) {
        return true;
    }

    att_security_requirement_t sec_req =
        auth_char->getUpdateSecurityRequirement();

    if (sec_req == att_security_requirement_t::NONE) {
        return true;
    }

#if BLE_FEATURE_SECURITY
    SecurityManager& security_manager = MaximSecurityManager::getInstance();
    ble::link_encryption_t encryption(ble::link_encryption_t::NOT_ENCRYPTED);
    ble_error_t err = security_manager.getLinkEncryption(connection, &encryption);
    if (err) {
        return false;
    }
#endif // BLE_FEATURE_SECURITY

    switch (sec_req.value()) {
#if BLE_FEATURE_SECURITY
        case att_security_requirement_t::UNAUTHENTICATED:
            if (encryption < ble::link_encryption_t::ENCRYPTED) {
                return false;
            }
            return true;

        case att_security_requirement_t::AUTHENTICATED:
            if (encryption < ble::link_encryption_t::ENCRYPTED_WITH_MITM) {
                return false;
            }
            return true;
#if BLE_FEATURE_SECURE_CONNECTIONS
        case att_security_requirement_t::SC_AUTHENTICATED:
            if (encryption != ble::link_encryption_t::ENCRYPTED_WITH_SC_AND_MITM) {
                return false;
            }
            return true;
#endif // BLE_FEATURE_SECURE_CONNECTIONS
#endif // BLE_FEATURE_SECURITY
        default:
            return false;
    }
}

void MaximGattServer::insert_service_attribute(
    GattService& service,
    attsAttr_t *&attribute_it
) {
    ++currentHandle;
    const UUID& service_uuid = service.getUUID();

    attribute_it->pUuid = attPrimSvcUuid;
    if (service_uuid.shortOrLong() == UUID::UUID_TYPE_LONG) {
        attribute_it->maxLen = UUID::LENGTH_OF_LONG_UUID;
    } else {
        attribute_it->maxLen = sizeof(UUID::ShortUUIDBytes_t);
    }
    attribute_it->pValue = (uint8_t*) alloc_block(attribute_it->maxLen);
    memcpy(attribute_it->pValue, service_uuid.getBaseUUID(), attribute_it->maxLen);
    attribute_it->pLen = &attribute_it->maxLen;
    attribute_it->settings = 0;
    attribute_it->permissions = ATTS_PERMIT_READ;

    ++attribute_it;
}

void* MaximGattServer::alloc_block(size_t block_size) {
    alloc_block_t* block = (alloc_block_t*) malloc(sizeof(alloc_block_t) + block_size);
    if (block == NULL) {
        return NULL;
    }

    if (allocated_blocks != NULL) {
        block->next = allocated_blocks;
    } else {
        block->next = NULL;
    }

    allocated_blocks = block;

    return block->data;
}

ble_error_t MaximGattServer::insert_characteristic(
    GattCharacteristic *characteristic,
    attsAttr_t *&attribute_it
) {
    bool valid = is_characteristic_valid(characteristic);
    if (!valid) {
        return BLE_ERROR_INVALID_PARAM;
    }

    uint8_t properties = characteristic->getProperties();

    // Create Characteristic Declaration Attribute
    insert_characteristic_declaration_attribute(characteristic, attribute_it);
    ble_error_t err = insert_characteristic_value_attribute(characteristic, attribute_it);
    if (err) {
        return err;
    }


    // insert descriptors
    bool cccd_created = false;
    for (size_t i = 0; i < characteristic->getDescriptorCount(); i++) {
        err = insert_descriptor(
            characteristic,
            characteristic->getDescriptor(i),
            attribute_it,
            cccd_created
        );
        if (err) {
            return err;
        }
    }

    // insert implicit CCCD
    if ((properties & UPDATE_PROPERTIES) && (cccd_created == false)) {
        err = insert_cccd(characteristic, attribute_it);
        if (err) {
            return err;
        }
    }

    return BLE_ERROR_NONE;
}

bool MaximGattServer::is_characteristic_valid(GattCharacteristic *characteristic) {
    uint8_t properties = characteristic->getProperties();

    // nothing to read while the characteristic is flagged as readable
    if ((characteristic->getValueAttribute().getValuePtr() == NULL) &&
        (characteristic->getValueAttribute().getMaxLength() == 0) &&
        (properties == READ_PROPERTY) &&
        (characteristic->isReadAuthorizationEnabled() == false)
    ) {
        return false;
    }

    // nothing to write while the characteristic is flagged as writable
    if ((characteristic->getValueAttribute().getValuePtr() == NULL) &&
        (characteristic->getValueAttribute().getMaxLength() == 0) &&
        (properties & WRITABLE_PROPERTIES) &&
        (characteristic->isWriteAuthorizationEnabled() == false)
    ) {
        return false;
    }

#if BLE_FEATURE_SIGNING
    // check for invalid permissions
    if ((properties == SIGNED_WRITE_PROPERTY) &&
        (characteristic->getWriteSecurityRequirement() == att_security_requirement_t::NONE
#if BLE_FEATURE_SECURE_CONNECTIONS
         || characteristic->getWriteSecurityRequirement() == att_security_requirement_t::SC_AUTHENTICATED

#endif // BLE_FEATURE_SECURE_CONNECTIONS
        )
    ) {
        return false;
    }
#endif // BLE_FEATURE_SIGNING

    return true;
}

void MaximGattServer::insert_characteristic_declaration_attribute(
    GattCharacteristic *characteristic,
    attsAttr_t *&attribute_it
) {
    const UUID& value_uuid = characteristic->getValueAttribute().getUUID();

    // move the current handle to point to the value handle
    currentHandle += 2;
    characteristic->getValueAttribute().setHandle(currentHandle);

    // fill the cordio attribute
    attribute_it->pUuid = attChUuid;
    attribute_it->maxLen = 1 + sizeof(currentHandle) + value_uuid.getLen();
    attribute_it->pLen = &attribute_it->maxLen;
    attribute_it->pValue = (uint8_t*) alloc_block(attribute_it->maxLen);
    attribute_it->settings = 0;
    attribute_it->permissions = ATTS_PERMIT_READ;

    // set the attribute value
    uint8_t *value_it = attribute_it->pValue;
    *value_it++ = characteristic->getProperties();
    memcpy(value_it, &currentHandle, sizeof(currentHandle));
    value_it += sizeof(currentHandle);
    memcpy(value_it, value_uuid.getBaseUUID(), value_uuid.getLen());

    ++attribute_it;
}

ble_error_t MaximGattServer::insert_characteristic_value_attribute(
    GattCharacteristic *characteristic,
    attsAttr_t *&attribute_it
) {
    GattAttribute &value_attribute = characteristic->getValueAttribute();
    uint8_t properties = characteristic->getProperties();

    // note: currentHandle has been already moved to the correct value

    // Create Value Attribute
    attribute_it->pUuid = value_attribute.getUUID().getBaseUUID();
    attribute_it->maxLen = characteristic->getValueAttribute().getMaxLength();
    attribute_it->pLen = (uint16_t*) alloc_block(attribute_it->maxLen + sizeof(uint16_t));
    *attribute_it->pLen = value_attribute.getLength();
    attribute_it->pValue = (uint8_t*) ((uint16_t*)attribute_it->pLen + 1);
    memcpy(attribute_it->pValue, value_attribute.getValuePtr(), *attribute_it->pLen);
    memset(attribute_it->pValue + *attribute_it->pLen, 0, attribute_it->maxLen - *attribute_it->pLen);

    // Set value attribute settings
    attribute_it->settings = 0;

    if (properties & READ_PROPERTY) {
        attribute_it->settings |= ATTS_SET_READ_CBACK;
    }
    if (properties & WRITABLE_PROPERTIES) {
        attribute_it->settings |= ATTS_SET_WRITE_CBACK;
    }
    if (value_attribute.getUUID().shortOrLong() == UUID::UUID_TYPE_LONG) {
        attribute_it->settings |= ATTS_SET_UUID_128;
    }
    if (value_attribute.hasVariableLength()) {
        attribute_it->settings |= ATTS_SET_VARIABLE_LEN;
    }
    if (properties & SIGNED_WRITE_PROPERTY) {
        attribute_it->settings |= ATTS_SET_ALLOW_SIGNED;
    }

    // setup permissions
    attribute_it->permissions = 0;

    // configure read permission
    if (properties & READ_PROPERTY) {
        attribute_it->permissions |= ATTS_PERMIT_READ;
        switch (characteristic->getReadSecurityRequirement().value()) {
            case att_security_requirement_t::NONE:
                break;
#if BLE_FEATURE_SECURITY
            case att_security_requirement_t::UNAUTHENTICATED:
                attribute_it->permissions |= ATTS_PERMIT_READ_ENC;
                break;
            case att_security_requirement_t::AUTHENTICATED:
                attribute_it->permissions |=
                    ATTS_PERMIT_READ_ENC |
                    ATTS_PERMIT_READ_AUTH;
                break;
#if BLE_FEATURE_SECURE_CONNECTIONS
            case att_security_requirement_t::SC_AUTHENTICATED:
                // Note: check done in the cordio stack doesn't cover LESC
                // so this one is done in attsAuthorCback
                attribute_it->permissions |=
                    ATTS_PERMIT_READ_ENC |
                    ATTS_PERMIT_READ_AUTH |
                    ATTS_PERMIT_READ_AUTHORIZ;
                break;
#endif // BLE_FEATURE_SECURE_CONNECTIONS
#endif // BLE_FEATURE_SECURITY
        }
    }

    // configure write permission
    if (properties & WRITABLE_PROPERTIES) {
        attribute_it->permissions |= ATTS_PERMIT_WRITE;
        switch (characteristic->getWriteSecurityRequirement().value()) {
            case att_security_requirement_t::NONE:
                break;
#if BLE_FEATURE_SECURITY
            case att_security_requirement_t::UNAUTHENTICATED:
                attribute_it->permissions |= ATTS_PERMIT_WRITE_ENC;
                break;
            case att_security_requirement_t::AUTHENTICATED:
                attribute_it->permissions |=
                    ATTS_PERMIT_WRITE_ENC |
                    ATTS_PERMIT_WRITE_AUTH;
                break;
#if BLE_FEATURE_SECURE_CONNECTIONS
            case att_security_requirement_t::SC_AUTHENTICATED:
                // Note: check done in the cordio stack doesn't cover LESC
                // so this one is done in attsAuthorCback
                attribute_it->permissions |=
                    ATTS_PERMIT_WRITE_ENC |
                    ATTS_PERMIT_WRITE_AUTH |
                    ATTS_PERMIT_WRITE_AUTHORIZ;
                break;
#endif // BLE_FEATURE_SECURE_CONNECTIONS
#endif // BLE_FEATURE_SECURITY
        }
    }

    // Register characteristic in authorisation list
    // Note: ATTS_PERMIT_*_AUTHORIZ is uniquely used to check if security
    // requirements are met according to the security requirements set in the
    // characteristic.
    // User defined security authorisation does not impact this flag
    if ((attribute_it->permissions & (ATTS_PERMIT_READ_AUTHORIZ | ATTS_PERMIT_WRITE_AUTHORIZ)) ||
        (properties & UPDATE_PROPERTIES) ||
        characteristic->isReadAuthorizationEnabled() ||
        characteristic->isWriteAuthorizationEnabled()
    ) {
        if ( _auth_char_count >= MAX_CHARACTERISTIC_AUTHORIZATION_CNT) {
            return BLE_ERROR_NO_MEM;
        }
        _auth_char[_auth_char_count] = characteristic;
        ++_auth_char_count;
    }

    ++attribute_it;

    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::insert_descriptor(
    GattCharacteristic *characteristic,
    GattAttribute* descriptor,
    attsAttr_t *&attribute_it,
    bool& cccd_created
) {
    uint8_t properties = characteristic->getProperties();

    currentHandle++;

    descriptor->setHandle(currentHandle);

    attribute_it->pUuid = descriptor->getUUID().getBaseUUID();
    attribute_it->maxLen = descriptor->getMaxLength();
    attribute_it->pLen = (uint16_t*) alloc_block(attribute_it->maxLen + sizeof(uint16_t));
    *attribute_it->pLen = descriptor->getLength();
    attribute_it->pValue = (uint8_t*) ((uint16_t*)attribute_it->pLen + 1);
    memcpy(attribute_it->pValue, descriptor->getValuePtr(), *attribute_it->pLen);
    memset(attribute_it->pValue + *attribute_it->pLen, 0, attribute_it->maxLen - *attribute_it->pLen);

    attribute_it->settings = 0;

    if (descriptor->getUUID().shortOrLong() == UUID::UUID_TYPE_LONG) {
        attribute_it->settings |= ATTS_SET_UUID_128;
    }

    // handle the special case of a CCCD
    if (descriptor->getUUID() == UUID(BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG)) {
        if (cccd_cnt >= MAX_CCCD_CNT) {
            return BLE_ERROR_NO_MEM;
        }

        if (descriptor->isReadAllowed() == false ||
            descriptor->getReadSecurityRequirement() != att_security_requirement_t::NONE
        ) {
            return BLE_ERROR_INVALID_PARAM;
        }

        cccd_created = true;
        attribute_it->settings |= ATTS_SET_CCC;

        cccds[cccd_cnt].handle = currentHandle;
        cccds[cccd_cnt].valueRange = 0;
        if (properties & NOTIFY_PROPERTY) {
            cccds[cccd_cnt].valueRange |= ATT_CLIENT_CFG_NOTIFY;
        }
        if (properties & INDICATE_PROPERTY) {
            cccds[cccd_cnt].valueRange |= ATT_CLIENT_CFG_INDICATE;
        }
        cccd_handles[cccd_cnt] = characteristic->getValueAttribute().getHandle();
        cccd_cnt++;
    }

    if (descriptor->hasVariableLength()) {
        attribute_it->settings |= ATTS_SET_VARIABLE_LEN;
    }

    // setup permissions
    attribute_it->permissions = 0;

    // configure read permission
    if (descriptor->isReadAllowed()) {
        attribute_it->permissions |= ATTS_PERMIT_READ;
        switch (descriptor->getReadSecurityRequirement().value()) {
            case att_security_requirement_t::NONE:
                break;
#if BLE_FEATURE_SECURITY
            case att_security_requirement_t::UNAUTHENTICATED:
                attribute_it->permissions |= ATTS_PERMIT_READ_ENC;
                break;
            case att_security_requirement_t::AUTHENTICATED:
                attribute_it->permissions |=
                    ATTS_PERMIT_READ_ENC |
                    ATTS_PERMIT_READ_AUTH;
                break;
#if BLE_FEATURE_SECURE_CONNECTIONS
            case att_security_requirement_t::SC_AUTHENTICATED:
                // Note: check done in the cordio stack doesn't cover LESC
                // so this one is done in attsAuthorCback
                attribute_it->permissions |=
                    ATTS_PERMIT_READ_ENC |
                    ATTS_PERMIT_READ_AUTH |
                    ATTS_PERMIT_READ_AUTHORIZ;
                break;
#endif // BLE_FEATURE_SECURE_CONNECTIONS
#endif // BLE_FEATURE_SECURITY
        }

        if (properties & READ_PROPERTY && !(attribute_it->settings & ATTS_SET_CCC)) {
            attribute_it->settings |= ATTS_SET_READ_CBACK;
        }
    }

    // configure write permission
    if (descriptor->isWriteAllowed()) {
        attribute_it->permissions |= ATTS_PERMIT_WRITE;
        switch (descriptor->getWriteSecurityRequirement().value()) {
            case att_security_requirement_t::NONE:
                break;
#if BLE_FEATURE_SECURITY
            case att_security_requirement_t::UNAUTHENTICATED:
                attribute_it->permissions |= ATTS_PERMIT_WRITE_ENC;
                break;
            case att_security_requirement_t::AUTHENTICATED:
                attribute_it->permissions |=
                    ATTS_PERMIT_WRITE_ENC |
                    ATTS_PERMIT_WRITE_AUTH;
                break;
#if BLE_FEATURE_SECURE_CONNECTIONS
            case att_security_requirement_t::SC_AUTHENTICATED:
                // Note: check done in the cordio stack doesn't cover LESC
                // so this one is done in attsAuthorCback
                attribute_it->permissions |=
                    ATTS_PERMIT_WRITE_ENC |
                    ATTS_PERMIT_WRITE_AUTH |
                    ATTS_PERMIT_WRITE_AUTHORIZ;
                break;
#endif // BLE_FEATURE_SECURE_CONNECTIONS
#endif // BLE_FEATURE_SECURITY
        }

        if (properties & WRITABLE_PROPERTIES && !(attribute_it->settings & ATTS_SET_CCC)) {
            attribute_it->settings |= ATTS_SET_WRITE_CBACK;
        }
    }

    attribute_it++;
    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::insert_cccd(
    GattCharacteristic *characteristic,
    attsAttr_t *&attribute_it
) {
    if (cccd_cnt >= MAX_CCCD_CNT) {
        return BLE_ERROR_NO_MEM;
    }

    uint8_t properties = characteristic->getProperties();

    currentHandle++;

    attribute_it->pUuid = CCCD_UUID.getBaseUUID();
    attribute_it->pValue = (uint8_t*) &cccd_values[cccd_cnt];
    attribute_it->maxLen = CCCD_SIZE;
    attribute_it->pLen = &attribute_it->maxLen;
    attribute_it->settings = ATTS_SET_CCC;
    attribute_it->permissions = (ATTS_PERMIT_READ | ATTS_PERMIT_WRITE);

    cccds[cccd_cnt].handle = currentHandle;
    cccds[cccd_cnt].valueRange = 0;
    if (properties & NOTIFY_PROPERTY) {
        cccds[cccd_cnt].valueRange |= ATT_CLIENT_CFG_NOTIFY;
    }
    if (properties & INDICATE_PROPERTY) {
        cccds[cccd_cnt].valueRange |= ATT_CLIENT_CFG_INDICATE;
    }
    cccds[cccd_cnt].secLevel = characteristic->getUpdateSecurityRequirement().value();
    cccd_handles[cccd_cnt] = characteristic->getValueAttribute().getHandle();

    cccd_cnt++;
    attribute_it++;

    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::read_(GattAttribute::Handle_t att_handle, uint8_t buffer[], uint16_t *const buffer_length)
{
    uint16_t att_length = 0;
    uint8_t* att_value = NULL;
    if (AttsGetAttr(att_handle, buffer_length, &buffer) != ATT_SUCCESS) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    if (buffer)
    {
        memcpy(buffer, att_value, std::min(*buffer_length, att_length));
    }

    *buffer_length = att_length;
    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::read_(ble::connection_handle_t connection, GattAttribute::Handle_t att_handle, uint8_t buffer[], uint16_t *buffer_length)
{
    // Check to see if this is a CCCD
    uint8_t cccd_index;
    if (get_cccd_index_by_cccd_handle(att_handle, cccd_index)) {
        if (connection == DM_CONN_ID_NONE) {
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }
        uint16_t cccd_value = AttsCccGet(connection, cccd_index);
        uint16_t cccd_length = sizeof(cccd_value);

        if (buffer) {
            memcpy(buffer, &cccd_value, std::min(*buffer_length, cccd_length));
        }

        *buffer_length = cccd_length;
        return BLE_ERROR_NONE;
    }

    // This is not a CCCD. Use the non-connection specific update method.
    return read(att_handle, buffer, buffer_length);
}

ble_error_t MaximGattServer::write_(GattAttribute::Handle_t att_handle, const uint8_t buffer[], uint16_t len, bool local_only)
{
    // Check to see if this is a CCCD, if it is the case update the value for all
    // connections
    uint8_t cccd_index;
    if (get_cccd_index_by_cccd_handle(att_handle, cccd_index)) {
        if (len != sizeof(uint16_t)) {
            return BLE_ERROR_INVALID_PARAM;
        }

        uint16_t cccd_value;
        memcpy(&cccd_value, buffer, sizeof(cccd_value));

        for (dmConnId_t conn_id = DM_CONN_MAX; conn_id > DM_CONN_ID_NONE; --conn_id) {
            if (DmConnInUse(conn_id) == true) {
                AttsCccSet(conn_id, cccd_index, cccd_value);
            }
        }

        return BLE_ERROR_NONE;
    }

    // write the value to the attribute handle
    if (AttsSetAttr(att_handle, len, (uint8_t*)buffer) != ATT_SUCCESS) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    // return if the update does not have to be propagated to peers
    if (local_only || !get_cccd_index_by_value_handle(att_handle, cccd_index)) {
        return BLE_ERROR_NONE;
    }

    // This characteristic has a CCCD attribute. Handle notifications and
    // indications for all active connections if the authentication is
    // successful
    size_t updates_sent = 0;

#if BLE_FEATURE_SECURITY
    for (dmConnId_t conn_id = DM_CONN_MAX; conn_id > DM_CONN_ID_NONE; --conn_id) {
        if (DmConnInUse(conn_id) == true) {
            if (is_update_authorized(conn_id, att_handle)) {
                uint16_t cccd_config = AttsCccEnabled(conn_id, cccd_index);
                if (cccd_config & ATT_CLIENT_CFG_NOTIFY) {
                    AttsHandleValueNtf(conn_id, att_handle, len, (uint8_t*)buffer);
                    updates_sent++;
                }
                if (cccd_config & ATT_CLIENT_CFG_INDICATE) {
                    AttsHandleValueInd(conn_id, att_handle, len, (uint8_t*)buffer);
                    updates_sent++;
                }
            }
        }
    }
#endif // BLE_FEATURE_SECURITY

    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::write_(Gap::Handle_t connection, GattAttribute::Handle_t att_handle, const uint8_t buffer[], uint16_t len, bool local_only)
{
    // Check to see if this is a CCCD
    uint8_t cccd_index;
    if (get_cccd_index_by_cccd_handle(att_handle, cccd_index)) {
        if ((connection == DM_CONN_ID_NONE) || (len != 2)) { // CCCDs are always 16 bits
            return BLE_ERROR_PARAM_OUT_OF_RANGE;
        }

        uint16_t cccd_value;
        memcpy(&cccd_value, buffer, sizeof(cccd_value));
        AttsCccSet(connection, cccd_index, cccd_value);
        return BLE_ERROR_NONE;
    }

    // write the value to the attribute handle
    if (AttsSetAttr(att_handle, len, (uint8_t*)buffer) != ATT_SUCCESS) {
        return BLE_ERROR_PARAM_OUT_OF_RANGE;
    }

    // return if the update does not have to be propagated to peers
    if (local_only || !get_cccd_index_by_value_handle(att_handle, cccd_index)) {
        return BLE_ERROR_NONE;
    }

    // This characteristic has a CCCD attribute. Handle notifications and indications.
    size_t updates_sent = 0;

#if BLE_FEATURE_SECURITY
    if (is_update_authorized(connection, att_handle)) {
        uint16_t cccEnabled = AttsCccEnabled(connection, cccd_index);
        if (cccEnabled & ATT_CLIENT_CFG_NOTIFY) {
            AttsHandleValueNtf(connection, att_handle, len, (uint8_t*)buffer);
            updates_sent++;
        }
        if (cccEnabled & ATT_CLIENT_CFG_INDICATE) {
            AttsHandleValueInd(connection, att_handle, len, (uint8_t*)buffer);
            updates_sent++;
        }
    }
#endif // BLE_FEATURE_SECURITY

    return BLE_ERROR_NONE;
}

ble_error_t MaximGattServer::areUpdatesEnabled_(const GattCharacteristic &characteristic, bool *enabled)
{
    for (size_t idx = 0; idx < cccd_cnt; idx++) {
        if (characteristic.getValueHandle() == cccd_handles[idx]) {
            for (dmConnId_t conn_id = DM_CONN_MAX; conn_id > DM_CONN_ID_NONE; --conn_id) {
                if (DmConnInUse(conn_id) == true) {
                    uint16_t cccd_value = AttsCccGet(conn_id, idx);
                    if (cccd_value & (ATT_CLIENT_CFG_NOTIFY | ATT_CLIENT_CFG_INDICATE)) {
                        *enabled = true;
                        return BLE_ERROR_NONE;
                    }

                }
            }
            *enabled = false;
            return BLE_ERROR_NONE;
        }
    }

    return BLE_ERROR_PARAM_OUT_OF_RANGE;
}

ble_error_t MaximGattServer::areUpdatesEnabled_(Gap::Handle_t connectionHandle, const GattCharacteristic &characteristic, bool *enabled)
{
    if (connectionHandle == DM_CONN_ID_NONE) {
        return BLE_ERROR_INVALID_PARAM;
    }

    for (uint8_t idx = 0; idx < cccd_cnt; idx++) {
        if (characteristic.getValueHandle() == cccd_handles[idx]) {
            uint16_t cccd_value = AttsCccGet(connectionHandle, idx);
            if (cccd_value & (ATT_CLIENT_CFG_NOTIFY | ATT_CLIENT_CFG_INDICATE)) {
                *enabled = true;
            } else {
                *enabled = false;
            }
            return BLE_ERROR_NONE;
        }
    }
    return BLE_ERROR_PARAM_OUT_OF_RANGE;
}

::Gap::ConnectionParams_t MaximGattServer::getPreferredConnectionParams()
{
    ::Gap::ConnectionParams_t params = { 0 };
    memcpy(&params.minConnectionInterval, generic_access_service.ppcp, 2);
    memcpy(&params.maxConnectionInterval, generic_access_service.ppcp + 2, 2);
    memcpy(&params.slaveLatency, generic_access_service.ppcp + 4, 2);
    memcpy(&params.connectionSupervisionTimeout, generic_access_service.ppcp + 6, 2);
    return params;
}

void MaximGattServer::setPreferredConnectionParams(const ::Gap::ConnectionParams_t& params)
{
    memcpy(generic_access_service.ppcp, &params.minConnectionInterval, 2);
    memcpy(generic_access_service.ppcp + 2, &params.maxConnectionInterval, 2);
    memcpy(generic_access_service.ppcp + 4, &params.slaveLatency, 2);
    memcpy(generic_access_service.ppcp + 6, &params.connectionSupervisionTimeout, 2);
}

ble_error_t MaximGattServer::setDeviceName(const uint8_t *deviceName)
{
    size_t length = 0;

    if (deviceName != NULL) {
        length = strlen((const char*)deviceName);
    }

    if (length == 0) {
        free(generic_access_service.device_name_value());
    } else {
        uint8_t* res = (uint8_t*) realloc(generic_access_service.device_name_value(), length);
        if (res == NULL) {
            return BLE_ERROR_NO_MEM;
        }

        generic_access_service.device_name_value() = res;
        memcpy(res, deviceName, length);
    }

    generic_access_service.device_name_length = length;

    return BLE_ERROR_NONE;
}

void MaximGattServer::getDeviceName(const uint8_t*& name, uint16_t& length)
{
    length = generic_access_service.device_name_length;
    name = generic_access_service.device_name_value();
}

void MaximGattServer::setAppearance(GapAdvertisingData::Appearance appearance)
{
    generic_access_service.appearance = appearance;
}

GapAdvertisingData::Appearance MaximGattServer::getAppearance()
{
    return (GapAdvertisingData::Appearance) generic_access_service.appearance;
}

ble_error_t MaximGattServer::reset_(void)
{
    Base::reset_();

    while (registered_service) {
        mxmService_t* s = (mxmService_t*)registered_service;
        registered_service = s->next;
        AttsRemoveGroup(s->attGroup.startHandle);
        delete s;
    }

    while (allocated_blocks) {
        alloc_block_t* b = allocated_blocks;
        allocated_blocks = b->next;
        free(b);
    }

    AttsRemoveGroup(generic_access_service.service.startHandle);
    AttsRemoveGroup(generic_attribute_service.service.startHandle);
    free(generic_access_service.device_name_value());

    currentHandle = 0;
    cccd_cnt = 0;

    _auth_char_count = 0;

    AttsCccRegister(cccd_cnt, (attsCccSet_t*)cccds, cccCback);

    return BLE_ERROR_NONE;
}

void MaximGattServer::cccCback(attsCccEvt_t *evt)
{
    GattServerEvents::gattEvent_t evt_type =
        evt->value & (ATT_CLIENT_CFG_NOTIFY | ATT_CLIENT_CFG_INDICATE) ?
            GattServerEvents::GATT_EVENT_UPDATES_ENABLED :
            GattServerEvents::GATT_EVENT_UPDATES_DISABLED;

    getInstance().handleEvent(evt_type, evt->handle);
}

void MaximGattServer::attCback(attEvt_t *pEvt)
{
    if (pEvt->hdr.status == ATT_SUCCESS) {
        getInstance().handleEvent(GattServerEvents::GATT_EVENT_DATA_SENT, pEvt->handle);
    }
}

uint8_t MaximGattServer::attsReadCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset, attsAttr_t *pAttr)
{
    GattCharacteristic* auth_char = getInstance().get_auth_char(handle);
    if (auth_char && auth_char->isReadAuthorizationEnabled()) {
        GattReadAuthCallbackParams read_auth_params = {
            connId,
            handle,
            offset,
            /* len */ 0,
            /* data */ NULL,
            AUTH_CALLBACK_REPLY_SUCCESS
        };

        GattAuthCallbackReply_t ret = auth_char->authorizeRead(&read_auth_params);
        if (ret != AUTH_CALLBACK_REPLY_SUCCESS) {
            return ret & 0xFF;
        }

        pAttr->pValue = read_auth_params.data;
        *pAttr->pLen = read_auth_params.len;
    }

    GattReadCallbackParams read_params = {
        connId,
        handle,
        offset,
        *pAttr->pLen,
        pAttr->pValue,
        /* status */ BLE_ERROR_NONE,
    };
    getInstance().handleDataReadEvent(&read_params);

    return ATT_SUCCESS;
}

uint8_t MaximGattServer::attsWriteCback(dmConnId_t connId, uint16_t handle, uint8_t operation, uint16_t offset, uint16_t len, uint8_t *pValue, attsAttr_t *pAttr)
{
    uint8_t err;

    /* TODO: offset is not handled properly */
    if ((err = AttsSetAttr(handle, len, pValue)) != ATT_SUCCESS) {
        return err;
    }

    GattWriteCallbackParams::WriteOp_t writeOp;
    switch (operation) {
        case ATT_PDU_WRITE_REQ:
            writeOp = GattWriteCallbackParams::OP_WRITE_REQ;
            break;
        case ATT_PDU_WRITE_CMD:
            writeOp = GattWriteCallbackParams::OP_WRITE_CMD;
            break;
#if false //BLE_FEATURE_SIGNING
        case ATT_PDU_SIGNED_WRITE_CMD:
            if (getInstance()._signing_event_handler) {
                getInstance()._signing_event_handler->on_signed_write_received(
                    connId,
                    AttsGetSignCounter(connId)
                );
            }
            writeOp = GattWriteCallbackParams::OP_SIGN_WRITE_CMD;
            break;
#endif // BLE_FEATURE_SIGNING
        case ATT_PDU_PREP_WRITE_REQ:
            writeOp = GattWriteCallbackParams::OP_PREP_WRITE_REQ;
            break;
        case ATT_PDU_EXEC_WRITE_REQ:
            writeOp = GattWriteCallbackParams::OP_EXEC_WRITE_REQ_NOW;
            break;
        default:
            writeOp = GattWriteCallbackParams::OP_INVALID;
            break;
    }

    GattCharacteristic* auth_char = getInstance().get_auth_char(handle);
    if (auth_char && auth_char->isWriteAuthorizationEnabled()) {
        GattWriteAuthCallbackParams write_auth_params = {
            connId,
            handle,
            offset,
            len,
            pValue,
            AUTH_CALLBACK_REPLY_SUCCESS
        };

        GattAuthCallbackReply_t ret = auth_char->authorizeWrite(&write_auth_params);
        if (ret!= AUTH_CALLBACK_REPLY_SUCCESS) {
            return ret & 0xFF;
        }
    }

    GattWriteCallbackParams write_params = {
        connId,
        handle,
        writeOp,
        offset,
        len,
        pValue
    };
    getInstance().handleDataWrittenEvent(&write_params);

    return ATT_SUCCESS;
}
MaximGattServer::MaximGattServer()
    : 
    GattServer(),
    _signing_event_handler(NULL),
    cccds(),
    cccd_values(),
    cccd_handles(),
    cccd_cnt(0),
    _auth_char(),
    _auth_char_count(0),
    generic_access_service(),
    generic_attribute_service(),
    registered_service(NULL),
    allocated_blocks(NULL),
    currentHandle(0),
    default_services_added(false) {
  /* empty */
}