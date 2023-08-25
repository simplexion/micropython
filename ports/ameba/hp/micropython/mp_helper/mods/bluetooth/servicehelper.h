#ifndef _SERVICE_HELPER_H
#define _SERVICE_HELPER_H

#include <gatt.h>

/** UUID heper */
#define VENDOR_UUID(UUID)              \
    {                                  \
        .time_low = 0x51360000 | UUID, \
        .time_mid = 0xc5e7,            \
        .time_hi_and_version = 0x47c7, \
        .clock_seq = 0x8a6e,           \
        .node = 0x47ebc99d80e8l,       \
    }

/** Attribute helper */
/* Primary Service Attributes */
#define VENDOR_PRIMARY_SERVICE_ATTRIBUTE(UUID)                \
    {                                                         \
        .flags.bits = {                                       \
            .le = true,                                       \
        },                                                    \
        .value_len = UUID_128BIT_SIZE, .type.PrimaryValue = { \
            .uuid = GATT_UUID_PRIMARY_SERVICE,                \
        },                                                    \
        .context.uuid_p = &((UUID_t)VENDOR_UUID(UUID)),       \
    }

#define PREDEF_PRIMARY_SERVICE_ATTRIBUTE(UUID)               \
    {                                                        \
        .flags.bits = {                                      \
            .le = true,                                      \
            .value_incl = true                               \
        },                                                   \
        .value_len = UUID_16BIT_SIZE, .type.PrimaryValue = { \
            .uuid = GATT_UUID_PRIMARY_SERVICE,               \
            .value = UUID,                                   \
        },                                                   \
    }

/* Characteristic Attributes */
#define CHARACTERISTIC_DECLARATION_ATTRIBUTE(FLAGS)                                    \
    {                                                                                  \
        .flags.bits = {                                                                \
            .value_incl = true,                                                        \
        },                                                                             \
        .type.UUIDValue = {                                                            \
            .uuid = GATT_UUID_CHARACTERISTIC,                                          \
            .PropertyFlags = FLAGS,                                                    \
        },                                                                             \
        .value_len = 1, .context.raw = NULL, .permissions.bits.read_authen_req = None, \
    }

#define VENDOR_CHARACTERISTIC_VALUE_ATTRIBUTE(UUID, LEN, R_PERM, W_PERM, N_PERM)        \
    {                                                                                   \
        .flags.bits = {                                                                 \
            .uuid_128bit = true,                                                        \
            .value_appl = true,                                                         \
        },                                                                              \
        .type.uuid = UUID, .value_len = LEN, .context.raw = NULL, .permissions.bits = { \
            .read_authen_req = R_PERM,                                                  \
            .write_authen_req = W_PERM,                                                 \
            .notify_authen_req = N_PERM,                                                \
        }                                                                               \
    }

#define CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTION_ATTRIBUTE(R_PERM, W_PERM, N_PERM) \
    {                                                                                     \
        .flags.bits = {                                                                   \
            .value_incl = true,                                                           \
            .cccd_appl = true,                                                            \
        },                                                                                \
        .type.CharClientValue = {                                                         \
            .uuid = GATT_UUID_CHAR_CLIENT_CONFIG,                                         \
            .bits = {                                                                     \
                .notify = 0,                                                              \
                .indicate = 0,                                                            \
            },                                                                            \
        },                                                                                \
        .value_len = 1, .context.raw = NULL, .permissions.bits = {                        \
            .read_authen_req = R_PERM,                                                    \
            .write_authen_req = W_PERM,                                                   \
            .notify_authen_req = N_PERM,                                                  \
        }                                                                                 \
    }

/** Characterisctic helper */
#define SIMPLE_CHARACTERISTIC(UUID, LEN, R_FLAG, R_PERM, W_FLAG, W_PERM, N_FLAG, N_PERM)           \
    {                                                                                              \
        CHARACTERISTIC_DECLARATION_ATTRIBUTE(((PropertyFlags_t) {                                  \
            .read = R_FLAG,                                                                        \
            .write = W_FLAG,                                                                       \
            .notify = N_FLAG,                                                                      \
        })),                                                                                       \
            VENDOR_CHARACTERISTIC_VALUE_ATTRIBUTE(VENDOR_UUID(UUID), LEN, R_PERM, W_PERM, N_PERM), \
    }

#define NOTIFY_CHARACTERISTIC(UUID, LEN, R_FLAG, R_PERM, W_FLAG, W_PERM, N_FLAG, N_PERM)           \
    {                                                                                              \
        CHARACTERISTIC_DECLARATION_ATTRIBUTE(((PropertyFlags_t) {                                  \
            .read = R_FLAG,                                                                        \
            .write = W_FLAG,                                                                       \
            .notify = N_FLAG,                                                                      \
        })),                                                                                       \
            VENDOR_CHARACTERISTIC_VALUE_ATTRIBUTE(VENDOR_UUID(UUID), LEN, R_PERM, W_PERM, N_PERM), \
            CLIENT_CHARACTERISTIC_CONFIGURATION_DESCRIPTION_ATTRIBUTE(R_PERM, W_PERM, N_PERM),     \
    }

#define WRITE_NO_RSP_CHARACTERISTIC(UUID, LEN, R_PERM, W_PERM, N_PERM)                             \
    {                                                                                              \
        CHARACTERISTIC_DECLARATION_ATTRIBUTE(((PropertyFlags_t) {                                  \
            .write_no_rsp = true,                                                                  \
        })),                                                                                       \
            VENDOR_CHARACTERISTIC_VALUE_ATTRIBUTE(VENDOR_UUID(UUID), LEN, R_PERM, W_PERM, N_PERM), \
    }

/** indexing helper */
#define VALUE_ATTR_IDX(DB, CHAR) offsetof(__typeof__(DB), CHAR.Value)/(sizeof(DB.CHAR.Value))
#define CCCD_ATTR_IDX(DB, CHAR) offsetof(__typeof__(DB), CHAR.CCCD)/(sizeof(DB.CHAR.CCCD))

/** Types */
typedef struct {
    unsigned uuid_128bit : 1; /**< Attribute uses 128 bit UUID */
    unsigned value_incl : 1; /**< Attribute value is included following 16 bit UUID */
    unsigned value_appl : 1; /**< Application has to supply write value */
    unsigned ascii_z : 1; /**< Attribute value is ASCII_Z string */
    unsigned cccd_appl : 1; /**< Application will be informed about CCCD value is changed */
    unsigned cccd_no_filter : 1; /**< Application will be informed about CCCD value when CCCD is write by client, no matter it is changed or not */
    unsigned _reserved0 : 5;
    unsigned le : 1; /**< Used only for primary service declaration attributes if GATT over BLE is supported */
    unsigned _reserved1 : 4;
} __attribute__((packed)) AttributeFlags_t;

typedef enum {
    None = GATT_PERM_NONE,
    All = GATT_PERM_ALL,
    AuthenReq = GATT_PERM_AUTHEN_REQ,
    AuthenMITMReq = GATT_PERM_AUTHEN_MITM_REQ,
} AuthenReq_t;

typedef struct {
    AuthenReq_t read_authen_req : 2;
    unsigned read_author_req : 1;
    unsigned read_encrypted_req : 1;

    AuthenReq_t write_authen_req : 2;
    unsigned write_author_req : 1;
    unsigned write_encrypted_req : 1;

    AuthenReq_t notify_authen_req : 2;
    unsigned notify_author_req : 1;
    unsigned notify_encrypted_req : 1;
} __attribute__((packed)) PermissionFlags_t;

typedef struct {
    unsigned broadcast : 1; /**< If set, permits broadcasts of the Characteristic Value using Server Characteristic Configuration Descriptor. */
    unsigned read : 1; /**< If set, permits reads of the Characteristic Value */
    unsigned write_no_rsp : 1; /**< If set, permit writes of the Characteristic Value without response */
    unsigned write : 1; /**< If set, permits writes of the Characteristic Value with response */
    unsigned notify : 1; /**< If set, permits notifications of a Characteristic Value without acknowledgment */
    unsigned indicate : 1; /**< If set, permits indications of a Characteristic Value with acknowledgment */
    unsigned write_authen_signed : 1; /**< If set, permits signed writes to the Characteristic Value */
    unsigned ext_prop : 1; /**< If set, additional characteristic properties are defined in the Characteristic Extended Properties Descriptor */
} __attribute__((packed)) PropertyFlags_t;

typedef struct {
    uint16_t uuid;
    uint16_t value;
} __attribute__((packed)) PrimaryTypeValue_t;

typedef struct {
    uint16_t uuid;
    union {
        uint8_t u8;
        PropertyFlags_t bits;
    } property_flags;
} __attribute__((packed)) UUIDTypeValue_t;

typedef struct {
    uint16_t uuid;
    struct {
        unsigned notify : 1;
        unsigned indicate : 1;
    } __attribute__((packed)) bits;
} __attribute__((packed)) CharClientTypeValue_t;

typedef struct {
    const unsigned long long node : 48;
    const uint16_t clock_seq;
    const uint16_t time_hi_and_version;
    const uint16_t time_mid;
    const uint32_t time_low;
} __attribute__((packed)) UUID_t;

typedef struct {
    union {
        uint16_t u16;
        AttributeFlags_t bits;
    } flags;
    union {
        UUID_t uuid;
        PrimaryTypeValue_t PrimaryValue;
        UUIDTypeValue_t UUIDValue;
        CharClientTypeValue_t CharClientValue;
    } type;
    uint16_t value_len; /**< Length of value */
    union {
        UUID_t* uuid_p;
        void* raw;
    } context; /**< Pointer to value if @ref ATTRIB_FLAG_VALUE_INCL
    and @ref ATTRIB_FLAG_VALUE_APPL not set */
    union {
        uint32_t u32;
        PermissionFlags_t bits;
    } permissions; /**< Attribute permission @ref GATT_ATTRIBUTE_PERMISSIONS */
} __attribute__((packed)) Attribute_t;

typedef struct {
    const Attribute_t Declaration, Value;
} SimpleCharacteristic_t, WriteNoRspCharacteristic_t;

typedef struct {
    const Attribute_t Declaration, Value, CCCD;
} NotifyCharacteristic_t;

#endif //_SERVICE_HELPER_H