#pragma once

#include <stdint.h>
#include <string.h>

#include "can_platform.h"   /**< Platform specific aliases, can_msg_t becomes twai_message_t on esp32 */
#include "canbus_project.h" /**< required for can message definitions */

/* ============================================================================
 *  CONSTANTS
 * ========================================================================== */

#define MAX_ROUTES                 (8U)
#define ROUTE_ENTRY_PARAM_LEN      (8U)
#define ROUTE_ENTRY_SOURCE_ID_LEN  (4U)
#define ROUTE_ACTION_PARAM_LEN     (4U)
#define ROUTE_TAKE_NO_ACTION       (0xFFFFU)
#define ROUTE_RX_COMPLETE          (0x303U)
#define ROUTE_CMD_START            (0x300U)
#define ROUTE_CMD_END              (0x31FU)
#define ROUTE_DATA_PAYLOAD_LEN     (3U)
#define ROUTE_INVALID_RX           (0xFFU)
#define ROUTE_ENTRY_SIZE           (sizeof(route_entry_t)) /**< size of a single route entry */
#define ROUTE_CHUNK_SIZE           3                       /**< payload bytes per route entry */
#define ROUTE_CHUNKS_PER_ROUTE     ((ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) / ROUTE_CHUNK_SIZE)

/* ============================================================================
 *  DATA STRUCTURES
 * ========================================================================== */

typedef struct {
    uint8_t  parameters[ROUTE_ENTRY_PARAM_LEN];          /* configuration parameters (8 bytes) */
    uint8_t  source_node_id[ROUTE_ENTRY_SOURCE_ID_LEN];  /* producer node ID (4 bytes) */

    uint16_t source_msg_id;      /* source CAN message ID */
    uint16_t target_msg_id;      /* target CAN message ID */
    uint8_t  source_msg_dlc;     /* source DLC */
    uint8_t  target_msg_dlc;     /* target DLC */

    uint8_t  source_sub_idx;     /* source submodule index */
    uint8_t  target_sub_idx;     /* target submodule index */

    uint8_t  event_type;         /* event type */
    uint8_t  action_type;        /* action type */

    uint8_t  param_len;          /* number of parameter bytes used */

    uint8_t  enabled;            /* enabled flag */
} route_entry_t;



/* structure to hold route entry crc and timestamp */
typedef struct {
    bool     in_use; /**< in use flag */
    uint16_t crc;    /**< 16-bit CRC */
    uint32_t ts;     /**< UNIX Timestamp */
} route_entry_crc_t;

typedef enum {
    EVENT_ALWAYS,        /**< Fire on every matching message */
    EVENT_ON_CHANGE,     /**< Fire only when payload changes */
    EVENT_ON_RISING,     /**< Fire when value goes 0 → 1 */
    EVENT_ON_FALLING,    /**< Fire when value goes 1 → 0 */
    EVENT_ON_MATCH       /**< Fire when payload matches parameters[] */
} event_type_t;

typedef enum {
    ROUTE_PARAM_0 = 0,
    ROUTE_PARAM_1,
    ROUTE_PARAM_2,
    ROUTE_PARAM_3
} route_param_t;

typedef struct {
    uint16_t actionMsgId;                       // Action is based on existing 16-bit CAN-bus message IDs
    uint8_t  actionMsgDlc;                      /**< Data length code for action message */
    uint8_t  valid;                             // 1 = action exists
    uint8_t  sub_idx;                           // which submodule to act on
    uint8_t  param[ROUTE_ACTION_PARAM_LEN];     // small parameter block
} router_action_t;

/* ============================================================================
 *  GLOBALS
 * ========================================================================== */

extern route_entry_t     g_routes[MAX_ROUTES];
extern route_entry_crc_t g_routesCrc[MAX_ROUTES];

extern volatile bool     g_routeSaveRequested;
extern volatile bool     g_routeLoadRequested;

/* ============================================================================
 *  CALLBACK INTERFACE
 * ========================================================================== */

/* access the CRC routine provided by the application */
#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t (*router_crc_fn_t)(const uint8_t *data, uint16_t length);

void router_set_crc_callback(router_crc_fn_t fn);

#ifdef __cplusplus
}
#endif

/* ============================================================================
 *  C LINKAGE
 * ========================================================================== */

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 *  ROUTING API
 * ========================================================================== */

bool checkRoutes(const can_msg_t *msg, router_action_t *out);
bool evaluate_event(const uint8_t idx, const can_msg_t *msg);
bool payload_matches_parameters(const can_msg_t *msg, uint8_t idx);
bool detect_change(const can_msg_t *msg, uint8_t idx);

/* ============================================================================
 *  ROUTE CONFIGURATION HANDLERS
 * ========================================================================== */

void    handleRouteBegin(const  can_msg_t *msg);
void    handleRouteData(const   can_msg_t *msg);
uint8_t handleRouteEnd(const    can_msg_t *msg);
void    handleRouteDelete(const can_msg_t *msg);
void    handleRoutePurge(const  can_msg_t *msg);

/* ============================================================================
 *  HELPER AND WRAPPER FUNCTIONS
 * ========================================================================== */

void handleRouteWriteNVS(void);
void handleRouteReadNVS(void);

/* ============================================================================
 *  END C LINKAGE
 * ========================================================================== */

#ifdef __cplusplus
}
#endif