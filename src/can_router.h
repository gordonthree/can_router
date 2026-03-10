#pragma once

#include <stdint.h>
#include "can_platform.h" /**< Platform specific aliases, can_msg_t becomes twai_message_t on esp32 */

/* ============================================================================
 *  CONSTANTS
 * ========================================================================== */

#define MAX_ROUTES                 (8U)
#define ROUTE_ENTRY_PARAM_LEN      (8U)
#define ROUTE_ENTRY_SOURCE_ID_LEN  (4U)
#define ROUTE_ACTION_PARAM_LEN     (4U)
#define ROUTE_TAKE_NO_ACTION       (0xFFFFU)
#define ROUTE_CMD_START            (0x300U)
#define ROUTE_CMD_END              (0x3FFU)

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

typedef enum {
    MSG_DATA_0 = 0,
    MSG_DATA_1,
    MSG_DATA_2,
    MSG_DATA_3,
    MSG_DATA_4,
    MSG_DATA_5,
    MSG_DATA_6,
    MSG_DATA_7
} msg_data_t;


typedef struct {
    uint16_t actionMsgId;                       // Action is based on existing 16-bit CAN-bus message IDs
    uint8_t  valid;                             // 1 = action exists
    uint8_t  sub_idx;                           // which submodule to act on
    uint8_t  param[ROUTE_ACTION_PARAM_LEN];     // small parameter block
} router_action_t;

/* ============================================================================
 *  GLOBALS
 * ========================================================================== */

extern route_entry_t   g_routes[MAX_ROUTES];
extern volatile bool g_routeSaveRequested;
extern volatile bool g_routeLoadRequested;

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

void handleRouteBegin(const  can_msg_t *msg);
void handleRouteData(const   can_msg_t *msg);
void handleRouteEnd(const    can_msg_t *msg);
void handleRouteDelete(const can_msg_t *msg);
void handleRoutePurge(const  can_msg_t *msg);

/* ============================================================================
 *  HELPER AND WRAPPER FUNCTIONS
 * ========================================================================== */

void handleRouteWriteNVS(void);
void handleRouteReadNVS(void);

/* ============================================================================
 *  NVS LOAD/SAVE API (platform-specific)
 * ========================================================================== */

void loadRouteTableFromNVS(void);
void saveRouteTableToNVS(void);
