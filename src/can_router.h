#pragma once

#include <stdint.h>
#include <string.h>

#include "can_platform.h" /**< Platform specific aliases, can_msg_t becomes twai_message_t on esp32 */
#include "canbus_project.h" /**< required for can message definitions */

/* ============================================================================
 *  CONSTANTS
 * ========================================================================== */

#define MAX_ROUTES (MAX_SUB_MODULES * 2)
#define ROUTE_ENTRY_PARAM_LEN (8U)
#define ROUTE_ENTRY_SOURCE_ID_LEN (4U)
#define ROUTE_ACTION_PARAM_LEN (4U)
#define ROUTE_TAKE_NO_ACTION (0xFFFFU)
#define ROUTE_RX_COMPLETE (0x303U)
#define ROUTE_CMD_START (0x300U)
#define ROUTE_CMD_END (0x31FU)
#define ROUTE_DATA_PAYLOAD_LEN (3U)
#define ROUTE_INVALID_RX (0xFFU)
#define ROUTE_ENTRY_SIZE                                                       \
  (sizeof(route_entry_t))  /**< size of a single route entry */
#define ROUTE_CHUNK_SIZE 3 /**< payload bytes per route entry */
#define ROUTE_CHUNKS_PER_ROUTE                                                 \
  ((ROUTE_ENTRY_SIZE + ROUTE_CHUNK_SIZE - 1) / ROUTE_CHUNK_SIZE)


/* Route control flags*/
#define ROUTE_FLAG_ENABLED         (1u << 0) /**< routing is enabled */
#define ROUTE_FLAG_WILDCARD        (1u << 1) /**< route will match any node id */
#define ROUTE_FLAG_DYNAMIC_PARAMS  (1u << 2) /**< route has uses dynamic parameters (not typical) */
#define ROUTE_FLAG_INUSE           (1u << 3) /**< slot is populated */

/* ============================================================================
 *  DATA STRUCTURES
 * ========================================================================== */

typedef union route_flags {
  struct {
    uint8_t enabled : 1;          /**< routing is enabled */
    uint8_t wildcard : 1;         /**< route will match any node id */
    uint8_t dynamic_params : 1;   /**< route has uses parameters (not typical) */
    uint8_t inUse : 1;            /**< slot is populated */
    uint8_t reserved : 4;         /**< reserved for future use */
  } bits;
  uint8_t raw;
} route_flags_t;

typedef struct __attribute__((packed)) route_entry {
  /* Source information fields */
  uint32_t source_node_id; /* source (producer) node ID */
  uint16_t source_msg_id;  /* source CAN message ID */
  uint8_t source_msg_dlc;  /* source DLC */
  uint8_t source_sub_idx;  /* source submodule index */

  /* Target information fields */
  uint16_t target_msg_id; /* target CAN message ID */
  uint8_t target_msg_dlc; /* target DLC */
  uint8_t target_sub_idx; /* target submodule index */

  /* Routing information fields */
  uint8_t event_type;      /* See event_type_t enum and EVENT_ */
  uint8_t action_type;     /* See action_type_t enum and ACTION_ */
  uint8_t route_flags;     /* Route control flags, see ROUTE_FLAG_ */

  /* Comparison fields */
  union __attribute__((packed)) {
    uint32_t match_value;   /* strict match, masked match, identifier match */
    uint32_t threshold_low; /* lower bound for range */
    uint32_t threshold_center; /* EVENT_WITHIN_TOLERANCE center */
    uint32_t id_low;           /* identifier range low */
  };

  union __attribute__((packed)) {
    uint32_t match_mask;     /* masked match, bit index (0–23) */
    uint32_t threshold_high; /* upper bound for range */
    uint32_t id_high;        /* identifier range high */
    uint32_t bit_index;      /* bit-level comparisons (0–23) */
  };

  /* Static payload parameters */
  uint8_t parameters[3]; /* 3 bytes of static payload parameters */

  /* Reserved for future use */
  uint8_t reserved; /* 1 byte of padding */
} route_entry_t;

typedef enum event_type {
  EVENT_ALWAYS = (0U),            /**< Fire on every matching message */
  EVENT_ON_BINARY_CHANGE = (1U),  /**< Fire only when payload changes */
  EVENT_ON_BINARY_RISING = (2U),  /**< Fire when value goes 0 → 1 */
  EVENT_ON_BINARY_FALLING = (3U), /**< Fire when value goes 1 → 0 */
  EVENT_ON_BINARY_MATCH = (4U), /**< Fire when 3-byte payload is a full match */
  EVENT_ON_FULL_MATCH = (5U),   /**< Fire when 4-byte payload is a full match */
  EVENT_MASKED_BINARY_MATCH =
      (6U), /**< Fire when 3-byte payload is a partial match */
  EVENT_MASKED_FULL_MATCH =
      (7U),                  /**< Fire when 4-byte payload is a partial match */
  EVENT_THRESHOLD_GT = (8U), /**< Fire when value is greater than threshold */
  EVENT_THRESHOLD_LT = (9U), /**< Fire when value is less than threshold */
  EVENT_THRESHOLD_IN_RANGE = (10U),  /**< Fire when value is within range */
  EVENT_THRESHOLD_OUT_RANGE = (11U), /**< Fire when value is outside range */
  EVENT_THRESHOLD_TOLERANCE = (12U), /**< Fire when value is within tolerance */
  EVENT_BIT_SET = (13U),             /**< Fire when bit is set */
  EVENT_BIT_CLEAR = (14U),           /**< Fire when bit is clear */
  EVENT_BIT_TOGGLE = (15U),          /**< Fire when bit is toggled */
  EVENT_BIT_RISING = (16U),          /**< Fire when bit goes 0 → 1 */
  EVENT_BIT_FALLING = (17U),         /**< Fire when bit goes 1 → 0 */
  EVENT_MATCH_BYTE4 = (18U),         /**< Fire when byte 4 matches */
  EVENT_MATCH_BYTE5 = (19U),         /**< Fire when byte 5 matches */
  EVENT_MATCH_BYTE6 = (20U),         /**< Fire when byte 6 matches */
  EVENT_MATCH_BYTE7 = (21U),         /**< Fire when byte 7 matches */
  EVENT_IDENTIFIER_MATCH = (22U),    /**< Fire when identifier matches */
  EVENT_IDENTIFIER_RANGE = (23U), /**< Fire when identifier is within range */
  EVENT_NEVER = (31U)             /**< Never match any message - stop here */

} event_type_t;

typedef enum action_type {
  /** No action */
  ACTION_NO_ACTION = (0U),
  /** Follow binary value */
  ACTION_FOLLOW_BINARY = (1U),
  /** Send synthetic message with dynamic payload */
  ACTION_SEND_DYNAMIC_PAYLOAD = (2U),
  /** Send synthetic message with static payload */
  ACTION_SEND_STATIC_PAYLOAD = (3U),
  /** Send synthetic message with 32-bit payload */
  ACTION_USE_32BIT_PAYLOAD = (4U),
  /** Send synthetic message with 64-bit payload */
  ACTION_USE_64BIT_PAYLOAD = (5U),
  /** Reserved for future use */
  ACTION_RESERVED_6 = (6U),
  /**< Reserved for future use - stop here */
  ACTION_RESERVED_7 = (7U)
} action_type_t;

/* structure to hold route entry data such as last value, crc, etc
   when adding new fields to this typedef be sure to update
   the sanitizer function as well to remove any volatile fields */
typedef struct __attribute__((packed)) route_entry_crc {
  /* utility */
  uint16_t crc; /**< 16-bit CRC */
  uint32_t ts;  /**< last udate in timestamp */
} route_entry_crc_t;

typedef struct __attribute__((packed)) route_entry_data {
  uint32_t ts_matched;   /* last time this route matched */
  uint32_t last_payload; /* last observed incoming payload */
  uint32_t hit_count;    /* number of successful matches */
} route_entry_data_t;

typedef struct router_action {
  /* message identifier and dlc*/
  uint16_t actionMsgId; // 16-bit CAN message ID
  uint8_t actionMsgDlc; // DLC for synthetic message

  /* go or no go flag*/
  uint8_t valid; // 1 = action exists

  /* target submodule*/
  uint8_t sub_idx; // submodule index byte 4

  /* for special actions */
  uint8_t payload_flag;   /**< 0 = use param0:3, 1 = use payload32, 2 = use
                             payload64 */
  uint32_t ext_payload32; // 32-bit payload
  uint64_t ext_payload64; // 64-bit payload

  /* up to three bytes of payload */
  uint8_t param0; // byte 5
  uint8_t param1; // byte 6
  uint8_t param2; // byte 7
} router_action_t;

typedef enum payload_flags {
  PAYLOAD_FLAG_NONE = 0,
  PAYLOAD_FLAG_32BIT = 1,
  PAYLOAD_FLAG_64BIT = 2
} payload_flag_t;

typedef struct event_result {
  bool matched;
  uint32_t value; /* semantic value extracted from message */
} event_result_t;

/* ============================================================================
 *  GLOBALS
 * ========================================================================== */

extern route_entry_t g_routes[MAX_ROUTES];
extern route_entry_crc_t g_routeCrc[MAX_ROUTES];
extern route_entry_data_t g_routeData[MAX_ROUTES];

extern volatile bool g_routeSaveRequested;
extern volatile bool g_routeLoadRequested;

/* ============================================================================
 *  CALLBACK INTERFACE
 * ========================================================================== */

/* access the CRC routine provided by the application */
#ifdef __cplusplus
extern "C" {
#endif

/* crc callback */
typedef uint16_t (*router_crc_fn_t)(const uint8_t *data, uint16_t length);
// typedef void (*lib_log_fn_t)(int level, const char *fmt, ...);

void router_set_crc_callback(router_crc_fn_t fn);
// void libSetLogger(lib_log_fn_t fn);

/* timestamp callback */
typedef uint32_t (*router_timestamp_fn_t)(void);

void router_set_timestamp_callback(router_timestamp_fn_t fn);

/* logging callback */
typedef void (*router_log_fn_t)(const char *msg);

void router_set_log_callback(router_log_fn_t fn);

// static inline route_entry_crc_t makeSanitizedRouteEntryData(const
// route_entry_crc_t *src)
// {
//     route_entry_crc_t out = *src;

//     /* loop through the route data entries */
//     for (uint8_t i = 0; i < MAX_ROUTES; i++)
//     {
//         out.lastValue = 0;

//     }
//     return out;
// }

/* ============================================================================
 *  ROUTING API
 * ========================================================================== */

bool checkRoutes(can_msg_t *msg, uint32_t my_node_id, router_action_t *out);

/* route flag accessors */
bool routerIsRouteInUse(const uint8_t i);
bool routerIsRouteEnabled(const uint8_t i);
bool routerIsRouteWildcard(const uint8_t i);

void routerSetRouteUseFlag(const uint8_t i);
void routerClearRouteUseFlag(const uint8_t i);

void routerSetRouteEnableFlag(const uint8_t i);
void routerClearRouteEnableFlag(const uint8_t i);

void routerClearRouteWildcardFlag(const uint8_t i);
void routerSetRouteWildcardFlag(const uint8_t i);


uint8_t routerGetRouteCount(void);
uint8_t routerGetEnabledRouteCount(void);
uint8_t routerGetMaxRouteCount(void);
void prettyPrintRoutes(void);

/* route table serialization */
int router_serialize_all_routes(const uint32_t node_id, can_msg_t *out, int max_msgs);
int router_get_active_route_count(void);
uint8_t router_get_msgs_per_route(void);
uint16_t router_get_total_msgs_required(void);

/* ============================================================================
 *  END C LINKAGE
 * ========================================================================== */

#ifdef __cplusplus
}
#endif