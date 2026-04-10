#include "can_router.h"

#include "esp_log.h"

const char *TAG = "can_router";

typedef bool (*binary_predicate_fn)(uint32_t old_val, uint32_t new_val);

/* ============================================================================
 *  LOCAL FUNCTION DECLARATIONS
 * ========================================================================== */

/* helpers */
static uint32_t pack_payload(const can_msg_t *msg);
static uint32_t pack_full_payload(const can_msg_t *msg);
static uint8_t countActiveRoutes(void);
static inline uint8_t get_binary_state(uint8_t idx);

/* action generator functions */
static void action_generator(const route_entry_t *r, const uint8_t idx,
                             router_action_t *out);

static void generate_action_follow_binary(const route_entry_t *r,
                                          const uint8_t idx,
                                          router_action_t *out);

static void generate_action_send_static_payload(const route_entry_t *r,
                                                uint8_t idx,
                                                router_action_t *out);

static void generate_action_send_dynamic_payload(const route_entry_t *r,
                                                 uint8_t idx,
                                                 router_action_t *out);

/* predicate functions */
static bool pred_binary_change(uint32_t old_val, uint32_t new_val);
static bool pred_binary_rising(uint32_t old_val, uint32_t new_val);
static bool pred_binary_falling(uint32_t old_val, uint32_t new_val);

/* comparison functions */
static bool detect_binary_event(const can_msg_t *msg, uint8_t idx,
                                binary_predicate_fn pred);
static bool detect_binary_rising(const can_msg_t *msg, uint8_t idx);
static bool detect_binary_falling(const can_msg_t *msg, uint8_t idx);
static bool detect_binary_change(const can_msg_t *msg, uint8_t idx);
static bool detect_binary_match(const can_msg_t *msg, uint8_t idx);
static bool detect_binary_match_full(const can_msg_t *msg, uint8_t idx);
static bool detect_masked_binary_match(const can_msg_t *msg, uint8_t idx);
static bool detect_masked_binary_match_full(const can_msg_t *msg, uint8_t idx);

/* event evaluation */
static event_result_t evaluate_event(const uint8_t idx, const can_msg_t *msg);

/* message and command handlers */
static bool process_router_command(const can_msg_t *msg, uint8_t *rxIdx_out,
                                   router_action_t *out);

static void handleRouteBegin(const can_msg_t *msg);
static void handleRouteData(const can_msg_t *msg);
static uint8_t handleRouteEnd(const can_msg_t *msg);
static bool routeValidateSave(const uint8_t rxIdx, router_action_t *out);
static void handleRouteDelete(const can_msg_t *msg);
static void handleRoutePurge(const can_msg_t *msg);

/* NVS handlers */
static void handleRouteWriteNVS(void);
static void handleRouteReadNVS(void);

/* serialization */
static int serializeRoute(uint8_t route_idx, can_msg_t *out, int maxMsgs);

/* ============================================================================
 *  GLOBAL STORAGE
 * ========================================================================== */

route_entry_t g_routes[MAX_ROUTES] = {0}; /* zero initializes the struct */
route_entry_crc_t g_routeCrc[MAX_ROUTES] = {
    0}; /* zero initializes the struct */
route_entry_data_t g_routeData[MAX_ROUTES] = {
    0}; /* zero initializes the struct */

volatile bool g_routeSaveRequested = false;
volatile bool g_routeLoadRequested = false;

// static uint8_t g_last_values[MAX_ROUTES] = {0};

/* ============================================================================
 *  LOCAL STORAGE
 * ========================================================================== */

/* Temporary buffer for multi-frame route reception */
static route_entry_t rxRouteBuffer = {0};
static uint8_t rxRouteSlot = 0xFF; /**< slot in the route table */
static uint8_t rxRouteTotal = 0;   /**< store expected chunk count */
static uint8_t rxRouteReceived =
    0; /**< track how many data frames we received */

/* ============================================================================
 *  CALLBACK FUNCTIONS
 * ========================================================================== */

/* Callback setters */
static router_crc_fn_t g_crc_callback = NULL;
static router_timestamp_fn_t g_timestamp_cb = NULL;
static router_log_fn_t g_router_log_cb = NULL;

/** @brief Set the CRC callback */
void router_set_crc_callback(router_crc_fn_t fn) { 
    g_crc_callback = fn; 
}

/** @brief Set the timestamp callback */
void router_set_timestamp_callback(router_timestamp_fn_t fn) {
  g_timestamp_cb = fn;
}

/** @brief Set the logging callback */
void router_set_log_callback(router_log_fn_t fn) { 
    g_router_log_cb = fn; 
}

/* ============================================================================
 *  HELPER FUNCTIONS
 * ========================================================================== */

/**
 * @brief Pack 3 bytes of a CAN message into a single 32-bit value
 * @param msg Pointer to a CAN message structure
 * @return A single 32-bit value with the 3 bytes packed in
 *
 * The 3 bytes are packed as follows:
 *   - LSB: `msg->data[MSG_DATA_5]`
 *   - Middle byte: `msg->data[MSG_DATA_6]` shifted left by 8 bits
 *   - MSB: `msg->data[MSG_DATA_7]` shifted left by 16 bits
 */
static uint32_t pack_payload(const can_msg_t *msg) {
  uint32_t packed = 0;
  packed |= msg->data[MSG_DATA_5];       /* LSB */
  packed |= msg->data[MSG_DATA_6] << 8;  /* middle */
  packed |= msg->data[MSG_DATA_7] << 16; /* MSB */
  return packed;
}

/**
 * @brief Pack all 4 bytes of a CAN message into a single 32-bit value
 * @param msg Pointer to a CAN message structure
 * @return A single 32-bit value with the 4 bytes packed in
 *
 * The 4 bytes are packed as follows:
 *   - LSB: `msg->data[MSG_DATA_4]`
 *   - Next byte: `msg->data[MSG_DATA_5]` shifted left by 8 bits
 *   - Next byte: `msg->data[MSG_DATA_6]` shifted left by 16 bits
 *   - MSB: `msg->data[MSG_DATA_7]` shifted left by 24 bits
 */
static uint32_t pack_full_payload(const can_msg_t *msg) {
  uint32_t packed = 0;
  packed |= (uint32_t)msg->data[MSG_DATA_4];       /* LSB */
  packed |= (uint32_t)msg->data[MSG_DATA_5] << 8;  /* next */
  packed |= (uint32_t)msg->data[MSG_DATA_6] << 16; /* next */
  packed |= (uint32_t)msg->data[MSG_DATA_7] << 24; /* MSB */
  return packed;
}

static inline uint8_t get_binary_state(uint8_t idx) {
  return (g_routeData[idx].last_payload & 0x01U); /* return LSB only */
}

/* ============================================================================
 *  ROUTE DETECTION FUNCTIONS
 * ========================================================================== */

static bool detect_binary_event(const can_msg_t *msg, uint8_t idx,
                                binary_predicate_fn pred) {
  const uint32_t new_val = pack_payload(msg);
  const uint32_t old_val = g_routeData[idx].last_payload;

  g_routeData[idx].last_payload = new_val;

  return pred(old_val, new_val);
}

static bool pred_binary_change(uint32_t old_val, uint32_t new_val) {
  return (old_val != new_val);
}

static bool pred_binary_rising(uint32_t old_val, uint32_t new_val) {
  return (old_val == 0 && new_val != 0);
}

static bool pred_binary_falling(uint32_t old_val, uint32_t new_val) {
  return (old_val != 0 && new_val == 0);
}

/**
 * @brief Detect a rising edge (0 → 1) on the entire payload.
 *
 * @param msg The CAN message to check.
 * @param idx The index of the route entry to check against.
 *
 * @return true if the payload matches the parameters and a rising edge
 * occurred.
 */
static bool detect_binary_rising(const can_msg_t *msg, uint8_t idx) {
  return detect_binary_event(msg, idx, pred_binary_rising);
}

/**
 * @brief Detect a falling edge (1 → 0) on the entire payload.
 *
 * @param msg The CAN message to check.
 * @param idx The index of the route entry to check against.
 *
 * @return true if the payload matches the parameters and a falling edge
 * occurred.
 */
static bool detect_binary_falling(const can_msg_t *msg, uint8_t idx) {
  return detect_binary_event(msg, idx, pred_binary_falling);
}

/**
 * @brief Detect a change (0 → 1 or 1 → 0) on the entire payload.
 *
 * @param msg The CAN message to check.
 * @param idx The index of the route entry to check against.
 *
 * @return true if the payload matches the parameters and a change occurred.
 */
static bool detect_binary_change(const can_msg_t *msg, uint8_t idx) {
  return detect_binary_event(msg, idx, pred_binary_change);
}

/**
 * @brief Detect a match on the entire 3-byte payload.
 *
 * @param msg The CAN message to check.
 * @param idx The index of the route entry to check against.
 *
 * @return true if the 3-byte payload matches the parameters.
 */
static bool detect_binary_match(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* pack only 3-byte payload */
  g_routeData[idx].last_payload = incoming;    /* update lastPayload */

  return (incoming == r->match_value);
}

/**
 * @brief Check if the incoming payload matches the parameters set in the
 *  route entry.
 *
 * @param msg The CAN message to check.
 * @param idx The index of the route entry to check against.
 *
 * @return true if the payload matches the parameters, false otherwise.
 */
static bool detect_binary_match_full(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];

  const uint32_t incoming = pack_full_payload(msg);
  g_routeData[idx].last_payload = incoming; /* update lastPayload */

  return (incoming == r->match_value);
}

static bool detect_masked_binary_match(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* 3-byte payload */
  g_routeData[idx].last_payload = incoming;

  return ((incoming & r->match_mask) == (r->match_value & r->match_mask));
}

static bool detect_masked_binary_match_full(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_full_payload(msg); /* 4-byte payload */
  g_routeData[idx].last_payload = incoming;

  return ((incoming & r->match_mask) == (r->match_value & r->match_mask));
}

static bool detect_threshold_gt(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* 3-byte payload */
  g_routeData[idx].last_payload = incoming;

  return (incoming > r->threshold_low);
}
static bool detect_threshold_lt(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  g_routeData[idx].last_payload = incoming;

  return (incoming < r->threshold_low);
}
static bool detect_threshold_in_range(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  g_routeData[idx].last_payload = incoming;

  return (incoming >= r->threshold_low && incoming <= r->threshold_high);
}

static bool detect_threshold_out_range(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* 24-bit payload */
  g_routeData[idx].last_payload = incoming;

  return (incoming < r->threshold_low || incoming > r->threshold_high);
}

static bool detect_within_tolerance(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  g_routeData[idx].last_payload = incoming;

  const uint32_t center = r->match_value;
  const uint32_t tol = r->threshold_low;

  return (incoming >= (center - tol) && incoming <= (center + tol));
}

static bool detect_bit_set(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* 24-bit payload */
  g_routeData[idx].last_payload = incoming;

  const uint8_t bit = (uint8_t)r->match_mask; /* 0–23 */
  return ((incoming >> bit) & 1U) == 1U;
}

static bool detect_bit_clear(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  g_routeData[idx].last_payload = incoming;

  const uint8_t bit = (uint8_t)r->match_mask;
  return ((incoming >> bit) & 1U) == 0U;
}

static bool detect_bit_toggle(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  const uint32_t old = g_routeData[idx].last_payload;

  g_routeData[idx].last_payload = incoming;

  const uint8_t bit = (uint8_t)r->match_mask;

  const uint8_t old_bit = (old >> bit) & 1U;
  const uint8_t new_bit = (incoming >> bit) & 1U;

  return old_bit != new_bit;
}

static bool detect_bit_rising(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg); /* 24-bit payload */
  const uint32_t old = g_routeData[idx].last_payload;

  g_routeData[idx].last_payload = incoming;

  const uint8_t bit = (uint8_t)r->match_mask; /* 0–23 */

  const uint8_t old_bit = (old >> bit) & 1U;
  const uint8_t new_bit = (incoming >> bit) & 1U;

  return (old_bit == 0U && new_bit == 1U);
}

static bool detect_bit_falling(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];
  const uint32_t incoming = pack_payload(msg);
  const uint32_t old = g_routeData[idx].last_payload;

  g_routeData[idx].last_payload = incoming;

  const uint8_t bit = (uint8_t)r->match_mask;

  const uint8_t old_bit = (old >> bit) & 1U;
  const uint8_t new_bit = (incoming >> bit) & 1U;

  return (old_bit == 1U && new_bit == 0U);
}

static bool detect_match_byte(const can_msg_t *msg, uint8_t idx,
                              uint8_t byte_index) {
  if (byte_index < 4 || byte_index > 7)
    return false; /* invalid byte_index */

  const route_entry_t *r = &g_routes[idx];

  const uint32_t incoming = pack_full_payload(msg);
  g_routeData[idx].last_payload = incoming;

  /* byte_index must be 4–7 */
  const uint8_t value = msg->data[byte_index];

  return (value == (uint8_t)r->match_value);
}

static bool detect_identifier_match(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];

  /* Update last_payload for consistency, even though ID is not payload */
  const uint32_t incoming = pack_full_payload(msg);
  g_routeData[idx].last_payload = incoming;

  return (msg->identifier == r->match_value);
}

static bool detect_identifier_range(const can_msg_t *msg, uint8_t idx) {
  const route_entry_t *r = &g_routes[idx];

  const uint32_t incoming = pack_full_payload(msg);
  g_routeData[idx].last_payload = incoming;

  const uint32_t id = msg->identifier;

  return (id >= r->threshold_low && id <= r->threshold_high);
}

/* ============================================================================
 *  ACTION GENERATOR FUNCTIONS
 * ========================================================================== */

static void action_generator(const route_entry_t *r, const uint8_t idx,
                             router_action_t *out) {

  // router_action_t act = {0}; /* zero out action buffer */

  switch (r->ea.bits.action_type) {
    case ACTION_FOLLOW_BINARY:
      {
        generate_action_follow_binary(r, idx, out);
        break;
      }

    case ACTION_SEND_STATIC_PAYLOAD:
      {
        generate_action_send_static_payload(r, idx, out);
        break;
      }

    case ACTION_SEND_DYNAMIC_PAYLOAD:
      {
        generate_action_send_dynamic_payload(r, idx, out);
        break;
      }

    default:
      /* no action or not implemented yet */
      break;
  }
}

static void generate_action_send_static_payload(const route_entry_t *r,
                                                uint8_t idx,
                                                router_action_t *out) {
  /* Route matched — fill out semantic action */
  out->valid = 1;                      /* 1 = valid command */
  out->actionMsgId = r->target_msg_id; /* may be 0xFFFF */
  out->actionMsgDlc = r->target_msg_dlc;
  out->sub_idx = r->target_sub_idx; /* msg data byte 4 */

  /* Load paramater bytes 0 to 3 with data from the route table */
  out->param0 = r->parameters[0];
  out->param1 = r->parameters[1];
  out->param2 = r->parameters[2];
}

static void generate_action_send_dynamic_payload(const route_entry_t *r,
                                                 uint8_t idx,
                                                 router_action_t *out) {
  /* Mark action as valid */
  out->valid = 1;

  /* Routing metadata (caller may have set these already) */
  out->actionMsgId = r->target_msg_id;
  out->actionMsgDlc = r->target_msg_dlc;
  out->sub_idx = r->target_sub_idx;

  /* Extract the last observed payload (24-bit packed) */
  const uint32_t p = g_routeData[idx].last_payload;

  /* Fill the synthetic message payload */
  out->param0 = (uint8_t)(p & 0xFFU);
  out->param1 = (uint8_t)((p >> 8) & 0xFFU);
  out->param2 = (uint8_t)((p >> 16) & 0xFFU);
}

static void generate_action_follow_binary(const route_entry_t *r, uint8_t idx,
                                          router_action_t *out) {
  /* Extract binary state (0 or 1) */
  const uint8_t state = get_binary_state(idx);

  /* User parameters: parameters[0] = OFF, parameters[1] = ON */
  const uint8_t off_val = r->parameters[0];
  const uint8_t on_val = r->parameters[1];

  const uint8_t payload = state ? on_val : off_val;

  out->actionMsgId = r->target_msg_id;
  out->actionMsgDlc = r->target_msg_dlc;
  out->sub_idx = r->target_sub_idx;

  out->param0 = payload;
  out->param1 = 0;
  out->param2 = 0;

  out->valid = 1;
}

/* ============================================================================
 *  ROUTING: MULTI-FRAME SENDER
 * ========================================================================== */

/**
 * @brief Count and return the number of active routes.
 *
 * This function returns the number of active routes in the routing table.
 * Active routes are routes that are in use (i.e., in_use == true).
 *
 * @return int The number of active routes.
 */
int routerGetActiveRouteCount(void) {
  int ret = 0;
  for (uint8_t i = 0; i < MAX_ROUTES; i++) {
    if (!routerIsRouteInUse(i))
      continue;

    ret++;
  }
  return ret;
}

/**
 * @brief Check if a route is in use.
 *
 * This function checks if a route at index i is in use.
 *
 * @param i The index of the route to check.
 * @return bool True if the route is in use, false otherwise.
 */
bool routerIsRouteInUse(const uint8_t i) {
  return g_routes[i].flags.bits.inUse;
}

void routerSetRouteUseFlag(const uint8_t i, const bool use) {
  g_routes[i].flags.bits.inUse = use;
}

bool routerIsRouteEnabled(const uint8_t i) {
  return g_routes[i].flags.bits.enabled;
}

void routerSetRouteEnableFlag(const uint8_t i, const bool enable) {
  g_routes[i].flags.bits.enabled = enable;
}

bool routerIsRouteWildcard(const uint8_t i) {
  return g_routes[i].flags.bits.wildcard;
}

void routerSetRouteWildcardFlag(const uint8_t i, const bool enable) {
  g_routes[i].flags.bits.wildcard = enable;
}

uint16_t routerGetMsgsPerRoute(void) {
  const uint16_t chunks =
      (sizeof(route_entry_t) + ROUTE_CHUNK_SIZE - 1) / ROUTE_CHUNK_SIZE;

  return chunks + 2; // +1 for RouteBegin, +1 for RouteEnd
}

int serializeAllRoutes(can_msg_t *out, int maxMsgs) {
  int total = 0;

  for (uint8_t i = 0; i < MAX_ROUTES; i++) {
    if (!routerIsRouteInUse(i))
      continue;

    int written = serializeRoute(i, &out[total], maxMsgs - total);
    if (written < 0)
      return -1; // caller didn't allocate enough space

    total += written;
  }

  return total;
}

static int serializeRoute(uint8_t route_idx, can_msg_t *out, int maxMsgs) {
  int count = 0;

  if (maxMsgs < ROUTE_CHUNKS_PER_ROUTE + 2)
    return -1; // not enough space

  /* RouteBegin */
  can_msg_t begin = {0};
  begin.identifier = CFG_ROUTE_BEGIN_ID;
  begin.data_length_code = CFG_ROUTE_BEGIN_DLC;
  begin.data[4] = route_idx;
  begin.data[5] = ROUTE_CHUNKS_PER_ROUTE;
  out[count++] = begin;

  /* RouteData */
  const uint8_t *src = (const uint8_t *)&g_routes[route_idx];

  for (uint8_t chunk = 0; chunk < ROUTE_CHUNKS_PER_ROUTE; chunk++) {
    can_msg_t msg = {0};
    msg.identifier = CFG_ROUTE_DATA_ID;
    msg.data_length_code = CFG_ROUTE_DATA_DLC;

    msg.data[4] = chunk;

    uint8_t base = chunk * ROUTE_DATA_PAYLOAD_LEN;
    msg.data[5] = src[base + 0];
    msg.data[6] = src[base + 1];
    msg.data[7] = src[base + 2];

    out[count++] = msg;
  }

  /* RouteEnd */
  can_msg_t end = {0};
  end.identifier = CFG_ROUTE_END_ID;
  end.data_length_code = CFG_ROUTE_END_DLC;
  end.data[4] = route_idx;
  end.data[5] = 1; // commit flag
  out[count++] = end;

  return count;
}

/* ============================================================================
 *  ROUTING: MULTI-FRAME RECEIVER
 * ========================================================================== */

static void handleRouteBegin(const can_msg_t *msg) {
  // printf("[ROUTER] handleRouteBegin()\n");

  if (msg->data_length_code < (uint8_t)CFG_ROUTE_BEGIN_DLC) {
    printf("[ROUTER] Error: invalid data length code, command %u ignored\n",
           msg->identifier);
    return;
  }

  uint8_t route_idx =
      msg->data[MSG_DATA_4]; /* data byte 4 is the index value */
  uint8_t total_chunks =
      msg->data[5]; /* data byte 5 is the total chunk count */

  if (route_idx >= (uint8_t)MAX_ROUTES) {
    printf("[ROUTER] Error: invalid route index, command %u ignored\n",
           msg->identifier);
    return;
  }

  if (total_chunks != (uint8_t)ROUTE_CHUNKS_PER_ROUTE) {
    printf("[ROUTER] Error: invalid chunk count %u we expected %u, command %u "
           "ignored\n",
           total_chunks, (uint8_t)ROUTE_CHUNKS_PER_ROUTE, msg->identifier);
    return;
  }

  rxRouteSlot = route_idx;
  rxRouteTotal = total_chunks; /* store expected chunk count */
  rxRouteReceived = 0;         /* track how many we got */

  g_routeCrc[route_idx].crc = 0xFFFF;         /* reset crc */
  g_routeCrc[route_idx].ts = 0x00;            /* reset timestamp */
  routerSetRouteUseFlag(route_idx, true);     /* reset in_use flag */
  routerSetRouteEnableFlag(route_idx, false); /* reset enabled flag */

  memset(&rxRouteBuffer, 0, sizeof(rxRouteBuffer));
  printf("[ROUTER] Route begin, index: %u chunks: %u\n", route_idx,
         total_chunks);
}

static void handleRouteData(const can_msg_t *msg) {
  if (msg->data_length_code < CFG_ROUTE_DATA_DLC)
    return;
  if (rxRouteSlot >= MAX_ROUTES)
    return;

  uint8_t chunk_idx = msg->data[4]; // <-- NEW: chunk index
  uint8_t *dst = (uint8_t *)&rxRouteBuffer;

  /* Each chunk carries 3 bytes: D5, D6, D7 */
  uint8_t base = chunk_idx * ROUTE_DATA_PAYLOAD_LEN; /* payload is 3 bytes */

  if (base >= sizeof(route_entry_t))
    return;

  if (base + 0 < sizeof(route_entry_t))
    dst[base + 0] = msg->data[5];
  if (base + 1 < sizeof(route_entry_t))
    dst[base + 1] = msg->data[6];
  if (base + 2 < sizeof(route_entry_t))
    dst[base + 2] = msg->data[7];

  rxRouteReceived++; // <-- NEW: count chunks
  printf("[ROUTER] Route data: Route %u chunk received %u\n", rxRouteReceived,
         chunk_idx);
}

static uint8_t handleRouteEnd(const can_msg_t *msg) {

  if (msg->data_length_code < 6)
    return ROUTE_INVALID_RX;

  uint8_t route_idx = msg->data[4];
  uint8_t commitFlag = msg->data[5];

  if (route_idx != rxRouteSlot) {
    ESP_LOGW(TAG, "[ROUTER] Error: route_idx (%u) != rxRouteSlot (%u)",
             route_idx, rxRouteSlot);
    return ROUTE_INVALID_RX;
  }

  if (!commitFlag) {
    ESP_LOGW(TAG, "[ROUTER] Error: commit flag not set");
    return ROUTE_INVALID_RX; // ignore incomplete transfers
  }

  if (rxRouteReceived != rxRouteTotal) {
    ESP_LOGW(TAG, "[ROUTER] Error: rxRouteReceived (%u) != rxRouteTotal (%u)",
             rxRouteReceived, rxRouteTotal);
    return ROUTE_INVALID_RX; // incomplete or missing chunks
  }

  memcpy(&g_routes[rxRouteSlot], &rxRouteBuffer, sizeof(route_entry_t));
  routerSetRouteEnableFlag(
      rxRouteSlot, true); /* set enabled flag even if the sender did not */

  ESP_LOGI(TAG, "[ROUTER] Route %u programming complete", rxRouteSlot);

  const uint8_t savedIdx = rxRouteSlot;

  routerSetRouteUseFlag(savedIdx, true); /* set in_use flag */

  uint16_t crc = 0xFFFF;

  if (g_crc_callback) {
    crc = g_crc_callback((const uint8_t *)&g_routes[savedIdx],
                         sizeof(route_entry_t));
  }

  g_routeCrc[savedIdx].crc = crc;

  /* save routing table to nvs */
  handleRouteWriteNVS();

  /* reset pointer */
  rxRouteSlot = 0xFF;

  /* let the route command handler know the rx was complete */
  return savedIdx;
}

static bool routeValidateSave(const uint8_t rxIdx, router_action_t *out) {
  if (rxIdx !=
      ROUTE_INVALID_RX) { /* test if route saved successfully, if so send ack */
    out->valid = 1;
    out->actionMsgId = DATA_ROUTE_ACK_ID;
    out->actionMsgDlc = DATA_ROUTE_ACK_DLC;
    out->sub_idx = rxIdx; /* return the route index */
    out->param0 = ((g_routeCrc[rxIdx].crc >> 8) & 0xFF);
    out->param1 = (g_routeCrc[rxIdx].crc & 0xFF);
    out->param2 = 0; // zero it out, not used
    printf("[ROUTER] Route index: %u saved with CRC16: 0x%04X\n", rxIdx,
           g_routeCrc[rxIdx].crc);

    return true;
  }

  /* route save was not successful */
  out->valid = 1; /* valid command */
  /* router commands have no action in the consumer */
  out->actionMsgId = ROUTE_TAKE_NO_ACTION;
  printf("[ROUTER] Route index: %u save failed.\n", rxIdx);

  return false;
}

static void handleRouteDelete(const can_msg_t *msg) {
  if (msg->data_length_code < CFG_ROUTE_DELETE_DLC)
    return;

  uint8_t idx = msg->data[MSG_DATA_4];
  if (idx >= MAX_ROUTES)
    return;

  memset(&g_routes[idx], 0, sizeof(route_entry_t));
}

static void handleRoutePurge(const can_msg_t *msg) {
  (void)msg;
  memset(g_routes, 0, sizeof(g_routes));
}

static void handleRouteWriteNVS(void) { g_routeSaveRequested = true; }

static void handleRouteReadNVS(void) { g_routeLoadRequested = true; }

/* ============================================================================
 *  ROUTE EXECUTION HOOK
 * ========================================================================== */

static bool process_router_command(const can_msg_t *msg, uint8_t *rxIdx_out,
                                   router_action_t *out) {
  switch (msg->identifier) {
    case CFG_ROUTE_BEGIN_ID: handleRouteBegin(msg); return true;

    case CFG_ROUTE_DATA_ID: handleRouteData(msg); return true;

    case CFG_ROUTE_END_ID:
      {
        const uint8_t rxIdx = handleRouteEnd(msg);
        *rxIdx_out = rxIdx; /* zero is a valid return value */

        if (rxIdx != ROUTE_INVALID_RX) { /* test if route saved successfully,
                                            0xFF indicates failure */
          const bool success = routeValidateSave(rxIdx, out);
          return success;
        }

        return false;
      }

    case REQ_ROUTE_LIST_ID:
      out->valid = 1;
      out->actionMsgId = REQ_ROUTE_LIST_ID;
      out->actionMsgDlc = 0;
      return true;

    case CFG_ROUTE_DELETE_ID: handleRouteDelete(msg); return true;

    case CFG_ROUTE_PURGE_ID: handleRoutePurge(msg); return true;

    case CFG_ROUTE_WRITE_NVS_ID: handleRouteWriteNVS(); return true;

    case CFG_ROUTE_READ_NVS_ID: handleRouteReadNVS(); return true;

    default:
      printf("[ROUTER] INVALID CONFIG MESSAGE: 0x%03X\n", msg->identifier);
      return true; // still consumed
  }
}

bool checkRoutes(can_msg_t *msg, router_action_t *out) {

  // Default: no action
  out->valid = 0;
  out->actionMsgId = ROUTE_TAKE_NO_ACTION;
  out->sub_idx = 0;
  out->param0 = 0;
  out->param1 = 0;
  out->param2 = 0;

  // printf("[ROUTER] CHECKING MESSAGE: 0x%03X\n", msg->identifier);

  /* ---------------------------------------------------------
   * CONFIGURATION COMMANDS (0x300–0x3FF)
   * --------------------------------------------------------- */
  if (msg->identifier >= ROUTE_CMD_START &&
      msg->identifier <=
          ROUTE_CMD_END) // command message is between 0x300 and 0x31F
  {
    return process_router_command(msg, NULL, out);
  }

  /* ---------------------------------------------------------
   * ROUTE EXECUTION (normal CAN traffic)
   * --------------------------------------------------------- */
  for (uint8_t i = 0; i < MAX_ROUTES; i++) {

    route_entry_t *r = &g_routes[i];

    /* Ignore disabled routes */
    if (!routerIsRouteEnabled(i))
      continue;

    /* Ignore routes that don't match the source message ID */
    if (msg->identifier != r->source_msg_id)
      continue;

    /* retrieve source node ID from payload, formatted big endian */
    const uint32_t sourceNodeId =
        (uint32_t)(msg->data[MSG_DATA_0] << 24 | msg->data[MSG_DATA_1] << 16 |
                   msg->data[MSG_DATA_2] << 8 | msg->data[MSG_DATA_3]);

    /* Ignore routes that don't match the source node ID,
    unless this route has the wildcard bit set, in which case it
    should match all producers */
    if (!routerIsRouteWildcard(i)) {
      if (sourceNodeId != r->source_node_id)
        continue;
    }

    /* Evaluate event type */
    const event_result_t er = evaluate_event(i, msg);
    if (!er.matched)
      continue;

    /* Generate semantic action */
    action_generator(r, i, out);

    return true; /* route matched and has data for the consumer */
  }

  return false; // <--- NO ROUTE MATCHED
}

/**
 * @brief Determine whether a route should fire based on its event type.
 *
 * @param idx The index of the route entry to check.
 * @param msg The CAN message to check.
 * @return true if the route should fire, false otherwise.
 * @note See event_type_t in can_router.h for a list of event types.
 *
 */
static event_result_t evaluate_event(const uint8_t idx, const can_msg_t *msg) {
  event_result_t er = {0}; /* initialize event result */
  er.matched = false;      /* default to no match */

  bool return_value = false;

  const route_entry_t *r = &g_routes[idx];

  switch (r->ea.bits.event_type) {
    case EVENT_ALWAYS:
      {
        return_value = true;
        break;
      }
    case EVENT_ON_BINARY_CHANGE:
      {
        return_value = detect_binary_change(msg, idx);
        break;
      }
    case EVENT_ON_BINARY_RISING:
      {
        return_value = detect_binary_rising(msg, idx);
        break;
      }
    case EVENT_ON_BINARY_FALLING:
      {
        return_value = detect_binary_falling(msg, idx);
        break;
      }
    case EVENT_ON_BINARY_MATCH:
      {
        return_value = detect_binary_match(msg, idx);
        break;
      }
    case EVENT_ON_FULL_MATCH:
      {
        return_value = detect_binary_match_full(msg, idx);
        break;
      }
    case EVENT_THRESHOLD_GT:
      {
        return_value = detect_threshold_gt(msg, idx);
        break;
      }
    case EVENT_THRESHOLD_LT:
      {
        return_value = detect_threshold_lt(msg, idx);
        break;
      }
    case EVENT_THRESHOLD_IN_RANGE:
      {
        return_value = detect_threshold_in_range(msg, idx);
        break;
      }
    case EVENT_THRESHOLD_OUT_RANGE:
      {
        return_value = detect_threshold_out_range(msg, idx);
        break;
      }
    case EVENT_THRESHOLD_TOLERANCE:
      {
        return_value = detect_within_tolerance(msg, idx);
        break;
      }
    case EVENT_BIT_SET:
      {
        return_value = detect_bit_set(msg, idx);
        break;
      }
    case EVENT_BIT_CLEAR:
      {
        return_value = detect_bit_clear(msg, idx);
        break;
      }
    case EVENT_BIT_TOGGLE:
      {
        return_value = detect_bit_toggle(msg, idx);
        break;
      }
    case EVENT_BIT_RISING:
      {
        return_value = detect_bit_rising(msg, idx);
        break;
      }
    case EVENT_BIT_FALLING:
      {
        return_value = detect_bit_falling(msg, idx);
        break;
      }
    case EVENT_MATCH_BYTE4:
      {
        return_value = detect_match_byte(msg, idx, MSG_DATA_4);
        break;
      }
    case EVENT_MATCH_BYTE5:
      {
        return_value = detect_match_byte(msg, idx, MSG_DATA_5);
        break;
      }
    case EVENT_MATCH_BYTE6:
      {
        return_value = detect_match_byte(msg, idx, MSG_DATA_6);
        break;
      }

    case EVENT_MATCH_BYTE7:
      {
        return_value = detect_match_byte(msg, idx, MSG_DATA_7);
        break;
      }

    case EVENT_IDENTIFIER_MATCH:
      {
        return_value = detect_identifier_match(msg, idx);
        break;
      }

    case EVENT_IDENTIFIER_RANGE:
      {
        return_value = detect_identifier_range(msg, idx);
        break;
      }

    case EVENT_NEVER: /* bookend event type 31 */
    default:
      {
        return_value = false;
        break;
      }
  }

  er.matched = return_value;
  er.value = 0;

  if (return_value) {
    g_routeData[idx].hit_count++;

    if (g_timestamp_cb)
      g_routeData[idx].ts_matched = g_timestamp_cb();
    else
      g_routeData[idx].ts_matched = 0; /* fallback */
  }

  return er;
}

/**
 * @brief Return the number of populated routing table entries.
 *
 * This function returns the number of slots in the routing table
 * that are currently populated (i.e., inuse == true).
 *
 * @return uint8_t Number of populated routing table entries.
 */
static uint8_t countActiveRoutes(void) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_ROUTES; i++) {
    if (routerIsRouteInUse) /* slot is populated */
      count++;
  }

  return count;
}

static uint8_t countEnabledRoutes(void) {
  uint8_t count = 0;

  for (uint8_t i = 0; i < MAX_ROUTES; i++) {
    if (routerIsRouteEnabled) /* slot is enabled */
      count++;
  }

  return count;
}

/**
 * @brief Return the number of active routes.
 *
 * This function returns the number of active routes in the routing table.
 * Active routes are routes that are currently populated (i.e., inuse == true).
 *
 * @return uint8_t Number of active routes.
 */
uint8_t routerGetRouteCount(void) { return countActiveRoutes(); }

/**
 * @brief Get the maximum number of routes that can be stored.
 *
 * This function returns the maximum number of routes that can be stored
 * in the routing table. This is a constant and does not change at
 * runtime.
 *
 * @return uint8_t The maximum number of routes that can be stored.
 */
uint8_t routerGetMaxRouteCount(void) { return MAX_ROUTES; }

/**
 * @brief Get the number of enabled routes.
 *
 * This function returns the number of enabled routes in the routing table.
 * Enabled routes are routes that are  and  enabled (i.e., enabled == true).
 *
 * @return uint8_t The number of enabled routes.
 */
uint8_t routerGetEnabledRouteCount(void) { return countEnabledRoutes(); }

/* Write a function that uses printf to pretty-print the contents of
 * the routing table, based on the struct route_entry_t */

void prettyPrintRoutes(void) {
  printf("[ROUTER] Routing Table:\n");
  for (uint8_t i = 0; i < MAX_ROUTES; i++) {
    if (routerIsRouteInUse(i)) {
      printf("  Route %u: Enabled: %c\n", i,
             routerIsRouteEnabled(i) ? 'Y' : 'N');
      printf("    CRC: 0x%04X\n", g_routeCrc[i].crc);
      printf("    Source Node Id: 0x%08X\n", g_routes[i].source_node_id);
      printf("    Source: msg_id = 0x%03X, sub_idx = %u, event_type = %u\n",
             g_routes[i].source_msg_id, g_routes[i].source_sub_idx,
             g_routes[i].ea.bits.event_type);
      printf("    Target: msg_id = 0x%03X, sub_idx = %u, action_type = %u\n",
             g_routes[i].target_msg_id, g_routes[i].target_sub_idx),
          g_routes[i].ea.bits.action_type;
      printf("    match_value: %lu, match_mask: %lu\n", g_routes[i].match_value,
             g_routes[i].match_mask);
      printf("    Payload parameters: 0: 0x%02X 1: 0x%02X 2: 0x%02X\n ",
             g_routes[i].parameters[0], g_routes[i].parameters[1],
             g_routes[i].parameters[2]);
      printf("    Reserved bytes: 0: 0x%02X 1: 0x%02X\n",
             g_routes[i].reserved[0], g_routes[i].reserved[1]);
    }
  }
}
