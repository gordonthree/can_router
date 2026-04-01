#include "can_router.h"

#include "esp_log.h"

const char *TAG = "can_router";

/* ============================================================================
 *  LOCAL FUNCTION DECLARATIONS
 * ========================================================================== */

static bool detect_change(const can_msg_t *msg, uint8_t idx);
static bool payload_matches_parameters(const can_msg_t *msg, uint8_t idx);
static bool evaluate_event(const uint8_t idx, const can_msg_t *msg);

static void handleRouteBegin(const can_msg_t *msg);
static void handleRouteData(const can_msg_t *msg);
static uint8_t handleRouteEnd(const can_msg_t *msg);
static void handleRouteDelete(const can_msg_t *msg);
static void handleRoutePurge(const can_msg_t *msg);
static void handleRouteWriteNVS(void);
static void handleRouteReadNVS(void);
static uint8_t countActiveRoutes(void);
static void libLog(int level, const char *fmt, ...);


/* ============================================================================
 *  GLOBAL STORAGE
 * ========================================================================== */

route_entry_t g_routes[MAX_ROUTES];
route_entry_crc_t g_routesCrc[] = {0}; /* zero initializes the struct */

volatile bool g_routeSaveRequested = false;
volatile bool g_routeLoadRequested = false;

static uint8_t g_last_values[MAX_ROUTES] = {0};

/* ============================================================================
 *  LOCAL STORAGE
 * ========================================================================== */

/* Temporary buffer for multi-frame route reception */
static route_entry_t rxRouteBuffer = {0};
static uint8_t rxRouteSlot = 0xFF;  /**< slot in the route table */
static uint8_t rxRouteTotal = 0;    /**< store expected chunk count */
static uint8_t rxRouteReceived = 0; /**< track how many data frames we received */

/* Callback setter */
static router_crc_fn_t g_crc_callback = NULL;

void router_set_crc_callback(router_crc_fn_t fn)
{
    g_crc_callback = fn;
}

/* ============================================================================
 *  ROUTING: EDGE DETECTION
 * ========================================================================== */

/**
 * @brief Detect a rising edge (0 → 1) on msg->data[5].
 */
bool detect_rising_edge(const can_msg_t *msg, uint8_t idx)
{
    uint8_t new_val = msg->data[MSG_DATA_5];
    uint8_t old_val = g_last_values[idx];

    g_last_values[idx] = new_val; // Update stored state

    return (old_val == 0 && new_val == 1);
}

/**
 * @brief Detect a falling edge (1 → 0) on msg->data[5].
 */
static bool detect_falling_edge(const can_msg_t *msg, uint8_t idx)
{
    uint8_t new_val = msg->data[MSG_DATA_5];
    uint8_t old_val = g_last_values[idx];

    g_last_values[idx] = new_val; // Update stored state

    return (old_val == 1 && new_val == 0);
}

static bool detect_change(const can_msg_t *msg, uint8_t idx)
{
    return (msg->data[MSG_DATA_5] != g_last_values[idx]);
}

static bool payload_matches_parameters(const can_msg_t *msg, uint8_t idx)
{ // TODO: confirm the return is accurate
    const route_entry_t *r = &g_routes[idx];

    return (msg->data[MSG_DATA_4] == r->parameters[ROUTE_PARAM_0] &&
            msg->data[MSG_DATA_5] == r->parameters[ROUTE_PARAM_1] &&
            msg->data[MSG_DATA_6] == r->parameters[ROUTE_PARAM_2] &&
            msg->data[MSG_DATA_7] == r->parameters[ROUTE_PARAM_3]);
}

/* ============================================================================
 *  ROUTING: MULTI-FRAME RECEIVER
 * ========================================================================== */

static void handleRouteBegin(const can_msg_t *msg)
{
    // printf("[ROUTER] handleRouteBegin()\n");

    if (msg->data_length_code < (uint8_t)CFG_ROUTE_BEGIN_DLC)
    {
        printf("[ROUTER] Error: invalid data length code, command %d ignored\n", msg->identifier);
        return;
    }

    uint8_t route_idx = msg->data[MSG_DATA_4]; /* data byte 4 is the index value */
    uint8_t total_chunks = msg->data[5];       /* data byte 5 is the total chunk count */

    if (route_idx >= (uint8_t)MAX_ROUTES)
    {
        printf("[ROUTER] Error: invalid route index, command %d ignored\n", msg->identifier);
        return;
    }

    if (total_chunks != (uint8_t)ROUTE_CHUNKS_PER_ROUTE)
    {
        printf("[ROUTER] Error: invalid chunk count %d we expected %d, command %d ignored\n", total_chunks, (uint8_t)ROUTE_CHUNKS_PER_ROUTE, msg->identifier);
        return;
    }

    rxRouteSlot = route_idx;
    rxRouteTotal = total_chunks; /* store expected chunk count */
    rxRouteReceived = 0;         /* track how many we got */

    g_routesCrc[route_idx].crc = 0xFFFF;   /* reset crc */
    g_routesCrc[route_idx].ts = 0x00;      /* reset timestamp */
    g_routesCrc[route_idx].in_use = false; /* reset in_use flag */

    memset(&rxRouteBuffer, 0, sizeof(rxRouteBuffer));
    printf("[ROUTER] Route begin, index: %d chunks: %d\n", route_idx, total_chunks);

}

static void handleRouteData(const can_msg_t *msg)
{
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
    printf("[ROUTER] Route data: Route %d chunk received %d\n", rxRouteReceived, chunk_idx);
}

static uint8_t handleRouteEnd(const can_msg_t *msg)
{

    if (msg->data_length_code < 6)
        return ROUTE_INVALID_RX;

    uint8_t route_idx = msg->data[4];
    uint8_t commitFlag = msg->data[5];

    if (route_idx != rxRouteSlot)
    {
        ESP_LOGW(TAG, "[ROUTER] Error: route_idx (%d) != rxRouteSlot (%d)", route_idx, rxRouteSlot);
        return ROUTE_INVALID_RX;
    }

    if (!commitFlag)
    {
        ESP_LOGW(TAG, "[ROUTER] Error: commit flag not set");
        return ROUTE_INVALID_RX; // ignore incomplete transfers
    }

    if (rxRouteReceived != rxRouteTotal)
    {
        ESP_LOGW(TAG, "[ROUTER] Error: rxRouteReceived (%d) != rxRouteTotal (%d)", rxRouteReceived, rxRouteTotal);
        return ROUTE_INVALID_RX; // incomplete or missing chunks
    }

    memcpy(&g_routes[rxRouteSlot], &rxRouteBuffer, sizeof(route_entry_t));
    g_routes[rxRouteSlot].enabled = 1;

    ESP_LOGI(TAG, "[ROUTER] Route %d programming complete", rxRouteSlot);

    const uint8_t savedIdx = rxRouteSlot;

    g_routesCrc[savedIdx].in_use = true; /* set in_use flag */

    uint16_t crc = 0xFFFF;

    if (g_crc_callback)
    {
        crc = g_crc_callback((const uint8_t *)&g_routes[savedIdx],
                             sizeof(route_entry_t));
    }

    g_routesCrc[savedIdx].crc = crc;

    /* save routing table to nvs */
    handleRouteWriteNVS();

    /* reset pointer */
    rxRouteSlot = 0xFF;

    /* let the route command handler know the rx was complete */
    return savedIdx;
}

static void handleRouteDelete(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_DELETE_DLC)
        return;

    uint8_t idx = msg->data[MSG_DATA_4];
    if (idx >= MAX_ROUTES)
        return;

    memset(&g_routes[idx], 0, sizeof(route_entry_t));
}

static void handleRoutePurge(const can_msg_t *msg)
{
    (void)msg;
    memset(g_routes, 0, sizeof(g_routes));
}

static void handleRouteWriteNVS(void)
{
    g_routeSaveRequested = true;
}

static void handleRouteReadNVS(void)
{
    g_routeLoadRequested = true;
}

/* ============================================================================
 *  ROUTE EXECUTION HOOK
 * ========================================================================== */

bool checkRoutes(const can_msg_t *msg, router_action_t *out)
{

    // Default: no action
    out->valid = 0;
    out->actionMsgId = ROUTE_TAKE_NO_ACTION;
    out->sub_idx = 0;
    memset(out->param, 0, ROUTE_ACTION_PARAM_LEN);

    printf("[ROUTER] CONFIG MESSAGE: 0x%03X\n", msg->identifier);

    /* ---------------------------------------------------------
     * CONFIGURATION COMMANDS (0x300–0x3FF)
     * --------------------------------------------------------- */
    if (msg->identifier >= ROUTE_CMD_START && 
        msg->identifier <= ROUTE_CMD_END) // test if command message is between 0x300 and 0x31F
    {

        switch (msg->identifier)
        {
        /* ---------------- ROUTER CONFIG ---------------- */
        case CFG_ROUTE_BEGIN_ID:
            printf("[ROUTER] CFG_ROUTE_BEGIN_ID\n");
            handleRouteBegin(msg);
            break;

        case CFG_ROUTE_DATA_ID:
            printf("[ROUTER] CFG_ROUTE_DATA_ID\n");
            handleRouteData(msg);
            break;

        case CFG_ROUTE_END_ID:
        {
            printf("[ROUTER] CFG_ROUTE_END_ID\n");
            const uint8_t rxIdx = handleRouteEnd(msg);

            if (rxIdx != ROUTE_INVALID_RX)
            { /* test if route saved successfully, if so send ack */
                out->valid = 1;
                out->actionMsgId = DATA_ROUTE_ACK_ID;
                out->actionMsgDlc = DATA_ROUTE_ACK_DLC;
                out->sub_idx = rxIdx; /* return the route index */
                out->param[0] = ((g_routesCrc[rxIdx].crc >> 8) & 0xFF);
                out->param[1] = (g_routesCrc[rxIdx].crc & 0xFF);
            printf("[ROUTER] Route index: %d saved with CRC16: 0x%04X\n", rxIdx, g_routesCrc[rxIdx].crc);

                return true;
            }

            /* route save was not successful */
            out->valid = 1; /* valid command */
            /* router commands have no action in the consumer */
            out->actionMsgId = ROUTE_TAKE_NO_ACTION;
            printf("[ROUTER] Route index: %d save failed.\n", rxIdx);

            return true;
        }
        case REQ_ROUTE_LIST_ID:
        { /* instruct the consumer to send a route list */
            out->valid = 1;
            out->actionMsgId = REQ_ROUTE_LIST_ID;
            out->actionMsgDlc = 0;
            return true;
        }
        case CFG_ROUTE_DELETE_ID:
            handleRouteDelete(msg);
            break;
        case CFG_ROUTE_PURGE_ID:
            handleRoutePurge(msg);
            break;
        case CFG_ROUTE_WRITE_NVS_ID:
            handleRouteWriteNVS();
            break;
        case CFG_ROUTE_READ_NVS_ID:
            handleRouteReadNVS();
            break;

        default:
            printf("[ROUTER] INVALID CONFIG MESSAGE: 0x%03X\n", msg->identifier);
            break;
        }

        /* if no valid action was defined, send no action message to consumer */
        if (out->valid == 0)
        {
            out->valid = 1;                          /* valid command */
            out->actionMsgId = ROUTE_TAKE_NO_ACTION; /* router commands have no action in the consumer */
        }
        return true; /* exit early */
    }

    /* ---------------------------------------------------------
     * ROUTE EXECUTION (normal CAN traffic)
     * --------------------------------------------------------- */
    for (uint8_t i = 0; i < MAX_ROUTES; i++)
    {

        route_entry_t *r = &g_routes[i];

        if (!r->enabled)
            continue;

        if (msg->identifier != r->source_msg_id)
            continue;

        if (!evaluate_event(i, msg))
            continue;

        /* Route matched — fill out semantic action */
        out->valid = 1;
        out->actionMsgId = r->target_msg_id; // may be 0xFFFF
        out->sub_idx = r->target_sub_idx; /* msg data byte 4 */

        /* Load paramater bytes 0 to 3 with message data received with the source message */
        out->param[ROUTE_PARAM_0] = msg->data[MSG_DATA_5];
        out->param[ROUTE_PARAM_1] = msg->data[MSG_DATA_6];
        out->param[ROUTE_PARAM_2] = msg->data[MSG_DATA_7];
        out->param[ROUTE_PARAM_3] = 0;

        return true; // <--- ROUTE MATCHED
    }

    return false; // <--- NO ROUTE MATCHED
}

/**
 * @brief Determine whether a route should fire based on its event type.
 *
 * EVENT_ALWAYS:
 *      Fire every time the source message ID matches.
 *
 * EVENT_ON_CHANGE:
 *      Fire only when the payload changes compared to last time.
 *
 * EVENT_ON_RISING:
 *      Fire when the value transitions 0 → 1.
 *
 * EVENT_ON_FALLING:
 *      Fire when the value transitions 1 → 0.
 *
 * EVENT_ON_MATCH:
 *      Fire when the payload exactly matches the route parameters[].
 */
static bool evaluate_event(const uint8_t idx, const can_msg_t *msg)
{
    const route_entry_t *r = &g_routes[idx];

    switch (r->event_type)
    {
    case EVENT_ALWAYS:
        return true;

    case EVENT_ON_CHANGE:
        // TODO: store last value per route
        return detect_change(msg, idx);

    case EVENT_ON_RISING:
        return detect_rising_edge(msg, idx);

    case EVENT_ON_FALLING:
        return detect_falling_edge(msg, idx);

    case EVENT_ON_MATCH:
        return payload_matches_parameters(msg, idx);

    default:
        return false;
    }
}

/**
 * @brief Return the number of populated routing table entries.
 *
 * This function returns the number of slots in the routing table
 * that are currently populated (i.e., inuse == true).
 *
 * @return uint8_t Number of populated routing table entries.
 */
static uint8_t countActiveRoutes(void)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < MAX_ROUTES; i++)
    {
        if (g_routesCrc[i].in_use)   /* slot is populated */
            count++;
    }

    return count;
}

static uint8_t countEnabledRoutes(void)
{
    uint8_t count = 0;

    for (uint8_t i = 0; i < MAX_ROUTES; i++)
    {
        if (g_routes[i].enabled)   /* slot is enabled */
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
uint8_t routerGetRouteCount(void)
{
    return countActiveRoutes();
}

/**
 * @brief Get the maximum number of routes that can be stored.
 *
 * This function returns the maximum number of routes that can be stored
 * in the routing table. This is a constant and does not change at
 * runtime.
 *
 * @return uint8_t The maximum number of routes that can be stored.
 */
uint8_t routerGetMaxRouteCount(void)
{
    return MAX_ROUTES;
}

/**
 * @brief Get the number of enabled routes.
 *
 * This function returns the number of enabled routes in the routing table.
 * Enabled routes are routes that are  and  enabled (i.e., enabled == true).
 *
 * @return uint8_t The number of enabled routes.
 */
uint8_t routerGetEnabledRouteCount(void)
{
    return countEnabledRoutes();
}

/* Write a function that uses printf to pretty-print the contents of
 * the routing table, based on the struct route_entry_t */

void prettyPrintRoutes(void)
{
    printf("[ROUTER] Routing Table:\n");
    for (uint8_t i = 0; i < MAX_ROUTES; i++)
    {
        if (g_routes[i].enabled && g_routesCrc[i].in_use)
        {
            printf("  Route %d: Enabled, In-use\n", i);
            printf("       CRC: 0x%04X\n", g_routesCrc[i].crc);
            printf("    Source: msg_id = 0x%04X, sub_idx = %d, event_type = %d\n", 
                    g_routes[i].source_msg_id, g_routes[i].source_sub_idx, g_routes[i].event_type);
            printf("    Target: msg_id = 0x%04X, sub_idx = %d\n", 
                    g_routes[i].target_msg_id, g_routes[i].target_sub_idx);
            printf("    Parameters: ");
            for (uint8_t j = 0; j < ROUTE_ENTRY_PARAM_LEN; j++)
            {
                printf("0x%02X ", g_routes[i].parameters[j]);
            }
            printf("\n");
        }
    }
    
}

// static lib_log_fn_t g_logger = NULL;

// void libSetLogger(lib_log_fn_t fn) {
//     g_logger = fn;
// }

// static void libLog(int level, const char *fmt, ...) 
// {
//     if (!g_logger)
//         return;

//     va_list args;
//     va_start(args, fmt);
//     g_logger(level, fmt, args);
//     va_end(args);
// }
