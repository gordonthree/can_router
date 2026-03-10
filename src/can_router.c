#include "can_router.h"


bool detect_change(const can_msg_t *msg, uint8_t idx);
bool payload_matches_parameters(const can_msg_t *msg, uint8_t idx);


/* ============================================================================
 *  GLOBAL STORAGE
 * ========================================================================== */

route_entry_t   g_routes[MAX_ROUTES];

volatile bool g_routeSaveRequested = false;
volatile bool g_routeLoadRequested = false;

static uint8_t g_last_values[MAX_ROUTES] = {0};

/* ============================================================================
 *  LOCAL STORAGE
 * ========================================================================== */

/* Temporary buffer for multi-frame route reception */
static route_entry_t rxRouteBuffer;
static uint8_t       rxRouteIndex = 0;
static uint8_t       rxRouteSlot  = 0xFF;


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

    g_last_values[idx] = new_val;   // Update stored state

    return (old_val == 0 && new_val == 1);
}

/**
 * @brief Detect a falling edge (1 → 0) on msg->data[5].
 */
bool detect_falling_edge(const can_msg_t *msg, uint8_t idx)
{
    uint8_t new_val = msg->data[MSG_DATA_5]; 
    uint8_t old_val = g_last_values[idx];

    g_last_values[idx] = new_val;   // Update stored state

    return (old_val == 1 && new_val == 0);
}

bool detect_change(const can_msg_t *msg, uint8_t idx)
{
    return (msg->data[MSG_DATA_5] != g_last_values[idx]); 
}

bool payload_matches_parameters(const can_msg_t *msg, uint8_t idx)
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

void handleRouteBegin(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_BEGIN_DLC)
        return;

    uint8_t route_idx = msg->data[MSG_DATA_4]; /* data byte 4 is the index value */
    if (route_idx >= MAX_ROUTES)
        return;

    rxRouteSlot  = route_idx;
    rxRouteIndex = 0;
    memset(&rxRouteBuffer, 0, sizeof(rxRouteBuffer));
}


void handleRouteData(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_DATA_DLC)
        return;
    if (rxRouteSlot >= MAX_ROUTES)
        return;

    /* Copy 4 bytes (D4..D7) into the struct buffer */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t dst = rxRouteIndex++;
        if (dst >= sizeof(route_entry_t))
            break;
        ((uint8_t*)&rxRouteBuffer)[dst] = msg->data[MSG_DATA_4 + i];
    }
}

void handleRouteEnd(const can_msg_t *msg)
{
    (void)msg;

    if (rxRouteSlot >= MAX_ROUTES)
        return;

    memcpy(&g_routes[rxRouteSlot], &rxRouteBuffer, sizeof(route_entry_t));
    g_routes[rxRouteSlot].enabled = 1;

}

void handleRouteDelete(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_DELETE_DLC)
        return;

    uint8_t idx = msg->data[MSG_DATA_4];
    if (idx >= MAX_ROUTES)
        return;

    memset(&g_routes[idx], 0, sizeof(route_entry_t));
}

void handleRoutePurge(const can_msg_t *msg)
{
    (void)msg;
    memset(g_routes, 0, sizeof(g_routes));
}

void handleRouteWriteNVS(void)
{
    g_routeSaveRequested = true;
}

void handleRouteReadNVS(void)
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

    /* ---------------------------------------------------------
        * CONFIGURATION COMMANDS (0x300–0x3FF)
        * --------------------------------------------------------- */
    if (msg->identifier >= ROUTE_CMD_START && msg->identifier <= ROUTE_CMD_END) {

        switch (msg->identifier)
        {
            case CFG_ROUTE_BEGIN_ID:     handleRouteBegin(msg);  break;
            case CFG_ROUTE_DATA_ID:      handleRouteData(msg);   break;
            case CFG_ROUTE_END_ID:       handleRouteEnd(msg);    break;
            case CFG_ROUTE_DELETE_ID:    handleRouteDelete(msg); break;
            case CFG_ROUTE_PURGE_ID:     handleRoutePurge(msg);  break;
            case CFG_ROUTE_WRITE_NVS_ID: handleRouteWriteNVS();  break;
            case CFG_ROUTE_READ_NVS_ID:  handleRouteReadNVS();   break;
            default: break;
        }

        return false;   // <--- CONFIG MESSAGES DO NOT PRODUCE ACTIONS
    }

    /* ---------------------------------------------------------
        * ROUTE EXECUTION (normal CAN traffic)
        * --------------------------------------------------------- */
    for (uint8_t i = 0; i < MAX_ROUTES; i++) {

        route_entry_t *r = &g_routes[i];

        if (!r->enabled)
            continue;

        if (msg->identifier != r->source_msg_id)
            continue;

        if (!evaluate_event(i, msg))
            continue;

        /* Route matched — fill out semantic action */
        out->valid = 1;
        out->actionMsgId = r->target_msg_id;   // may be 0xFFFF
        out->sub_idx = r->target_sub_idx;

        // Copy parameters into param[]
        out->param[ROUTE_PARAM_0] = msg->data[MSG_DATA_5];
        out->param[ROUTE_PARAM_1] = msg->data[MSG_DATA_6];
        out->param[ROUTE_PARAM_2] = msg->data[MSG_DATA_7];
        out->param[ROUTE_PARAM_3] = 0;

        return true;    // <--- ROUTE MATCHED
    }

    return false;       // <--- NO ROUTE MATCHED
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
bool evaluate_event(const uint8_t idx, const can_msg_t *msg)
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

