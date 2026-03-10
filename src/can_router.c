#include "can_router.h"


/* ============================================================================
 *  GLOBAL STORAGE
 * ========================================================================== */

route_entry_t   g_routes[MAX_ROUTES];
producer_cfg_t  g_producerCfg[MAX_SUB_MODULES];

/* Temporary buffer for multi-frame route reception */
static route_entry_t rxRouteBuffer;
static uint8_t       rxRouteIndex = 0;
static uint8_t       rxRouteSlot  = 0xFF;

/**
 * @brief Stores the last-seen primary data byte for each route.
 *
 * This array is used by EVENT_ON_CHANGE, EVENT_ON_RISING, and EVENT_ON_FALLING.
 *
 */
static uint8_t g_last_values[MAX_ROUTES] = {0};


/* ============================================================================
 *  ROUTING: EDGE DETECTION
 * ========================================================================== */

/**
 * @brief Detect a rising edge (0 → 1) on msg->data[0].
 */
bool detect_rising_edge(const can_msg_t *msg, uint8_t idx)
{
    uint8_t new_val = msg->data[0];
    uint8_t old_val = g_last_values[idx];

    g_last_values[idx] = new_val;   // Update stored state

    return (old_val == 0 && new_val == 1);
}

/**
 * @brief Detect a falling edge (1 → 0) on msg->data[0].
 */
bool detect_falling_edge(const can_msg_t *msg, uint8_t idx)
{
    uint8_t new_val = msg->data[0];
    uint8_t old_val = g_last_values[idx];

    g_last_values[idx] = new_val;   // Update stored state

    return (old_val == 1 && new_val == 0);
}



/* ============================================================================
 *  ROUTING: MULTI-FRAME RECEIVER
 * ========================================================================== */

void handleRouteBegin(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_BEGIN_DLC)
        return;

    uint8_t route_idx = msg->data[4];
    if (route_idx >= MAX_ROUTES)
        return;

    rxRouteSlot  = route_idx;
    rxRouteIndex = 0;
    memset(&rxRouteBuffer, 0, sizeof(rxRouteBuffer));

    printf("RouteBegin: slot %u\n", route_idx);
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
        ((uint8_t*)&rxRouteBuffer)[dst] = msg->data[4 + i];
    }

    printf("RouteData: slot %u idx %u\n", rxRouteSlot, rxRouteIndex);
}

void handleRouteEnd(const can_msg_t *msg)
{
    (void)msg;

    if (rxRouteSlot >= MAX_ROUTES)
        return;

    memcpy(&g_routes[rxRouteSlot], &rxRouteBuffer, sizeof(route_entry_t));
    g_routes[rxRouteSlot].enabled = 1;

    printf("RouteEnd: stored route %u\n", rxRouteSlot);

    rxRouteSlot  = 0xFF;
    rxRouteIndex = 0;
}

void handleRouteDelete(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_ROUTE_DELETE_DLC)
        return;

    uint8_t idx = msg->data[4];
    if (idx >= MAX_ROUTES)
        return;

    memset(&g_routes[idx], 0, sizeof(route_entry_t));
    printf("RouteDelete: idx %u\n", idx);
}

void handleRoutePurge(const can_msg_t *msg)
{
    (void)msg;
    memset(g_routes, 0, sizeof(g_routes));
    printf("RoutePurge: all cleared\n");
}

/* ============================================================================
 *  NVS STUBS FOR NON-ESP32 PLATFORMS
 * ========================================================================== */

#ifndef ESP32

void loadRouteTableFromNVS(void) {}
void saveRouteTableToNVS(void) {}
void loadProducerCfgFromNVS(void) {}
void saveProducerCfgToNVS(void) {}

#endif

void handleRouteWriteNVS(void)
{
    saveRouteTableToNVS();
}

void handleRouteReadNVS(void)
{
    loadRouteTableFromNVS();
}

/* ============================================================================
 *  PRODUCER CONFIG HANDLERS
 * ========================================================================== */

void handleProducerCfg(const can_msg_t *msg)
{
    if (msg->data_length_code < CFG_PRODUCER_CFG_DLC)
        return;

    uint8_t idx = msg->data[4];
    if (idx >= MAX_SUB_MODULES)
        return;

    g_producerCfg[idx].sub_idx  = idx;
    g_producerCfg[idx].rate_hz  = msg->data[5];
    g_producerCfg[idx].flags    = msg->data[6];
    g_producerCfg[idx].reserved = msg->data[7];

    printf("ProducerCfg: sub %u rate %u flags 0x%02X\n",
           idx,
           g_producerCfg[idx].rate_hz,
           g_producerCfg[idx].flags);
}

void handleProducerPurge(const can_msg_t *msg)
{
    (void)msg;
    memset(g_producerCfg, 0, sizeof(g_producerCfg));
    printf("ProducerCfg: purged\n");
}

void handleProducerDefaults(const can_msg_t *msg)
{
    (void)msg;
    memset(g_producerCfg, 0, sizeof(g_producerCfg));
    printf("ProducerCfg: defaults applied\n");
}

void handleProducerApply(void)
{
    /* No-op: config is live immediately */
    printf("ProducerCfg: apply\n");
}

void handleReqProducerCfg(const can_msg_t *msg)
{
    (void)msg;
    /* The caller (ESP32) will send RESP_PRODUCER_CFG_ID frames.
       This module is platform-agnostic, so we do nothing here. */
    printf("ProducerCfg: request received (platform must respond)\n");
}

void handleProducerWriteNVS(void)
{
    saveProducerCfgToNVS();
    printf("ProducerCfg: saved\n");
}

/* ============================================================================
 *  ROUTE EXECUTION HOOK
 * ========================================================================== */

void checkRoutes(const can_msg_t *msg)
{
    /* ---------------------------------------------------------
     * CONFIGURATION COMMANDS (0x300–0x3FF)
     * --------------------------------------------------------- */
    if (msg->identifier >= 0x300 && msg->identifier <= 0x3FF) {

        switch (msg->identifier)
        {
            case CFG_ROUTE_BEGIN_ID:
                handleRouteBegin(msg);
                break;

            case CFG_ROUTE_DATA_ID:
                handleRouteData(msg);
                break;

            case CFG_ROUTE_END_ID:
                handleRouteEnd(msg);
                break;

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

            case CFG_PRODUCER_CFG_ID:
                handleProducerCfg(msg);
                break;

            case CFG_PRODUCER_PURGE_ID:
                handleProducerPurge(msg);
                break;

            case CFG_PRODUCER_DEFAULTS_ID:
                handleProducerDefaults(msg);
                break;

            case CFG_PRODUCER_APPLY_ID:
                handleProducerApply();
                break;

            case REQ_PRODUCER_CFG_ID:
                handleReqProducerCfg(msg);
                break;

            case CFG_PRODUCER_WRITE_NVS_ID:
                handleProducerWriteNVS();
                break;

            default:
                break;
        }

        return; // <--- IMPORTANT
    }

    /* ---------------------------------------------------------
    * ROUTE EXECUTION (normal CAN traffic)
    * --------------------------------------------------------- */
    for (uint8_t i = 0; i < MAX_ROUTES; i++) {

        route_entry_t *r = &g_routes[i];

        if (!r->enabled) /* skip disabled routes */
            continue;

        if (msg->identifier != r->source_msg_id) /* skip non-matching messages */
            continue;

        printf("RouteHit: idx %u src 0x%03X\n", i, msg->identifier);

        /* Evaluate event condition */
        if (!evaluate_event(i, msg)) /* skip non-matching events */
            continue;

        /* Execute the action */
        execute_action(i, msg);
    }

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
bool evaluate_event(uint8_t idx, const can_msg_t *msg)
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
            return payload_matches_parameters(msg, r);

        default:
            return false;
    }
}

/**
 * @brief Execute the action associated with a route.
 *
 * ACTION_FORWARD:
 *      Forward the incoming CAN frame unchanged to the target message ID.
 *
 * ACTION_TOGGLE:
 *      Send a 1-byte "toggle" command to the target submodule.
 *
 * ACTION_SET_VALUE:
 *      Send the route's parameter bytes to the target message ID.
 *
 * ACTION_MAP_BYTE:
 *      Extract a byte from the source payload and send it to the target.
 *
 * ACTION_PWM / ACTION_STROBE:
 *      Invoke the output personality logic (blink, pwm, strobe).
 */
void execute_action(uint8_t idx, const can_msg_t *msg)
{
    route_entry_t *r = &g_routes[idx];

    switch (r->action_type)
    {
        case ACTION_FORWARD:
            /* Forward the original payload unchanged */
            send_message(r->target_msg_id, msg->data, msg->data_length_code);
            break;

        case ACTION_TOGGLE: {
            /* Toggle commands are always 1-byte payloads */
            uint8_t payload = 1;
            send_message(r->target_msg_id, &payload, 1);
            break;
        }

        case ACTION_SET_VALUE:
            /* Send the route's parameter bytes */
            send_message(r->target_msg_id, r->parameters, r->param_len);
            break;

        case ACTION_MAP_BYTE: {
            /* Extract byte N from the source payload */
            uint8_t index = r->parameters[0];
            uint8_t value = msg->data[index];
            send_message(r->target_msg_id, &value, 1);
            break;
        }

        case ACTION_PWM:
            handle_pwm_action(r, msg);
            break;

        case ACTION_STROBE:
            handle_strobe_action(r, msg);
            break;

        default:
            /* Unknown or unimplemented action */
            break;
    }
}
