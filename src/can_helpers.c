#include <stdio.h>

#include "canbus_project.h"   // for subModule_t, node, etc.
#include "can_router.h"
#include "node_state.h"


#include "can_router.h"
#include "node_state.h"      // gives access to node and trackers
#include <stdio.h>

/**
 * ACTION_PWM:
 * Router requests PWM output on a submodule.
 * We update the submodule config and let TaskOutput + handleHardwarePwm() do the work.
 */
void handle_pwm_action(const route_entry_t *r, const can_msg_t *msg)
{
    uint8_t sub = r->target_sub_idx;

    if (sub >= MAX_SUB_MODULES)
        return;

    // Extract PWM parameters
    // You can refine this later — this is just a clean starting point.
    uint8_t freq = msg->data[0];
    uint8_t duty = msg->data[1];

    // Update node configuration
    node.subModule[sub].config.gpioOutput.mode   = OUT_MODE_PWM;
    node.subModule[sub].config.gpioOutput.param1 = freq;
    node.subModule[sub].config.gpioOutput.param2 = duty;

    // Mark tracker as configured
    trackers[sub].isConfigured = true;
    trackers[sub].isActive     = true;

    printf("Router: PWM → sub %u freq=%u duty=%u\n", sub, freq, duty);
}

/**
 * ACTION_STROBE:
 * Router requests a strobe pattern.
 * Again, we update config and let TaskOutput + handleStrobeLogic() run it.
 */
void handle_strobe_action(const route_entry_t *r, const can_msg_t *msg)
{
    uint8_t sub = r->target_sub_idx;

    if (sub >= MAX_SUB_MODULES)
        return;

    uint8_t pattern = msg->data[0];

    node.subModule[sub].config.gpioOutput.mode   = OUT_MODE_STROBE;
    node.subModule[sub].config.gpioOutput.param2 = pattern;

    trackers[sub].isConfigured = true;
    trackers[sub].isActive     = true;

    printf("Router: STROBE → sub %u pattern=%u\n", sub, pattern);
}
