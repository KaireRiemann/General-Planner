#pragma once

namespace fsm {

    enum class GoalHeightMode {
        CONFIGURED_CLICK_HEIGHT,
        MESSAGE_HEIGHT,
    };

    inline bool usesMessageHeight(const double click_height,
                                  const GoalHeightMode mode) {
        return mode == GoalHeightMode::MESSAGE_HEIGHT || !(click_height > -5.0);
    }

    inline double resolveGoalHeight(const double message_height,
                                    const double click_height,
                                    const GoalHeightMode mode) {
        if (usesMessageHeight(click_height, mode)) {
            return message_height;
        }
        return click_height;
    }

    inline const char *goalHeightSourceName(const double click_height,
                                            const GoalHeightMode mode) {
        return usesMessageHeight(click_height, mode) ? "message_z" : "click_height";
    }

} // namespace fsm
