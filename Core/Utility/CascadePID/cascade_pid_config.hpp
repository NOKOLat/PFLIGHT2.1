#pragma once

namespace CascadePidConfig {

    namespace Command {

        constexpr float PITCH_ROLL_MAX_ANGLE_DEG = 30.0f;
        constexpr float YAW_MAX_RATE_DEG_PER_SEC = 45.0f;
    }

    namespace Pitch {
        namespace Angle {
            constexpr float KP = 1.0f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
        namespace Rate {
            constexpr float KP = 0.5f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
    }

    namespace Roll {
        namespace Angle {
            constexpr float KP = 1.0f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
        namespace Rate {
            constexpr float KP = 0.5f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
    }

    namespace Yaw {
        namespace Angle {
            constexpr float KP = 1.0f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
        namespace Rate {
            constexpr float KP = 1.0f;
            constexpr float KI = 0.0f;
            constexpr float KD = 0.0f;
        }
    }
}
