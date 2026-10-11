// Optional 40-byte read-only DShot300 LAB diagnostics.
// Only compiled for -DAIM_DSHOT_RPM_LAB_RX=ON.
// No change to legacy type 4 command or type 16 RPM PDO.
#include "task_defs.hpp"
#include "buffer_utils.hpp"
#include "dshot_rpm_lab.hpp"

#ifdef AIM_DSHOT_RPM_LAB_RX
extern "C" {
#include "stm32h7xx_hal.h"
}

namespace aim::ecat::task::pwm {
    DSHOT_RPM_DIAG::DSHOT_RPM_DIAG(buffer::Buffer *buf)
        : CustomRunnable(false, TaskType::DSHOT_RPM_DIAG) {
        dshot_id_ = buf->read_uint8(buffer::EndianType::LITTLE);
    }

    void DSHOT_RPM_DIAG::write_to_master(buffer::Buffer *out) {
        uint32_t counters[10]{};
        if (dshot_id_ == 2) {
            dshot_rpm::lab::read_diagnostics(counters, HAL_GetTick());
        }
        for (const uint32_t value : counters) {
            out->write_uint32(buffer::EndianType::LITTLE, value);
        }
    }
}
#endif
