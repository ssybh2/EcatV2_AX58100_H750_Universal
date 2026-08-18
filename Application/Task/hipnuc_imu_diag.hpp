#ifndef ECATV2_AX58100_H750_UNIVERSAL_HIPNUC_IMU_DIAG_HPP
#define ECATV2_AX58100_H750_UNIVERSAL_HIPNUC_IMU_DIAG_HPP

#include <cstdint>

namespace aim::ecat::task::hipnuc_imu {
    constexpr uint8_t HIPNUC_DIAG_IMU_COUNT = 6;

    struct HipnucImuDiagSnapshot {
        uint16_t sample_seq[HIPNUC_DIAG_IMU_COUNT]{};
        uint16_t incomplete_samples[HIPNUC_DIAG_IMU_COUNT]{};
    };

    void get_diag_snapshot(HipnucImuDiagSnapshot *snapshot);
}

#endif // ECATV2_AX58100_H750_UNIVERSAL_HIPNUC_IMU_DIAG_HPP
