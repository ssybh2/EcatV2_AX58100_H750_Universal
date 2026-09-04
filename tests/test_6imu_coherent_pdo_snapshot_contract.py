#!/usr/bin/env python3
"""Regression contract for coherent 6-IMU PDO snapshots.

Each H750 IMU task must commit the 21-byte payload and its 16-bit sequence as
one snapshot.  The EtherCAT reader must then use that same snapshot for both
the 21-byte PDO payload and bytes 126..137 sequence diagnostics.

The implementation intentionally uses fixed buffers plus atomics in the CAN
assembly state: no mutex and no heap allocation are introduced in the CAN ISR.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HIPNUC = (ROOT / "Application" / "Task" / "hipnuc_imu_can.cpp").read_text()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    require(
        "uint8_t committed_snapshot[2][23]{};" in HIPNUC,
        "IMU state must double-buffer 21-byte payload + 2-byte sequence",
    )
    require(
        "std::atomic<uint8_t> active_snapshot{};" in HIPNUC,
        "committed snapshot publication must use an atomic active index",
    )
    require(
        "std::atomic<uint32_t> snapshot_generation{};" in HIPNUC,
        "reader must be able to detect a publication during its memcpy",
    )
    require(
        "ThreadSafeValue<uint16_t> last_pdo_sample_seq{};" in HIPNUC,
        "PDO diagnostics must remember the sequence paired with the copied payload",
    )
    require(
        "state->active_snapshot.store(write_snapshot_idx, std::memory_order_release);" in HIPNUC,
        "CAN packet3 commit must publish the finished snapshot only after copying it",
    )
    require(
        "generation_before == generation_after" in HIPNUC,
        "PDO reader must retry if a snapshot publication happened during its copy",
    )
    require(
        "snapshot->sample_seq[i] = imu_states[i].last_pdo_sample_seq.get();" in HIPNUC,
        "bytes 126..137 must use the sequence captured with this PDO payload",
    )
    require(
        "snapshot->sample_seq[i] = static_cast<uint16_t>(imu_states[i].complete_samples.get()" not in HIPNUC,
        "diagnostic sequence must not be reread from the live completion counter",
    )


if __name__ == "__main__":
    main()
