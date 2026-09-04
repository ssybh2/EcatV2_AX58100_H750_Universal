#!/usr/bin/env python3
"""Regression contract for coherent 6-IMU PDO snapshots.

The H750 must publish each IMU's 21-byte payload and the sequence value that
identifies that payload from the same committed snapshot.  Reading the payload
first and the live completion counter later allows a CAN IRQ between those two
operations to pair sample N data with sample N+1's sequence.
"""

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
TASK_DEFS = (ROOT / "Application" / "task_defs.hpp").read_text()
HIPNUC = (ROOT / "Application" / "Task" / "hipnuc_imu_can.cpp").read_text()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def main() -> None:
    require(
        "ThreadSafeBuffer buf_{23};" in TASK_DEFS,
        "HIPNUC committed buffer must contain 21-byte payload + 2-byte sequence",
    )
    require(
        "get_last_pdo_sample_seq" in TASK_DEFS,
        "HIPNUC task must expose the sequence captured with its PDO payload",
    )
    require(
        "buf_.write(committed_snapshot, 23);" in HIPNUC,
        "CAN packet3 commit must publish payload and sequence in one buffer swap",
    )
    require(
        "buf_.read(current_snapshot, 23);" in HIPNUC,
        "PDO read must take payload and sequence from one committed snapshot",
    )
    require(
        "state.owner->get_last_pdo_sample_seq()" in HIPNUC,
        "diagnostic sequence must come from the same snapshot written to this PDO",
    )


if __name__ == "__main__":
    main()
