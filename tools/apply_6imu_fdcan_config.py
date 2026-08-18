#!/usr/bin/env python3
"""Keep the six-IMU FDCAN FIFO settings consistent after CubeMX regeneration.

This script intentionally changes only the known FIFO-depth fields needed by
feature/6imu-large-pdo. It does not touch CAN bit timing, pins, clocks, or other
CubeMX settings.
"""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
IOC = ROOT / "EcatV2_AX58100_H750_Universal.ioc"
FDCAN_C = ROOT / "Core" / "Src" / "fdcan.c"

IOC_REPLACEMENTS = {
    "FDCAN1.RxFifo0ElmtsNbr=10": "FDCAN1.RxFifo0ElmtsNbr=32",
    "FDCAN2.RxFifo1ElmtsNbr=10": "FDCAN2.RxFifo1ElmtsNbr=32",
}

C_REPLACEMENTS = {
    "hfdcan1.Init.RxFifo0ElmtsNbr = 10;": "hfdcan1.Init.RxFifo0ElmtsNbr = 32;",
    "hfdcan2.Init.RxFifo1ElmtsNbr = 10;": "hfdcan2.Init.RxFifo1ElmtsNbr = 32;",
}

EXPECTED_C = (
    "hfdcan1.Init.RxFifo0ElmtsNbr = 32;",
    "hfdcan1.Init.TxFifoQueueElmtsNbr = 32;",
    "hfdcan2.Init.MessageRAMOffset = 1280;",
    "hfdcan2.Init.RxFifo1ElmtsNbr = 32;",
    "hfdcan2.Init.TxFifoQueueElmtsNbr = 32;",
)


def patch_file(path: Path, replacements: dict[str, str]) -> bool:
    text = path.read_text(encoding="utf-8")
    original = text
    for old, new in replacements.items():
        text = text.replace(old, new)
    if text != original:
        path.write_text(text, encoding="utf-8", newline="\n")
        return True
    return False


def verify() -> list[str]:
    errors: list[str] = []
    ioc = IOC.read_text(encoding="utf-8")
    fdcan = FDCAN_C.read_text(encoding="utf-8")

    for required in (
        "FDCAN1.RxFifo0ElmtsNbr=32",
        "FDCAN2.RxFifo1ElmtsNbr=32",
        "FDCAN2.MessageRAMOffset=1280",
    ):
        if required not in ioc:
            errors.append(f"IOC missing: {required}")

    for required in EXPECTED_C:
        if required not in fdcan:
            errors.append(f"fdcan.c missing: {required}")

    return errors


def main() -> int:
    if not IOC.exists() or not FDCAN_C.exists():
        print("Run this script from the repository containing the CubeMX project.", file=sys.stderr)
        return 2

    changed_ioc = patch_file(IOC, IOC_REPLACEMENTS)
    changed_c = patch_file(FDCAN_C, C_REPLACEMENTS)

    errors = verify()
    if errors:
        print("6-IMU FDCAN configuration check FAILED:", file=sys.stderr)
        for error in errors:
            print(f"  - {error}", file=sys.stderr)
        return 1

    print("6-IMU FDCAN configuration OK")
    print(f"  CubeMX .ioc patched: {'yes' if changed_ioc else 'already correct'}")
    print(f"  generated fdcan.c patched: {'yes' if changed_c else 'already correct'}")
    print("  CAN1 RX FIFO0 = 32")
    print("  CAN2 RX FIFO1 = 32")
    print("  CAN1/CAN2 TX FIFO = 32")
    print("  FDCAN2 Message RAM offset = 1280 words")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
