# ProductCode 0x06 — 6IMU + DJI RC + DShot

This profile preserves ProductCode 0x05 and adds a new identity:

- ProductCode: `0x06`
- Master -> Slave application data: `80 B`
- Slave -> Master application data: `192 B`

S->M: 0..125 six IMUs, 126..159 diagnostics, 160..178 DJI DBUS/RC,
179..191 reserved. M->S: 0..7 DShot four uint16 values.

The first six configured tasks must be HIPNUC IMU tasks.

`ecat/device/patch_esi.py` does not compile ESI/ESX to SII. Regenerate
`slave.bin` with the project's EtherCAT SDK/code generator, then run
`python3 ecat/device/patch_esi.py`. Never rename/reuse an old 0x05 image.
