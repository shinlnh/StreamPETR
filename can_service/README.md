# Module: can_service

## Description: Support send data to / receive data from other node (STM32, ...) by CANopenNode

## 1. Object dictionary

In CANopen communication network, object dictionary (OD) is a collection of objects used to describe the necessary information for the operation of CANopen devices.

OD source: CANopenLinux/CANopenNode/example/OD.c

OD used in my application, see OD map in ***can_service/CANopenLinux/CANopenNode/example/BV_OD.xdd (run on EDSEditor.exe)***

For more detail, see
[CANopenNode/CANopenDemo: CANopenNode tutorial and testing (github.com)](https://github.com/CANopenNode/CANopenDemo)

OD Structure (for example):

| Index  | Name                 | Sub-index | Type | Description                  |
| ------ | -------------------- | --------- | ---- | ---------------------------- |
| 0x1200 | SDO server parameter | 0x00      | u8   | Highest sub-index supported  |
|        |                      | 0x01      | u8   | COB-ID client to server (rx) |
|        |                      | 0x02      | u8   | COB-ID server to client (tx) |

## 2. Structure of can_service

* **CANopenLinux:** *modified* from open source: https://github.com/CANopenNode/CANopenLinux.git (similar to a submodule). Its output is in CANopenLinux/libcanopend.so, which can be used when build CMake. If you would like to modify it (as required by the project), see part 5.
* src/can_service.cpp: main source file
* test/*: **inherit from previous project (Haven't used can open).**Can be considered for removal

## 3. Build and run

If you want to modify CANopenLinux (expand/shrink TPDO, RPDO, use more features, ...), go to part 5

If not, you can consider CANopenLinux as a submodule (output libcanopend.so is ready).

| source ~/sdk/v1.0.1/environment-setup-aarch64-poky-linux |
| -------------------------------------------------------- |

From adas_sdk

| mkdir build |
| ----------- |

| cd build |
| -------- |

| cmake .. |
| -------- |

| make |
| ---- |

Test

can_if: can interface, default: can0

If you would like to test with virtual CAN (ex: vcan0)

```
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set up vcan0
```

USB, PCI or similar CAN interface (ex: bitrate 125000)

```

ip link set up can0 type can bitrate 125000
```

Run (on the secondterminal)

cd build/can_service

./canservice `can_if` -i `node_id`

Check message (on the first terminal)

./candump `can_if`

After *canopend* startup, first messages are:

```
vcan0  704   [1]  00                        # Boot-up message.
```

## 4. Summary of source code

For more detail

[CANopenNode: CANopenNode](https://canopennode.github.io/CANopenNode/index.html)

- **Mainline thread:** Time consuming task (SDO server, Emergency, Heartbeat, ...). Timer interval is 0.1s (#define MAIN_THREAD_INTERVAL_US100000)
- **Timer interval thread**: Realtime thread with constant interval (#defineTMR_THREAD_INTERVAL_US1000000 - can be changed), network synchronized, Copy RPDO to OD, OD to TPDO, ...
- **send_control_signal:** send data to other nodes. Data is transmit_frame struct
- **get_current_param**: get data received from other node to receive_frame (COD-ID must matched)

struct transmit_control

{

    int8_t function_code;

    int8_t priority;

    int8_t throttle_value;

    int8_t steering_angle_value;

} transmit_frame;


struct receive_param

{

    int8_t function_code;

    int8_t throttle_value;

    int8_t steering_angle_value;

    int8_t speed;

} receive_frame;

## 5. CANopenNode Linux

Can use CANopenEditor to modify object dictionary

https://github.com/CANopenNode/CANopenEditor.git


In my project, I clone source from https://github.com/CANopenNode/CANopenLinux.git

Add 2 OD_index, 0x6000 (Sub 1->8) for TPDO and 0x6200 (sub 1->8) for RPDO


Note: TPDO, RPDO setup

#### 0x1400 - RPDO communication parameter

COB-ID used by RPDO: 0x180 + receive_node_id (ví dụ muốn gửi cho node 4 thì set là 0x184)

#### 0x1600  - RPDO mapping parameter

sub index 0: Number of mapped application objects in PDO

sub index 1 -> 8: Application object

* bit 16-31: index
* bit 08-15: sub-index
* bit 0-7: data length (bits)

TPDO: similar
