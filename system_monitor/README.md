# System Monitor for Autoware

**Further improvement of system monitor functionality for Autoware.**

## Description
This package provides the following nodes for monitoring system:
* CPU Monitor
* HDD Monitor
* Memory Monitor
* Network Monitor
* NTP Monitor
* Process Monitor
* GPU Monitor

### Suppoted architecture
* x86_64
* ARM

### Operation confirmed platform
* PC system intel core i7
* NVIDIA Jetson AGX Xavier
* Raspberry Pi4 Model B

## How to use
Use colcon build and launch in the same way as other packages.
```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
roslaunch system_monitor system_monitor.launch
```

CPU and GPU monitoring method differs depending on platform.<br>
CMake automatically chooses source to be built according to build environment.<br>
If you build this package on intel platform, CPU monitor and GPU monitor which run on intel platform are built.

## ROS topics published by system monitor

Every topic is published in 1 minute interval.

- [CPU Monitor](topics_cpu_monitor.md)
- [HDD Monitor](topics_hdd_monitor.md)
- [Mem Monitor](topics_mem_monitor.md)
- [Net Monitor](topics_net_monitor.md)
- [NTP Monitor](topics_ntp_monitor.md)
- [Process Monitor](topics_process_monitor.md)
- [GPU Monitor](topics_gpu_monitor.md)

[Usage] ✓：Supported, -：Not supported

| Node | Message | Intel | arm64(tegra) | arm64(raspi) | Notes |
| ---- | --- | :---: | :---: | :---: | --- |
| CPU Monitor     | CPU Temperature        | ✓ | ✓ | ✓ | |
|                 | CPU Usage              | ✓ | ✓ | ✓ | |
|                 | CPU Load Average       | ✓ | ✓ | ✓ | |
|                 | CPU Thermal Throttling | ✓ | - | ✓ | |
|                 | CPU Frequency          | ✓ | ✓ | ✓ | Notification of frequency only, normally error not generated. |
| HDD Monitor     | HDD Temperature        | ✓ | ✓ | ✓ | |
|                 | HDD Usage              | ✓ | ✓ | ✓ | |
| Memory Monitor  | Memory Usage           | ✓ | ✓ | ✓ | |
| Net Monitor     | Network Usage          | ✓ | ✓ | ✓ | |
| NTP Monitor     | NTP Offset             | ✓ | ✓ | ✓ | |
| Process Monitor | Tasks Summary          | ✓ | ✓ | ✓ | |
|                 | High-load Proc[0-9]    | ✓ | ✓ | ✓ | |
|                 | High-mem Proc[0-9]	   | ✓ | ✓ | ✓ | |
| GPU Monitor     | GPU Temperature        | ✓ | ✓ | - | |
|                 | GPU Usage              | ✓ | ✓ | - | |
|                 | GPU Memory Usage       | ✓ | - | - | |
|                 | GPU Thermal Throttling | ✓ | - | - | |
|                 | GPU Frequency          | - | ✓ | - | |

## ROS parameters

 See [ROS parameters](ros_parameters.md).

# Notes

## <u>CPU monitor for intel platform</u>
CPU Monitor which runs on intel platform reads contents of MSR(Model Specific Register) to monitor thermal throttling event.<br>
Kernel module 'msr' must be loaded into your target system, and accessing MSR is only allowed for root, 
so you must set up passwordless sudo for some specific commands.
```
sudo visudo
```

Add the following line to your /etc/sudoers. Replace 'username' with user name which runs system_monitor on your target system.
```
username ALL=(ALL) NOPASSWD: /sbin/modprobe msr,/usr/sbin/rdmsr -af 0\:0 0x1b1
```

Also you need to install MSR tools into your target system.
```
sudo apt install -y msr-tools
```

## <u>HDD Monitor</u>
HDD Monitor uses S.M.A.R.T. Monitoring Tools to monitor HDD temperature.<br>
This is only allowed for root, so you must set up passwordless sudo.
```
sudo visudo
```

Append the following command to NOPASSWD in your /etc/sudoers. Replace 'disk_name' with disk name to monitor temperature.
```
/usr/sbin/smartctl -a /dev/disk_name
```

Also you need to install S.M.A.R.T. Monitoring Tools into your target system.
```
sudo apt install -y smartmontools
```

## <u>GPU Monitor for intel platform</u>
Currently GPU monitor for intel platform only supports NVIDIA GPU whose information can be accessed by NVML API.

Also you need to instal CUDA libraries.
For installation instructions for CUDA 10.0, see [NVIDIA CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/archive/10.0/cuda-installation-guide-linux/index.html).

# UML diagrams
 See [Class diagrams](class_diagrams.md).
 See [Sequence diagrams](seq_diagrams.md).

