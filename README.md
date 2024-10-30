# Lib Mem Dmabuf

## Overview

lib_mem_dmabuf is a library for access and interact with Linux DMA buffer.

### Features

* Alloc and destroy DMA buffer
* Return file descriptor (fd) associated with DMA buffer
* DMA buffer synchronization operation
* Auto release DMA buffer and fd when leave code scope
* Register DMA buffer release callback

## Getting Started

### Prerequisites

- Linux kernel version 5.12 and later, for kernel dma-buf support.

<details><summary>Cross Compile with QCLINUX SDK</summary>

### Cross Compile with QCLINUX SDK

Setup QCLINUX SDK environments:
- Reference [QRB ROS Documents: Getting Started](https://quic-qrb-ros.github.io/getting_started/environment_setup.html)

Create workspace in QCLINUX SDK environment and clone source code

```bash
mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
```

Build source code with QCLINUX SDK

```bash
export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

colcon build --merge-install --cmake-args \
  -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
  -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
  -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
  -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
  -DBUILD_TESTING=OFF
```

</details>

<details open><summary>Native Build on Ubuntu</summary>

### Native Build on Ubuntu

Prerequisites

- ROS 2, this package built with ROS 2 build tools: [Install ROS2 on Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

Create workspace and clone source code from GitHub:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
```
Build source code

```bash
cd ~/ros2_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build
```

</details>

### Use `lib_mem_dmabuf` in your project

Add dependencies in your package.xml

```xml
<depend>lib_mem_dmabuf</depend>
```

Find dependencies in your CMakeLists.txt

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
```

Alloc DMA buffer with provided APIs

```c++
#include "lib_mem_dmabuf/dmabuf.hpp"

// alloc dmabuf with size and DMA heap name
auto buf = lib_mem_dmabuf::DmaBuffer::alloc(size, "/dev/dma_heap/system");

// get fd of buffer
std::cout << "fd: " << buf->fd() << std::endl;

// get CPU accessable address
if (buf->map()) {
   std::cout << "CPU address: " << buf->addr() << std::endl;
   // read / write buffer
   // ...
}

// fd will auto close when buf leave scope
```

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | `LE.QCROBOTICS.1.0`, `Canonical Ubuntu Image for RB3 gen2` |

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

## Authors

* **Peng Wang** - *Maintainer* - [penww](https://github.com/penww)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.
