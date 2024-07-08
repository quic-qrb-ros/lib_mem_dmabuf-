## Overview

lib_mem_dmabuf is a library for access and interact with Linux DMA buffer.

## Feature highlights

* Alloc and destroy DMA buffer
* Return file descriptor (fd) associated with DMA buffer
* DMA buffer synchronization operation
* Auto release DMA buffer and fd when leave code scope
* Register DMA buffer release callback

## Quickstart

### Code Sync and Build

Currently, we only support build with QCLINUX SDK.

1. Setup QCLINUX SDK environments follow this document: [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

     ```bash
     mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     ```

3. Clone this project under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

     ```bash
     cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     ```

4. Build projects

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

### Use lib_mem_dmabuf in your project

1. Add dependencies in your package.xml

   ```xml
   <depend>lib_mem_dmabuf</depend>
   ```
2. Use ament_cmake_auto to find dependencies in your CMakeLists.txt

   ```cmake
   find_package(ament_cmake_auto REQUIRED)
   ament_auto_find_build_dependencies()
   ```
3. Using lib_mem_dmabuf to alloc DMA buffer

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

   // buf will auto release when leave scope
   ```

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcom RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | LE.QCROBOTICS.1.0 |

## Contributions

Thanks for your interest in contributing to lib_mem_dmabuf! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

lib_mem_dmabuf is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.
