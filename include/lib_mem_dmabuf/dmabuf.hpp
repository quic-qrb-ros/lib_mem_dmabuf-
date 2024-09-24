// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef LIB_MEM_DMABUF__DMABUF_HPP_
#define LIB_MEM_DMABUF__DMABUF_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

namespace lib_mem_dmabuf
{
class DmaBuffer
{
public:
  DmaBuffer(int fd, std::size_t size);
  ~DmaBuffer();
  static std::shared_ptr<DmaBuffer> alloc(std::size_t size, const std::string & heap_name);

  bool map();
  bool unmap();
  bool sync_start();
  bool sync_end();
  bool release();
  void set_auto_release(bool auto_release);
  void set_destroy_callback(std::function<void(DmaBuffer *)> cb);
  bool set_data(void * data, std::size_t size);

  int fd() const { return fd_; }
  int size() const { return size_; }
  void * addr() const { return addr_; }

private:
  int fd_{ 0 };
  int size_{ 0 };
  void * addr_{ nullptr };
  bool auto_release_{ true };
  std::function<void(DmaBuffer *)> destroy_callback_{ nullptr };
};

}  // namespace lib_mem_dmabuf

#endif  // LIB_MEM_DMABUF__DMABUF_HPP_
