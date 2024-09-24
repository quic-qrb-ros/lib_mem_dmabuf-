// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "lib_mem_dmabuf/dmabuf.hpp"

#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <iostream>

namespace lib_mem_dmabuf
{
DmaBuffer::DmaBuffer(int fd, std::size_t size) : fd_(fd), size_(size) {}

DmaBuffer::~DmaBuffer()
{
  if (destroy_callback_ != nullptr) {
    destroy_callback_(this);
  }
  if (auto_release_ && !release()) {
    std::cerr << "dma buffer release failed, fd: " << fd_ << ", size: " << size_ << std::endl;
  }
}

std::shared_ptr<DmaBuffer> DmaBuffer::alloc(std::size_t size, const std::string & heap_name)
{
  int heap_fd = open(heap_name.c_str(), O_RDONLY | O_CLOEXEC);
  if (heap_fd < 0) {
    std::cerr << "open dma heap failed, heap: " << heap_name << std::endl;
    return nullptr;
  }

  struct dma_heap_allocation_data heap_data = {};
  heap_data.len = size;
  heap_data.fd_flags = O_RDWR | O_CLOEXEC;

  if (ioctl(heap_fd, DMA_HEAP_IOCTL_ALLOC, &heap_data) != 0) {
    std::cerr << "dma heap alloc failed, len: " << heap_data.len << std::endl;
    close(heap_fd);
    return nullptr;
  }
  close(heap_fd);
  return std::make_shared<DmaBuffer>(heap_data.fd, heap_data.len);
}

bool DmaBuffer::map()
{
  addr_ = mmap(NULL, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
  if (addr_ == MAP_FAILED) {
    std::cerr << "dma heap mmap failed, fd: " << fd_ << std::endl;
    return false;
  }
  return true;
}

bool DmaBuffer::unmap()
{
  if (addr_ != nullptr) {
    if (munmap(addr_, size_) != 0) {
      return false;
    }
  }
  addr_ = nullptr;
  return true;
}

bool DmaBuffer::sync_start()
{
  struct dma_buf_sync sync;
  sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
  if (ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
    std::cerr << "DmaBuffer: sync start failed" << std::endl;
    return false;
  }
  return true;
}

bool DmaBuffer::sync_end()
{
  struct dma_buf_sync sync;
  sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW;
  if (ioctl(fd_, DMA_BUF_IOCTL_SYNC, &sync) != 0) {
    std::cerr << "DmaBuffer: sync end failed" << std::endl;
    return false;
  }
  return true;
}

bool DmaBuffer::release()
{
  if (!unmap()) {
    return false;
  }
  close(fd_);
  return true;
}

void DmaBuffer::set_auto_release(bool auto_release)
{
  auto_release_ = auto_release;
}

void DmaBuffer::set_destroy_callback(std::function<void(DmaBuffer *)> cb)
{
  destroy_callback_ = cb;
}

bool DmaBuffer::set_data(void * data, std::size_t size)
{
  if (addr_ == nullptr && !map()) {
    return false;
  }
  if (!sync_start()) {
    return false;
  }
  std::memcpy(addr_, data, size);
  if (!sync_end()) {
    return false;
  }
  return true;
}

}  // namespace lib_mem_dmabuf