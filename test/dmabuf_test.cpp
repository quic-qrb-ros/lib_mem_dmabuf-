// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "lib_mem_dmabuf/dmabuf.hpp"

#include <cstring>

#include "gtest/gtest.h"

TEST(dmabuf_transport, dmabuf_test)
{
  auto buf = lib_mem_dmabuf::DmaBuffer::alloc(64, "/dev/dma_heap/system");
  if (buf == nullptr) {
    std::cerr << "alloc dmabuf failed" << std::endl;
  }

  ASSERT_NE(buf, nullptr);

  std::cout << "fd: " << buf->fd() << std::endl;
  ASSERT_GT(buf->fd(), 0);

  ASSERT_TRUE(buf->map());

  auto data = "mock data";
  auto ret = buf->set_data((void *)data, std::strlen(data));
  ASSERT_TRUE(ret);

  ASSERT_TRUE(buf->unmap());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
