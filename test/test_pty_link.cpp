#include <gtest/gtest.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>
#include <string>

#include "rabbits_sim/pty_link.hpp"

namespace rabbits_sim
{

class PtyLinkTest : public ::testing::Test
{
protected:
  static constexpr const char * LINK = "/tmp/rabbits_sim_test_pty";

  void SetUp() override
  {
    unlink(LINK);
    ASSERT_TRUE(link_.Open(LINK));
  }

  void TearDown() override
  {
    if (peer_ >= 0) {
      close(peer_);
    }
    link_.Close();
    unlink(LINK);
  }

  // bridge の側を演じる: リンクを開いて raw にする
  int OpenPeer()
  {
    peer_ = open(LINK, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (peer_ < 0) {
      return peer_;
    }
    struct termios tio;
    tcgetattr(peer_, &tio);
    cfmakeraw(&tio);
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 1;
    tcsetattr(peer_, TCSANOW, &tio);
    return peer_;
  }

  static std::string ReadLink(const char * path)
  {
    char buf[256] = {};
    const ssize_t n = readlink(path, buf, sizeof(buf) - 1);
    return n < 0 ? std::string() : std::string(buf, static_cast<size_t>(n));
  }

  PtyLink link_;
  int peer_ = -1;
};

TEST_F(PtyLinkTest, OpenCreatesLinkToSlave)
{
  EXPECT_TRUE(link_.is_open());
  EXPECT_EQ(link_.link_path(), LINK);
  EXPECT_EQ(link_.slave_path().rfind("/dev/pts/", 0), 0u);
  EXPECT_EQ(ReadLink(LINK), link_.slave_path());
}

TEST_F(PtyLinkTest, PeerWritesAreReadable)
{
  ASSERT_GE(OpenPeer(), 0);
  const uint8_t out[5] = {1, 2, 3, 4, 5};
  ASSERT_EQ(write(peer_, out, sizeof(out)), 5);
  usleep(20000);
  uint8_t in[16] = {};
  const int n = link_.Read(in, sizeof(in));
  ASSERT_EQ(n, 5);
  EXPECT_EQ(std::memcmp(in, out, 5), 0);
}

TEST_F(PtyLinkTest, WriteReachesPeer)
{
  ASSERT_GE(OpenPeer(), 0);
  const uint8_t out[3] = {0xA5, 0x5A, 0x01};
  ASSERT_EQ(link_.Write(out, sizeof(out)), 3);
  usleep(20000);
  uint8_t in[16] = {};
  const ssize_t n = read(peer_, in, sizeof(in));
  ASSERT_EQ(n, 3);
  EXPECT_EQ(std::memcmp(in, out, 3), 0);
}

TEST_F(PtyLinkTest, ReadWithoutDataReturnsZero)
{
  ASSERT_GE(OpenPeer(), 0);
  uint8_t in[16];
  EXPECT_EQ(link_.Read(in, sizeof(in)), 0);
}

TEST_F(PtyLinkTest, WriteWithoutReaderDoesNotBlock)
{
  uint8_t out[4096];
  std::memset(out, 0x55, sizeof(out));
  for (int i = 0; i < 10; ++i) {
    EXPECT_GE(link_.Write(out, sizeof(out)), 0);
  }
}

TEST_F(PtyLinkTest, CloseRemovesLink)
{
  link_.Close();
  EXPECT_FALSE(link_.is_open());
  struct stat st;
  EXPECT_NE(lstat(LINK, &st), 0);
}

}  // namespace rabbits_sim

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
