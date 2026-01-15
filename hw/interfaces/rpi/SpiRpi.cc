#include "hw/interfaces/rpi/SpiRpi.hh"

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <cstring>

namespace {
constexpr uint32_t SPI_SPEED_HZ = 500000;
constexpr uint8_t SPI_MODE = SPI_MODE_0;
constexpr uint8_t SPI_BITS_PER_WORD = 8;
}  // namespace

namespace gl::hw {

SpiRpi::SpiRpi(const std::string& path) {
  fd = open(path.c_str(), O_RDWR);
  ioctl(fd, SPI_IOC_WR_MODE, SPI_MODE);
  ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, SPI_BITS_PER_WORD);
  ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, SPI_SPEED_HZ);
}

SpiRpi::~SpiRpi() {
  close(fd);
}

bool SpiRpi::writeReadBytes(std::span<const std::byte> dataWrite, std::span<std::byte> dataRead) {
  if (fd < 0) {
    return false;
  }

  struct spi_ioc_transfer tr[2] = {};
  tr[0].tx_buf = reinterpret_cast<uint64_t>(dataWrite.data());
  tr[0].rx_buf = 0;
  tr[0].len = static_cast<uint32_t>(dataWrite.size());
  tr[0].speed_hz = SPI_SPEED_HZ;
  tr[0].bits_per_word = SPI_BITS_PER_WORD;

  tr[1].tx_buf = 0;
  tr[1].rx_buf = reinterpret_cast<uint64_t>(dataRead.data());
  tr[1].len = static_cast<uint32_t>(dataRead.size());
  tr[1].speed_hz = SPI_SPEED_HZ;
  tr[1].bits_per_word = SPI_BITS_PER_WORD;

  return ioctl(fd, SPI_IOC_MESSAGE(2), tr) >= 0;
}

bool SpiRpi::readBytes(std::span<std::byte> data) {
  if (fd < 0) {
    return false;
  }

  struct spi_ioc_transfer tr;
  tr.tx_buf = 0;
  tr.rx_buf = reinterpret_cast<uint64_t>(data.data());
  tr.len = static_cast<uint32_t>(data.size());
  tr.speed_hz = SPI_SPEED_HZ;
  tr.bits_per_word = SPI_BITS_PER_WORD;

  return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}

bool SpiRpi::writeBytes(std::span<const std::byte> data) {
  if (fd < 0) {
    return false;
  }

  struct spi_ioc_transfer tr = {};
  tr.tx_buf = reinterpret_cast<uint64_t>(data.data());
  tr.rx_buf = 0;
  tr.len = static_cast<uint32_t>(data.size());
  tr.speed_hz = SPI_SPEED_HZ;
  tr.bits_per_word = SPI_BITS_PER_WORD;

  return ioctl(fd, SPI_IOC_MESSAGE(1), &tr) >= 0;
}
}  // namespace gl::hw
