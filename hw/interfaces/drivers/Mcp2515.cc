#include "hw/interfaces/drivers/Mcp2515.hh"

#include <cstring>

namespace {
// Commands
constexpr uint8_t CMD_WRITE = 0x02;
constexpr uint8_t CMD_READ = 0x03;
constexpr uint8_t CMD_MODIFY = 0x05;
constexpr uint8_t CMD_RESET = 0xC0;
constexpr uint8_t CMD_READ_STATUS = 0xA0;

// Register Addresses
constexpr uint8_t RXF0SIDH = 0x00;
constexpr uint8_t RXF1SIDH = 0x04;
constexpr uint8_t RXF2SIDH = 0x08;
constexpr uint8_t RXF3SIDH = 0x10;
constexpr uint8_t RXF4SIDH = 0x14;
constexpr uint8_t RXF5SIDH = 0x18;
constexpr uint8_t CANCTRL = 0x0F;
constexpr uint8_t RXM0SIDH = 0x20;
constexpr uint8_t RXM1SIDH = 0x24;
constexpr uint8_t CNF3 = 0x28;
constexpr uint8_t CNF2 = 0x29;
constexpr uint8_t CNF1 = 0x2A;
constexpr uint8_t CANINTE = 0x2B;
constexpr uint8_t CANINTF = 0x2C;
constexpr uint8_t TXB0CTRL = 0x30;
constexpr uint8_t TXB1CTRL = 0x40;
constexpr uint8_t TXB2CTRL = 0x50;
constexpr uint8_t RXB0CTRL = 0x60;
constexpr uint8_t RXB0SIDH = 0x61;
constexpr uint8_t RXB1CTRL = 0x70;
constexpr uint8_t RXB1SIDH = 0x71;

// CanCtrl register values
constexpr uint8_t MODE_NORMAL = 0x00;
constexpr uint8_t MODE_CONFIG = 0x80;
constexpr uint8_t MODE_MASK = 0xE0;

// Receive mode values
constexpr uint8_t RXB_RX_STDEXT = 0x00;
constexpr uint8_t RXB_RX_MASK = 0x60;
constexpr uint8_t RXB_BUKT_MASK = (1 << 2);

// Speed register values. TODO this is slow, we could try and increase
constexpr uint8_t MHz8_20kBPS_CFG1 = 0x07;
constexpr uint8_t MHz8_20kBPS_CFG2 = 0xBF;
constexpr uint8_t MHz8_20kBPS_CFG3 = 0x87;

// Bits in TxCtrl register
constexpr uint8_t TXB_TXREQ_M = 0x08;

// Bits in status register
constexpr uint8_t STAT_RX0IF = 1 << 0;
constexpr uint8_t STAT_RX1IF = 1 << 1;

// Bits in CANINF register
constexpr uint8_t RX0IF = 0x01;
constexpr uint8_t RX1IF = 0x02;

// Bits in SID register
constexpr uint8_t SIDH = 0;
constexpr uint8_t SIDL = 1;
constexpr uint8_t DLC_MASK = 0x0F;

std::byte readRegister(gl::hw::ISpi& spi, uint8_t addr) {
  const std::array<std::byte, 2> data{static_cast<std::byte>(CMD_READ), static_cast<std::byte>(addr)};
  std::byte result;
  spi.writeReadBytes(data, std::span(&result, 1));
  return result;
}

void readRegisters(gl::hw::ISpi& spi, uint8_t addr, std::span<std::byte> data, uint8_t len) {
  const std::array<std::byte, 2> readRequest{static_cast<std::byte>(CMD_READ), static_cast<std::byte>(addr)};
  spi.writeReadBytes(readRequest, data);
}

void writeRegister(gl::hw::ISpi& spi, uint8_t addr, uint8_t val) {
  std::array<std::byte, 3> data = {static_cast<std::byte>(CMD_WRITE), static_cast<std::byte>(addr),
                                   static_cast<std::byte>(val)};
  spi.writeBytes(data);
}

void writeRegisters(gl::hw::ISpi& spi, uint8_t addr, std::span<const std::byte> data, uint8_t len) {
  std::vector<std::byte> msg(2 + len);
  msg[0] = static_cast<std::byte>(CMD_WRITE);
  msg[1] = static_cast<std::byte>(addr);
  memcpy(msg.data() + 2, data.data(), len);
  spi.writeBytes(msg);
}

void modifyRegister(gl::hw::ISpi& spi, uint8_t addr, uint8_t mask, uint8_t val) {
  std::array<std::byte, 4> data = {static_cast<std::byte>(CMD_MODIFY), static_cast<std::byte>(addr),
                                   static_cast<std::byte>(mask), static_cast<std::byte>(val)};
  spi.writeBytes(data);
}

void setMode(gl::hw::ISpi& spi, const uint8_t mode) {
  modifyRegister(spi, CANCTRL, MODE_MASK, mode);
  while (!((readRegister(spi, CANCTRL) & static_cast<std::byte>(MODE_MASK)) == static_cast<std::byte>(mode))) {}
}

}  // namespace

namespace gl::hw {
Mcp2515::Mcp2515(ISpi& spi, const utils::IClock& clock, CanId canId, std::vector<CanId> subscriptions)
    : spi(spi), clock(clock), canId(canId), subscriptions(std::move(subscriptions)) {
  // Reset
  spi.writeBytes(std::array{static_cast<std::byte>(CMD_RESET)});
  clock.wait(utils::Time::msec(300));
  // Set to config
  setMode(spi, MODE_CONFIG);
  // Configure speed
  writeRegister(spi, CNF1, MHz8_20kBPS_CFG1);
  writeRegister(spi, CNF2, MHz8_20kBPS_CFG2);
  writeRegister(spi, CNF3, MHz8_20kBPS_CFG3);
  // Reset control registers
  uint8_t tx0 = TXB0CTRL;
  uint8_t tx1 = TXB1CTRL;
  uint8_t tx2 = TXB2CTRL;
  for (uint32_t i = 0; i < 14; i++) {
    writeRegister(spi, tx0, 0);
    writeRegister(spi, tx1, 0);
    writeRegister(spi, tx2, 0);
    tx0++;
    tx1++;
    tx2++;
  }
  writeRegister(spi, RXB0CTRL, 0);
  writeRegister(spi, RXB1CTRL, 0);
  // Disable interrupts
  writeRegister(spi, CANINTE, 0);
  // Enable both receive buffers
  modifyRegister(spi, RXB0CTRL, RXB_RX_MASK | RXB_BUKT_MASK, RXB_RX_STDEXT | RXB_BUKT_MASK);
  modifyRegister(spi, RXB1CTRL, RXB_RX_MASK, RXB_RX_STDEXT);
  // Set masks for both receive registers to check the full id
  const auto setFilter = [&](uint8_t sidhAddr, uint16_t id) {
    std::array<std::byte, 4> maskData = {static_cast<std::byte>(id >> 3), static_cast<std::byte>(id << 5), std::byte{0},
                                         std::byte{0}};
    writeRegisters(spi, sidhAddr, maskData, 4);
  };
  setFilter(RXM0SIDH, 0x07FF);
  setFilter(RXM1SIDH, 0x07FF);
  // Set filters. Ideally receive on both receive buffers. TODO: This ignores subscriptions if too many are requested
  const std::vector<uint8_t> filterAddrRx0 = {RXF0SIDH, RXF1SIDH};
  const std::vector<uint8_t> filterAddrRx1 = {RXF2SIDH, RXF3SIDH, RXF4SIDH, RXF5SIDH};
  for (uint32_t i = 0; i < std::min<uint32_t>(filterAddrRx0.size(), subscriptions.size()); i++) {
    setFilter(filterAddrRx0[i], static_cast<uint16_t>(subscriptions[i]));
  }
  const uint32_t maxSubscriptions = filterAddrRx0.size() + filterAddrRx1.size();
  const int32_t subDiff = static_cast<int32_t>(subscriptions.size()) - static_cast<int32_t>(maxSubscriptions);
  const uint32_t offset = static_cast<uint32_t>(std::max<int32_t>(std::min<int32_t>(subDiff, filterAddrRx0.size()), 0));
  for (uint32_t i = 0; i < std::min<uint32_t>(filterAddrRx1.size(), subscriptions.size() - offset); i++) {
    setFilter(filterAddrRx1[i], static_cast<uint16_t>(subscriptions[i + offset]));
  }

  // Set to normal mode
  setMode(spi, MODE_NORMAL);
}

bool Mcp2515::send(const std::array<std::byte, 8>& data, uint8_t len) {
  const auto findFreeTx = [&](uint8_t* availableTxCtrl) {
    uint8_t ctrlregs[3] = {TXB0CTRL, TXB1CTRL, TXB2CTRL};
    for (uint8_t i = 0; i < 3; i++) {
      if ((static_cast<uint8_t>(readRegister(spi, ctrlregs[i])) & TXB_TXREQ_M) == 0) {
        *availableTxCtrl = ctrlregs[i];
        return true;
      }
    }
    return false;
  };

  // Find free tx buffer
  uint8_t freeTxCtrl;
  if (!findFreeTx(&freeTxCtrl)) {
    return false;
  }
  // Set message
  writeRegisters(spi, freeTxCtrl + 6, data, len);
  writeRegister(spi, freeTxCtrl + 5, len);
  // Set Id
  std::array<std::byte, 4> idData;
  idData[0] = static_cast<std::byte>(static_cast<uint8_t>(canId) >> 3);
  idData[1] = static_cast<std::byte>((static_cast<uint8_t>(canId) & 0x07) << 5);
  idData[2] = std::byte{0};
  idData[3] = std::byte{0};
  writeRegisters(spi, freeTxCtrl + 1, idData, 4);
  // Start the transmission
  modifyRegister(spi, freeTxCtrl, TXB_TXREQ_M, TXB_TXREQ_M);
  return true;
}

std::pair<uint8_t, uint16_t> Mcp2515::read(std::array<std::byte, 8>& data) {
  std::byte status;
  spi.writeReadBytes(std::array{static_cast<std::byte>(CMD_READ_STATUS)}, std::span(&status, 1));

  const auto readCanMsg = [&](uint8_t sidhAddr) {
    std::array<std::byte, 4> idData;
    readRegisters(spi, sidhAddr, idData, 4);
    const uint16_t id = (static_cast<uint16_t>(idData[SIDH]) << 3) + static_cast<uint16_t>(idData[SIDL] >> 5);
    const std::byte ctrl = readRegister(spi, sidhAddr - 1);
    const uint32_t numBytes = static_cast<uint32_t>(readRegister(spi, sidhAddr + 4)) & DLC_MASK;
    readRegisters(spi, sidhAddr + 5, data, numBytes);
    return std::make_pair(numBytes, id);
  };

  std::pair<uint8_t, uint16_t> ret = {0, 0};
  if (static_cast<uint8_t>(status) & STAT_RX0IF) {
    ret = readCanMsg(RXB0SIDH);
    modifyRegister(spi, CANINTF, RX0IF, 0);
  } else if (static_cast<uint8_t>(status) & STAT_RX1IF) {
    ret = readCanMsg(RXB1SIDH);
    modifyRegister(spi, CANINTF, RX1IF, 0);
  }
  return ret;
}

}  // namespace gl::hw
