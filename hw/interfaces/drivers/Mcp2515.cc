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

}  // namespace

namespace gl::hw {
uint8_t Mcp2515::readRegister(uint8_t addr) {
  uint8_t data[2] = {CMD_READ, addr};
  uint8_t result;
  spi->writeReadBytes(data, &result, 2, 1);
  return result;
}

void Mcp2515::readRegisters(uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t readRequest[2] = {CMD_READ, addr};
  spi->writeReadBytes(readRequest, data, 2, len);
}

void Mcp2515::writeRegister(uint8_t addr, uint8_t val) {
  uint8_t data[3] = {CMD_WRITE, addr, val};
  spi->writeBytes(data, 3);
}

void Mcp2515::writeRegisters(uint8_t addr, const uint8_t* data, uint8_t len) {
  std::vector<uint8_t> msg(2 + len);
  msg[0] = CMD_WRITE;
  msg[1] = addr;
  memcpy(msg.data() + 2, data, len);
  spi->writeBytes(msg.data(), msg.size());
}

void Mcp2515::modifyRegister(uint8_t addr, uint8_t mask, uint8_t val) {
  uint8_t data[4] = {CMD_MODIFY, addr, mask, val};
  spi->writeBytes(data, 4);
}

void Mcp2515::setMode(const uint8_t mode) {
  modifyRegister(CANCTRL, MODE_MASK, mode);
  while (!((readRegister(CANCTRL) & MODE_MASK) == mode)) {}
}

Mcp2515::Mcp2515(std::shared_ptr<ISpi> spi, std::shared_ptr<utils::IClock> clock, CanId canId,
                 const std::vector<CanId>& subscriptions)
    : spi(spi), clock(clock), canId(canId), subscriptions(subscriptions) {
  // Reset
  spi->writeBytes(&CMD_RESET, 1);
  clock->wait(utils::Time::msec(300));
  // Set to config
  setMode(MODE_CONFIG);
  // Configure speed
  writeRegister(CNF1, MHz8_20kBPS_CFG1);
  writeRegister(CNF2, MHz8_20kBPS_CFG2);
  writeRegister(CNF3, MHz8_20kBPS_CFG3);
  // Reset control registers
  uint8_t tx0 = TXB0CTRL;
  uint8_t tx1 = TXB1CTRL;
  uint8_t tx2 = TXB2CTRL;
  for (uint32_t i = 0; i < 14; i++) {
    writeRegister(tx0, 0);
    writeRegister(tx1, 0);
    writeRegister(tx2, 0);
    tx0++;
    tx1++;
    tx2++;
  }
  writeRegister(RXB0CTRL, 0);
  writeRegister(RXB1CTRL, 0);
  // Disable interrupts
  writeRegister(CANINTE, 0);
  // Enable both receive buffers
  modifyRegister(RXB0CTRL, RXB_RX_MASK | RXB_BUKT_MASK, RXB_RX_STDEXT | RXB_BUKT_MASK);
  modifyRegister(RXB1CTRL, RXB_RX_MASK, RXB_RX_STDEXT);
  // Set masks for both receive registers to check the full id
  const auto setFilter = [&](uint8_t sidhAddr, uint16_t id) {
    uint8_t maskData[4] = {static_cast<uint8_t>(id >> 3), static_cast<uint8_t>(id << 5), 0, 0};
    writeRegisters(sidhAddr, maskData, 4);
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
  setMode(MODE_NORMAL);
}

bool Mcp2515::send(const std::array<uint8_t, 8>& data, uint8_t len) {
  const auto findFreeTx = [&](uint8_t* availableTxCtrl) {
    uint8_t ctrlregs[3] = {TXB0CTRL, TXB1CTRL, TXB2CTRL};
    uint8_t txBuffSidh;
    for (uint8_t i = 0; i < 3; i++) {
      if ((readRegister(ctrlregs[i]) & TXB_TXREQ_M) == 0) {
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
  writeRegisters(freeTxCtrl + 6, data.data(), len);
  writeRegister(freeTxCtrl + 5, len);
  // Set Id
  uint8_t idData[4];
  idData[0] = static_cast<uint8_t>(canId) >> 3;
  idData[1] = (static_cast<uint8_t>(canId) & 0x07) << 5;
  idData[2] = 0;
  idData[3] = 0;
  writeRegisters(freeTxCtrl + 1, idData, 4);
  // Start the transmission
  modifyRegister(freeTxCtrl, TXB_TXREQ_M, TXB_TXREQ_M);
  return true;
}

std::pair<uint8_t, CanId> Mcp2515::read(std::array<uint8_t, 8>* data) {
  uint8_t status;
  spi->writeReadBytes(&CMD_READ_STATUS, &status, 1, 1);

  const auto readCanMsg = [&](uint8_t sidhAddr, uint8_t* data) {
    uint8_t idData[4];
    readRegisters(sidhAddr, idData, 4);
    const CanId id{(idData[SIDH] << 3) + (idData[SIDL] >> 5)};
    const uint8_t ctrl = readRegister(sidhAddr - 1);
    const uint8_t numBytes = readRegister(sidhAddr + 4) & DLC_MASK;
    readRegisters(sidhAddr + 5, data, numBytes);
    return std::make_pair(numBytes, id);
  };

  std::pair<uint8_t, CanId> ret = {0, CanId::NONE};
  if (status & STAT_RX0IF) {
    ret = readCanMsg(RXB0SIDH, data->data());
    modifyRegister(CANINTF, RX0IF, 0);
  } else if (status & STAT_RX1IF) {
    ret = readCanMsg(RXB1SIDH, data->data());
    modifyRegister(CANINTF, RX1IF, 0);
  }
  return ret;
}

}  // namespace gl::hw