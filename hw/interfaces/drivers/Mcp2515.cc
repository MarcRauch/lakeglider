#include "hw/interfaces/drivers/Mcp2515.hh"

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

uint8_t readRegister(gl::hw::ISpi* spi, uint8_t addr) {
  uint8_t data[2] = {CMD_READ, addr};
  spi->writeBytes(data, 2);
  uint8_t result;
  spi->readBytes(1, &result);
  return result;
}

void readRegisters(gl::hw::ISpi* spi, uint8_t addr, uint8_t* data, uint8_t len) {
  uint8_t data[2] = {CMD_READ, addr};
  spi->writeBytes(data, 2);
  const uint8_t numBytes = std::min(len, static_cast<uint8_t>(8));
  for (uint32_t i = 0; i < numBytes; i++) {
    data[i] = spi->readBytes(numBytes, data);
  }
}

void writeRegister(gl::hw::ISpi* spi, uint8_t addr, uint8_t val) {
  uint8_t data[3] = {CMD_WRITE, addr, val};
  spi->writeBytes(data, 3);
}

void writeRegisters(gl::hw::ISpi* spi, uint8_t addr, const uint8_t* data, uint8_t len) {
  std::vector<uint8_t> msg(2 + len);
  msg[0] = CMD_WRITE;
  msg[1] = addr;
  memcpy(msg.data() + 2, data, len);
  spi->writeBytes(msg.data(), msg.size());
}

void modifyRegister(gl::hw::ISpi* spi, uint8_t addr, uint8_t mask, uint8_t val) {
  uint8_t data[4] = {CMD_MODIFY, addr, mask, val};
  spi->writeBytes(data, 4);
}

}  // namespace

namespace gl::hw {
Mcp2515::Mcp2515(ISpi* spi, PinGpioSensor csPin, CanId canId, std::vector<CanId> subscriptions)
    : spi(spi), csPin(csPin), canId(canId), subscriptions(subscriptions) {
  // Reset
  spi->writeBytes(&CMD_RESET, 1);
  // Set to config
  modifyRegister(spi, CANCTRL, MODE_CONFIG, MODE_MASK);
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
    uint8_t maskData[4] = {static_cast<uint8_t>(id >> 3), static_cast<uint8_t>(id & 0x07 << 5), 0, 0};
    writeRegisters(spi, sidhAddr, maskData, 4);
  };
  setFilter(RXM0SIDH, 0x07FF);
  setFilter(RXM1SIDH, 0x07FF);
  // Set filters
  uint8_t filterAddr[6] = {RXF0SIDH, RXF1SIDH, RXF2SIDH, RXF3SIDH, RXF4SIDH, RXF5SIDH};
  for (size_t i = 0; i < std::min<size_t>(5, subscriptions.size()); i++) {
    setFilter(filterAddr[i], static_cast<uint16_t>(subscriptions[i]));
  }

  // Set to normal mode
  modifyRegister(spi, CANCTRL, MODE_NORMAL, MODE_MASK);
}

void Mcp2515::send(const std::array<uint8_t, 8>* data, uint8_t len) {
  const auto findFreeTx = [&](uint8_t* availableTxCtrl) {
    uint8_t ctrlregs[3] = {TXB0CTRL, TXB1CTRL, TXB2CTRL};
    uint8_t txBuffSidh;
    for (uint8_t i = 0; i < 3; i++) {
      if (readRegister(spi, ctrlregs[i]) & TXB_TXREQ_M == 0) {
        *availableTxCtrl = ctrlregs[i];
        return true;
      }
    }
    return false;
  };

  // Find free tx buffer
  uint8_t freeTxCtrl;
  while (!findFreeTx(&freeTxCtrl)) {}
  // Set message
  writeRegisters(spi, freeTxCtrl + 6, data->data(), len);
  writeRegister(spi, freeTxCtrl + 5, len);
  // Set Id
  uint8_t idData[4];
  idData[0] = static_cast<uint8_t>(canId) >> 3;
  idData[1] = (static_cast<uint8_t>(canId) & 0x07) << 5;
  idData[2] = 0;
  idData[3] = 0;
  writeRegisters(spi, freeTxCtrl + 1, idData, 4);
  // Start the transmission
  modifyRegister(spi, freeTxCtrl, TXB_TXREQ_M, TXB_TXREQ_M);
}

uint8_t Mcp2515::read(std::array<uint8_t, 8>* data) {
  spi->writeBytes(&CMD_READ_STATUS, 1);
  uint8_t status;
  spi->readBytes(1, &status);

  const auto readCanMsg = [&](uint8_t sidhAddr, uint8_t* data) {
    uint8_t idData[4];
    readRegisters(spi, sidhAddr, idData, 4);
    const uint8_t id = (idData[SIDH] << 3) + (idData[SIDL] >> 5);
    const uint8_t ctrl = readRegister(spi, sidhAddr - 1);
    const uint8_t numBytes = readRegister(spi, sidhAddr + 4) & DLC_MASK;
    readRegisters(spi, sidhAddr + 5, data, numBytes);
    return numBytes;
  };

  int8_t msgLen = -1;
  if (status & STAT_RX0IF) {
    msgLen = readCanMsg(RXB0SIDH, data->data());
    modifyRegister(spi, CANINTF, RX0IF, 0);
  } else if (status & STAT_RX1IF) {
    msgLen = readCanMsg(RXB1SIDH, data->data());
    modifyRegister(spi, CANINTF, RX1IF, 0);
  }
  return msgLen;
}

}  // namespace gl::hw