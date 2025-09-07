#ifndef GL_HW_INTERFACES_PICO_UARTPICO_H_
#define GL_HW_INTERFACES_PICO_UARTPICO_H_

#include <hardware/uart.h>
#include <stdint.h>

#include "hw/Pins.hh"
#include "hw/interfaces/IUart.hh"

namespace gl::hw {

/**
Implements the UART interface for the rp2350
*/
class UartPico : public IUart {
 public:
  /**
   * Constructs an UART object and initializes it
   * @param[in] uartInst UART instance to be used
   * @param[in] pinTx hardware pin number for TX
   * @param[in] pinRx hardware pin number for RX
   * @param[in] baudRate Baudrate to be used
   * @returns UART object
   */
  template <ConceptPinGpio T>
  UartPico(uart_inst_t* uartInst, T pinTx, T pinRx, uint32_t baudRate) : uartInst(uartInst) {
    uart_init(uartInst, baudRate);
    gpio_set_function(static_cast<pinTx>, GPIO_FUNC_UART);
    gpio_set_function(static_cast<pinRx>, GPIO_FUNC_UART);
  }

  bool readBytes(uint32_t numBytes, uint8_t* dest) override;
  bool writeBytes(const uint8_t* data, uint32_t numBytes) override;

 private:
  uart_inst_t* uartInst = nullptr;
};

}  // namespace gl::hw

#endif  // GL_HW_INTERFACES_PICO_UARTPICO_H_
