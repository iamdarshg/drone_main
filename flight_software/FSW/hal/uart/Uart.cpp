#include <flight_software/FSW/hal/uart/Uart.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <flight_software/FSW/hal/PinMap.hpp>

// Dummy hardware abstraction layer
void uart_init(U32 tx, U32 rx) {}
void uart_write(U8* data, U32 size) {}
void uart_read(U8* data, U32 size) {}

namespace Hal {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Uart ::
    Uart(
        const char *const compName
    ) :
      UartComponentBase(compName)
  {

  }

  void Uart ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    UartComponentBase::init(instance);
    uart_init(Hal::PinMap::UART1_TX, Hal::PinMap::UART1_RX);
  }

  Uart ::
    ~Uart()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Uart ::
    uartWrite_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    uart_write(data.getData(), data.getSize());
  }

  void Uart ::
    uartRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    uart_read(data.getData(), data.getSize());
  }

} // end namespace Hal
