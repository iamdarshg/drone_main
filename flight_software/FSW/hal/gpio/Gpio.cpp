#include <flight_software/FSW/hal/gpio/Gpio.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <flight_software/FSW/hal/PinMap.hpp>

// Dummy hardware abstraction layer
void digitalWrite(U32 pin, bool state) {}
bool digitalRead(U32 pin) { return false; }

namespace Hal {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Gpio ::
    Gpio(
        const char *const compName
    ) :
      GpioComponentBase(compName)
  {

  }

  void Gpio ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    GpioComponentBase::init(instance);
  }

  Gpio ::
    ~Gpio()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Gpio ::
    gpioWrite_handler(
        const NATIVE_INT_TYPE portNum,
        bool state
    )
  {
    // This is a placeholder for the actual pin number
    U32 pin = 0;
    if (portNum == 0) {
        pin = Hal::PinMap::PYRO1;
    }
    digitalWrite(pin, state);
  }

  bool Gpio ::
    gpioRead_handler(
        const NATIVE_INT_TYPE portNum
    )
  {
    // This is a placeholder for the actual pin number
    U32 pin = 0;
    if (portNum == 0) {
        pin = Hal::PinMap::PYRO1;
    }
    return digitalRead(pin);
  }

} // end namespace Hal
