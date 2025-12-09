#include <flight_software/FSW/actuators/motor/Motor.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Actuators {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Motor ::
    Motor(
        const char *const compName
    ) :
      Fw::PassiveComponentBase(compName)
  {

  }

  void Motor ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::PassiveComponentBase::init(instance);
  }

  Motor ::
    ~Motor()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Motor ::
    speedIn_handler(
        NATIVE_INT_TYPE portNum,
        F32 speed
    )
  {
    // TODO: Implement motor speed control
  }

} // end namespace Actuators
