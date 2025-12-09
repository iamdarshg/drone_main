#include <flight_software/FSW/control/pid/Pid.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Control {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Pid ::
    Pid(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void Pid ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  Pid ::
    ~Pid()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Pid ::
    attitudeIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &attitude
    )
  {
    // TODO: Implement PID control logic
    // TODO: Output actuator commands
  }

  void Pid ::
    setpointIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &setpoint
    )
  {
    // TODO: Store the setpoint for the PID controller
  }

} // end namespace Control
