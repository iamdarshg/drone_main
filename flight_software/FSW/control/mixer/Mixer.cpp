#include <flight_software/FSW/control/mixer/Mixer.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Control {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Mixer ::
    Mixer(
        const char *const compName
    ) :
      MixerComponentBase(compName)
  {

  }

  void Mixer ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    MixerComponentBase::init(instance);
  }

  Mixer ::
    ~Mixer()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Mixer ::
    actuatorIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Actuator &actuator
    )
  {
    F32 roll = actuator.getroll();
    F32 pitch = actuator.getpitch();
    F32 yaw = actuator.getyaw();
    F32 thrust = actuator.getthrust();

    // Quadcopter X configuration mixer
    F32 motor1 = thrust - roll - pitch + yaw;
    F32 motor2 = thrust - roll + pitch - yaw;
    F32 motor3 = thrust + roll + pitch + yaw;
    F32 motor4 = thrust + roll - pitch - yaw;

    // Output motor speeds
    this->motor1Out_out(0, motor1);
    this->motor2Out_out(0, motor2);
    this->motor3Out_out(0, motor3);
    this->motor4Out_out(0, motor4);
  }

} // end namespace Control
