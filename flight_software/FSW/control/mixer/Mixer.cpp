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
      Fw::PassiveComponentBase(compName)
  {

  }

  void Mixer ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::PassiveComponentBase::init(instance);
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
    // TODO: Implement mixer logic
  }

} // end namespace Control
