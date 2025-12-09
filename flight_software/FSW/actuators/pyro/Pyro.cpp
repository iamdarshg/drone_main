#include <flight_software/FSW/actuators/pyro/Pyro.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Actuators {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Pyro ::
    Pyro(
        const char *const compName
    ) :
      Fw::PassiveComponentBase(compName)
  {

  }

  void Pyro ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::PassiveComponentBase::init(instance);
  }

  Pyro ::
    ~Pyro()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Pyro ::
    fireIn_handler(
        NATIVE_INT_TYPE portNum,
        U32 channel,
        U32 duration
    )
  {
    // TODO: Implement pyro channel firing
  }

} // end namespace Actuators
