#include <flight_software/FSW/sensors/neo6m/Neo6m.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Sensors {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Neo6m ::
    Neo6m(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void Neo6m ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  Neo6m ::
    ~Neo6m()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Neo6m ::
    schedIn_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO: Read GPS data from the sensor
    // TODO: Output GPS data
  }

} // end namespace Sensors
