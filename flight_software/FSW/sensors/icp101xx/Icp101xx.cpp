#include <flight_software/FSW/sensors/icp101xx/Icp101xx.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Sensors {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Icp101xx ::
    Icp101xx(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void Icp101xx ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  Icp101xx ::
    ~Icp101xx()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Icp101xx ::
    schedIn_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO: Read pressure and temperature data from the sensor
    // TODO: Output pressure and temperature data
  }

} // end namespace Sensors
