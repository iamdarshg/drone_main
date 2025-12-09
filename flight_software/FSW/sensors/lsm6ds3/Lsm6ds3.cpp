#include <flight_software/FSW/sensors/lsm6ds3/Lsm6ds3.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Sensors {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Lsm6ds3 ::
    Lsm6ds3(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void Lsm6ds3 ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  Lsm6ds3 ::
    ~Lsm6ds3()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Lsm6ds3 ::
    schedIn_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO: Read IMU data from the sensor
    // TODO: Output IMU data
  }

} // end namespace Sensors
