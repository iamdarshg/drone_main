#include <flight_software/FSW/sensors/kx122/Kx122.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Sensors {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Kx122 ::
    Kx122(
        const char *const compName
    ) :
      Kx122ComponentBase(compName)
  {

  }

  void Kx122 ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Kx122ComponentBase::init(instance);
  }

  Kx122 ::
    ~Kx122()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Kx122 ::
    schedIn_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
    )
  {
    // TODO: Read accelerometer data from the sensor

    // Placeholder dummy data
    Fw::Accelerometer accel_data;
    accel_data.setx(0.1);
    accel_data.sety(0.2);
    accel_data.setz(9.8);

    // Output accelerometer data
    this->accelOut_out(0, accel_data);

    // Telemetry
    this->tlmWrite_AccelX(0.1);
    this->tlmWrite_AccelY(0.2);
    this->tlmWrite_AccelZ(9.8);
  }

} // end namespace Sensors
