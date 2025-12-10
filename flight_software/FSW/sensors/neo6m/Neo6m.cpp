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
      Neo6mComponentBase(compName)
  {

  }

  void Neo6m ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Neo6mComponentBase::init(instance);
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

    // Placeholder dummy data
    Fw::Gps gps_data;
    gps_data.setlatitude(34.0522);
    gps_data.setlongitude(-118.2437);
    gps_data.setaltitude(71.0);
    gps_data.setsatellites(8);

    // Output GPS data
    this->gpsOut_out(0, gps_data);

    // Telemetry
    this->tlmWrite_Latitude(34.0522);
    this->tlmWrite_Longitude(-118.2437);
    this->tlmWrite_Altitude(71.0);
    this->tlmWrite_Satellites(8);
  }

} // end namespace Sensors
