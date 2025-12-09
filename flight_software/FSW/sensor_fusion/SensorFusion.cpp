#include <flight_software/FSW/sensor_fusion/SensorFusion.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace FSW {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  SensorFusion ::
    SensorFusion(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void SensorFusion ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  SensorFusion ::
    ~SensorFusion()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void SensorFusion ::
    imuIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Imu &imu
    )
  {
    // TODO: Implement sensor fusion algorithm
    // TODO: Output fused attitude data
  }

  void SensorFusion ::
    pressureIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Pressure &pressure
    )
  {
    // TODO: Use pressure data in sensor fusion algorithm
  }

  void SensorFusion ::
    imuIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Imu &imu
    )
  {
    // TODO: Implement sensor fusion algorithm
    // TODO: Output fused attitude data
  }

  void SensorFusion ::
    pressureIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Pressure &pressure
    )
  {
    // TODO: Use pressure data in sensor fusion algorithm
  }

  void SensorFusion ::
    gpsIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Gps &gps
    )
  {
    // TODO: Use GPS data in sensor fusion algorithm
  }

} // end namespace FSW
