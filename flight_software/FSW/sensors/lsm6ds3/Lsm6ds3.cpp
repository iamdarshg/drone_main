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
      Lsm6ds3ComponentBase(compName)
  {

  }

  void Lsm6ds3 ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Lsm6ds3ComponentBase::init(instance);
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

    // Placeholder dummy data
    Fw::Imu imu_data;
    imu_data.setaccel_x(1.0);
    imu_data.setaccel_y(2.0);
    imu_data.setaccel_z(3.0);
    imu_data.setgyro_x(4.0);
    imu_data.setgyro_y(5.0);
    imu_data.setgyro_z(6.0);

    // Output IMU data
    this->imuOut_out(0, imu_data);

    // Telemetry
    this->tlmWrite_AccelX(1.0);
    this->tlmWrite_AccelY(2.0);
    this->tlmWrite_AccelZ(3.0);
    this->tlmWrite_GyroX(4.0);
    this->tlmWrite_GyroY(5.0);
    this->tlmWrite_GyroZ(6.0);
  }

} // end namespace Sensors
