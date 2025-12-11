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
      SensorFusionComponentBase(compName)
  {
    // Initialize the EKF
    // TODO: Initialize the state and covariance matrices
  }

  void SensorFusion ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    SensorFusionComponentBase::init(instance);
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
    // TODO: Implement sensor redundancy
    // For now, just store the data
    m_imu1 = imu;
  }

  void SensorFusion ::
    imuIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Imu &imu
    )
  {
    // TODO: Implement sensor redundancy
    // For now, average the data and run the EKF
    Fw::Imu avg_imu;
    avg_imu.setaccel_x((m_imu1.getaccel_x() + imu.getaccel_x()) / 2.0);
    avg_imu.setaccel_y((m_imu1.getaccel_y() + imu.getaccel_y()) / 2.0);
    avg_imu.setaccel_z((m_imu1.getaccel_z() + imu.getaccel_z()) / 2.0);
    avg_imu.setgyro_x((m_imu1.getgyro_x() + imu.getgyro_x()) / 2.0);
    avg_imu.setgyro_y((m_imu1.getgyro_y() + imu.getgyro_y()) / 2.0);
    avg_imu.setgyro_z((m_imu1.getgyro_z() + imu.getgyro_z()) / 2.0);

    // Create the control vector
    Control u;
    u << avg_imu.getgyro_x(), avg_imu.getgyro_y(), avg_imu.getgyro_z(),
         avg_imu.getaccel_x(), avg_imu.getaccel_y(), avg_imu.getaccel_z();

    // Predict the state
    ekf.predict(u);

    // Create the measurement vector
    ImuMeasurement z;
    z << avg_imu.getaccel_x(), avg_imu.getaccel_y(), avg_imu.getaccel_z(),
         avg_imu.getgyro_x(), avg_imu.getgyro_y(), avg_imu.getgyro_z();

    // Update the state
    ekf.update(z);

    // Get the state
    State x = ekf.getState();

    // Output the attitude
    m_attitude.set(x(6), x(7), x(8), x(9));
    this->attitudeOut_out(0, m_attitude);

    // Telemetry
    this->tlmWrite_AttitudeQ1(m_attitude.getq1());
    this->tlmWrite_AttitudeQ2(m_attitude.getq2());
    this->tlmWrite_AttitudeQ3(m_attitude.getq3());
    this->tlmWrite_AttitudeQ4(m_attitude.getq4());
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
    pressureIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Pressure &pressure
    )
  {
    // TODO: Implement sensor redundancy
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
