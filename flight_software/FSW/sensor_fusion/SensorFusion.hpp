#ifndef SENSOR_FUSION_HPP
#define SENSOR_FUSION_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Imu.hpp>
#include <Fw/Types/Pressure.hpp>
#include <Fw/Types/Quaternion.hpp>
#include <Fw/Types/Gps.hpp>
#include <kalman/ekf.hpp>

namespace FSW {

  // State vector
  typedef Kalman::Vector<10> State;
  // Control vector
  typedef Kalman::Vector<6> Control;
  // Measurement vector for IMU
  typedef Kalman::Vector<6> ImuMeasurement;
  // Measurement vector for GPS
  typedef Kalman::Vector<3> GpsMeasurement;
  // Measurement vector for Barometer
  typedef Kalman::Vector<1> BaroMeasurement;

  class SensorFusion : public Fw::ActiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct SensorFusion object
      SensorFusion(
          const char *const compName //!< The component name
      );

      //! Initialize SensorFusion object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy SensorFusion object
      ~SensorFusion();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for imuIn
      void imuIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Imu &imu
      );

      //! Handler implementation for imuIn2
      void imuIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Imu &imu
      );

      //! Handler implementation for pressureIn
      void pressureIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Pressure &pressure
      );

      //! Handler implementation for pressureIn2
      void pressureIn2_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Pressure &pressure
      );

      //! Handler implementation for gpsIn
      void gpsIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Gps &gps
      );

    PRIVATE:
        // ----------------------------------------------------------------------
        // Member variables
        // ----------------------------------------------------------------------

        //! The fused attitude data
        Fw::Quaternion m_attitude;

        //! EKF
        Kalman::EKF<State, Control, ImuMeasurement, GpsMeasurement, BaroMeasurement> ekf;

        //! System model
        // TODO: Implement the system model

        //! Measurement models
        // TODO: Implement the measurement models
  };

} // end namespace FSW

#endif
