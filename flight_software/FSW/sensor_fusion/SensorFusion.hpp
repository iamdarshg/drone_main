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

namespace FSW {

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

        //! EKF state vector
        // TODO: Define the EKF state vector

        //! EKF covariance matrix
        // TODO: Define the EKF covariance matrix
  };

} // end namespace FSW

#endif
