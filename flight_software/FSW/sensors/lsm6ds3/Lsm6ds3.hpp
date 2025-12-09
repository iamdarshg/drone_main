#ifndef SENSORS_LSM6DS3_HPP
#define SENSORS_LSM6DS3_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Imu.hpp>

namespace Sensors {

  class Lsm6ds3 : public Fw::ActiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Lsm6ds3 object
      Lsm6ds3(
          const char *const compName //!< The component name
      );

      //! Initialize Lsm6ds3 object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Lsm6ds3 object
      ~Lsm6ds3();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for schedIn
      void schedIn_handler(
        NATIVE_INT_TYPE portNum,
        NATIVE_UINT_TYPE context
      );

    PRIVATE:
        // ----------------------------------------------------------------------
        // Member variables
        // ----------------------------------------------------------------------

        //! The IMU data
        Fw::Imu m_imu;
  };

} // end namespace Sensors

#endif
