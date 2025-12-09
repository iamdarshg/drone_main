#ifndef SENSORS_ICP101XX_HPP
#define SENSORS_ICP101XX_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Pressure.hpp>
#include <Fw/Types/Temperature.hpp>

namespace Sensors {

  class Icp101xx : public Fw::ActiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Icp101xx object
      Icp101xx(
          const char *const compName //!< The component name
      );

      //! Initialize Icp101xx object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Icp101xx object
      ~Icp101xx();

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

        //! The pressure data
        Fw::Pressure m_pressure;

        //! The temperature data
        Fw::Temperature m_temperature;
  };

} // end namespace Sensors

#endif
