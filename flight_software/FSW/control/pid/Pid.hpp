#ifndef CONTROL_PID_HPP
#define CONTROL_PID_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Quaternion.hpp>
#include <Fw/Types/Actuator.hpp>

namespace Control {

  class Pid : public Fw::ActiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Pid object
      Pid(
          const char *const compName //!< The component name
      );

      //! Initialize Pid object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Pid object
      ~Pid();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for attitudeIn
      void attitudeIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &attitude
      );

      //! Handler implementation for setpointIn
      void setpointIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &setpoint
      );

    PRIVATE:
        // ----------------------------------------------------------------------
        // Member variables
        // ----------------------------------------------------------------------

        //! The actuator commands
        Fw::Actuator m_actuator;
  };

} // end namespace Control

#endif
