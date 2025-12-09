#ifndef ACTUATORS_MOTOR_HPP
#define ACTUATORS_MOTOR_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Actuators {

  class Motor : public Fw::PassiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Motor object
      Motor(
          const char *const compName //!< The component name
      );

      //! Initialize Motor object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Motor object
      ~Motor();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for speedIn
      void speedIn_handler(
        NATIVE_INT_TYPE portNum,
        F32 speed
      );
  };

} // end namespace Actuators

#endif
