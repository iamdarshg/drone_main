#ifndef CONTROL_MIXER_HPP
#define CONTROL_MIXER_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/PassiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Actuator.hpp>

namespace Control {

  class Mixer : public Fw::PassiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Mixer object
      Mixer(
          const char *const compName //!< The component name
      );

      //! Initialize Mixer object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Mixer object
      ~Mixer();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for actuatorIn
      void actuatorIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Actuator &actuator
      );
  };

} // end namespace Control

#endif
