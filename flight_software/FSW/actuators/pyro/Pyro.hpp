#ifndef ACTUATORS_PYRO_HPP
#define ACTUATORS_PYRO_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Actuators {

  class Pyro : public Fw::PassiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Pyro object
      Pyro(
          const char *const compName //!< The component name
      );

      //! Initialize Pyro object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Pyro object
      ~Pyro();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for fireIn
      void fireIn_handler(
        NATIVE_INT_TYPE portNum,
        U32 channel,
        U32 duration
      );
  };

} // end namespace Actuators

#endif
