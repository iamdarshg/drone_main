#ifndef NAVIGATION_NAVIGATION_HPP
#define NAVIGATION_NAVIGATION_HPP

#include <Fw/Core/InputPortBase.hpp>
#include <Fw/Core/OutputPortBase.hpp>
#include <Fw/Comp/ActiveComponentBase.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <Fw/Types/Gps.hpp>
#include <Fw/Types/Guidance.hpp>

namespace Navigation {

  class Navigation : public Fw::ActiveComponentBase {
    public:
      // ----------------------------------------------------------------------
      // Construction, initialization, and destruction
      // ----------------------------------------------------------------------

      //! Construct Navigation object
      Navigation(
          const char *const compName //!< The component name
      );

      //! Initialize Navigation object
      void init(
          const NATIVE_INT_TYPE instance = 0 //!< The instance number
      );

      //! Destroy Navigation object
      ~Navigation();

    PRIVATE:
      // ----------------------------------------------------------------------
      // Handler implementations for user-defined typed input ports
      // ----------------------------------------------------------------------

      //! Handler implementation for gpsIn
      void gpsIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Gps &gps
      );

      //! Handler implementation for cmdIn
      void cmdIn_handler(
        NATIVE_INT_TYPE portNum,
        FwOpcodeType opCode,
        U32 cmdSeq
      );

    PRIVATE:
        // ----------------------------------------------------------------------
        // Member variables
        // ----------------------------------------------------------------------

        //! The guidance data
        Fw::Guidance m_guidance;
  };

} // end namespace Navigation

#endif
