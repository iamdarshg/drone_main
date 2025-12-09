#include <flight_software/FSW/navigation/Navigation.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Navigation {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Navigation ::
    Navigation(
        const char *const compName
    ) :
      Fw::ActiveComponentBase(compName)
  {

  }

  void Navigation ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    Fw::ActiveComponentBase::init(instance);
  }

  Navigation ::
    ~Navigation()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Navigation ::
    gpsIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Gps &gps
    )
  {
    // TODO: Implement navigation logic
    // TODO: Output guidance data
  }

  void Navigation ::
    cmdIn_handler(
        NATIVE_INT_TYPE portNum,
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    // TODO: Implement waypoint management
  }

} // end namespace Navigation
