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
      NavigationComponentBase(compName)
  {

  }

  void Navigation ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    NavigationComponentBase::init(instance);
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
    // TODO: Implement navigation logic to switch waypoints
    if (waypoints.size() > 0) {
        Fw::Guidance guidance;
        guidance.setlatitude(waypoints[0].getlatitude());
        guidance.setlongitude(waypoints[0].getlongitude());
        guidance.setaltitude(waypoints[0].getaltitude());
        this->guidanceOut_out(0, guidance);
    }
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void Navigation ::
    ADD_WAYPOINT_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        F64 latitude,
        F64 longitude,
        F32 altitude
    )
  {
    Fw::Gps waypoint;
    waypoint.setlatitude(latitude);
    waypoint.setlongitude(longitude);
    waypoint.setaltitude(altitude);
    waypoints.push_back(waypoint);
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Navigation ::
    CLEAR_WAYPOINTS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    waypoints.clear();
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

  void Navigation ::
    LIST_WAYPOINTS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq
    )
  {
    // TODO: Implement waypoint listing
    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

} // end namespace Navigation
