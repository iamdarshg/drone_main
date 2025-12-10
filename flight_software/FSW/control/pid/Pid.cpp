#include <flight_software/FSW/control/pid/Pid.hpp>
#include <Fw/Types/BasicTypes.hpp>

namespace Control {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Pid ::
    Pid(
        const char *const compName
    ) :
      PidComponentBase(compName),
      p_gain(1.0),
      i_gain(0.0),
      d_gain(0.0),
      integral(0.0),
      prev_error(0.0)
  {
    // Set a default setpoint
    setpoint.set(1.0, 0.0, 0.0, 0.0);
  }

  void Pid ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    PidComponentBase::init(instance);
  }

  Pid ::
    ~Pid()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Pid ::
    attitudeIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &attitude
    )
  {
    // TODO: Implement a proper PID controller for a quaternion
    // This is a simplified version for demonstration purposes.

    // Calculate error
    Fw::Quaternion error = setpoint * attitude.conjugate();

    // Proportional term
    F32 p_term = p_gain * error.getq1();

    // Integral term
    integral += error.getq1();
    F32 i_term = i_gain * integral;

    // Derivative term
    F32 d_term = d_gain * (error.getq1() - prev_error);
    prev_error = error.getq1();

    // Calculate output
    Fw::Actuator actuator_output;
    actuator_output.setroll(p_term + i_term + d_term);
    actuator_output.setpitch(0.0);
    actuator_output.setyaw(0.0);
    actuator_output.setthrust(0.0);

    // Output actuator commands
    this->actuatorOut_out(0, actuator_output);
  }

  void Pid ::
    setpointIn_handler(
        NATIVE_INT_TYPE portNum,
        Fw::Quaternion &setpoint
    )
  {
    this->setpoint = setpoint;
  }

  // ----------------------------------------------------------------------
  // Command handler implementations
  // ----------------------------------------------------------------------

  void Pid ::
    SET_GAINS_cmdHandler(
        FwOpcodeType opCode,
        U32 cmdSeq,
        F32 p,
        F32 i,
        F32 d
    )
  {
    this->p_gain = p;
    this->i_gain = i;
    this->d_gain = d;

    // Telemetry
    this->tlmWrite_p_gain(p);
    this->tlmWrite_i_gain(i);
    this->tlmWrite_d_gain(d);

    this->cmdResponse_out(opCode, cmdSeq, Fw::CmdResponse::OK);
  }

} // end namespace Control
