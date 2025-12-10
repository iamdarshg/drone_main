#include <flight_software/FSW/hal/can/Can.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <flight_software/FSW/hal/PinMap.hpp>

// Dummy hardware abstraction layer
void can_init(U32 tx, U32 rx) {}
void can_write(U32 id, U8* data, U8 size) {}
void can_read(U32& id, U8* data, U8& size) {}

namespace Hal {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Can ::
    Can(
        const char *const compName
    ) :
      CanComponentBase(compName)
  {

  }

  void Can ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    CanComponentBase::init(instance);
    can_init(Hal::PinMap::CAN1_TX, Hal::PinMap::CAN1_RX);
  }

  Can ::
    ~Can()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Can ::
    canWrite_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    // This is a placeholder for the actual CAN ID
    U32 id = 0;
    can_write(id, data.getData(), data.getSize());
  }

  void Can ::
    canRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    // This is a placeholder for the actual CAN ID
    U32 id = 0;
    U8 size = 0;
    can_read(id, data.getData(), size);
    data.setSize(size);
  }

} // end namespace Hal
