#include <flight_software/FSW/hal/i2c/I2c.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <flight_software/FSW/hal/PinMap.hpp>

// Dummy hardware abstraction layer
void i2c_init(U32 scl, U32 sda) {}
void i2c_write(U8 address, U8* data, U32 size) {}
void i2c_read(U8 address, U8* data, U32 size) {}

namespace Hal {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  I2c ::
    I2c(
        const char *const compName
    ) :
      I2cComponentBase(compName)
  {

  }

  void I2c ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    I2cComponentBase::init(instance);
    i2c_init(Hal::PinMap::I2C1_SCL, Hal::PinMap::I2C1_SDA);
  }

  I2c ::
    ~I2c()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void I2c ::
    i2cWrite_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    // This is a placeholder for the actual I2C address
    U8 address = 0;
    i2c_write(address, data.getData(), data.getSize());
  }

  void I2c ::
    i2cRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    // This is a placeholder for the actual I2C address
    U8 address = 0;
    i2c_read(address, data.getData(), data.getSize());
  }

} // end namespace Hal
