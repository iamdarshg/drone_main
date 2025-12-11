#include <flight_software/FSW/hal/spi/Spi.hpp>
#include <Fw/Types/BasicTypes.hpp>
#include <flight_software/FSW/hal/PinMap.hpp>

// Dummy hardware abstraction layer
void spi_init(U32 clk, U32 miso, U32 mosi) {}
void spi_write(U8* data, U32 size) {}
void spi_read(U8* data, U32 size) {}

namespace Hal {

  // ----------------------------------------------------------------------
  // Construction, initialization, and destruction
  // ----------------------------------------------------------------------

  Spi ::
    Spi(
        const char *const compName
    ) :
      SpiComponentBase(compName)
  {

  }

  void Spi ::
    init(
        const NATIVE_INT_TYPE instance
    )
  {
    SpiComponentBase::init(instance);
    spi_init(Hal::PinMap::SPI1_CLK, Hal::PinMap::SPI1_MISO, Hal::PinMap::SPI1_MOSI);
  }

  Spi ::
    ~Spi()
  {

  }

  // ----------------------------------------------------------------------
  // Handler implementations for user-defined typed input ports
  // ----------------------------------------------------------------------

  void Spi ::
    spiWrite_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    spi_write(data.getData(), data.getSize());
  }

  void Spi ::
    spiRead_handler(
        const NATIVE_INT_TYPE portNum,
        Fw::Buffer &data
    )
  {
    spi_read(data.getData(), data.getSize());
  }

} // end namespace Hal
