import PyBCM2835 as bcm
import time


class 
    # The RPI GPIO to use for chip select and ready polling
    CS                  = bcm.RPI_GPIO_P1_15 
    DRDY                = bcm.RPI_GPIO_P1_11o
    SPI_BIT_ORDER       = bcm.SPI_BIT_ORDER_LSBFIRST
    SPI_MODE            = bcm.SPI_MODE1
    SPI_CLOCK_DIVIDER   = bcm.SPI_CLOCK_DIVIDER_8192

    # Register definitions
    REG_STATUS  = 0
    REG_MUX     = 1
    REG_ADCON   = 2
    REG_DRATE   = 3
    REG_IO      = 4
    REG_OFC0    = 5
    REG_OFC1    = 6
    REG_OFC2    = 7
    REG_FSC0    = 8 
    REG_FSC1    = 9
    REG_FSC2    = 10

    # Gain levels
    GAIN_1      = 0
    GAIN_2      = 1
    GAIN_4      = 2
    GAIN_6      = 3
    GAIN_8      = 4
    GAIN_16     = 5
    GAIN_32     = 6

    # Data rates, samples per second
    DRATE_30000 = 0
    DRATE_15000 = 1
    DRATE_7500  = 2
    DRATE_3750  = 3
    DRATE_2000  = 4
    DRATE_1000  = 5
    DRATE_500   = 6
    DRATE_100   = 7
    DRATE_60    = 8
    DRATE_50    = 9
    DRATE_30    = 10
    DRATE_25    = 11
    DRATE_15    = 12
    DRATE_10    = 13
    DRATE_5     = 15
    DRATE_2     = 16

    # Commands
    CMD_WAKEUP  = 0x00 # Completes SYNC and exits standby mode
    CMD_RDATA   = 0x01 # Read data
    CMD_RDATAC  = 0x03 # Start read data continuously
    CMD_SDATAC  = 0x0F # Stop read data continuously
    CMD_RREG    = 0x10 # Read from register
    CMD_WREG    = 0x50 # Write to register
    CMD_SELFCAL = 0xF0 # Offset and gain self-calibration
    CMD_SELFOCAL= 0xF1 # Offset self-calibration
    CMD_SELFGCAL= 0xF2 # Gain self-calibration
    CMD_SYSOCAL = 0xF3 # System offset calibration
    CMD_SYSGCAL = 0xF4 # System gain calibration
    CMD_SYNC    = 0xFC # Synchronize the A/D conversion
    CMD_STANDBY = 0xFD # Begin standby mode
    CMD_RESET   = 0xFE # Reset to power-on values



    def init(self):
        """
        Initializes the - the CS line and DRDY pins must be defined
        :returns: empty string if successful, error string if unsuccessful
        """
        retval = ""

        bcm.init()
        bcm.spi_begin()
        bcm.spi_setBitOrder(self.SPI_BIT_ORDER)
        bcm.spi_setDataMode(self.SPI_DATA_MODE)
        bcm.spi_setClockDivider(self.SPI_CLOCK_DIVIDER)
        bcm.gpio_fsel(self.CS, bcm.GPIO_FSEL_OUTP)
        bcm.gpio_write(self.CS, bcm.HIGH)
        bcm.gpio_fsel(self.DRDY, bcm.GPIO_FSEL_INPT)
        bcm.gpio_set_pud(self.DRDY, bcm.GPIO_PUD_UP)



    def ads_ReadID():
        return ads_ReadReg(




    if __name__ == "__main__":
        main()
