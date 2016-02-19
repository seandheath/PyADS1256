import PyBCM2835 as bcm
import time


class 
    # The RPI GPIO to use for chip select and ready polling
    CS                  = bcm.RPI_GPIO_P1_15 
    DRDY                = bcm.RPI_GPIO_P1_11o
    SPI_BIT_ORDER       = bcm.SPI_BIT_ORDER_LSBFIRST
    SPI_MODE            = bcm.SPI_MODE1
    SPI_CLOCK_DIVIDER   = bcm.SPI_CLOCK_DIVIDER_8192
    DRDY_TIMEOUT        = 0.5 # Seconds to wait for DRDY when communicating
    DATA_TIMEOUT        = 0.00001 # 10uS delay for sending data

    # ADS Chip State
    CURRENT_GAIN    = -1
    CURRENT_DRATE   = -1

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

    def WaitDRDY(self):
        """
        Delays until DRDY line goes low, allowing for automatic calibration
        """
        start = time.time()
        elapsed = time.time() - start

        # Waits for DRDY to go to zero or TIMEOUT seconds to pass
        while bcm.gpio_lev(self.DRDY) != 0 and elapsed < self.DRDY_TIMEOUT:
            elapsed = time.time() - start

        if elapsed >= self.DRDY_TIMEOUT:
            print("WaitDRDY() Timeout\r\n")

    def SendByte(self, cmd):
        """
        Sends a byte to the indicated command register of the ads chip
        """
        bcm.spi_transfer(cmd)

    def ReadByte(self):
        """
        Reads a byte from the SPI bus
        :returns: byte read from the bus
        """
        byte = bcm.spi_transfer(0xFF)
        return byte

    def DataDelay(self):
        """
        Delay from last SCLK edge to first SCLK rising edge
        """
        start = time.time()
        elapsed = time.time() - start

        # Wait for TIMEOUT to elapse
        while elapsed < self.DATA_TIMEOUT:
            elapsed = time.time() - start
    

    def ReadReg(self, reg):
        """
        Read the provided register
        :returns: the contents of the provided register
        """

        result = []

        # Pull the SPI bus low
        bcm.gpio_write(self.CS, bcm.LOW)
        
        # Send the byte command
        self.SendByte(self.CMD_RREG | reg)

        # Clear the bus
        self.SendByte(0x00)
        self.DataDelay()

        # Read the register contents
        read = self.ReadByte()

        # Release the SPI bus
        bcm.gpio_write(self.CS, bcm.HIGH)

        return read

    def ReadData(self):
        """
        Reads ADC data
        """

        # Pull the SPI bus low
        bcm.gpio_write(self.CS, bcm.LOW)

        # Send the read command
        self.SendByte(self.CMD_RDATA)
        self.DataDelay()

        # The result is 24 bits
        result.append(self.ReadByte())
        result.append(self.ReadByte())
        result.append(self.ReadByte())

        # Release the SPI bus
        bcm.gpio_write(self.CS, bcm.HIGH)

        # Concatenate the bytes
        total = result[0] << 16
        total |= result[1] << 8
        total |= result[2]

        return total

    def CfgADC(self, gain, data_rate):
        """
        Configures the gain and data rate of the ADS chip
        
        Status Register:
        Bit 0: DRDY - Data Ready

            Read only

        Bit 1: BUFEN - Analog Input Buffer Enable

            0 = Buffer disabled (default)
            1 = Buffer enabled

        Bit 2: ACAL - Auto-calibrate

            0 = Auto-calibration disabled (default)
            1 = Auto-calibration enabled
            
            When auto-calibration is enabled self-calibration begins at the
            completion of the WREG command that changes the PGA (bits 0-2 of
            ADCON register), DR (bits 7-0 in the DRATE register), or BUFEN
            (bit 1 of the STATUS register) values.

        Bit 3: ORDER - Data Output Bit Order
            
            0 = Most significant bit first (default)
            1 = Least significant bit first

            Input data is always shifted in most significant byte and bit
            first. Output data is always shifted out most significant byte
            first. The ORDER bit controls the bit order of the output data
            within the byte.

        Bits 4-7: ID0, ID1, ID2, ID3 - Factory Programmed ID Bits

            Read only


        """
        self.CURRENT_GAIN = gain
        self.CURRENT_DRATE = data_rate

        self.WaitDRDY()
        

        # Disable the internal buffer
        parameter1 = 0
        parameter1 = 1 << 2 # ORDER

        



    def ReadID(self):
        """
        Read the ID from the ADS chip
        :returns: numeric identifier of the ADS chip
        """
        self.WaitDRDY()
        id = ReadReg(self.REG_STATUS)
        return id >> 4
