import time
import wiringpi2 as wp


class ADS1256:
    """ Wiring Diagram
     +-----+-----+---------+------+---+---Pi 2---+---+------+---------+-----+-----+
     | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
     +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
     |     |     |    3.3v |      |   |  1 || 2  |   |      | 5v      |     |     |
     |   2 |   8 |   SDA.1 |   IN | 1 |  3 || 4  |   |      | 5V      |     |     |
     |   3 |   9 |   SCL.1 |   IN | 1 |  5 || 6  |   |      | 0v      |     |     |
     |   4 |   7 | GPIO. 7 |   IN | 1 |  7 || 8  | 1 | ALT0 | TxD     | 15  | 14  |
     |     |     |      0v |      |   |  9 || 10 | 1 | ALT0 | RxD     | 16  | 15  |
     |  17 |   0 | GPIO. 0 |   IN | 0 | 11 || 12 | 1 | IN   | GPIO. 1 | 1   | 18  |
     |  27 |   2 | GPIO. 2 |   IN | 1 | 13 || 14 |   |      | 0v      |     |     |
     |  22 |   3 | GPIO. 3 |   IN | 0 | 15 || 16 | 0 | IN   | GPIO. 4 | 4   | 23  |
     |     |     |    3.3v |      |   | 17 || 18 | 0 | IN   | GPIO. 5 | 5   | 24  |
     |  10 |  12 |    MOSI | ALT0 | 0 | 19 || 20 |   |      | 0v      |     |     |
     |   9 |  13 |    MISO | ALT0 | 0 | 21 || 22 | 0 | IN   | GPIO. 6 | 6   | 25  |
     |  11 |  14 |    SCLK | ALT0 | 0 | 23 || 24 | 1 | OUT  | CE0     | 10  | 8   |
     |     |     |      0v |      |   | 25 || 26 | 1 | OUT  | CE1     | 11  | 7   |
     |   0 |  30 |   SDA.0 |   IN | 1 | 27 || 28 | 1 | IN   | SCL.0   | 31  | 1   |
     |   5 |  21 | GPIO.21 |   IN | 1 | 29 || 30 |   |      | 0v      |     |     |
     |   6 |  22 | GPIO.22 |   IN | 1 | 31 || 32 | 0 | IN   | GPIO.26 | 26  | 12  |
     |  13 |  23 | GPIO.23 |   IN | 0 | 33 || 34 |   |      | 0v      |     |     |
     |  19 |  24 | GPIO.24 |   IN | 0 | 35 || 36 | 0 | IN   | GPIO.27 | 27  | 16  |
     |  26 |  25 | GPIO.25 |   IN | 0 | 37 || 38 | 0 | IN   | GPIO.28 | 28  | 20  |
     |     |     |      0v |      |   | 39 || 40 | 0 | IN   | GPIO.29 | 29  | 21  |
     +-----+-----+---------+------+---+----++----+---+------+---------+-----+-----+
     | BCM | wPi |   Name  | Mode | V | Physical | V | Mode | Name    | wPi | BCM |
     +-----+-----+---------+------+---+---Pi 2---+---+------+---------+-----+-----+

    ADS1256 COMMANDS
    WAKEUP Completes SYNC and Exits Standby Mode 0000 0000 (00h)
    RDATA Read Data 0000 0001 (01h)
    RDATAC Read Data Continuously 0000 0011 (03h)
    SDATAC Stop Read Data Continuously 0000 1111 (0Fh)
    done - RREG Read from REG rrr 0001 rrrr (1xh) 0000 nnnn
    done - WREG Write to REG rrr 0101 rrrr (5xh) 0000 nnnn
    SELFCAL Offset and Gain Self-Calibration 1111 0000 (F0h)
    SELFOCAL Offset Self-Calibration 1111 0001 (F1h)
    SELFGCAL Gain Self-Calibration 1111 0010 (F2h)
    SYSOCAL System Offset Calibration 1111 0011 (F3h)
    SYSGCAL System Gain Calibration 1111 0100 (F4h)
    SYNC Synchronize the A/D Conversion 1111 1100 (FCh)
    STANDBY Begin Standby Mode 1111 1101 (FDh)
    RESET Reset to Power-Up Values 1111 1110 (FEh)
    WAKEUP Completes SYNC and Exits Standby Mode 1111 1111 (FFh)
    """

    SPI_BIT_ORDER       = True
    SPI_MODE            = 0
    SPI_FREQUENCY       = 244000 # frequency suggested on pg6 of the data
                                 # sheet (SCLK period min 4 max 10 of base
                                 # clock, which is 7.68MHz)
    DRDY_TIMEOUT        = 0.5 # Seconds to wait for DRDY when communicating
    DATA_TIMEOUT        = 0.00001 # 10uS delay for sending data
    FCLK_FREQUENCY      = 7680000 # default clock rate is 7.68MHz

    # ADS Chip State
    CURRENT_GAIN    = -1
    CURRENT_DRATE   = -1

    # Register addresses
    REG_STATUS  = 0x00
    REG_MUX     = 0x01
    REG_ADCON   = 0x02
    REG_DRATE   = 0x03
    REG_IO      = 0x04
    REG_OFC0    = 0x05
    REG_OFC1    = 0x06
    REG_OFC2    = 0x07
    REG_FSC0    = 0x08 
    REG_FSC1    = 0x09
    REG_FSC2    = 0x010
    NUM_REG     = 11 
    
import spidev
import RPi.GPIO as GPIO

spi = null
"""

This is a list of the commands accepted by the ADS1256

WAKEUP Completes SYNC and Exits Standby Mode 0000 0000 (00h)
RDATA Read Data 0000 0001 (01h)
RDATAC Read Data Continuously 0000 0011 (03h)
SDATAC Stop Read Data Continuously 0000 1111 (0Fh)
done - RREG Read from REG rrr 0001 rrrr (1xh) 0000 nnnn
done - WREG Write to REG rrr 0101 rrrr (5xh) 0000 nnnn
SELFCAL Offset and Gain Self-Calibration 1111 0000 (F0h)
SELFOCAL Offset Self-Calibration 1111 0001 (F1h)
SELFGCAL Gain Self-Calibration 1111 0010 (F2h)
SYSOCAL System Offset Calibration 1111 0011 (F3h)
SYSGCAL System Gain Calibration 1111 0100 (F4h)
SYNC Synchronize the A/D Conversion 1111 1100 (FCh)
STANDBY Begin Standby Mode 1111 1101 (FDh)
RESET Reset to Power-Up Values 1111 1110 (FEh)
WAKEUP Completes SYNC and Exits Standby Mode 1111 1111 (FFh)
"""

# The RPI GPIO to use for chip select and ready polling
CS                  = 15 
DRDY                = 11
SPI_BIT_ORDER       = True
SPI_MODE            = 0b00
DRDY_TIMEOUT        = 0.5 # Seconds to wait for DRDY when communicating
DATA_TIMEOUT        = 0.00001 # 10uS delay for sending data
SCLK_FREQUENCY      = 7680000 # default clock rate is 7.68MHz

# ADS Chip State
CURRENT_GAIN    = -1
CURRENT_DRATE   = -1

# Register addresses
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

"""
DRATE Register: A/D Data Rate Address 0x03 The 16 valid Data Rate settings are shown below. Make sure to select a
valid setting as the invalid settings may produce unpredictable results.

Bits 7-0 DR[7: 0]: Data Rate Setting(1)

    11110000 = 30,000SPS (default)
    11100000 = 15,000SPS
    11010000 = 7,500SPS
    11000000 = 3,750SPS
    10110000 = 2,000SPS
    10100001 = 1,000SPS
    10010010 = 500SPS
    10000010 = 100SPS
    01110010 = 60SPS
    01100011 = 50SPS
    01010011 = 30SPS
    01000011 = 25SPS
    00110011 = 15SPS
    00100011 = 10SPS
    00010011 = 5SPS
    00000011 = 2.5SPS

    (1) for fCLKIN = 7.68MHz. Data rates scale linearly with fCLKIN
"""
# Data rates
DRATE_30000     = 0b11110000 # 30,000SPS (default)
DRATE_15000     = 0b11100000 # 15,000SPS
DRATE_7500      = 0b11010000 # 7,500SPS
DRATE_3750      = 0b11000000 # 3,750SPS
DRATE_2000      = 0b10110000 # 2,000SPS
DRATE_1000      = 0b10100001 # 1,000SPS
DRATE_500       = 0b10010010 # 500SPS
DRATE_100       = 0b10000010 # 100SPS
DRATE_60        = 0b01110010 # 60SPS
DRATE_50        = 0b01100011 # 50SPS
DRATE_30        = 0b01010011 # 30SPS
DRATE_25        = 0b01000011 # 25SPS
DRATE_15        = 0b00110011 # 15SPS
DRATE_10        = 0b00100011 # 10SPS
DRATE_5         = 0b00010011 # 5SPS
DRATE_2_5       = 0b00000011 # 2.5SPS

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

"""
Status Register Configuration - logically OR all desired options together
to form a 1 byte command and write it to the STATUS register

STATUS REGISTER - ADDRESS 0x00
Bits 7-4 ID3, ID2, ID1, ID0 Factory Programmed Identification Bits 
(Read Only)

Bit 3 ORDER: Data Output Bit Order

    0 = Most Significant Bit First (default)
    1 = Least Significant Bit First

    Input data is always shifted in most significant byte and bit first.
    Output data is always shifted out most significant byte first. The
    ORDER bit only controls the bit order of the output data within the
    byte.

Bit 2 ACAL: Auto-Calibration

    0 = Auto-Calibration Disabled (default)
    1 = Auto-Calibration Enabled

    When Auto-Calibration is enabled, self-calibration begins at the
    completion of the WREG command that changes the PGA (bits 0-2 of ADCON
    register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the
    STATUS register) values.

Bit 1 BUFEN: Analog Input Buffer Enable

    0 = Buffer Disabled (default)
    1 = Buffer Enabled

Bit 0 DRDY: Data Ready (Read Only)

    This bit duplicates the state of the DRDY pin, which is inverted logic.
"""
STATUS_BUFFER_ENABLE    = 0x02
STATUS_AUTOCAL_ENABLE   = 0x04
STATUS_ORDER_LSB        = 0x08


"""
A/D Control Register - Address 0x02

Bit 7 Reserved, always 0 (Read Only)

Bits 6-5 CLK1, CLK0: D0/CLKOUT Clock Out Rate Setting

    00 = Clock Out OFF
    01 = Clock Out Frequency = fCLKIN (default)
    10 = Clock Out Frequency = fCLKIN/2
    11 = Clock Out Frequency = fCLKIN/4

    When not using CLKOUT, it is recommended that it be turned off. These
    bits can only be reset using the RESET pin.

Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources

    00 = Sensor Detect OFF (default)
    01 = Sensor Detect Current = 0.5uA
    10 = Sensor Detect Current = 2uA
    11 = Sensor Detect Current = 10uA

    The Sensor Detect Current Sources can be activated to verify the
    integrity of an external sensor supplying a signal to the ADS1255/6.
    A shorted sensor produces a very small signal while an open-circuit
    sensor produces a very large signal.

Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
    000 = 1 (default)
    001 = 2
    010 = 4
    011 = 8
    100 = 16
    101 = 32
    110 = 64
    111 = 64
"""
# Gain levels
AD_GAIN_1      = 0x00
AD_GAIN_2      = 0x01
AD_GAIN_4      = 0x02
AD_GAIN_8      = 0x03
AD_GAIN_16     = 0x04
AD_GAIN_32     = 0x05
AD_GAIN_64     = 0x06

# Sensor Detect Current Sources
AD_SDCS_500pA   = 0x08
AD_SDCS_2uA     = 0x10
AD_SDCS_10uA    = 0x18
>>>>>>> c99d3783c8890785d4f2443fd90043f7b1c75dc9

    """
    DRATE Register: A/D Data Rate Address 0x03 The 16 valid Data Rate settings are
    shown below. Make sure to select a valid setting as the invalid settings may
    produce unpredictable results.

    Bits 7-0 DR[7: 0]: Data Rate Setting(1)

        11110000 = 30,000SPS (default)
        11100000 = 15,000SPS
        11010000 = 7,500SPS
        11000000 = 3,750SPS
        10110000 = 2,000SPS
        10100001 = 1,000SPS
        10010010 = 500SPS
        10000010 = 100SPS
        01110010 = 60SPS
        01100011 = 50SPS
        01010011 = 30SPS
        01000011 = 25SPS
        00110011 = 15SPS
        00100011 = 10SPS
        00010011 = 5SPS
        00000011 = 2.5SPS

        (1) for fCLKIN = 7.68MHz. Data rates scale linearly with fCLKIN
    """
    # Data rates
    DRATE_30000     = 0b11110000 # 30,000SPS (default)
    DRATE_15000     = 0b11100000 # 15,000SPS
    DRATE_7500      = 0b11010000 # 7,500SPS
    DRATE_3750      = 0b11000000 # 3,750SPS
    DRATE_2000      = 0b10110000 # 2,000SPS
    DRATE_1000      = 0b10100001 # 1,000SPS
    DRATE_500       = 0b10010010 # 500SPS
    DRATE_100       = 0b10000010 # 100SPS
    DRATE_60        = 0b01110010 # 60SPS
    DRATE_50        = 0b01100011 # 50SPS
    DRATE_30        = 0b01010011 # 30SPS
    DRATE_25        = 0b01000011 # 25SPS
    DRATE_15        = 0b00110011 # 15SPS
    DRATE_10        = 0b00100011 # 10SPS
    DRATE_5         = 0b00010011 # 5SPS
    DRATE_2_5       = 0b00000011 # 2.5SPS

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

    """
    Status Register Configuration - logically OR all desired options together
    to form a 1 byte command and write it to the STATUS register

    STATUS REGISTER - ADDRESS 0x00
    Bits 7-4 ID3, ID2, ID1, ID0 Factory Programmed Identification Bits 
    (Read Only)

    Bit 3 ORDER: Data Output Bit Order

        0 = Most Significant Bit First (default)
        1 = Least Significant Bit First

        Input data is always shifted in most significant byte and bit first.
        Output data is always shifted out most significant byte first. The
        ORDER bit only controls the bit order of the output data within the
        byte.

    Bit 2 ACAL: Auto-Calibration

        0 = Auto-Calibration Disabled (default)
        1 = Auto-Calibration Enabled

        When Auto-Calibration is enabled, self-calibration begins at the
        completion of the WREG command that changes the PGA (bits 0-2 of ADCON
        register), DR (bits 7-0 in the DRATE register) or BUFEN (bit 1 in the
        STATUS register) values.

    Bit 1 BUFEN: Analog Input Buffer Enable

        0 = Buffer Disabled (default)
        1 = Buffer Enabled

    Bit 0 DRDY: Data Ready (Read Only)

        This bit duplicates the state of the DRDY pin, which is inverted logic.
    """
    STATUS_BUFFER_ENABLE    = 0x02
    STATUS_AUTOCAL_ENABLE   = 0x04
    STATUS_ORDER_LSB        = 0x08


    """
    A/D Control Register - Address 0x02

    Bit 7 Reserved, always 0 (Read Only)

    Bits 6-5 CLK1, CLK0: D0/CLKOUT Clock Out Rate Setting

        00 = Clock Out OFF
        01 = Clock Out Frequency = fCLKIN (default)
        10 = Clock Out Frequency = fCLKIN/2
        11 = Clock Out Frequency = fCLKIN/4

        When not using CLKOUT, it is recommended that it be turned off. These
        bits can only be reset using the RESET pin.

    Bits 4-3 SDCS1, SCDS0: Sensor Detect Current Sources

        00 = Sensor Detect OFF (default)
        01 = Sensor Detect Current = 0.5uA
        10 = Sensor Detect Current = 2uA
        11 = Sensor Detect Current = 10uA

        The Sensor Detect Current Sources can be activated to verify the
        integrity of an external sensor supplying a signal to the ADS1255/6.
        A shorted sensor produces a very small signal while an open-circuit
        sensor produces a very large signal.

    Bits 2-0 PGA2, PGA1, PGA0: Programmable Gain Amplifier Setting
        000 = 1 (default)
        001 = 2
        010 = 4
        011 = 8
        100 = 16
        101 = 32
        110 = 64
        111 = 64
    """
    # Gain levels
    AD_GAIN_1      = 0x00
    AD_GAIN_2      = 0x01
    AD_GAIN_4      = 0x02
    AD_GAIN_8      = 0x03
    AD_GAIN_16     = 0x04
    AD_GAIN_32     = 0x05
    AD_GAIN_64     = 0x06

    # Sensor Detect Current Sources
    AD_SDCS_500pA   = 0x08
    AD_SDCS_2uA     = 0x10
    AD_SDCS_10uA    = 0x18

    # Clock divider
    AD_CLK_EQUAL    = 0x20
    AD_CLK_HALF     = 0x40
    AD_CLK_FOURTH   = 0x60

    # The RPI GPIO to use for chip select and ready polling
    def __init__(self)
        # Set up the wiringpi object to use physical pin numbers
        wp.wiringPiSetupPhys()

        # Initialize the DRDY pin
        wp.pinMode(self.DRDY_PIN, wp.INPUT)

        # Initialize the reset pin
        wp.pinMode(self.RESET_PIN, wp.OUTPUT)

        # Initialize PDWN pin
        wp.pinMode(self.PDWN_PIN, wp.OUTPUT)

        # Initialize CS pin
        wp.pinMode(self.CS_PIN, wp.OUTPUT)

        # Initialize SPI
        wp.wiringpiSPISetup(self.SPI_CHANNEL, self.SPI_FREQUENCY)


    def chip_select(self):
        wp.digitalWrite(CS_PIN, wp.LOW)

    def chip_release(self):
        wp.digitalWrite(CS_PIN, wp.HIGH)

    def WaitDRDY(self):
        """
        Delays until DRDY line goes low, allowing for automatic calibration
        """
        start = time.time()
        elapsed = time.time() - start

        # Waits for DRDY to go to zero or TIMEOUT seconds to pass
        drdy_level = wp.digitalRead(DRDY_PIN)
        while drdy_level and elapsed < DRDY_TIMEOUT:
            elapsed = time.time() - start

        if elapsed >= DRDY_TIMEOUT:
            print("WaitDRDY() Timeout\r\n")

    def SendByte(self, data, size):
        """
        Sends a byte to the SPI bus
        """
        wp.wiringPiSPIDataR

    def ReadByte(self):
        """
        Reads a byte from the SPI bus
        :returns: byte read from the bus
        """
        byte = spi.readbytes(1)
        return byte

    def DataDelay(self):
        """
        Delay from last SCLK edge to first SCLK rising edge

        Master clock rate is typically 7.68MHz, this is adjustable through the
        SCLK_FREQUENCY variable
        
        Datasheet states that the delay between requesting data and reading the
        bus must be minimum 50x SCLK period, this function reads data after
        60 x SCLK period.
        """
        timeout = (60 / SCLK_FREQUENCY)


        start = time.time()
        elapsed = time.time() - start

        # Wait for TIMEOUT to elapse
        while elapsed < DATA_TIMEOUT:
            elapsed = time.time() - start


    def ReadReg(self, start_reg, num_to_read):
        """
        Read the provided register, implements:
        
        RREG: Read from Registers

        Description: Output the data from up to 11 registers starting with the
        register address specified as part of the command. The number of
        registers read will be one plus the second byte of the command. If the
        count exceeds the remaining registers, the addresses will wrap back to
        the beginning.

        1st Command Byte: 0001 rrrr where rrrr is the address of the first
        register to read.

        2nd Command Byte: 0000 nnnn where nnnn is the number of bytes to read
        1. See the Timing Characteristics for the required delay between the
        end of the RREG command and the beginning of shifting data on DOUT: t6.
        """

        result = []

        # Pull the SPI bus low
        self.chip_select()
        
        # Send the byte command
        self.SendByte(CMD_RREG | start_reg)
        self.SendByte(num_to_read)

        # Wait for appropriate data delay
        DataDelay()

        # Read the register contents
        read = ReadByte()

        # Release the SPI bus
        chip_release()

        return read

    def WriteReg(self, start_register, data):
        """
        Writes data to the register, implements: 
        
        WREG: Write to Register

        Description: Write to the registers starting with the register
        specified as part of the command. The number of registers that
        will be written is one plus the value of the second byte in the
        command.

        1st Command Byte: 0101 rrrr where rrrr is the address to the first
        register to be written.
        
        2nd Command Byte: 0000 nnnn where nnnn is the number of bytes-1 to be
        written
        
        TODO: Implement multiple register write
        """

        # Select the ADS chip
        chip_select()

        # Tell the ADS chip which register to start writing at
        SendByte(CMD_WREG | register)

        # Tell the ADS chip how many additional registers to write
        SendByte(0x00)

        # Send the data
        SendByte(data)

        # Release the ADS chip
        chip_release()

    def ReadADC(self):
        """
        Reads ADC data, implements:

        RDATA: Read Data

        Description: Issue this command after DRDY goes low to read a single
        conversion result. After all 24 bits have been shifted out on DOUT,
        DRDY goes high. It is not necessary to read back all 24 bits, but DRDY
        will then not return high until new data is being updated. See the
        Timing Characteristics for the required delay between the end of the
        RDATA command and the beginning of shifting data on DOUT: t6
        """

        # Pull the SPI bus low
        chip_select()

        # Wait for data to be ready
        WaitDRDY()

        # Send the read command
        SendByte(CMD_RDATA)
        
        # Wait through the data pause
        DataDelay()

        # The result is 24 bits
        result.append(ReadByte())
        result.append(ReadByte())
        result.append(ReadByte())

        # Release the SPI bus
        chip_release()

        # Concatenate the bytes
        total  = (result[0] << 16)
        total |= (result[1] << 8)
        total |= result[2]

        return total


    def ReadID(self): 
        """
        Read the ID from the ADS chip
        :returns: numeric identifier of the ADS chip
        """
        WaitDRDY()
        id = ReadReg(REG_STATUS)
        return (id >> 4)
