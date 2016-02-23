import time
import wiringpi2

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

# Clock divider
AD_CLK_EQUAL    = 0x20
AD_CLK_HALF     = 0x40
AD_CLK_FOURTH   = 0x60

# SPI Bus and Device for RPi talk
spi = spidev.SpiDev()
BUS = 0
DEV = 1

def chip_select():
    spi.open(BUS, DEV)

def chip_release():
    spi.close()


def init():
    """
    Initializes the - the CS line and DRDY pins must be defined
    :returns: empty string if successful, error string if unsuccessful
    """
    spi.mode = SPI_MODE

    gpio.setmode(gpio.BOARD)
    gpio.setup(DRDY, gpio.IN)


def WaitDRDY():
    """
    Delays until DRDY line goes low, allowing for automatic calibration
    """
    start = time.time()
    elapsed = time.time() - start

    # Waits for DRDY to go to zero or TIMEOUT seconds to pass
    while gpio.input(DRDY) and elapsed < DRDY_TIMEOUT:
        elapsed = time.time() - start

    if elapsed >= DRDY_TIMEOUT:
        print("WaitDRDY() Timeout\r\n")

def SendByte(byte):
    """
    Sends a byte to the SPI bus
    """
    spi.writebytes(chr(byte))

def ReadByte():
    """
    Reads a byte from the SPI bus
    :returns: byte read from the bus
    """
    byte = spi.readbytes(1)
    return byte

def DataDelay():
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


def ReadReg(reg):
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

    TODO: Implement 2nd command byte
    """

    result = []

    # Pull the SPI bus low
    chip_select()
    
    # Send the byte command
    SendByte(CMD_RREG | reg)
    SendByte(0x00)

    # Wait for appropriate data delay
    DataDelay()

    # Read the register contents
    read = ReadByte()

    # Release the SPI bus
    chip_release()

    return read

def WriteReg(start_register, data):
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

def ReadADC():
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


def ReadID(): 
    """
    Read the ID from the ADS chip
    :returns: numeric identifier of the ADS chip
    """
    WaitDRDY()
    id = ReadReg(REG_STATUS)
    return (id >> 4)
