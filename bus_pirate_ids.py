from pyftdi.ftdi import Ftdi
from pyftdi.eeprom import FtdiEeprom


# dump EEPROM
ftdi = Ftdi()
ftdi.open(0x0403, 0x6001)
eeprom = ftdi.read_eeprom()
print(eeprom)
ftdi.close()

# read stuff
eeprom = FtdiEeprom()
eeprom.open('ftdi://ftdi:232r/1')
print('Manufacturer:', eeprom.manufacturer)
print('Product:     ', eeprom.product)
print('Serial:      ', eeprom.serial)
eeprom.close()

