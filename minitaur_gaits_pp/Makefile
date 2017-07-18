

# MODIFY THESE
PORT=COM4
# mblc0.5.2: koduino:stm32:f303v:hse=8m,bootloader=stm32loader
# grmbm: koduino:stm32:f446:bootloader=stm32loader
# grbl*: koduino:stm32:f301:flashsize=32k,hse=8m,bootloader=stlink
ARDUINO_BOARD=koduino:stm32:f303v:hse=8m,bootloader=stm32loader
# blank, or --verbose, or --verbose-upload
ARDUINO_OPTS=

# DON'T MODIFY THESE
null:=
SPACE:= $(null) $(null)
SKETCH=$(CURDIR)/$(notdir $(subst $(SPACE),_,$(CURDIR))).ino
# May need to modify this
ARDUINO_DEBUG=$(subst \,/,$(USERPROFILE))/Documents/arduino-1.6.1/arduino_debug

all:
	$(ARDUINO_DEBUG) $(ARDUINO_OPTS) --upload --board $(ARDUINO_BOARD) --port $(PORT) "$(SKETCH)"

# # 

# # # Mainboard2
# # VARIANT = f40_41x
# # UPLOAD_METHOD = SERIAL
# # HSE_VALUE = 16000000UL
# # # dtr_rts or rts_trpl_inv
# # UPLOAD_ENTRY = rts_trpl_inv
# # UPLOAD_BAUD = 115200
# # SECTOR_ERASE = 1

# # mblc0.4
# VARIANT = f303v
# UPLOAD_METHOD = SERIAL
# UPLOAD_ENTRY = mblc
# UPLOAD_PORT = COM4
# HSE_VALUE = 8000000UL
# KODUINO_ISRS = INLINE
# # 72MHz
# CLKSRC = HSE_8M
# # overclocked to 112 MHz
# # CLKSRC = HSE_8M_OC112
# EEP_START = 0x0800F000
# # EEP_LEN = 0
# MAX_PROGRAM_SIZE = 61440

# # # bllc0.6
# # VARIANT = f301
# # #VARIANT = f37x
# # UPLOAD_METHOD = SERIAL
# # HSE_VALUE = 16000000UL
# # # dtr_rts or rts_trpl_inv
# # UPLOAD_ENTRY = none
# # SECTOR_ERASE = 1
# # MAX_PROGRAM_SIZE = 28672
# # EEP_START = 0x08007000
# # CLKSRC = HSE_8M



# LIBRARIES = Dxl MotorController DxlMotor SPI IMU Eigen OpenLog Robot
# # LIBRARIES = MotorController SPI IMU Eigen OpenLog Robot

# include $(KODUINO_DIR)/project.mk
