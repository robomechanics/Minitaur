
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
