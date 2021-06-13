
# Copyright (c) 2021 ozforester. All rights reserved.
# Use of this source code is goverened by a MIT license
# that can be found in the LICENSE file.

TARGET	 = avr_hvpp
SOURCES := $(wildcard *.c)
OBJECTS  = $(SOURCES:.S=.o)
OPT = -Os

all:
	avr-gcc -DF_CPU=4000000LL -c -Wall ${OPT} -mmcu=atmega8 -o ${TARGET}.o ${TARGET}.S
	avr-gcc -Wall -Os -mmcu=atmega8 -o ${TARGET} ${TARGET}.o
	avr-objcopy -R .eeprom -O ihex ${TARGET} ${TARGET}.hex
	avr-objcopy -j .eeprom --set-section-flags=.eeprom="alloc,load" --change-section-lma .eeprom=0 -O ihex ${TARGET} ${TARGET}.eep
	#avr-size --format=avr --mcu=atmega8 ${TARGET}.o
	avr-size ${TARGET}.o
	avr-size ${TARGET}.eep
flash:
	avrdude -c usbasp -p m8 -B 3 -U flash:w:${TARGET}.hex -U eeprom:w:${TARGET}.eep

clean:
	rm -f $(OBJECTS) $(TARGET) $($(TARGET).hex TARGET).eep $(TARGET).o
