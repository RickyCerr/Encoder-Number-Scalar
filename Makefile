PRG            =lab4


# You should not have to change anything below here, but go ahead and
# try to understand what is going on.

OBJ                  = $(PRG).o  
MCU_TARGET           = atxmega16a4u
MCU_PROGRAM_TARGET   = x16a4 
#OPTIMIZE            = -O2    # options are 1, 2, 3, s
OPTIMIZE             = -Os    # options are 1, 2, 3, s
DEFS                 =
LIBS                 =
CC                   = avr-gcc

F_CPU  = 2000000UL

########################3
## Options common to compile, link and assembly rules
COMMON = -mmcu=$(MCU_TARGET)

# Compile options common for all C compilation units.
CFLAGS = $(COMMON)
CFLAGS += -Wall -gdwarf-2 -std=gnu99  -DF_CPU=2000000UL -Os -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

# Linker flags
LDFLAGS = $(COMMON)
LDFLAGS +=  -Wl,-Map=XMega16A4.map

########################3

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf lst text eeprom

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean: 
	rm -rf *.o $(PRG).elf *.bin *.hex *.eps *.png *.pdf *.bak  *.lst *.elf *.srec
	rm -rf *.lst *.map $(EXTRA_CLEAN_FILES) *~

program: $(PRG).hex
	avrdude -v -p $(MCU_PROGRAM_TARGET) -P usb -c avrisp2 -e -U flash:w:$(PRG).hex -U eeprom:w:$(PRG)_eeprom.hex

lst:  $(PRG).lst

%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

# Rules for building the .text rom images

text: hex bin srec

hex:  $(PRG).hex
bin:  $(PRG).bin
srec: $(PRG).srec

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@ 
%.srec: %.elf
	$(OBJCOPY) -j .text -j .data -O srec $< $@

%.bin: %.elf
	$(OBJCOPY) -j .text -j .data -O binary $< $@

# Rules for building the .eeprom rom images

eeprom: ehex ebin esrec

ehex:  $(PRG)_eeprom.hex
ebin:  $(PRG)_eeprom.bin
esrec: $(PRG)_eeprom.srec

%_eeprom.hex: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O ihex $< $@

%_eeprom.srec: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O srec $< $@

%_eeprom.bin: %.elf
	$(OBJCOPY) -j .eeprom --change-section-lma .eeprom=0 -O binary $< $@

