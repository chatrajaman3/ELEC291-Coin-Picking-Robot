SHELL=cmd
CC=arm-none-eabi-gcc
AS=arm-none-eabi-as
LD=arm-none-eabi-ld
CCFLAGS=-mcpu=cortex-m0 -mthumb -g
LDFLAGS=-Os -u _printf_float -nostdlib -lnosys -lgcc -T ldscripts\stm32l051xx.ld --cref
# LDFLAGS=-Os -nostdlib -lnosys -lgcc -T ldscripts\stm32l051xx.ld --cref

GCCPATH=$(subst \bin\arm-none-eabi-gcc.exe,\,$(shell where $(CC)))
LIBPATH1=$(subst \libgcc.a,,$(shell dir /s /b "$(GCCPATH)*libgcc.a" | find "v6-m"))
LIBPATH2=$(subst \libc_nano.a,,$(shell dir /s /b "$(GCCPATH)*libc_nano.a" | find "v6-m"))
LIBSPEC=-L"$(LIBPATH1)" -L"$(LIBPATH2)"

OBJS=main.o util.o serial.o startup.o newlib_stubs.o

PORTN=$(shell type COMPORT.inc)

all: main.elf flash

main.elf : $(OBJS)
	$(LD) $(OBJS) $(LIBSPEC) $(LDFLAGS) -Map main.map -o main.elf
	arm-none-eabi-objcopy -O ihex main.elf main.hex

main.o: main.c
	$(CC) -c $(CCFLAGS) main.c -o main.o

util.o: util.c
	$(CC) -c $(CCFLAGS) util.c -o util.o

startup.o: lib/startup.c
	$(CC) -c $(CCFLAGS) -DUSE_USART1 lib/startup.c -o startup.o

serial.o: lib/serial.c
	$(CC) -c $(CCFLAGS) lib/serial.c -o serial.o

newlib_stubs.o: lib/newlib_stubs.c
	$(CC) -c $(CCFLAGS) lib/newlib_stubs.c -o newlib_stubs.o

clean:
	del $(OBJS) 2> NUL
	del main.elf main.hex main.map 2> NUL
	del *.lst 2> NUL

flash:
	@taskkill /f /im putty.exe /t /fi "status eq running" > NUL
	@echo stm32flash -w main.hex -v -g 0x0 ^^> sflash.bat
	@BO230 -b >> sflash.bat
	@sflash
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^> sputty.bat
	@BO230 -r >> sputty.bat
	@sputty

putty:
	@echo cmd /c start putty.exe -sercfg 115200,8,n,1,N -serial ^^> sputty.bat
	@BO230 -r >> sputty.bat
	@sputty
