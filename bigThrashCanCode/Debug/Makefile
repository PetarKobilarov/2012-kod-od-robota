################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL := cmd.exe
RM := rm -rf

USER_OBJS :=

LIBS := 
PROJ := 

O_SRCS := 
C_SRCS := 
S_SRCS := 
S_UPPER_SRCS := 
OBJ_SRCS := 
ASM_SRCS := 
PREPROCESSING_SRCS := 
OBJS := 
OBJS_AS_ARGS := 
C_DEPS := 
C_DEPS_AS_ARGS := 
EXECUTABLES := 
OUTPUT_FILE_PATH :=
OUTPUT_FILE_PATH_AS_ARGS :=
AVR_APP_PATH :=$$$AVR_APP_PATH$$$
QUOTE := "
ADDITIONAL_DEPENDENCIES:=
OUTPUT_FILE_DEP:=

# Every subdirectory with source files must be described here
SUBDIRS := 


# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS +=  \
../adc.c \
../bigThrashCanCode.c \
../funkcije.c \
../kretanje.c \
../lcd.c \
../odometry.c \
../system.c \
../timers.c \
../usart.c


PREPROCESSING_SRCS += 


ASM_SRCS += 


OBJS +=  \
adc.o \
bigThrashCanCode.o \
funkcije.o \
kretanje.o \
lcd.o \
odometry.o \
system.o \
timers.o \
usart.o


OBJS_AS_ARGS +=  \
adc.o \
bigThrashCanCode.o \
funkcije.o \
kretanje.o \
lcd.o \
odometry.o \
system.o \
timers.o \
usart.o


C_DEPS +=  \
adc.d \
bigThrashCanCode.d \
funkcije.d \
kretanje.d \
lcd.d \
odometry.d \
system.d \
timers.d \
usart.d


C_DEPS_AS_ARGS +=  \
adc.d \
bigThrashCanCode.d \
funkcije.d \
kretanje.d \
lcd.d \
odometry.d \
system.d \
timers.d \
usart.d


OUTPUT_FILE_PATH +=bigThrashCanCode.elf

OUTPUT_FILE_PATH_AS_ARGS +=bigThrashCanCode.elf

ADDITIONAL_DEPENDENCIES:=

OUTPUT_FILE_DEP:= ./makedep.mk

# AVR32/GNU C Compiler



















./%.o: .././%.c
	@echo Building file: $<
	@echo Invoking: AVR/GNU C Compiler : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -funsigned-char -funsigned-bitfields -O0 -fpack-struct -fshort-enums -g2 -Wall -pedantic-errors -c -std=gnu99 -MD -MP -MF "$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -MT"$(@:%.o=%.o)"  -mmcu=atmega128   -o"$@" "$<" 
	@echo Finished building: $<



# AVR32/GNU Preprocessing Assembler



# AVR32/GNU Assembler




ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: $(OUTPUT_FILE_PATH) $(ADDITIONAL_DEPENDENCIES)

$(OUTPUT_FILE_PATH): $(OBJS) $(USER_OBJS) $(OUTPUT_FILE_DEP)
	@echo Building target: $@
	@echo Invoking: AVR/GNU Linker : (WinAVR 20100110) 4.3.3
	$(QUOTE)C:\WinAVR-20100110\bin\avr-gcc.exe$(QUOTE) -o$(OUTPUT_FILE_PATH_AS_ARGS) $(OBJS_AS_ARGS) $(USER_OBJS) $(LIBS) -Wl,-Map="bigThrashCanCode.map" -Wl,--start-group -Wl,-lm  -Wl,--end-group  -mmcu=atmega128  
	@echo Finished building target: $@
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O ihex -R .eeprom -R .fuse -R .lock -R .signature  "bigThrashCanCode.elf" "bigThrashCanCode.hex"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -j .eeprom  --set-section-flags=.eeprom=alloc,load --change-section-lma .eeprom=0  --no-change-warnings -O ihex "bigThrashCanCode.elf" "bigThrashCanCode.eep" || exit 0
	"C:\WinAVR-20100110\bin\avr-objdump.exe" -h -S "bigThrashCanCode.elf" > "bigThrashCanCode.lss"
	"C:\WinAVR-20100110\bin\avr-objcopy.exe" -O srec -R .eeprom -R .fuse -R .lock -R .signature  "bigThrashCanCode.elf" "bigThrashCanCode.srec"
	"C:\WinAVR-20100110\bin\avr-size.exe" "bigThrashCanCode.elf"
	





# Other Targets
clean:
	-$(RM) $(OBJS_AS_ARGS)$(C_DEPS_AS_ARGS) $(EXECUTABLES) 
	rm -rf "bigThrashCanCode.elf" "bigThrashCanCode.a" "bigThrashCanCode.hex" "bigThrashCanCode.lss" "bigThrashCanCode.eep" "bigThrashCanCode.map" "bigThrashCanCode.srec"
	