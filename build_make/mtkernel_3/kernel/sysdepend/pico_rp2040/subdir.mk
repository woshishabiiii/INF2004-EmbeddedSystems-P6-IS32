################################################################################
# micro T-Kernel 3.0 BSP makefile
################################################################################

OBJS += \
./mtkernel_3/kernel/sysdepend/pico_rp2040/cpu_clock.o \
./mtkernel_3/kernel/sysdepend/pico_rp2040/devinit.o \
./mtkernel_3/kernel/sysdepend/pico_rp2040/hw_setting.o \
./mtkernel_3/kernel/sysdepend/pico_rp2040/power_save.o \

C_DEPS += \
./mtkernel_3/kernel/sysdepend/pico_rp2040/cpu_clock.d \
./mtkernel_3/kernel/sysdepend/pico_rp2040/devinit.d \
./mtkernel_3/kernel/sysdepend/pico_rp2040/hw_setting.d \
./mtkernel_3/kernel/sysdepend/pico_rp2040/power_save.d \

mtkernel_3/kernel/sysdepend/pico_rp2040/%.o: ../kernel/sysdepend/pico_rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
