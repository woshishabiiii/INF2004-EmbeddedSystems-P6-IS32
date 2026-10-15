################################################################################
# micro T-Kernel 3.0 BSP makefile
################################################################################

OBJS += \
./mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.o \
./mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.o 

C_DEPS += \
./mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/int_rp2040.d \
./mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/ptimer_rp2040.d 

mtkernel_3/lib/libtk/sysdepend/cpu/rp2040/%.o: ../lib/libtk/sysdepend/cpu/rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
