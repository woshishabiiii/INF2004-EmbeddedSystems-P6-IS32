################################################################################
# micro T-Kernel 3.00.03  makefile
################################################################################

OBJS += \
./mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.o \
./mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.o 

C_DEPS += \
./mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/int_armv6m.d \
./mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/wusec_armv6m.d 


mtkernel_3/lib/libtk/sysdepend/cpu/core/armv6m/%.o: ../lib/libtk/sysdepend/cpu/core/armv6m/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
