################################################################################
# micro T-Kernel 3.0 BSP makefile
################################################################################

OBJS += \
./mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.o 

C_DEPS += \
./mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/libbsp_pwm.d  

mtkernel_3/lib/libbsp/sysdepend/cpu/rp2040/%.o: ../lib/libbsp/sysdepend/cpu/rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
