################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

OBJS += \
./mtkernel_3/device/adc/sysdepend/rp2040/adc_rp2040.o 

C_DEPS += \
./mtkernel_3/device/adc/sysdepend/rp2040/adc_rp2040.d 


mtkernel_3/device/adc/sysdepend/rp2040/%.o: ../device/adc/sysdepend/rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
