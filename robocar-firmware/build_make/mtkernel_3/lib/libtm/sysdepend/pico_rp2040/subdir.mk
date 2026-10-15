################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

OBJS += \
./mtkernel_3/lib/libtm/sysdepend/pico_rp2040/tm_com.o 

C_DEPS += \
./mtkernel_3/lib/libtm/sysdepend/pico_rp2040/tm_com.d 

mtkernel_3/lib/libtm/sysdepend/pico_rp2040/%.o: ../lib/libtm/sysdepend/pico_rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
