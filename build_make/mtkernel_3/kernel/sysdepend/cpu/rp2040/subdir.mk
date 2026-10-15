################################################################################
# micro T-Kernel 3.0 BSP makefile
################################################################################

OBJS += \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/boot_2nd.o \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/vector_tbl.o \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/smp_rp2040.o \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/smp_atomic.o 

C_DEPS += \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/boot_2nd.d \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/vector_tbl.d \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/smp_rp2040.d \
./mtkernel_3/kernel/sysdepend/cpu/rp2040/smp_atomic.d 

mtkernel_3/kernel/sysdepend/cpu/rp2040/%.o: ../kernel/sysdepend/cpu/rp2040/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
