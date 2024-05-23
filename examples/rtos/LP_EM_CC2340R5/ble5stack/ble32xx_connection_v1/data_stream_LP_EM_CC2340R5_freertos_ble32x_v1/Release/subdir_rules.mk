################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-544373231: ../data_stream.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/TI/sysconfig_1.18.1/sysconfig_cli.bat" -s "C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/.metadata/product.json" --script "C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/data_stream.syscfg" -o "syscfg" --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_ble_config.h: build-544373231 ../data_stream.syscfg
syscfg/ti_ble_config.c: build-544373231
syscfg/ti_devices_config.c: build-544373231
syscfg/ti_radio_config.c: build-544373231
syscfg/ti_radio_config.h: build-544373231
syscfg/ti_drivers_config.c: build-544373231
syscfg/ti_drivers_config.h: build-544373231
syscfg/ti_utils_build_linker.cmd.genlibs: build-544373231
syscfg/ti_utils_build_compiler.opt: build-544373231
syscfg/syscfg_c.rov.xs: build-544373231
syscfg/FreeRTOSConfig.h: build-544373231
syscfg/ti_freertos_config.c: build-544373231
syscfg/ti_freertos_portable_config.c: build-544373231
syscfg/: build-544373231

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/TI/ccs1240/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c @"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/build_components.opt" @"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/factory_config.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/Release" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/app" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/controller/cc26xx/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/rom" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/target/_common" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx/npi/stack" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/heapmgr" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/dev_info" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/simple_profile" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/npi/src" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/osal/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/saddr" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/sdata" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/nv" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/cc26xx" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/profiles/health_thermometer" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/services/health_thermometer" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/drivers/rcl" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/posix/ticlang" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/include" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/portable/GCC/ARM_CM0" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/kernel/freertos" -DICALL_NO_APP_EVENTS -DCC23X0 -DNVOCMP_NWSAMEITEM=1 -DFLASH_ONLY_BUILD -DUSE_RCL -DFREERTOS -DNVOCMP_POSIX_MUTEX -gdwarf-3 -ffunction-sections -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/Release/syscfg" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/TI/ccs1240/ccs/tools/compiler/ti-cgt-armllvm_2.1.3.LTS/bin/tiarmclang.exe" -c @"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/build_components.opt" @"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/config/factory_config.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -Oz -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/Release" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/app" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/controller/cc26xx/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/rom" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/target/_common" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/common/cc26xx/npi/stack" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/hal/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/heapmgr" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/dev_info" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/profiles/simple_profile" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/npi/src" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/osal/src/inc" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/saddr" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/services/src/sdata" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/nv" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/common/cc26xx" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/ble5stack_flash/icall/src" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/profiles/health_thermometer" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/bleapp/services/health_thermometer" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/drivers/rcl" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/ti/posix/ticlang" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/include" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/source/third_party/freertos/portable/GCC/ARM_CM0" -I"C:/TI/simplelink_lowpower_f3_sdk_7_40_00_64/kernel/freertos" -DICALL_NO_APP_EVENTS -DCC23X0 -DNVOCMP_NWSAMEITEM=1 -DFLASH_ONLY_BUILD -DUSE_RCL -DFREERTOS -DNVOCMP_POSIX_MUTEX -gdwarf-3 -ffunction-sections -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Projects/ccs12.4/cc2340r5_7_20/data_stream_LP_EM_CC2340R5_freertos_ble32x_v1/Release/syscfg" -std=c99 $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


