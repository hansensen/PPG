#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = gnu.targets.arm.M3{1,0,4.8,4
#
ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g.dep
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g: package/package_ti.drivers.ports.tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.sm3g: package/package_ti.drivers.ports.tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/ClockP_tirtos.om3g.dep
package/lib/lib/tirtosport/ClockP_tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/ClockP_tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/ClockP_tirtos.om3g: ClockP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/ClockP_tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/ClockP_tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/ClockP_tirtos.sm3g: ClockP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/ClockP_tirtos.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/DebugP_tirtos.om3g.dep
package/lib/lib/tirtosport/DebugP_tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/DebugP_tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/DebugP_tirtos.om3g: DebugP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/DebugP_tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/DebugP_tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/DebugP_tirtos.sm3g: DebugP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/DebugP_tirtos.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/HwiP_tirtos.om3g.dep
package/lib/lib/tirtosport/HwiP_tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/HwiP_tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/HwiP_tirtos.om3g: HwiP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/HwiP_tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/HwiP_tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/HwiP_tirtos.sm3g: HwiP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/HwiP_tirtos.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/../ListP.om3g.dep
package/lib/lib/tirtosport/../ListP.om3g.dep: ;
endif

package/lib/lib/tirtosport/../ListP.om3g: | .interfaces
package/lib/lib/tirtosport/../ListP.om3g: ../ListP.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/../ListP.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/../ListP.sm3g: | .interfaces
package/lib/lib/tirtosport/../ListP.sm3g: ../ListP.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/../ListP.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/MutexP_tirtos.om3g.dep
package/lib/lib/tirtosport/MutexP_tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/MutexP_tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/MutexP_tirtos.om3g: MutexP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/MutexP_tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/MutexP_tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/MutexP_tirtos.sm3g: MutexP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/MutexP_tirtos.sm3g: export LD_LIBRARY_PATH=

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g.dep
package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g.dep: ;
endif

package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g: | .interfaces
package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g: SemaphoreP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c  -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS  -I/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm//libs/install-native/arm-none-eabi/include   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g: export LD_LIBRARY_PATH=

package/lib/lib/tirtosport/SemaphoreP_tirtos.sm3g: | .interfaces
package/lib/lib/tirtosport/SemaphoreP_tirtos.sm3g: SemaphoreP_tirtos.c lib/tirtosport.am3g.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clm3g -S $< ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-gcc -c -MD -MF $@.dep -x c -S -Wunused -Wunknown-pragmas -ffunction-sections -fdata-sections  -mcpu=cortex-m3 -mthumb -mabi=aapcs -g -Dfar= -D__DYNAMIC_REENT__  -g -D gcc  -Dxdc_target_name__=M3 -Dxdc_target_types__=gnu/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_4_8_4  -O2  -DTIRTOS   $(XDCINCS)  -o $@ $<
	-@$(FIXDEP) $@.dep $@.dep
	
package/lib/lib/tirtosport/SemaphoreP_tirtos.sm3g: export LD_LIBRARY_PATH=

clean,m3g ::
	-$(RM) package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/ClockP_tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/DebugP_tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/HwiP_tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/../ListP.om3g
	-$(RM) package/lib/lib/tirtosport/MutexP_tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g
	-$(RM) package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.sm3g
	-$(RM) package/lib/lib/tirtosport/ClockP_tirtos.sm3g
	-$(RM) package/lib/lib/tirtosport/DebugP_tirtos.sm3g
	-$(RM) package/lib/lib/tirtosport/HwiP_tirtos.sm3g
	-$(RM) package/lib/lib/tirtosport/../ListP.sm3g
	-$(RM) package/lib/lib/tirtosport/MutexP_tirtos.sm3g
	-$(RM) package/lib/lib/tirtosport/SemaphoreP_tirtos.sm3g

lib/tirtosport.am3g: package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.om3g package/lib/lib/tirtosport/ClockP_tirtos.om3g package/lib/lib/tirtosport/DebugP_tirtos.om3g package/lib/lib/tirtosport/HwiP_tirtos.om3g package/lib/lib/tirtosport/../ListP.om3g package/lib/lib/tirtosport/MutexP_tirtos.om3g package/lib/lib/tirtosport/SemaphoreP_tirtos.om3g lib/tirtosport.am3g.mak

clean::
	-$(RM) lib/tirtosport.am3g.mak
