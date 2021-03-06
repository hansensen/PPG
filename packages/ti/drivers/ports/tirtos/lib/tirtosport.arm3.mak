#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#
#  target compatibility key = iar.targets.arm.M3{1,0,7.40,2
#
ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3.dep
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3: package/package_ti.drivers.ports.tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.srm3: package/package_ti.drivers.ports.tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/ClockP_tirtos.orm3.dep
package/lib/lib/tirtosport/ClockP_tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/ClockP_tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/ClockP_tirtos.orm3: ClockP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/ClockP_tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/ClockP_tirtos.srm3: ClockP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/DebugP_tirtos.orm3.dep
package/lib/lib/tirtosport/DebugP_tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/DebugP_tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/DebugP_tirtos.orm3: DebugP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/DebugP_tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/DebugP_tirtos.srm3: DebugP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/HwiP_tirtos.orm3.dep
package/lib/lib/tirtosport/HwiP_tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/HwiP_tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/HwiP_tirtos.orm3: HwiP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/HwiP_tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/HwiP_tirtos.srm3: HwiP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/../ListP.orm3.dep
package/lib/lib/tirtosport/../ListP.orm3.dep: ;
endif

package/lib/lib/tirtosport/../ListP.orm3: | .interfaces
package/lib/lib/tirtosport/../ListP.orm3: ../ListP.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/../ListP.srm3: | .interfaces
package/lib/lib/tirtosport/../ListP.srm3: ../ListP.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/MutexP_tirtos.orm3.dep
package/lib/lib/tirtosport/MutexP_tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/MutexP_tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/MutexP_tirtos.orm3: MutexP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/MutexP_tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/MutexP_tirtos.srm3: MutexP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

ifeq (,$(MK_NOGENDEPS))
-include package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3.dep
package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3.dep: ;
endif

package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3: | .interfaces
package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3: SemaphoreP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

package/lib/lib/tirtosport/SemaphoreP_tirtos.srm3: | .interfaces
package/lib/lib/tirtosport/SemaphoreP_tirtos.srm3: SemaphoreP_tirtos.c lib/tirtosport.arm3.mak
	@$(RM) $@.dep
	$(RM) $@
	@$(MSG) clrm3 $< ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iccarm  --silent --aeabi --cpu=Cortex-M3 --diag_suppress=Pa050,Go005 --endian=little -e --thumb  -Dewarm -DIAR -Dxdc_target_name__=M3 -Dxdc_target_types__=iar/targets/arm/std.h -Dxdc_bld__profile_release -Dxdc_bld__vers_1_0_7_40_2 -Ohs --dlib_config $(iar.targets.arm.M3.rootDir)/inc/c/DLib_Config_Normal.h -DTIRTOS   $(XDCINCS)  -o $@  $<
	
	-@$(FIXDEP) $@.dep $@.dep
	

clean,rm3 ::
	-$(RM) package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/ClockP_tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/DebugP_tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/HwiP_tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/../ListP.orm3
	-$(RM) package/lib/lib/tirtosport/MutexP_tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3
	-$(RM) package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.srm3
	-$(RM) package/lib/lib/tirtosport/ClockP_tirtos.srm3
	-$(RM) package/lib/lib/tirtosport/DebugP_tirtos.srm3
	-$(RM) package/lib/lib/tirtosport/HwiP_tirtos.srm3
	-$(RM) package/lib/lib/tirtosport/../ListP.srm3
	-$(RM) package/lib/lib/tirtosport/MutexP_tirtos.srm3
	-$(RM) package/lib/lib/tirtosport/SemaphoreP_tirtos.srm3

lib/tirtosport.arm3: package/lib/lib/tirtosport/package/package_ti.drivers.ports.tirtos.orm3 package/lib/lib/tirtosport/ClockP_tirtos.orm3 package/lib/lib/tirtosport/DebugP_tirtos.orm3 package/lib/lib/tirtosport/HwiP_tirtos.orm3 package/lib/lib/tirtosport/../ListP.orm3 package/lib/lib/tirtosport/MutexP_tirtos.orm3 package/lib/lib/tirtosport/SemaphoreP_tirtos.orm3 lib/tirtosport.arm3.mak

clean::
	-$(RM) lib/tirtosport.arm3.mak
