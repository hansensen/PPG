#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#

unexport MAKEFILE_LIST
MK_NOGENDEPS := $(filter clean,$(MAKECMDGOALS))
override PKGDIR = ti/tirtos/utils
XDCINCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(XPKGPATH))))
XDCCFGDIR = package/cfg/

#
# The following dependencies ensure package.mak is rebuilt
# in the event that some included BOM script changes.
#
ifneq (clean,$(MAKECMDGOALS))
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/utils.js:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/utils.js
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xdc.tci:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xdc.tci
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/template.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/template.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/om2.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/om2.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xmlgen.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xmlgen.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xmlgen2.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/xmlgen2.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/Warnings.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/Warnings.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/IPackage.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/IPackage.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/package.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/package.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/services/global/Clock.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/services/global/Clock.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/services/global/Trace.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/services/global/Trace.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/bld.js:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/bld.js
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/BuildEnvironment.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/BuildEnvironment.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/PackageContents.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/PackageContents.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/_gen.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/_gen.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Library.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Library.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Executable.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Executable.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Repository.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Repository.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Configuration.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Configuration.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Script.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Script.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Manifest.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Manifest.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Utils.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/Utils.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget2.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget2.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget3.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITarget3.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITargetFilter.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/ITargetFilter.xs
/db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/package.xs:
package.mak: /db/rtree/library/trees/xdcprod/xdcprod-u33/product/xdccore/Linux/xdctools_3_31_01_33_core/packages/xdc/bld/package.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/tirtos.bld:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/tirtos.bld
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/ITarget.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/ITarget.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/C28_large.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/C28_large.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/C28_float.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/C28_float.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/package.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/package.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/arm/elf/IArm.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/arm/elf/IArm.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/arm/elf/package.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/arm/elf/package.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/iar/targets/arm/ITarget.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/iar/targets/arm/ITarget.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/iar/targets/arm/package.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/iar/targets/arm/package.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/ITarget.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/ITarget.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/_utils.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/_utils.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/Linux86.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/Linux86.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/Mingw.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/Mingw.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/package.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/package.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/ITarget.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/ITarget.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/GCArmv6.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/GCArmv6.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/GCArmv7A.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/GCArmv7A.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/IM.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/IM.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M3.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M3.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M4.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M4.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M4F.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/M4F.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A8F.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A8F.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A9F.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A9F.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A15F.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/A15F.xs
/db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/package.xs:
package.mak: /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/package.xs
package.mak: package.bld
endif

ti.targets.arm.elf.M3.rootDir ?= /db/vendors/ti/arm/5.2.2/Linux
ti.targets.arm.elf.packageBase ?= /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/ti/targets/arm/elf/
ti.targets.arm.elf.M4.rootDir ?= /db/vendors/ti/arm/5.2.2/Linux
iar.targets.arm.M3.rootDir ?= /db/vendors/iar/arm/7.40.1a/Linux
iar.targets.arm.packageBase ?= /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/iar/targets/arm/
iar.targets.arm.M4.rootDir ?= /db/vendors/iar/arm/7.40.1a/Linux
gnu.targets.arm.M3.rootDir ?= /db/vendors/linaro/gcc-arm-none-eabi-4_8-2014q3
gnu.targets.arm.packageBase ?= /db/vtree/library/trees/zumaprod/zumaprod-g06/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages/gnu/targets/arm/
gnu.targets.arm.M4.rootDir ?= /db/vendors/linaro/gcc-arm-none-eabi-4_8-2014q3
.PRECIOUS: $(XDCCFGDIR)/%.oem3
.PHONY: all,em3 .dlls,em3 .executables,em3 test,em3
all,em3: .executables,em3
.executables,em3: .libraries,em3
.executables,em3: .dlls,em3
.dlls,em3: .libraries,em3
.libraries,em3: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,em3
	@$(ECHO) xdc .executables,em3
	@$(ECHO) xdc .libraries,em3
	@$(ECHO) xdc .dlls,em3

.PRECIOUS: $(XDCCFGDIR)/%.oem4
.PHONY: all,em4 .dlls,em4 .executables,em4 test,em4
all,em4: .executables,em4
.executables,em4: .libraries,em4
.executables,em4: .dlls,em4
.dlls,em4: .libraries,em4
.libraries,em4: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,em4
	@$(ECHO) xdc .executables,em4
	@$(ECHO) xdc .libraries,em4
	@$(ECHO) xdc .dlls,em4

.PRECIOUS: $(XDCCFGDIR)/%.orm3
.PHONY: all,rm3 .dlls,rm3 .executables,rm3 test,rm3
all,rm3: .executables,rm3
.executables,rm3: .libraries,rm3
.executables,rm3: .dlls,rm3
.dlls,rm3: .libraries,rm3
.libraries,rm3: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,rm3
	@$(ECHO) xdc .executables,rm3
	@$(ECHO) xdc .libraries,rm3
	@$(ECHO) xdc .dlls,rm3

.PRECIOUS: $(XDCCFGDIR)/%.orm4
.PHONY: all,rm4 .dlls,rm4 .executables,rm4 test,rm4
all,rm4: .executables,rm4
.executables,rm4: .libraries,rm4
.executables,rm4: .dlls,rm4
.dlls,rm4: .libraries,rm4
.libraries,rm4: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,rm4
	@$(ECHO) xdc .executables,rm4
	@$(ECHO) xdc .libraries,rm4
	@$(ECHO) xdc .dlls,rm4

.PRECIOUS: $(XDCCFGDIR)/%.om3g
.PHONY: all,m3g .dlls,m3g .executables,m3g test,m3g
all,m3g: .executables,m3g
.executables,m3g: .libraries,m3g
.executables,m3g: .dlls,m3g
.dlls,m3g: .libraries,m3g
.libraries,m3g: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,m3g
	@$(ECHO) xdc .executables,m3g
	@$(ECHO) xdc .libraries,m3g
	@$(ECHO) xdc .dlls,m3g

.PRECIOUS: $(XDCCFGDIR)/%.om4g
.PHONY: all,m4g .dlls,m4g .executables,m4g test,m4g
all,m4g: .executables,m4g
.executables,m4g: .libraries,m4g
.executables,m4g: .dlls,m4g
.dlls,m4g: .libraries,m4g
.libraries,m4g: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,m4g
	@$(ECHO) xdc .executables,m4g
	@$(ECHO) xdc .libraries,m4g
	@$(ECHO) xdc .dlls,m4g


all: .executables 
.executables: .libraries .dlls
.libraries: .interfaces

PKGCFGS := $(wildcard package.xs) package/build.cfg
.interfaces: package/package.xdc.inc package/package.defs.h package.xdc $(PKGCFGS)

-include package/package.xdc.dep
package/%.xdc.inc package/%_ti.tirtos.utils.c package/%.defs.h: %.xdc $(PKGCFGS)
	@$(MSG) generating interfaces for package ti.tirtos.utils" (because $@ is older than $(firstword $?))" ...
	$(XSRUN) -f xdc/services/intern/cmd/build.xs $(MK_IDLOPTS) -m package/package.xdc.dep -i package/package.xdc.inc package.xdc

ifeq (,$(MK_NOGENDEPS))
-include package/package.cfg.dep
endif

package/package.cfg.xdc.inc: .interfaces $(XDCROOT)/packages/xdc/cfg/cfginc.js package.xdc
	@$(MSG) generating schema include file list ...
	$(CONFIG) -f $(XDCROOT)/packages/xdc/cfg/cfginc.js ti.tirtos.utils $@

.libraries,em3 .libraries: lib/release/ti.tirtos.utils.aem3

-include lib/release/ti.tirtos.utils.aem3.mak
lib/release/ti.tirtos.utils.aem3: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem3 package/lib/lib/release/ti.tirtos.utils/UARTMon.oem3  into $@ ...
	$(ti.targets.arm.elf.M3.rootDir)/bin/armar  rq $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem3 package/lib/lib/release/ti.tirtos.utils/UARTMon.oem3 
lib/release/ti.tirtos.utils.aem3: export C_DIR=
lib/release/ti.tirtos.utils.aem3: PATH:=$(ti.targets.arm.elf.M3.rootDir)/bin/:$(PATH)

clean,em3 clean::
	-$(RM) lib/release/ti.tirtos.utils.aem3
.libraries,em4 .libraries: lib/release/ti.tirtos.utils.aem4

-include lib/release/ti.tirtos.utils.aem4.mak
lib/release/ti.tirtos.utils.aem4: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem4 package/lib/lib/release/ti.tirtos.utils/UARTMon.oem4  into $@ ...
	$(ti.targets.arm.elf.M4.rootDir)/bin/armar  rq $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem4 package/lib/lib/release/ti.tirtos.utils/UARTMon.oem4 
lib/release/ti.tirtos.utils.aem4: export C_DIR=
lib/release/ti.tirtos.utils.aem4: PATH:=$(ti.targets.arm.elf.M4.rootDir)/bin/:$(PATH)

clean,em4 clean::
	-$(RM) lib/release/ti.tirtos.utils.aem4
.libraries,rm3 .libraries: lib/release/ti.tirtos.utils.arm3

-include lib/release/ti.tirtos.utils.arm3.mak
lib/release/ti.tirtos.utils.arm3: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm3 package/lib/lib/release/ti.tirtos.utils/UARTMon.orm3  into $@ ...
	LC_ALL=C $(iar.targets.arm.M3.rootDir)/bin/iarchive --silent -o $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm3 package/lib/lib/release/ti.tirtos.utils/UARTMon.orm3 

clean,rm3 clean::
	-$(RM) lib/release/ti.tirtos.utils.arm3
.libraries,rm4 .libraries: lib/release/ti.tirtos.utils.arm4

-include lib/release/ti.tirtos.utils.arm4.mak
lib/release/ti.tirtos.utils.arm4: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm4 package/lib/lib/release/ti.tirtos.utils/UARTMon.orm4  into $@ ...
	LC_ALL=C $(iar.targets.arm.M4.rootDir)/bin/iarchive --silent -o $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm4 package/lib/lib/release/ti.tirtos.utils/UARTMon.orm4 

clean,rm4 clean::
	-$(RM) lib/release/ti.tirtos.utils.arm4
.libraries,m3g .libraries: lib/release/ti.tirtos.utils.am3g

-include lib/release/ti.tirtos.utils.am3g.mak
lib/release/ti.tirtos.utils.am3g: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om3g package/lib/lib/release/ti.tirtos.utils/UARTMon.om3g  into $@ ...
	$(gnu.targets.arm.M3.rootDir)/bin/arm-none-eabi-ar  cr $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om3g package/lib/lib/release/ti.tirtos.utils/UARTMon.om3g 
lib/release/ti.tirtos.utils.am3g: export LD_LIBRARY_PATH=

clean,m3g clean::
	-$(RM) lib/release/ti.tirtos.utils.am3g
.libraries,m4g .libraries: lib/release/ti.tirtos.utils.am4g

-include lib/release/ti.tirtos.utils.am4g.mak
lib/release/ti.tirtos.utils.am4g: 
	$(RM) $@
	@$(MSG) archiving package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om4g package/lib/lib/release/ti.tirtos.utils/UARTMon.om4g  into $@ ...
	$(gnu.targets.arm.M4.rootDir)/bin/arm-none-eabi-ar  cr $@   package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om4g package/lib/lib/release/ti.tirtos.utils/UARTMon.om4g 
lib/release/ti.tirtos.utils.am4g: export LD_LIBRARY_PATH=

clean,m4g clean::
	-$(RM) lib/release/ti.tirtos.utils.am4g
test:;
%,copy:
	@$(if $<,,$(MSG) don\'t know how to build $*; exit 1)
	@$(MSG) cp $< $@
	$(RM) $@
	$(CP) $< $@
package_ti.tirtos.utils.oem3,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem3
package_ti.tirtos.utils.sem3,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.sem3
UARTMon.oem3,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.oem3
UARTMon.sem3,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.sem3
package_ti.tirtos.utils.oem4,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.oem4
package_ti.tirtos.utils.sem4,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.sem4
UARTMon.oem4,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.oem4
UARTMon.sem4,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.sem4
package_ti.tirtos.utils.orm3,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm3
package_ti.tirtos.utils.srm3,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.srm3
UARTMon.orm3,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.orm3
UARTMon.srm3,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.srm3
package_ti.tirtos.utils.orm4,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.orm4
package_ti.tirtos.utils.srm4,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.srm4
UARTMon.orm4,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.orm4
UARTMon.srm4,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.srm4
package_ti.tirtos.utils.om3g,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om3g
package_ti.tirtos.utils.sm3g,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.sm3g
UARTMon.om3g,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.om3g
UARTMon.sm3g,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.sm3g
package_ti.tirtos.utils.om4g,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.om4g
package_ti.tirtos.utils.sm4g,copy : package/lib/lib/release/ti.tirtos.utils/package/package_ti.tirtos.utils.sm4g
UARTMon.om4g,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.om4g
UARTMon.sm4g,copy : package/lib/lib/release/ti.tirtos.utils/UARTMon.sm4g

$(XDCCFGDIR)%.c $(XDCCFGDIR)%.h $(XDCCFGDIR)%.xdl: $(XDCCFGDIR)%.cfg $(XDCROOT)/packages/xdc/cfg/Main.xs | .interfaces
	@$(MSG) "configuring $(_PROG_NAME) from $< ..."
	$(CONFIG) $(_PROG_XSOPTS) xdc.cfg $(_PROG_NAME) $(XDCCFGDIR)$*.cfg $(XDCCFGDIR)$*

.PHONY: release,ti_tirtos_utils
ifeq (,$(MK_NOGENDEPS))
-include package/rel/ti_tirtos_utils.tar.dep
endif
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: package/package.bld.xml
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: package/build.cfg
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: package/package.xdc.inc
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: package/package.cfg.xdc.inc
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.aem3
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.aem4
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.arm3
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.arm4
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.am3g
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: lib/release/ti.tirtos.utils.am4g
package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml: .force
	@$(MSG) generating external release references $@ ...
	$(XS) $(JSENV) -f $(XDCROOT)/packages/xdc/bld/rel.js $(MK_RELOPTS) . $@

ti_tirtos_utils.tar: package/rel/ti_tirtos_utils.xdc.inc package/rel/ti_tirtos_utils/ti/tirtos/utils/package/package.rel.xml
	@$(MSG) making release file $@ "(because of $(firstword $?))" ...
	-$(RM) $@
	$(call MKRELTAR,package/rel/ti_tirtos_utils.xdc.inc,package/rel/ti_tirtos_utils.tar.dep)


release release,ti_tirtos_utils: all ti_tirtos_utils.tar
clean:: .clean
	-$(RM) ti_tirtos_utils.tar
	-$(RM) package/rel/ti_tirtos_utils.xdc.inc
	-$(RM) package/rel/ti_tirtos_utils.tar.dep

clean:: .clean
	-$(RM) .libraries $(wildcard .libraries,*)
clean:: 
	-$(RM) .dlls $(wildcard .dlls,*)
#
# The following clean rule removes user specified
# generated files or directories.
#
	-$(RMDIR) lib/

ifneq (clean,$(MAKECMDGOALS))
ifeq (,$(wildcard lib))
    $(shell $(MKDIR) lib)
endif
ifeq (,$(wildcard lib/release))
    $(shell $(MKDIR) lib/release)
endif
ifeq (,$(wildcard package))
    $(shell $(MKDIR) package)
endif
ifeq (,$(wildcard package/cfg))
    $(shell $(MKDIR) package/cfg)
endif
ifeq (,$(wildcard package/lib))
    $(shell $(MKDIR) package/lib)
endif
ifeq (,$(wildcard package/rel))
    $(shell $(MKDIR) package/rel)
endif
ifeq (,$(wildcard package/internal))
    $(shell $(MKDIR) package/internal)
endif
ifeq (,$(wildcard package/lib/lib))
    $(shell $(MKDIR) package/lib/lib)
endif
ifeq (,$(wildcard package/lib/lib/release))
    $(shell $(MKDIR) package/lib/lib/release)
endif
ifeq (,$(wildcard package/lib/lib/release/ti.tirtos.utils))
    $(shell $(MKDIR) package/lib/lib/release/ti.tirtos.utils)
endif
ifeq (,$(wildcard package/lib/lib/release/ti.tirtos.utils/package))
    $(shell $(MKDIR) package/lib/lib/release/ti.tirtos.utils/package)
endif
endif
clean::
	-$(RMDIR) package


