# invoke SourceDir generated makefile for appBLE.prm3
appBLE.prm3: .libraries,appBLE.prm3
.libraries,appBLE.prm3: package/cfg/appBLE_prm3.xdl
	$(MAKE) -f D:\PPG\Implementation\ble_cc26xx_2_01_01_44627_0925\Projects\ble\SimpleBLEPeripheral\CC26xx\IAR\Config/src/makefile.libs

clean::
	$(MAKE) -f D:\PPG\Implementation\ble_cc26xx_2_01_01_44627_0925\Projects\ble\SimpleBLEPeripheral\CC26xx\IAR\Config/src/makefile.libs clean

