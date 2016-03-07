
all:
	~/Ac6/SystemWorkbench/eclipse  -nosplash --launcher.suppressErrors -application org.eclipse.cdt.managedbuilder.core.headlessbuild -build 'nucleo-f746/Debug'

clean:
	rm -rf ./SW4STM32/nucleo-f746/Debug/


