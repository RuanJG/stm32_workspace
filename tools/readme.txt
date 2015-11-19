#=============================== stm32flash: 
#####Get device information:
./stm32flash /dev/ttyS0
####Write with verify and then start execution:
./stm32flash -w filename -v -g 0x0 /dev/ttyS0
####Read flash to file:
./stm32flash -r filename /dev/ttyS0
Start execution:
####./stm32flash -g 0x0 /dev/ttyS0


#============= build.sh
cd stm32_workspace dir , run ./tools/build.sh , it will make the program

#============= env_build.sh
cd stm32_workspace dir , run ./tools/env_build.sh, it will build the libopencm3 , which is need for build program
