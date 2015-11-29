#============= build.sh
cd stm32_workspace dir , run ./tools/build.sh , it will make the program

#============= env_build.sh
cd stm32_workspace dir , run ./tools/env_build.sh, it will build the libopencm3 and orther tools , which is need for build program

#============= burn.sh
./tools/burn.sh [uart/stlink] #default is stlink

#============= debug by gdb
###you can see usage in Makefile
first terminal : ./tools/stlink/st-util 
second terminal : arm-none-eabi-gdb --batch -ex 'target extended-remote :4242' app.elf
or
second terminal : ddd -debugger arm-none-eabi-gdb app_main.elf, and then input 'target extended-remote :4242' in gdb cmd console




############################################################## tools build for private use
#=============================== stm32flash: 
##### Build stm32flash
#in stm32flash dir, as I have fill in env_build.sh 
make
#####Get device information:
./stm32flash /dev/ttyS0
####Write with verify and then start execution:
./stm32flash -w filename -v -g 0x0 /dev/ttyS0
####Read flash to file:
./stm32flash -r filename /dev/ttyS0
Start execution:
####./stm32flash -g 0x0 /dev/ttyS0

#============================== stlink
##### Build 
# in stlink dir, has fill in env_build
./autogen.sh
./configure
make


