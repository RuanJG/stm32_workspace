#=============================== stm32flash: 
#####Get device information:
./stm32flash /dev/ttyS0
####Write with verify and then start execution:
./stm32flash -w filename -v -g 0x0 /dev/ttyS0
####Read flash to file:
./stm32flash -r filename /dev/ttyS0
Start execution:
####./stm32flash -g 0x0 /dev/ttyS0
