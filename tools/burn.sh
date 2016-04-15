#tool='uart'
tool='stlink'
if [ $1'x' != 'x' ];then
	tool=$1
fi
if [ $tool'x' == 'uartx' ]; then
	app=$(find ./ -name *.hex)
	if [ $app'x' == 'x' ];then
		echo no *.hex find or input, run: make hex 
		exit 1
	fi
	sudo ./tools/stm32flash/stm32flash -w $app -v -g 0x0 /dev/ttyUSB0
fi
if [ $tool'x' == 'stlinkx' ]; then
	app=$(find ./ -name *.bin)
	if [ $app'x' == 'x' ];then
		echo no *.bin find or input, run: make bin 
		exit 1
	fi
	sudo ./tools/stlink/st-flash write $app 0x8000000
fi

