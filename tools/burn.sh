if [ $1'x' == 'x' ];then
	echo no filename 
	exit 1
fi
sudo ./tools/stm32flash -w $1 -v -g 0x0 /dev/ttyUSB0
