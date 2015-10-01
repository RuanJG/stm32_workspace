app=$(find ./ -name *.hex)
if [ $1'x' != 'x' ];then
	app=$1
fi
if [ $app'x' == 'x' ];then
	echo no *.hex find or input 
	exit 1
fi
sudo ./tools/stm32flash -w $app -v -g 0x0 /dev/ttyUSB0
