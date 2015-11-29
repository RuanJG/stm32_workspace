make -C ./libopencm3 clean
make -C ./libopencm3
if [ -d ./tools/stm32flash ];then
	echo "Start build stm32flash for uart burn "
	make -C ./tools/stm32flash
fi
if [ -d ./tools/stlink ]; then
	echo "Start build stlink for stlink burn "
	cd ./tools/stlink 
	./autogen.sh
	./configure
	make
	cd -
fi
