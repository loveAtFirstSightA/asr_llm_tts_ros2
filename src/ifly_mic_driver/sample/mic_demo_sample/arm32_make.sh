#编译32位可执行文件raspberrypi
make clean;make LINUX_ARM32=1
#设置libmsc.so库搜索路径
export LD_LIBRARY_PATH=$(pwd)/../../lib/arm32/
