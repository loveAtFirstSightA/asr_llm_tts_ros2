#编译64位可执行文件
make clean;make LINUX_ARM64=1
#设置libmsc.so库搜索路径
export LD_LIBRARY_PATH=$(pwd)/../../lib/arm64/
