rm -r build
mkdir -p build
cd build
cmake ../src/ && make
cp liboffline_record_lib.so ../../../lib/x64/
