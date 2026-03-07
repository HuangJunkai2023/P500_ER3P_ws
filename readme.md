sdk download： http://sw.rokae.com:8800/?dir=xcore/SDK/v3.1

cd xCoreSDK_cpp-v0.7.1
cmake -S . -B build -DXCORE_USE_XMATE_MODEL=OFF
cmake --build build --target arm_bridge -j4

conda activate tidybot2
python arm_server.py