LDFLAGS=-ltomlplusplus

# used for physical demos based on udp
raspi:
	c++ raspi.cpp main.cpp parsing.cpp ../thirdparty/Vector3/Vector3.cpp ../thirdparty/Vector3/Quaternion.cpp  -I ../thirdparty/Vector3/ ${LDFLAGS} -o raspi

# used for the frontend and cluster live data
simulated:
	c++ simulated.cpp main.cpp parsing.cpp ../thirdparty/Vector3/Vector3.cpp ../thirdparty/Vector3/Quaternion.cpp -I . -I ../thirdparty/Vector3/ ${LDFLAGS} -o simulated


run-live: simulated
	./simulated ./live1.toml &
	./simulated ./live2.toml &
	./simulated ./live3.toml &
	./simulated ./live4.toml | python3 main.py 1

stop:
	killall simulated