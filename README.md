# bmp180-driver-raspberry
This is driver support for module sensor BMP180 by BOSCH
- Provide by:
-   Vũ Trung Tâm (22146396)
-   Nguyễn Đình Thao (22146400)
-   Nguyễn Đoàn Anh Khoa (22146333)
  
# SENSOR BMP180
- The BMP180 consists of a piezo-resistive sensor, an analog to digital converter and a control unit with E2PROM and a serial I2C interface. The BMP180 delivers the uncompensated value of pressure and temperature. The E2PROM has stored 176 bit of individual calibration data. This is used to compensate offset, temperature dependence and other parameters of the sensor.
- Pin out:

![image](https://github.com/user-attachments/assets/02255fe9-8a92-47ed-9282-6dfc8ad5c694)

- Measurement flow diagram:

![image](https://github.com/user-attachments/assets/b5651012-618b-41b5-b32f-107d549b0659)

- BMP180 have different mode for pressure measurement, from 0 to 3. Mode 0 have the least wait time but affeted by more noise. On the other hand, mode 3 have more wait time but high acccuracy. We will include this mode for flow diagram of code.

![image](https://github.com/user-attachments/assets/c3aad8ce-191d-47c1-acdb-30f12a062a87)

- Before we start calculating pressure and temperature, we need to calibrate them. Each sensor have different calibration coefficient (Provide by manufactory), so we first read these coefficient in register. Here is the address map of calibration register:

![image](https://github.com/user-attachments/assets/fdb0d954-f26f-430c-bfa3-8d2903624461)

- After we get the calibration coefficient, we meassure and calculate the pressure and temperature. The flow code diagram is provided below.

![image](https://github.com/user-attachments/assets/e614deaf-bc08-4ae6-baf6-28c10925e72a)

# SET UP ENVIRONMENT
- The project using Raspberry pi 3 with Ubuntu version 22.04 and sensor BMP180. Before download the code, make sure that you have enable I2C in sudo raspi-config -> Interface Options. You can check if success by enter command below, if you see bmp180 address (0x77), you are successfully connect to it.

'''bash
i2cdetect -y 1
'''

- After that, check if you have downloaded linux-kernel-headers or raspberry-kernel-headers to import useful library in kernel space. You can check by cd to this path, if you see a folder name "build", this mean you have downloaded it.

'''bash
cd /lib/modules/$(uname -r)
'''

- In case you are not downloaded it, run these command. Try to reboot the os.

'''bash
sudo apt update
sudo apt upgrade
sudo apt install raspberrypi-kernel-headers
'''

- Git the code, after that create a Makefile in the same folder. Paste these code to it:

'''bash
obj-m += bmp180.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
'''

- Run the command below to build the makefile:

'''bash
make
'''

- The following command use to load the bmp180 driver into linux kernel:

'''bash
sudo insmod bmp180_driver.ko
dmesg | tail
'''

- After that, you can try to run it with your code
