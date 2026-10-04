## Introduction
This folder contains python scripts to plot what is sent by the e-puck2 (camera pixel's line in TP4 or the FFT in TP5). The bluetooth should work as well as the USB, except for the connection time but it depends on the bluetooth dongle/chip used by the computer.

Normally, if "VSCode for e-puck2 installer" has been used, python 3.11.2 and some packages are already installed and pyenv has been used to create the e-puck2 dedicated environment. However some packages are still missing to be able to run the python scripts provided within this practical. Depending on your OS, you must install them with these commands in a terminal located in your EPuck2_Workplace:

- Windows 10 & 11:
	```shell
	python -m pip install matplotlib msvc-runtime pyserial
	```
- MacOS:
	```shell
	python -m pip install matplotlib pyserial
	```
- Linux: 
	```shell
	python -m pip install matplotlib pyserial
	```

## Python script usage
For TP4, run the script plotImage.py in the terminal and give it the com port as argument:
```shell
python path_to_the_script/plotImage.py SerialMonitorDeviceName
```
For TP5, run the script plotFFT.py in the terminal and give it the com port as argument
```shell
python path_to_the_script/plotFFT.py SerialMonitorDeviceName
```

## Troubleshooting
If you get a lot of Timeout messages in the terminal while trying to receive something from the e-puck2, quit the program (Ctrl+C), unplug the USB cable (or disable the bluetooth if used instead of the USB), shutdown the epuck2 and retry. And above all make sure to use the right serial port.