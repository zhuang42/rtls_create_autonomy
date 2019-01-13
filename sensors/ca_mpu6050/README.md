# MPU6050

Interfacing Raspberry Pi with MPU6050:

![Connection between RPi and MPU6050](media/wiring.png)

## Calibrate sensor

```sh
$ roslaunch ca_mpu6050 calibrate.launch
```

Copy the resulting values (in green) into `/config/calibrated.yaml`.
