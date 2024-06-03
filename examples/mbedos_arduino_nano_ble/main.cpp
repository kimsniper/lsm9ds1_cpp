#include "lsm9ds1.h"
#include "mbed.h"

int main() {

    std::unique_ptr<Lsm9ds1> dev = std::make_unique<Lsm9ds1>();

    Lsm9ds1::Lsm9ds1_AccelData_t AccelData;
    Lsm9ds1::Lsm9ds1_GyroData_t GyroData;

    while (1){

        dev->Lsm9ds1_GetAccelData(AccelData);
        dev->Lsm9ds1_GetGyroData(GyroData);

        printf("Accelerometer X: %.02f\n", AccelData.Accel_X);
        printf("Accelerometer Y: %.02f\n", AccelData.Accel_Y);
        printf("Accelerometer Z: %.02f\n", AccelData.Accel_Z);

        printf("Gyro Roll: %.02f\n", GyroData.Gyro_X);
        printf("Gyro Pitch: %.02f\n", GyroData.Gyro_Y);
        printf("Gyro Yaw: %.02f\n", GyroData.Gyro_Z);

        ThisThread::sleep_for(1s);
    }

    return 0;
}