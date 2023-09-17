#include "mbed.h"
#include "MPU6050.h"

I2C i2c(I2C_SDA, I2C_SCL);
MPU6050 imu(&i2c);


int main()
{
    printf("Starting...\n\r");

    i2c.frequency(400000); // 400 kHz

    uint8_t whoami = imu.readByte(WHO_AM_I_MPU6050);

    if (whoami != 0x68) {
        printf("Invalid device...\n\r");
        return 1; // Correct device was not found
    }
    float devPerc[3]={0,0,0};

    imu.reset(); // Reset registers to default in preparation for device calibration
    imu.calibrate(); // Calibrate gyro and accelerometers

    imu.reset();
    imu.selfTest(devPerc); // Start self test
    printf("x-axis self test: acceleration trim within : %f % of factory value\n\r", devPerc[0]);
    printf("y-axis self test: acceleration trim within : %f % of factory value\n\r", devPerc[1]);
    printf("z-axis self test: acceleration trim within : %f % of factory value\n\r", devPerc[2]);

    wait_us(2000000); // Delay a while to let the device stabilize
    imu.reset();
    imu.init();

    float a[3], g[3];
    float tempscale = 340.0f;

    while (true) {

        // If data ready bit set, all data registers have new data
        if (imu.readByte(INT_STATUS) & 0x01)    // check if data ready interrupt
        {
            imu.readAccelData(a);  // Read the x/y/z acceleration
    
            printf("ax: %d, ay: %d, az: %d\n\r", (int)(a[0] * 100.0f), (int)(a[1] * 100.0f), (int)(a[2] * 100.0f)); //cm/s^2
    
            imu.readGyroData(g);  // Read the x/y/z gyroscope values       

            printf("gx: %d, gy: %d, gz: %d\n\r", (int)(g[0] ), (int)(g[1] ), (int)(g[2] ));      //deg/s

            float temp = imu.readTempData();

            printf("Temp: %d\n\r", (int)((temp / tempscale) + 36.53f));      //celsius
        }

        thread_sleep_for(100); // 100 Hz
    }
}
