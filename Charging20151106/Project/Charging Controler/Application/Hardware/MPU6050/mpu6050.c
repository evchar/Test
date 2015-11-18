////////////////////////////////////////////////////////////////////////////
//
//  This file is part of linux-mpu9150
//
//  Copyright (c) 2013 Pansenti, LLC
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of
//  this software and associated documentation files (the "Software"), to deal in
//  the Software without restriction, including without limitation the rights to use,
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the
//  Software, and to permit persons to whom the Software is furnished to do so,
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include <stdio.h>
#include <string.h>

#include "main.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"

static int data_ready();
static void calibrate_data(mpudata_t *mpu);
static void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ);
static int data_fusion(mpudata_t *mpu);
static unsigned short inv_row_2_scale(const signed char *row);
static unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx);

int debug_on;
int yaw_mixing_factor;

int use_accel_cal;
caldata_t accel_cal_data;

int use_mag_cal;
caldata_t mag_cal_data;


//float magkxx=0.005028472981121  ; //见文档《磁传感器标定程序使用说明》
//float magkyy= 0.005101793261312;
//float magkzz= 0.005265425302025;
//float magkyx=0.000061691738883;
//float magkzx=0.000034113294031;
//float magkzy=-0.000015094571459;
//
//float magxoffset= 7.322416637270568;
//float magyoffset=-41.679077201843214;
//float mag0ffset= 1.370773743195294;

float magkxx=0.005024271140198  ; //见文档《磁传感器标定程序使用说明》
float magkyy= 0.005070559456294;
float magkzz= 0.005297112014961;
float magkyx=0.000046207648216;
float magkzx= 0.000043705764021;
float magkzy= 0.000027426882536;

float magxoffset= 7.651452293735868;
float magyoffset=-42.357763782749167;
float mag0ffset= -0.821333772957188;


float compss[3]={0};

void mpu9150_set_debug(int on)
{
    debug_on = on;
}

//void run_self_test(void)
//{
//    int result;
//
//    long gyro[3], accel[3];
//
//    result = mpu_run_6500_self_test(gyro, accel,0);
//    if (result == 0x07) {
//        /* Test passed. We can trust the gyro data here, so let's push it down
//         * to the DMP.
//         */
//        float sens;
//        unsigned short accel_sens;
//        mpu_get_gyro_sens(&sens);
//        gyro[0] = (long)(gyro[0] * sens);
//        gyro[1] = (long)(gyro[1] * sens);
//        gyro[2] = (long)(gyro[2] * sens);
//        dmp_set_gyro_bias(gyro);
//        mpu_get_accel_sens(&accel_sens);
//        accel[0] *= accel_sens;
//        accel[1] *= accel_sens;
//        accel[2] *= accel_sens;
//        dmp_set_accel_bias(accel);
//        printf("self test pass!\r\n");
//    }
//    else
//    {
//       printf("self test falied!\r\n");
//    }
//
//
//}
#define USE_CAL_HW_REGISTERS 1
void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

#if defined (MPU6500) || defined (MPU9250)
    result = mpu_run_6500_self_test(gyro, accel, 0);
#elif defined (MPU6050) || defined (MPU9150)
    result = mpu_run_self_test(gyro, accel);
#endif
    if (result == 0x07) {
        printf("Passed!\n");
        printf("accel: %7.4f %7.4f %7.4f\n",
                    accel[0]/65536.f,
                    accel[1]/65536.f,
                    accel[2]/65536.f);
        printf("gyro: %7.4f %7.4f %7.4f\n",
                    gyro[0]/65536.f,
                    gyro[1]/65536.f,
                    gyro[2]/65536.f);
        /* Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for(i = 0; i<3; i++) {
          gyro[i] = (long)(gyro[i] * 32.8f); //convert to +-1000dps
          accel[i] *= 4096.f; //convert to +-8G
          accel[i] = accel[i] >> 16;
          gyro[i] = (long)(gyro[i] >> 16);
        }

        mpu_set_gyro_bias_reg(gyro);

#if defined (MPU6500) || defined (MPU9250)
        mpu_set_accel_bias_6500_reg(accel);
#elif defined (MPU6050) || defined (MPU9150)
        mpu_set_accel_bias_6050_reg(accel);
#endif
#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
     * biases in g's << 16.
     */
      unsigned short accel_sens;
      float gyro_sens;

    mpu_get_accel_sens(&accel_sens);
    accel[0] *= accel_sens;
    accel[1] *= accel_sens;
    accel[2] *= accel_sens;
    //inv_set_accel_bias(accel, 3);
    mpu_get_gyro_sens(&gyro_sens);
    gyro[0] = (long) (gyro[0] * gyro_sens);
    gyro[1] = (long) (gyro[1] * gyro_sens);
    gyro[2] = (long) (gyro[2] * gyro_sens);
    //inv_set_gyro_bias(gyro, 3);
#endif
    }
    else {
            if (!(result & 0x1))
                printf("Gyro failed.\n");
            if (!(result & 0x2))
                printf("Accel failed.\n");
            if (!(result & 0x4))
                printf("Compass failed.\n");
     }

}

int mpu6050_init(int sample_rate, int mix_factor)
{
//    signed char gyro_orientation[9] = { 1, 0, 0,
//                                        0, 0, 1,
//                                        0, -1, 0 };
    
    
        signed char gyro_orientation[9] = { 1, 0, 0,
                                            0, 1, 0,
                                            0, 0, 1 };
        
    if (sample_rate < MIN_SAMPLE_RATE || sample_rate > MAX_SAMPLE_RATE) {
        printf("Invalid sample rate %d\n", sample_rate);
        return -1;
    }

    if (mix_factor < 0 || mix_factor > 100) {
        printf("Invalid mag mixing factor %d\n", mix_factor);
        return -1;
    }

    yaw_mixing_factor = mix_factor;

    //linux_set_i2c_bus(i2c_bus);

    printf("\nInitializing IMU .");
    //fflush(stdout);

    if (mpu_init(NULL)) {
        printf("\nmpu_init() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL )) {
        printf("\nmpu_set_sensors() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL)) {
        printf("\nmpu_configure_fifo() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (mpu_set_sample_rate(sample_rate)) {
        printf("\nmpu_set_sample_rate() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

//    if (mpu_set_compass_sample_rate(sample_rate)) {
//        printf("\nmpu_set_compass_sample_rate() failed\n");
//        return -1;
//    }

    printf(".");
    //fflush(stdout);

    if (dmp_load_motion_driver_firmware()) {
        printf("\ndmp_load_motion_driver_firmware() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation))) {
        printf("\ndmp_set_orientation() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL
                        | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_PEDOMETER)) {
        printf("\ndmp_enable_feature() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (dmp_set_fifo_rate(sample_rate)) {
        printf("\ndmp_set_fifo_rate() failed\n");
        return -1;
    }

    printf(".");
    //fflush(stdout);

    if (mpu_set_dmp_state(1)) {
        printf("\nmpu_set_dmp_state(1) failed\n");
        return -1;
    }

    printf(" done\n\n");

    return 0;
}

void mpu9150_exit()
{
    // turn off the DMP on exit
    if (mpu_set_dmp_state(0))
        printf("mpu_set_dmp_state(0) failed\n");

    // TODO: Should turn off the sensors too
}

void mpu9150_set_accel_cal(caldata_t *cal)
{
    int i;
    //long bias[3];

    if (!cal) {
        use_accel_cal = 0;
        return;
    }

    memcpy(&accel_cal_data, cal, sizeof(caldata_t));

    for (i = 0; i < 3; i++) {
        if (accel_cal_data.range[i] < 1)
            accel_cal_data.range[i] = 1;
        else if (accel_cal_data.range[i] > ACCEL_SENSOR_RANGE)
            accel_cal_data.range[i] = ACCEL_SENSOR_RANGE;

        //bias[i] = -accel_cal_data.offset[i];
    }

    if (debug_on) {
        printf("\naccel cal (range : offset)\n");

        for (i = 0; i < 3; i++)
            printf("%d : %d\n", accel_cal_data.range[i], accel_cal_data.offset[i]);
    }

    //mpu_set_accel_bias(bias);

    use_accel_cal = 1;
}

void mpu9150_set_mag_cal(caldata_t *cal)
{
    int i;

    if (!cal) {
        use_mag_cal = 0;
        return;
    }

    memcpy(&mag_cal_data, cal, sizeof(caldata_t));

    for (i = 0; i < 3; i++) {
        if (mag_cal_data.range[i] < 1)
            mag_cal_data.range[i] = 1;
        else if (mag_cal_data.range[i] > MAG_SENSOR_RANGE)
            mag_cal_data.range[i] = MAG_SENSOR_RANGE;

        if (mag_cal_data.offset[i] < -MAG_SENSOR_RANGE)
            mag_cal_data.offset[i] = -MAG_SENSOR_RANGE;
        else if (mag_cal_data.offset[i] > MAG_SENSOR_RANGE)
            mag_cal_data.offset[i] = MAG_SENSOR_RANGE;
    }

    if (debug_on) {
        printf("\nmag cal (range : offset)\n");

        for (i = 0; i < 3; i++)
            printf("%d : %d\n", mag_cal_data.range[i], mag_cal_data.offset[i]);
    }

    use_mag_cal = 1;
}

int mpu9150_read_dmp(mpudata_t *mpu)
{
    short sensors;
    unsigned char more;

    if (!data_ready())
    {
      return -1;
    }

    if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
        //printf("dmp_read_fifo() failed\r\n");
        return -1;
    }

    while (more) {
        // Fell behind, reading again
        //printf("m\r\n");
        if (dmp_read_fifo(mpu->rawGyro, mpu->rawAccel, mpu->rawQuat, &mpu->dmpTimestamp, &sensors, &more) < 0) {
            //printf("dmp_read_fifo() failed\n");
            return -1;
        }
    }
    return 0;
}

int mpu9150_read_mag(mpudata_t *mpu)
{
    if (mpu_get_compass_reg(mpu->rawMag, &mpu->magTimestamp) < 0) {
        printf("mpu_get_compass_reg() failed\n");
        return -1;
    }

    return 0;
}

extern uint32 time2;
int mpu9150_read(mpudata_t *mpu)
{

    if (mpu9150_read_dmp(mpu) != 0)
       return -1;
//
//    if (mpu9150_read_mag(mpu) != 0)
//        return -1;
    //time2 = xTaskGetTickCount();
    calibrate_data(mpu);

    return data_fusion(mpu);
}

int data_ready()
{
    short status;

    if (mpu_get_int_status(&status) < 0) {
        //printf("mpu_get_int_status() failed\n");
        return -1;
    }
    //printf("s:%x\r\n",status);

    //unsigned char tmp[2];

    /*读取motion Detection Status
    if (i2c_read(0xd0, 0x61, 1, tmp))
        return -1;//hujiang
    printf("%d\r\n",tmp[0]);
    */


    /*读取温度，hujiang
    if (i2c_read(0xd0, 0x41, 1, tmp))
    return -1;//hujiang
    if (i2c_read(0xd0, 0x42, 2, &tmp[1]))
    return -1;//hujiang
    printf("%x%x\r\n",tmp[0],tmp[1]);
    */

    return (status &&(MPU_INT_STATUS_DATA_READY | MPU_INT_STATUS_DMP | MPU_INT_STATUS_DMP_0));
}

void calibrate_data(mpudata_t *mpu)
{
  #if 0
    if (use_mag_cal) {
      mpu->calibratedMag[VEC3_Y] = -(short)(((long)(mpu->rawMag[VEC3_X] - mag_cal_data.offset[VEC3_X])
            * (long)MAG_SENSOR_RANGE) / (long)mag_cal_data.range[VEC3_X]);

      mpu->calibratedMag[VEC3_X] = (short)(((long)(mpu->rawMag[VEC3_Y] - mag_cal_data.offset[VEC3_Y])
            * (long)MAG_SENSOR_RANGE) / (long)mag_cal_data.range[VEC3_Y]);

      mpu->calibratedMag[VEC3_Z] = (short)(((long)(mpu->rawMag[VEC3_Z] - mag_cal_data.offset[VEC3_Z])
            * (long)MAG_SENSOR_RANGE) / (long)mag_cal_data.range[VEC3_Z]);
    }
    else {
        mpu->calibratedMag[VEC3_Y] = -mpu->rawMag[VEC3_X];
        mpu->calibratedMag[VEC3_X] = mpu->rawMag[VEC3_Y];
        mpu->calibratedMag[VEC3_Z] = mpu->rawMag[VEC3_Z];
    }
#endif
#if 1
     if (use_mag_cal) {
      mpu->calibratedMag[VEC3_X] = (short)(((long)(mpu->rawMag[VEC3_X] - mag_cal_data.offset[VEC3_X])));
       
      mpu->calibratedMag[VEC3_Y] = -(short)(((long)(mpu->rawMag[VEC3_Z] - mag_cal_data.offset[VEC3_Z])));
           
      mpu->calibratedMag[VEC3_Z] = (short)(((long)(mpu->rawMag[VEC3_Y] - mag_cal_data.offset[VEC3_Y])));       
    }
    else {
        mpu->calibratedMag[VEC3_X] = mpu->rawMag[VEC3_X];
        mpu->calibratedMag[VEC3_Y] = -mpu->rawMag[VEC3_Z];
        mpu->calibratedMag[VEC3_Z] = mpu->rawMag[VEC3_Y];
    }
#endif
//      mpu->calibratedMag[VEC3_X] = (short)(((long)(mpu->rawMag[VEC3_X] - mag_cal_data.offset[VEC3_X])));
//       
//      mpu->calibratedMag[VEC3_Y] = (short)(((long)(mpu->rawMag[VEC3_Y] - mag_cal_data.offset[VEC3_Y])));
//           
//      mpu->calibratedMag[VEC3_Z] = (short)(((long)(mpu->rawMag[VEC3_Z] - mag_cal_data.offset[VEC3_Z])));       
      
      
//         compss[VEC3_X]=((mpu->rawMag[0]-magxoffset)*magkxx+(mpu->rawMag[1]-magyoffset)*magkyx+(mpu->rawMag[2]-mag0ffset)*magkzx)*10000; 
//         compss[VEC3_Y]=((mpu->rawMag[1]-magyoffset)*magkyy+(mpu->rawMag[2]-mag0ffset)*magkzy)*10000;
//         compss[VEC3_Z]=((mpu->rawMag[2]-mag0ffset)*magkzz)*10000;
//    
//        mpu->calibratedMag[VEC3_X] = (short)compss[VEC3_X];
//        mpu->calibratedMag[VEC3_Y] = -(short)compss[VEC3_Z];
//        mpu->calibratedMag[VEC3_Z] = (short)compss[VEC3_Y];
      

        
//    if (use_accel_cal) {
//      mpu->calibratedAccel[VEC3_X] = -(short)(((long)mpu->rawAccel[VEC3_X] * (long)ACCEL_SENSOR_RANGE)
//            / (long)accel_cal_data.range[VEC3_X]);
//
//      mpu->calibratedAccel[VEC3_Y] = (short)(((long)mpu->rawAccel[VEC3_Y] * (long)ACCEL_SENSOR_RANGE)
//            / (long)accel_cal_data.range[VEC3_Y]);
//
//      mpu->calibratedAccel[VEC3_Z] = (short)(((long)mpu->rawAccel[VEC3_Z] * (long)ACCEL_SENSOR_RANGE)
//            / (long)accel_cal_data.range[VEC3_Z]);
//    }
    //else {
        mpu->calibratedAccel[VEC3_X] = mpu->rawAccel[VEC3_X];
        mpu->calibratedAccel[VEC3_Y] = mpu->rawAccel[VEC3_Y];
        mpu->calibratedAccel[VEC3_Z] = mpu->rawAccel[VEC3_Z];
    //}
}

void tilt_compensate(quaternion_t magQ, quaternion_t unfusedQ)
{
    quaternion_t unfusedConjugateQ;
    quaternion_t tempQ;

    quaternionConjugate(unfusedQ, unfusedConjugateQ);
    quaternionMultiply(magQ, unfusedConjugateQ, tempQ);
    quaternionMultiply(unfusedQ, tempQ, magQ);
}

long inv_q29_mult(long a, long b)
{
#ifdef EMPL_NO_64BIT
    long result;
    result = (long)((float)a * b / (1L << 29));
    return result;
#else
    long long temp;
    long result;
    temp = (long long)a * b;
    result = (long)(temp >> 29);
    return result;
#endif
}

int inv_get_gravity(long *q,long *data)
{
    data[0] =
        inv_q29_mult(q[1], q[3]) - inv_q29_mult(q[2], q[0]);
    data[2] =
        inv_q29_mult(q[2], q[3]) + inv_q29_mult(q[1], q[0]);
    data[1] =
        (inv_q29_mult(q[3], q[3]) + inv_q29_mult(q[0], q[0])) -
        1073741824L;
    return 0;
}


int inv_get_linear_accel(long *data,short *accl,long *gravity)
{

    if (data != NULL)
    {
        data[0] = accl[0] - (gravity[0] >> 16);
        data[1] = accl[1] - (gravity[1] >> 16);
        data[2] = accl[2] - (gravity[2] >> 16);
        return 0;
    }
    else {
        return -1;
    }
}

//int dmpGetGravity(float *v, float *q) {
//    v[0] = 2 * (q[1]*q[3] - q[0]*q[2])*16384;
//    v[1] = 2 * (q[0]*q[1] + q[2]*q[3])*16384;
//    v[2] = (q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])*16384;
//    return 0;
//}
    
long g[3];
long linear_accl[3];
//extern float heading;

int data_fusion(mpudata_t *mpu)
{
    quaternion_t dmpQuat;
    vector3d_t dmpEuler;
    quaternion_t magQuat;
    quaternion_t unfusedQuat;
    float deltaDMPYaw;
    float deltaMagYaw;
    float newMagYaw;
    float newYaw;
    
    dmpQuat[QUAT_W] = (float)mpu->rawQuat[QUAT_W];
    dmpQuat[QUAT_X] = (float)mpu->rawQuat[QUAT_X];
    dmpQuat[QUAT_Y] = (float)mpu->rawQuat[QUAT_Y];
    dmpQuat[QUAT_Z] = (float)mpu->rawQuat[QUAT_Z];

    
    inv_get_gravity(mpu->rawQuat,g);
    inv_get_linear_accel(linear_accl,mpu->calibratedAccel,g);
      
    quaternionNormalize(dmpQuat);
    quaternionToEuler(dmpQuat, dmpEuler);

    //dmpGetGravity(g1,dmpQuat);
    
    mpu->fusedEuler[VEC3_X] = dmpEuler[VEC3_X];
    mpu->fusedEuler[VEC3_Y] = -dmpEuler[VEC3_Y];
    mpu->fusedEuler[VEC3_Z] = 0;

    eulerToQuaternion(mpu->fusedEuler, unfusedQuat);

    deltaDMPYaw = -dmpEuler[VEC3_Z] + mpu->lastDMPYaw;
    
    //dmp_yaw = (deltaDMPYaw + mpu->lastDMPYaw);
    mpu->lastDMPYaw = dmpEuler[VEC3_Z];

    magQuat[QUAT_W] = 0;
    magQuat[QUAT_X] = mpu->calibratedMag[VEC3_X];
    magQuat[QUAT_Y] = mpu->calibratedMag[VEC3_Y];
    magQuat[QUAT_Z] = mpu->calibratedMag[VEC3_Z];


    tilt_compensate(magQuat, unfusedQuat);

    newMagYaw = -atan2f(magQuat[QUAT_Y], magQuat[QUAT_X]);
    //printf("%d %d %d %d %d %d  ",mpu->rawMag[0],mpu->rawMag[1],mpu->rawMag[2],mpu->calibratedMag[0],
    //                                                        mpu->calibratedMag[1],mpu->calibratedMag[2]);
    //heading = newMagYaw;
//    if (heading > TWO_PI)
//        heading -= TWO_PI;
//    else if (heading < 0.0f)
//        heading += TWO_PI;
    
    if (newMagYaw != newMagYaw) {
        printf("newMagYaw NAN\n");
        return -1;
    }

    if (newMagYaw < 0.0f)
        newMagYaw = TWO_PI + newMagYaw;

    newYaw = mpu->lastYaw + deltaDMPYaw;

    if (newYaw > TWO_PI)
        newYaw -= TWO_PI;
    else if (newYaw < 0.0f)
        newYaw += TWO_PI;

    deltaMagYaw = newMagYaw - newYaw;

    if (deltaMagYaw >= (float)M_PI)
        deltaMagYaw -= TWO_PI;
    else if (deltaMagYaw < -(float)M_PI)
        deltaMagYaw += TWO_PI;

    if (yaw_mixing_factor > 0)
        newYaw += deltaMagYaw / yaw_mixing_factor;

    if (newYaw > TWO_PI)
        newYaw -= TWO_PI;
    else if (newYaw < 0.0f)
        newYaw += TWO_PI;

    mpu->lastYaw = newYaw;

    if (newYaw > (float)M_PI)
        newYaw -= TWO_PI;

    mpu->fusedEuler[VEC3_Z] = newYaw;

    eulerToQuaternion(mpu->fusedEuler, mpu->fusedQuat);
    return 0;
}


/* These next two functions convert the orientation matrix (see
 * gyro_orientation) to a scalar representation for use by the DMP.
 * NOTE: These functions are borrowed from InvenSense's MPL.
 */
unsigned short inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}

unsigned short inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    /*
       XYZ  010_001_000 Identity Matrix
       XZY  001_010_000
       YXZ  010_000_001
       YZX  000_010_001
       ZXY  001_000_010
       ZYX  000_001_010
     */
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;
    return scalar;
}


mpudata_t mpu;
void Mpu6050Task(void *pvParameters)
{
  int sample_rate = DEFAULT_SAMPLE_RATE_HZ;
  int yaw_mix_factor = DEFAULT_YAW_MIX_FACTOR;
  short tmp_pitch,tmp_roll,tmp_yaw;
  i2cdevInit(I2C1);

  mpu6050_init(sample_rate, yaw_mix_factor);

  run_self_test();

  portTickType xLastWakeTime;
  xLastWakeTime =  xTaskGetTickCount();
  for(;;)
  {
     mpu9150_read(&mpu);
     tmp_yaw = (int16)(mpu.fusedEuler[VEC3_Z] * RAD_TO_DEGREE*1);
     tmp_pitch  = (int16)(mpu.fusedEuler[VEC3_X] * RAD_TO_DEGREE*1);
     tmp_roll = (int16)(mpu.fusedEuler[VEC3_Y] * RAD_TO_DEGREE*1);
     //printf("%4d,%4d,%4d\r\n",tmp_pitch,tmp_roll,tmp_yaw);
     vTaskDelayUntil( &xLastWakeTime, ( 50 / portTICK_RATE_MS ) );
  }
}
