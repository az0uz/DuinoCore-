/*
 * MIT License
 *
 * Copyright (c) 2016 Antoine de Maleprade
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __MPU60X0DEF_H__
#define __MPU60X0DEF_H__

#define MPU60X0_ADDRESS 0x68

#define MPU60X0_ACCEL_FS 1000

#define MPU60X0_DLPF_260 0
#define MPU60X0_DLPF_188 1
#define MPU60X0_DLPF_98  2
#define MPU60X0_DLPF_44  3
#define MPU60X0_DLPF_21  4
#define MPU60X0_DLPF_10  5
#define MPU60X0_DLPF_5   6

#define MPU60X0_MOT_INT (1<<6)
#define MPU60X0_FIFO_OFLOW_INT (1<<4)
#define MPU60X0_DATA_READY_INT (1)

#define MPU60X0_FIFO_EN_VAL (1<<6)
#define MPU60X0_I2C_MST_EN (1<<5)
#define MPU60X0_FIFO_RESET (1<<2)
#define MPU60X0_I2C_MST_RESET (1<<1)
#define MPU60X0_SIG_RESET (1)

#define MPU60X0_DEVICE_RESET (1<<7)
#define MPU60X0_SLEEP (1<<6)
#define MPU60X0_CYCLE (1<<5)
#define MPU60X0_TEMP_DIS (1<<3)
#define MPU60X0_PLL_GX_REF (1)


#endif //__MPU60X0DEF_H__

