/*
 * i2c_slave.h
 *
 *  Created on: Aug 12, 2025
 *      Author: Juang
 */

#ifndef INC_I2C_SLAVE_H_
#define INC_I2C_SLAVE_H_

#ifdef __cplusplus
    extern "C" {
#endif

    void process_i2c_cmd(void); // Expose our new fn to main

#ifdef __cplusplus
}
#endif

#endif /* INC_I2C_SLAVE_H_ */
