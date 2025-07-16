//
// Created by 24319 on 2025/4/5.
//

#ifndef ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H
#define ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H

int16_t Read_Encoder_Angle4(void);
int16_t Read_Encoder_Angle5(void);
int16_t Read_Encoder_Angle6(void);

float convertRawAngleToDegrees1(int16_t newAngle);


#endif //ENGINEERING_USERDEFINE_VERSION2_HARDWARE_I2C1_H
