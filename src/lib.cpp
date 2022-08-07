#include "lib.h"

#define motor_1_on 27
#define motor_1_off 13
#define relayoff HIGH
#define relayon LOW

void motor_1_onstate()
{
    digitalWrite(motor_1_on, relayon);
    delay(2000);
    digitalWrite(motor_1_on, relayoff);
}

void motor_1_offstate()
{
    digitalWrite(motor_1_off, relayon);
    delay(2000);
    digitalWrite(motor_1_off, relayoff);
}
