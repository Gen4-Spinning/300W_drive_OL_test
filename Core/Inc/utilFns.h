#ifndef __UTILFNS_H
#define __UTILFNS_H



#define STATE_IDLE 0
#define STATE_RUN 1

//SECTIONS in each RUN
#define OFF 0
#define START 1
#define INCREASE_PWM 2
#define PK_PWM_WAIT 3
#define	DECREASE_PWM 4
#define	STOP 5

#define START_DELAY_TIME 4
#define M1_PWM_INCREASE_TIME 20
#define M2_PWM_INCREASE_TIME 15
#define PK_PWM_WAIT_TIME 15
#define M1_PWM_DECREASE_TIME 15
#define STOP_DELAY_TIME 5

#define M1_INC_START_PERCENTAGE 0
#define M1_INC_END_PERCENTAGE 100

//MODE2 DEFINE
#define M2_INC_START_PERCENTAGE 15
#define M2_INC_END_PERCENTAGE 80

#define DISABLE_D 1
#define ENABLE_D 2

char Read_inp1Pin(void);
char Read_inp2Pin(void);

void MotorDrive(char index);

#endif

