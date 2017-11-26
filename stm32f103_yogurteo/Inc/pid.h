#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct PidData PidData;

struct PidData
{
    int32_t Kp;
    int32_t Ti;
    int32_t Td;

    int32_t prevError;
    int32_t integral;
};

//Kp %, Ti s, Td s
void PidControllerInit(int32_t Kp, int32_t Ti, int32_t Td, int32_t numberOfCallsPerSecond, PidData* pid);
int32_t PidControllerExecute(int32_t setPoint, int32_t processValue, PidData* pid);
void PidControllerReset(PidData* pid);

int32_t PControllerExecute(int32_t setPoint, int32_t processValue, PidData* pid);

#endif
