#include "pid.h"

/*
U = K * ( Err + (1/Ti)*Int + Td*dErr )

U — мощность, которую следует выдать;
K — пропорциональный коэффициент;
Ti — постоянная времени интегрирования. Обратите внимание — в расчетах используется обратная величина;
Td — постоянная времени дифференцирования
Err — текущее рассогласование (разница между уставкой и измеренной температурой
dErr — производная рассогласования (разница между текущей и прошлой ошибкой)
Int — накопленный интеграл рассогласования (сумма всех Err'ов, кои мы видели)

U — имеет величину в % мощности.
Err — рассогласование, задаётся в градусах. Точнее — через 1/10 градуса.
Int — интеграл, представляет собой сумму градусов во времени — а значит, имеет размерность градус*сек. Точнее — (1/10 градуса)*(1/2 сек)
Ti — задаётся через 1/2 сек
(1/Ti)*Int — после вычисления даёт вклад, имеющий размерность (1/10 градуса).
dErr — производная, имеет размерность градус/сек, а точнее (1/10 градуса)/(1/2 сек)
Td — задаётся через 1/2 сек
Td*dErr — после произведения приводит вклад к размерности (1/10 градуса)
(...) — итак, все слагаемые под скобками приведены к размерности (1/10 градуса)
K — согласует U и (...), а значит имеет размерность процента-на-градус, точнее (1)%/(1/10 градуса)

Теперь производим расчет баланса разрядности. Для этого распишем полную формулу пошагово:
Eo = E; Нам нужна прошлая ошибка. Ошибки — по 16бит
E = Y-X; Вычисляем новое рассогласование. 16bit
Int = Int + (E+Eo)/2; Интегрируем ошибку. При этом считаем полусумму разности (разностная схема). 32bit = 32bit + 16bit
cI = Int * (1/Ti); Считаем интегральный вклад — 32bit * 32bit => 32bit
cD = Td * (E-Eo); Считаем диф вклад — 16*16 => 32bit
PID = E + cI + cD; Подскобочное; 16+32+32 => 32bit
U = K*PID/256; Коэфф; 32*16/8 bit => 40bit.
*/

void PidControllerInit(int32_t Kp, int32_t Ti, int32_t Td, int32_t numberOfCallsPerSecond, PidData *pid)
{
    pid->Kp = Kp;
    pid->Ti = Ti * numberOfCallsPerSecond;
    pid->Td = Td * numberOfCallsPerSecond;
    pid->prevError = 0;
    pid->integral = 0;
}


int32_t PidControllerExecute(int32_t setPoint, int32_t processValue, PidData* pid)
{
    int32_t error = setPoint - processValue;
    pid->integral += (error + pid->prevError) / 2;
    int32_t U = pid->Kp * (error + pid->integral / pid->Ti + pid->Td * (error - pid->prevError));
    pid->prevError = error;
    return U;
}

void PidControllerReset(PidData* pid)
{
  pid->integral = 0;
  pid->prevError = 0;
}

int32_t PControllerExecute(int32_t setPoint, int32_t processValue, PidData* pid)
{
    int32_t error = setPoint - processValue;
    int32_t U = pid->Kp * error;
    pid->prevError = error;
    return U;
}

