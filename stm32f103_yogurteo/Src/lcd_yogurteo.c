#include "lcd_yogurteo.h"

#include "main.h"
#include "stm32f1xx_hal.h"

typedef enum LcdComNumber LcdComNumber;

enum LcdComNumber
{
    LCD_COM_NONE,
    LCD_COM1,
    LCD_COM2,
    LCD_COM3
};

typedef enum LcdSegmentNumber LcdSegmentNumber;

enum LcdSegmentNumber
{
    LCD_SEG_NONE,
    LCD_SEG1,
    LCD_SEG2,
    LCD_SEG3,
    LCD_SEG4,
    LCD_SEG5
};

typedef struct Segment Segment;

struct Segment
{
    LcdComNumber com;
    LcdSegmentNumber seg;
};

typedef Segment Number[];

//Right digit
static Number numberLo0 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG1},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo1 = {
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo2 = {
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG1},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo3 = {
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo4 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo5 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG4},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo6 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG4},
    {LCD_COM2, LCD_SEG1},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo7 = {
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo8 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG1},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberLo9 = {
    {LCD_COM3, LCD_SEG1},
    {LCD_COM3, LCD_SEG4},
    {LCD_COM3, LCD_SEG5},
    {LCD_COM2, LCD_SEG4},
    {LCD_COM2, LCD_SEG5},
    {LCD_COM1, LCD_SEG4},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number* LoNumbers[10] = {
    &numberLo0,
    &numberLo1,
    &numberLo2,
    &numberLo3,
    &numberLo4,
    &numberLo5,
    &numberLo6,
    &numberLo7,
    &numberLo8,
    &numberLo9,
};

//Left digit
static Number numberHi0 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM1, LCD_SEG2},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi1 = {
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi2 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM1, LCD_SEG2},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi3 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi4 = {
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi5 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi6 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM1, LCD_SEG2},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi7 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi8 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM1, LCD_SEG2},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number numberHi9 = {
    {LCD_COM3, LCD_SEG3},
    {LCD_COM3, LCD_SEG2},
    {LCD_COM2, LCD_SEG3},
    {LCD_COM2, LCD_SEG2},
    {LCD_COM1, LCD_SEG1},
    {LCD_COM1, LCD_SEG3},
    {LCD_COM_NONE, LCD_SEG_NONE}
};

static Number* HiNumbers[10] = {
    &numberHi0,
    &numberHi1,
    &numberHi2,
    &numberHi3,
    &numberHi4,
    &numberHi5,
    &numberHi6,
    &numberHi7,
    &numberHi8,
    &numberHi9
};

static Number AllSegments = {
    {LCD_COM_NONE, LCD_SEG_NONE},
    {LCD_COM_NONE, LCD_SEG_NONE},
    {LCD_COM_NONE, LCD_SEG_NONE},
    {LCD_COM_NONE, LCD_SEG_NONE},
    {LCD_COM_NONE, LCD_SEG_NONE},
    {LCD_COM_NONE, LCD_SEG_NONE}
};


typedef enum LcdState LcdState;

enum LcdState
{
    LcdStateCom1Low,
    LcdStateCom1High,
    LcdStateCom2Low,
    LcdStateCom2High,
    LcdStateCom3Low,
    LcdStateCom3High
};

static LcdState lcdState = LcdStateCom1Low;

inline void GetLcdSegGpio(LcdSegmentNumber segmentNo, GPIO_TypeDef** portOut, uint16_t* pinOut)
{
    switch (segmentNo)
    {
        case LCD_SEG1:
            *portOut = LCD_SEG1_GPIO_Port;
            *pinOut  = LCD_SEG1_Pin;
            break;
        case LCD_SEG2:
            *portOut = LCD_SEG2_GPIO_Port;
            *pinOut  = LCD_SEG2_Pin;
            break;
        case LCD_SEG3:
            *portOut = LCD_SEG3_GPIO_Port;
            *pinOut  = LCD_SEG3_Pin;
            break;
        case LCD_SEG4:
            *portOut = LCD_SEG4_GPIO_Port;
            *pinOut  = LCD_SEG4_Pin;
            break;
        case LCD_SEG5:
            *portOut = LCD_SEG5_GPIO_Port;
            *pinOut  = LCD_SEG5_Pin;
            break;
    }
}

void PrintSegment(LcdSegmentNumber segmentNo, bool state)
{
    GPIO_TypeDef* port;
    uint16_t pin;
    GetLcdSegGpio(segmentNo, &port, &pin);
    HAL_GPIO_WritePin(port, pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void PrintSegments(Number* numbers, bool state)
{
    for (int i = 0; (*numbers)[i].com != LCD_COM_NONE; ++i)
    {
        PrintSegment((*numbers)[i].seg, state);
    }
}

inline void GetLcdComGpio(LcdComNumber com, GPIO_TypeDef** portOut, uint16_t* pinOut)
{
    switch (com)
    {
        case LCD_COM1:
            *portOut = LCD_COM1_GPIO_Port;
            *pinOut = LCD_COM1_Pin;
            break;
        case LCD_COM2:
            *portOut = LCD_COM2_GPIO_Port;
            *pinOut = LCD_COM2_Pin;
            break;
        case LCD_COM3:
            *portOut = LCD_COM3_GPIO_Port;
            *pinOut = LCD_COM3_Pin;
            break;
    }
}

bool IsSegmentExists(Segment* segment)
{
    for (int i = 0; AllSegments[i].com != LCD_COM_NONE; ++i)
    {
        if (AllSegments[i].seg == segment->seg)
            return true;
    }
    return false;
}

void GetUniqueSegments(LcdComNumber com, Number* leftNumber, Number* rightNumber)
{
    int allSegmentsCounter = 0;
    for (int i = 0; (*leftNumber)[i].com != LCD_COM_NONE; ++i)
    {
        if ((*leftNumber)[i].com == com)
        {
            AllSegments[allSegmentsCounter] = (*leftNumber)[i];
            ++allSegmentsCounter;
        }
    }
    AllSegments[allSegmentsCounter].com = LCD_COM_NONE;
    AllSegments[allSegmentsCounter].seg = LCD_SEG_NONE;
    for (int i = 0; (*rightNumber)[i].com != LCD_COM_NONE; ++i)
    {
        Segment* segment = &(*rightNumber)[i];
        if (segment->com == com && !IsSegmentExists(segment))
        {
            AllSegments[allSegmentsCounter].com = segment->com;
            AllSegments[allSegmentsCounter].seg = segment->seg;
            ++allSegmentsCounter;
            AllSegments[allSegmentsCounter].com = LCD_COM_NONE;
            AllSegments[allSegmentsCounter].seg = LCD_SEG_NONE;
        }
    }
}

void PrintLcdCom(LcdComNumber com, uint8_t leftDigit, uint8_t rightDigit, bool blink)
{
    GPIO_TypeDef* portCom;
    uint16_t pinCom;
    GetUniqueSegments(com, HiNumbers[leftDigit], LoNumbers[rightDigit]);
    GetLcdComGpio(com, &portCom, &pinCom);
    HAL_GPIO_WritePin(portCom, pinCom, blink ? GPIO_PIN_SET : GPIO_PIN_RESET);
    bool antiPhase = !blink;
    PrintSegments(&AllSegments, antiPhase);
}

void ByteToBcd(uint8_t value, uint8_t* leftDigit, uint8_t* rightDigit)
{
    *leftDigit = 0;
    while (value >= 10)
    {
        ++(*leftDigit);
        value -= 10;
    }
    *rightDigit = value;
}

void PrintLcd(uint8_t number, bool blink)
{
    uint8_t leftDigit;
    uint8_t rightDigit;
    ByteToBcd(number, &leftDigit, &rightDigit);
    
    switch (lcdState)
    {
        case LcdStateCom1Low:
        case LcdStateCom1High:
            PrintLcdCom(LCD_COM1, leftDigit, rightDigit, blink);
            break;
        case LcdStateCom2Low:
        case LcdStateCom2High:
            PrintLcdCom(LCD_COM2, leftDigit, rightDigit, blink);
            break;
        case LcdStateCom3Low:
        case LcdStateCom3High:
            PrintLcdCom(LCD_COM3, leftDigit, rightDigit, blink);
            break;
    }
    
    ++lcdState;
    if (lcdState > LcdStateCom3High)
        lcdState = LcdStateCom1Low;
}

