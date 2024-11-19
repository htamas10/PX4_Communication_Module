#include "uart.h"
#include "menu.h"
#include "main.h"
#include "lcd.h"

extern UART_HandleTypeDef huart2;
extern uint16_t weight;

typedef struct {
	double P;
	double I;
	double D;
} PID;

typedef struct {
	int weight;
	PID pid;
} Weight_PID_params;

Weight_PID_params data[] =
    {{0,   {0.05, 0.1, 0.025}},
    {150, {0.10, 0.175, 0.022}},
    {200, {0.12, 0.18, 0.020}},
    {250, {0.14, 0.19, 0.018}},
    {300, {0.16, 0.20, 0.016}},
    {350, {0.18, 0.21, 0.014}},
    {400, {0.20, 0.22, 0.012}},
    {450, {0.22, 0.23, 0.010}},
    {500, {0.25, 0.25, 0.008}}};

void Send_MAVLink_Message(int weight){

}
