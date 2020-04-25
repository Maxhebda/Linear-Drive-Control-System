#include "board.h"
#include <cr_section_macros.h>
#include "arm_math.h"
#include "oled.h"

/* ------------------------------------------------------------------------ */
/* -------------------------------- MACROS -------------------------------- */
/* ------------------------------------------------------------------------ */

/* PWM output frequency */
#define PWMFS 			100000
#define MOTOR_PORT 		1

/* Motor direction A/B pins */
#define MOTOR_DIRA 		14
#define MOTOR_DIRB 		15
#define ENCODER_PORT	1
/* Motor encoder A/B channels */
#define ENCODER_CHA 	31
#define ENCODER_CHB 	16

/* Input encoder SIA/SIB channels */
#define CONTROL_CHA 	27
#define CONTROL_CHB 	26
/* Input encoder switch */
#define CONTROL_CHC 	28

/* Limit switches */
#define LIMIT_SW_PORT	0
#define LIMIT_SW_L 		22
#define LIMIT_SW_R 		23

/* PWM output */
#define PWM_PORT 		1
#define PWM_PIN  		13

#define ENCODER_IOCON_FUNC (IOCON_FUNC0 | IOCON_DIGMODE_EN | IOCON_MODE_PULLUP | IOCON_HYS_EN)
#define LIMIT_SW_IOCON_FUNC	(IOCON_FUNC0 | IOCON_DIGMODE_EN | IOCON_MODE_PULLUP)

/* Speed */
#define CALIB_POWER 30

#define MAX_VAL_ON_RIGHT 12720

/* ------------------------------------------------------------------------ */
/* ------------------------------- VARIABLES ------------------------------ */
/* ------------------------------------------------------------------------ */

char sbuffer[30];
volatile int8_t dataControl = 0;
volatile int16_t motorPower = 0;
volatile float32_t motorVelocity = 0;
volatile bool run = false;
volatile uint16_t tdiv = 0;

/* PID related */
volatile int32_t dataEncoder = 0, dataEncoder_1 = 0;

arm_pid_instance_f32 velCoef, posCoef;
volatile float32_t velU = 0, velE = 0, velX = 0;
volatile float32_t posU = 0, posE = 0;

volatile bool leftLimitStatus = false;
volatile bool rightLimitStatus = false;

/* Position related */
uint32_t maxValOnRight;
uint32_t newPosition;

typedef enum { OLED_DESKTOP, OLED_CALIB, OLED_MANUAL, OLED_MANUAL_CONTROL } OLED_VIEW;
OLED_VIEW currentView;
typedef void(*CALLBACK)(void);

bool moveLimit = false;

/* calibration related */
#define CALIB_MODES_CNT 2
const char* calibModes[] = {"Fast", "Normal"};
uint8_t calibModeActive = 0;

/* menu related */
#define MENU_MODES_CNT 3
const char* menuModes[] = {"Manual", "Visit points", "Interval"};
uint8_t menuModeActive = 0;

/* manual related */
#define MANUAL_MODES_CNT 2
const char* manualModes[] = {"START", "EXIT"};
uint8_t manualModeActive = 0;

/* TODO Remove stub */
//int32_t tempPower;

/* ------------------------------------------------------------------------ */
/* --------------------------- PUBLIC FUNCTIONS --------------------------- */
/* ------------------------------------------------------------------------ */

static void OLED_desktop()
{
	OLED_Clear_Screen(0);
	OLED_Puts(0, 0, "      MAIN MENU");
	OLED_Draw_Line(0, 10, 127, 10);
	for (int i = 0; i < MENU_MODES_CNT; i++) {
		if (i == menuModeActive) {
			sprintf(sbuffer, "> %s", menuModes[i]);
		} else {
			sprintf(sbuffer, "  %s", menuModes[i]);
		}
		OLED_Puts(0, i+2, sbuffer);
	}

	OLED_Refresh_Gram();
}

static void OLED_drawPosition()
{
	uint8_t temp = (dataEncoder < 0 ? 0 : dataEncoder)*121 / maxValOnRight;
	OLED_Draw_Line(1+temp, 59, 1+temp, 63);
	OLED_Draw_Line(4+temp, 59, 4+temp, 63);
	OLED_Draw_Line(1+temp, 59, 4+temp, 59);
	OLED_Draw_Line(1+temp, 63, 4+temp, 63);
	OLED_Draw_Line(2+temp, 58, 3+temp, 58);

	OLED_Draw_Line(0, 60, 0, 63);
	OLED_Draw_Line(127, 60, 127, 63);
	OLED_Draw_Line(0, 63, 127, 63);
}

static void OLED_calibrate()
{
	OLED_Clear_Screen(0);
	OLED_Puts(0, 0, "   CALIBRATION MODE");
	OLED_Draw_Line(0, 10, 127, 10);

	if (run) {
		OLED_Puts(0, 4, "   Please wait ...");
	} else {
		for (int i = 0; i < CALIB_MODES_CNT; i++) {
			if (i == calibModeActive) {
				sprintf(sbuffer, "> %s", calibModes[i]);
			} else {
				sprintf(sbuffer, "  %s", calibModes[i]);
			}
			OLED_Puts(0, i+2, sbuffer);
		}
	}

	OLED_Refresh_Gram();
}

static void OLED_manual()
{
	OLED_Clear_Screen(0);
	OLED_Puts(0, 0, "      MANUAL MODE");
	OLED_Draw_Line(0, 10, 127, 10);
	for (int i = 0; i < MANUAL_MODES_CNT; i++) {
		if (i == manualModeActive) {
			sprintf(sbuffer, "> %s", manualModes[i]);
		} else {
			sprintf(sbuffer, "  %s", manualModes[i]);
		}
		OLED_Puts(0, i+2, sbuffer);
	}
	OLED_Refresh_Gram();
}

static void OLED_manualControl()
{
	OLED_Clear_Screen(0);
	OLED_Puts(0, 0, "      MANUAL MODE");
	OLED_Draw_Line(0, 10, 127, 10);
	OLED_Puts(0, 2, "> STOP");

	sprintf(sbuffer, "VEL: %3d.0 [U]", dataControl);
	OLED_Puts(0, 4, sbuffer);
	sprintf(sbuffer, "POS: %5.1f [%%]", ((float) dataEncoder/maxValOnRight)*100);
	OLED_Puts(0, 5, sbuffer);
	OLED_drawPosition();
	OLED_Refresh_Gram();
}

/* TODO Remove stub */
//void SysTick_Handler(void)
//{
//	if (run) {
//		if (tempPower > 0) { dataEncoder++; }
//		if (tempPower < 0) { dataEncoder--; }
//	}
//}

static void move(uint32_t position, uint32_t velocity)
{
	dataControl = velocity;
	run = true;
	newPosition = position;
	while (abs((int32_t) position - dataEncoder) > 5) {}
	run = false;
}

static void systemCalibrate(CALLBACK callback, bool normal)
{
	if (moveLimit) return;

	/* Reach left limit and reset encoder readouts */
	dataControl = -CALIB_POWER;
	run = true;
	while (leftLimitStatus) {
		if (callback) callback();
	}
	dataEncoder = dataEncoder_1 = 0;
	maxValOnRight = MAX_VAL_ON_RIGHT;

	if (normal) {
		/* Reach right limit and set max encoder ticks */
		dataControl = CALIB_POWER;
		while (rightLimitStatus) {
			if (callback) callback();
		}
		maxValOnRight = dataEncoder;

		run = false;
		dataControl = 0;
	}

	moveLimit = true;
	run = false;
	move(maxValOnRight/2, 50);
//	move(100, 50);
}

static void calcMotorPower(void)
{
	if (!moveLimit) {
		velU = dataControl;
	} else {
		/* Position PID */
		posE = (int32_t) newPosition - dataEncoder;
		posU = arm_pid_f32(&posCoef, posE);

		velX = dataControl / 10.0;
		if (posU < 0) velX = -velX;

		if (velX < 0 && posU < velX) posU = velX;
		if (velX > 0 && posU > velX) posU = velX;

		/* Velocity PID */
		velE = posU - motorVelocity;
		velU = arm_pid_f32(&velCoef, velE);
	}
}

static void resetCoef(void)
{
	velU = 0;
	velE = 0;
	posE = 0;
	posU = 0;
	arm_pid_reset_f32(&velCoef);
	arm_pid_reset_f32(&posCoef);
}

static void updateLimitSWStatus(void)
{
	rightLimitStatus = Chip_GPIO_GetPinState(LPC_GPIO_PORT, LIMIT_SW_PORT, LIMIT_SW_R) ?  true : false;
	leftLimitStatus = Chip_GPIO_GetPinState(LPC_GPIO_PORT, LIMIT_SW_PORT, LIMIT_SW_L) ? true : false;
}

static void motorPinsInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, MOTOR_PORT, MOTOR_DIRA, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, MOTOR_PORT, MOTOR_DIRB, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, ENCODER_CHA, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, ENCODER_CHB, ENCODER_IOCON_FUNC);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);

	/* Encoder A channel as falling edge interrupt */
	Chip_SYSCTL_SetPinInterrupt(0, ENCODER_PORT, ENCODER_CHA);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH0);
	NVIC_SetPriority(PIN_INT0_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
}

/**
 * @brief Init control encoder pins
 */
static void encoderPinsInit(void)
{
	/* Encoder pins as GPIO input with pullup enabled */
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHA, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHB, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHC, ENCODER_IOCON_FUNC);

	/* Encoder SIA and switch as falling edge interrupts */
	Chip_SYSCTL_SetPinInterrupt(1, ENCODER_PORT, CONTROL_CHA);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH1);
	NVIC_SetPriority(PIN_INT1_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	Chip_SYSCTL_SetPinInterrupt(2, ENCODER_PORT, CONTROL_CHC);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH2);
	NVIC_SetPriority(PIN_INT2_IRQn, 2);
	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
}

static void timerInit(void)
{
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();
	Chip_TIMER_Init(LPC_TIMER16_0);
	Chip_TIMER_PrescaleSet(LPC_TIMER16_0, (timerFreq/PWMFS)-1);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 0, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 1, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 2, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 3, 99);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_0, 3);
	LPC_TIMER16_0->PWMC = 7;
	Chip_TIMER_MatchEnableInt(LPC_TIMER16_0, 3);
	NVIC_SetPriority(TIMER_16_0_IRQn, 1);
	NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	Chip_TIMER_Reset(LPC_TIMER16_0);
	Chip_TIMER_Enable(LPC_TIMER16_0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, PWM_PORT, PWM_PIN, IOCON_FUNC2 ); /* PIO1_13 connected to MAT0 */
}

static void limitSWInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, LIMIT_SW_PORT, LIMIT_SW_L, LIMIT_SW_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, LIMIT_SW_PORT, LIMIT_SW_R, LIMIT_SW_IOCON_FUNC);

	/* Limit switches as falling edge interrupts */
	Chip_SYSCTL_SetPinInterrupt(3, LIMIT_SW_PORT, LIMIT_SW_L);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH3);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH3);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH3);
	NVIC_SetPriority(PIN_INT3_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT3_IRQn);
	NVIC_EnableIRQ(PIN_INT3_IRQn);

	Chip_SYSCTL_SetPinInterrupt(4, LIMIT_SW_PORT, LIMIT_SW_R);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH4);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH4);
	NVIC_SetPriority(PIN_INT4_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT4_IRQn);
	NVIC_EnableIRQ(PIN_INT4_IRQn);

	Chip_SYSCTL_SetPinInterrupt(5, LIMIT_SW_PORT, LIMIT_SW_L);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH5);
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH5);
	NVIC_SetPriority(PIN_INT5_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT5_IRQn);
	NVIC_EnableIRQ(PIN_INT5_IRQn);

	Chip_SYSCTL_SetPinInterrupt(6, LIMIT_SW_PORT, LIMIT_SW_R);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH6);
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH6);
	NVIC_SetPriority(PIN_INT6_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT6_IRQn);
	NVIC_EnableIRQ(PIN_INT6_IRQn);
}

/**
 * @brief 				DC motor control
 * @param power 		Motor power
 * @param fastbreak		True for fastbreak, false otherwise
 */
void motor(int16_t power, bool fastbreak)
{
	if (power > 100) power = 100;
	if (power < -100) power = -100;
	if (power < 0 && leftLimitStatus) {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 0);
	}
	else if (power > 0 && rightLimitStatus) {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 0);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);
	}
	else {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, fastbreak);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, fastbreak);
	}

	if ((power < 0 && !leftLimitStatus) || (power > 0 && !rightLimitStatus)) {
		resetCoef();
	}

	/* TODO Remove stub */
	//tempPower = power;

	power = (power < 0) ? -power : power;
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 0, 100 - power);
}

/**
 * @brief Motor encoder ISR handler
 */
void PIN_INT0_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);

	if (LPC_GPIO_PORT->B[ENCODER_PORT][ENCODER_CHB])
	{
		dataEncoder--;
	}
	else {
		dataEncoder++;
	}
}

/**
 * @brief Input encoder knob ISR handler
 */
void PIN_INT1_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);

	switch (currentView) {
	case OLED_CALIB:
		if (LPC_GPIO_PORT->B[ENCODER_PORT][CONTROL_CHB]) {
			if (calibModeActive < CALIB_MODES_CNT - 1) {
				calibModeActive++;
			}
		} else {
			if (calibModeActive > 0) {
				calibModeActive--;
			}
		}
		break;
	case OLED_DESKTOP:
		if (LPC_GPIO_PORT->B[ENCODER_PORT][CONTROL_CHB]) {
			if (menuModeActive < MENU_MODES_CNT - 1) {
				menuModeActive++;
			}
		} else {
			if (menuModeActive > 0) {
				menuModeActive--;
			}
		}
	case OLED_MANUAL:
		if (LPC_GPIO_PORT->B[ENCODER_PORT][CONTROL_CHB]) {
			if (manualModeActive < MANUAL_MODES_CNT - 1) {
				manualModeActive++;
			}
		} else {
			if (manualModeActive > 0) {
				manualModeActive--;
			}
		}
	case OLED_MANUAL_CONTROL:
		if (LPC_GPIO_PORT->B[ENCODER_PORT][CONTROL_CHB]) {
			if(dataControl < 40)
				dataControl++;
		} else {
			if(dataControl > -40)
				dataControl--;
		}
		break;
	}
}

/**
 * @brief Input encoder switch ISR handler
 */
void PIN_INT2_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);

	switch (currentView) {
	case OLED_CALIB:
		if (!moveLimit) {
			systemCalibrate(OLED_calibrate, calibModeActive);
			currentView = OLED_DESKTOP;
		}
		break;
	case OLED_DESKTOP:
		switch (menuModeActive) {
		case 0:
			currentView = OLED_MANUAL;
			break;
		case 1:
			break;
		case 2:
			break;
		}
		break;
	case OLED_MANUAL:
		switch (manualModeActive) {
		case 0:
			currentView = OLED_MANUAL_CONTROL;
			run = true;
			moveLimit = false;
			dataControl = 0;
			break;
		case 1:
			currentView = OLED_DESKTOP;
			break;
		}
		break;
	case OLED_MANUAL_CONTROL:
		currentView = OLED_MANUAL;
		run = false;
		moveLimit = true;
		break;
	}
}

void PIN_INT3_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH3);
	leftLimitStatus = false;
}

void PIN_INT4_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
	rightLimitStatus = false;
}

void PIN_INT5_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
	leftLimitStatus = true;
}

void PIN_INT6_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
	rightLimitStatus = true;
}

/**
 * @brief Timer ISR Handler
 */
void TIMER16_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER16_0, 3)) {
		Chip_TIMER_ClearMatch(LPC_TIMER16_0, 3);
		tdiv++;
		if(tdiv >= 100){
			tdiv = 0;
			motorVelocity= (dataEncoder - dataEncoder_1) / 48.0;
			dataEncoder_1 = dataEncoder;
		}
		if(run) {
			calcMotorPower();
		}
		else {
			resetCoef();
		}

		motorPower = (int) velU;
		motor(motorPower, 1);
	}
}

/**
 * @brief 	Application entry point
 * @return	Zero for success, non-zero for failure
 */
int main(void)
{
	/* Generic initialization */
	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, false);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PINT); /* PININT clock */

	/* TODO Remove stub */
	//SysTick_Config(SystemCoreClock / 500);

	velCoef.Kp = 10;
	velCoef.Ki = 0.1;
	velCoef.Kd = 0;
	arm_pid_init_f32(&velCoef, 1);

	posCoef.Kp = 0.05;
	posCoef.Ki = 0;
	posCoef.Kd = 5;
	arm_pid_init_f32(&posCoef, 1);

	motorPinsInit();
	encoderPinsInit();
	timerInit();
	limitSWInit();
	OLED_Init();
	OLED_Clear_Screen(0);
	updateLimitSWStatus();

	currentView = OLED_CALIB;

	while(1) {
		switch (currentView) {
		case OLED_DESKTOP:
			OLED_desktop();
			break;
		case OLED_CALIB:
			OLED_calibrate();
			break;
		case OLED_MANUAL:
			OLED_manual();
			break;
		case OLED_MANUAL_CONTROL:
			OLED_manualControl();
			break;
		}
	}
	return 0;
}
