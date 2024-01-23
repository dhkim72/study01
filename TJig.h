/*
 * TJig.h
 *
 *  Created on: 2023. 8. 22.
 *      Author: USER
 */

#ifndef SRC_TJIG_H_
#define SRC_TJIG_H_

//----------------------------------------------------------------------------------------------------------
#define FW_VERSION			1.00
#define expose_pause_time	30
#define EXPOSE_SW 			0x2
//#define acc_PT	// 누적 휴지시간 적용할 경우 활성화

//----------------------------------------------------------------------------------------------------------
#define BTN_ID			XPAR_GPIO_0_DEVICE_ID
#define LED_ID			XPAR_GPIO_1_DEVICE_ID
#define PWMEN_CTRL_ID		XPAR_GPIO_2_DEVICE_ID
#define POWER_CTRL_ID		XPAR_GPIO_3_DEVICE_ID
#define FV_FREQ_ID			XPAR_GPIO_4_DEVICE_ID
#define FV_PWM_ID			XPAR_GPIO_5_DEVICE_ID
#define HV_FREQ_ID			XPAR_GPIO_6_DEVICE_ID
#define HV_PWM_ID			XPAR_GPIO_7_DEVICE_ID

#define INTC_GPIO_INTERRUPT_ID		XPAR_INTC_0_GPIO_0_VEC_ID
#define INTC_DEVICE_ID				XPAR_INTC_0_DEVICE_ID
#define INTC						XIntc
#define INTC_HANDLER				XIntc_InterruptHandler
#define BUTTON_INTERRUPT			XGPIO_IR_CH2_MASK  /* Channel 2 Interrupt Mask */
#define TIMER0_DEVICE_ID			XPAR_AXI_TIMER_0_DEVICE_ID
#define TMRCTR_DEVICE_ID			XPAR_TMRCTR_0_DEVICE_ID
#define TMRCTR_INTERRUPT_ID			XPAR_INTC_0_TMRCTR_0_VEC_ID
#define UARTLITE_DEVICE_ID 		    XPAR_UARTLITE_0_DEVICE_ID
#define UARTLITE_INT_IRQ_ID     	XPAR_INTC_0_UARTLITE_0_VEC_ID
//#define SYSMON_DEVICE_ID			XPAR_SYSMON_0_DEVICE_ID
//#define SYSMON_INTR_ID			XPAR_INTC_0_SYSMON_0_VEC_ID

#define printf xil_printf 	/* Small foot-print printf function */
#define GPIO_CHANNEL1	1
#define GPIO_CHANNEL2	2
#define TIMER_CNTR_0	0
#define TIMER_CNTR_1	1
//#define RESET_VALUE	 0xFFFFD8EF // AXI_Timer_0 Counter value for 100us Interrupt : AXI Input CLK 100Mz, count 10000(FFFFFFFF-2710)
#define RESET_VALUE	 0xFFFFEC77 // AXI_Timer_0 Counter1 value for 100us Interrupt : AXI Input CLK 50Mz, count 5000(FFFFFFFF-1388)
//#define RESET_VALUE	 0xFFFFF63B // AXI_Timer_0 Counter value for 50us Interrupt : AXI Input CLK 50Mz, count 2500(FFFFFFFF-9C4)
//#define RESET_VALUE	 0xFFFFFC17 // AXI_Timer_0 Counter value for 20us Interrupt : AXI Input CLK 50Mz, count 1000(FFFFFFFF-3E8)
//#define RESET_VALUE	 0xFFF85EDF // AXI_Timer_0 Counter value for 10ms Interrupt : AXI Input CLK 50Mz, count 500000(FFFFFFFF-7A120)

#define TEST_BUFFER_SIZE        10
#define FB_BUFFER_SIZE        	10

//--------------------------------------------------------------------------------
// FREQ, PWM
#define FV_PWM_COUNT	500  // < 280KHz
#define FV_PWM_COUNT_HALF	FV_PWM_COUNT/2
#define HV_PWM_COUNT	500  // < 280KHz
#define HV_PWM_COUNT_HALF	HV_PWM_COUNT/2
#define BZ_PWM_COUNT	2000
#define BZ_PWM_COUNT_HALF	BZ_PWM_COUNT/2
#define LCD_PWM_COUNT	2000
#define LCD_PWM_COUNT_HALF	LCD_PWM_COUNT/2
#define STOP	1
#define RUN		0
#define unit100msec		556000
//#define HV_START_DUTY	10	//5%
#define HV_START_DUTY	2	//1%
#define FV_START_DUTY	10	//5%

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// HV module configuration : 63~77KV, 2mA 100mm corn > 130mR
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define HV_LOW_FREQ			2000	//HV_MIN = HV_DEFINE_FREQ - HV_LOW_FREQ(Hz)
#define HV_DEFINE_FREQ		45000
#define FV_DEFINE_FREQ		19000
#define HV_DUTY				10.0	//15.0%
#define FV_DUTY				30.0	//25.0%

#define FV_DUTY_DECREASE	1.0 	//step1(< 100ms) -1.5
#define FV_DUTY_DECREASE2	1.0 	//step2(> 100ms) FV_DUTY_DECREASE - FV_DUTY_DECREASE2
#define COOLING_STANDARD	150		//300:1
#define COOLING_TIME		180		//expose 0.3 : pause 90 sec (300:1) 120:1(90-36)=>54, 54*5 , 180초의 쿨링 총합이 필요하면

//define create instant object
XGpio BTN_Inst, LED_Inst, PWMEN_CTRL_Inst, POWER_CTRL_Inst;
XGpio FV_FREQ_Inst, FV_PWM_Inst, HV_FREQ_Inst, HV_PWM_Inst;
XTmrCtr TMR_Inst;
XUartLite UartLite;			/* The instance of the UartLite Device */
INTC Intc;

volatile u8 finish_check1, finish_check2, need_cooling_flag;
volatile static u8 expose_ready_ok, tRemote_expose, expose_enable, pause_time, expose_stop_signal, remote_flag;
volatile u16 set_time_data, test_time_data, last_end_time;
volatile static u16 flash_hv_freq, flash_fv_freq, flash_expose_pause_time;
volatile static u16 GlobalIntrMask; /* GPIO channel mask that is needed by the Interrupt Handler */
volatile static u32 SendBuffer[TEST_BUFFER_SIZE];
volatile static u32 ReceiveBuffer[TEST_BUFFER_SIZE];
volatile static int TotalReceivedCount, TotalSentCount, Expose_Count;
volatile static int accumulate_expose_time, pause_finish_time, pause_remaining_time, accumulate_pause_time ,rate_cal ;
volatile static float fv_pwmf, hv_pwmf, FV_dec_duty, FV_dec2_duty, flash_f_duty, heat_cmd, temp_preheat;
volatile static int fv_dec_count, fv_dec2_count, hv_dec_count, hv_add_count;
volatile static float flash_h_duty ;
volatile static u32 fv_pwm_cmd,hv_pwm_cmd, bottom_freq;
volatile static u32 fv_freq, hv_freq, fv_debug_freq;
volatile static u32 fv_status, hv_status;
volatile static u32 BeforeInputStatus, SW_EXPOSE_ON;
volatile static u32 preheat_time, power_stabilize, expose_time, expose_end_time, freq_out, preheat_end_time, flash_preheat_time;
volatile static u32 preheat_time_flag, power_stabilize_flag, expose_time_flag, expose_end_time_flag, preheat_end_time_flag;
volatile static u32 fast_preheat_enable, auto_expose_enable, production_process;
volatile static u32 HV_soft_start_enable, HV_soft_start_duty, HV_soft_duty, HV_dec_freq;
volatile static u32 led_blink, led_blink_status, blank_time;
volatile static int Timer_Interrupt, wait_time;
volatile static int TimerExpired, timer_1s, before_TimerExpired;
volatile static u32 btn_interval, intr_btn, timer_btn, blank_time;
volatile static u32 btn_Progress, BTN_press_time, enter_btn_time, btn_expose;
volatile int cal_f_duty, cal_h_duty;

//--------------------------------------------------------------------------------
//define function
int User_init(void);
void HVM_Freq_set(u32 MF, float MD, u32 FF, float f_duty);
void HV_Soft_Start_Duty(void);
void FV_HV_RUN_soft_start(void);
void Fast_preheat(void);
void Display_Info(void);
static int FloatToInt(float FloatNum);
void PowerOn(void);
void PowerOff(void);
void exposDisplay(u8 e_status);
void Expose_stop(void);
void Expose_stop_step2(void);
void PWM_CLK_enable(void);
void PWM_CLK_disable(void);
void Expose_flag_init(void);
void BTN_Handler(void *CallBackRef);
int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr, u16 DeviceId, u16 IntrId, u16 IntrMask);
void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,	u16 IntrId, u16 IntrMask);
int TmrCtrSetupIntrSystem(INTC* IntcInstancePtr,	XTmrCtr* InstancePtr, u16 DeviceId,	u16 IntrId,	u8 TmrCtrNumber);
void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber);
void TmrCtrDisableIntr(INTC* IntcInstancePtr, u16 IntrId);
int SetupInterruptSystem(XUartLite *UartLitePtr);
void SendHandler(void *CallBackRef, unsigned int EventData);
void RecvHandler(void *CallBackRef, unsigned int EventData);
void Receive_Control(u32 command_no);
void pause_process_1(void);
void pause_process_2(void);
void pause_process_3(void);

#endif /* SRC_TJIG_H_ */
