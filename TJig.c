/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
	Main Program Edit : 2023/08/22
	make by KDH
	XC7A35T-1, block ram(128K)
 */

#include <stdio.h>
#include "platform.h"
#include "xparameters.h"
#include "xgpio.h"
#include "xil_printf.h"
#include "xil_exception.h"
#include "xintc.h"
#include "xtmrctr.h"
#include "xuartlite.h"
#include "xstatus.h"
#include "math.h"
#include "TJig.h"

int main()
{
    init_platform();

	int Status;
    Status = User_init();
    if(Status != XST_SUCCESS){
    	return XST_FAILURE;
    }

    Display_Info();
    Expose_flag_init();		//Expose flag init
	XTmrCtr_Start(&TMR_Inst, TIMER_CNTR_0);

    while(1){

    	if((TimerExpired%100 == 0) && (finish_check2 == 0))
		{
			/* Test routine : interval 1s
			 *
			 */
			//xil_printf("\r\nDebug timer : %d",dc++);
			finish_check2 = 1;
		}else if((TimerExpired%100 != 0) && (finish_check2 == 1)){
			finish_check2 = 0;
		}
    }

    cleanup_platform();
    return 0;
}

void Display_Info(void)
{
	xil_printf("\r\n==================================================");
	xil_printf("\r\n TJig Version %d.%03d, XC7A35T-1 Date:2023.08.22",(int)FW_VERSION,FloatToInt(FW_VERSION));
	xil_printf("\r\n==================================================\r\n");
}

int FloatToInt(float FloatNum)
{
	float Temp;

	Temp = FloatNum;
	if (FloatNum < 0) {
		Temp = -(FloatNum);
	}

	return( ((int)((Temp -(float)((int)Temp)) * (1000.0f))));
}


int User_init(void)
{
	int Status ;
	int Index;

	//----------------------------------------------------
    // Initial GPIO
    //----------------------------------------------------

	// Initialize Control & Debug Buttons
    Status = XGpio_Initialize(&BTN_Inst, BTN_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

	// Initialize Status LEDs
    Status = XGpio_Initialize(&LED_Inst, LED_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize System PWM on/off
    Status = XGpio_Initialize(&PWMEN_CTRL_Inst, PWMEN_CTRL_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize System Power on/off
    Status = XGpio_Initialize(&POWER_CTRL_Inst, POWER_CTRL_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize FV_FREQ
    Status = XGpio_Initialize(&FV_FREQ_Inst, FV_FREQ_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize FV_PWM
    Status = XGpio_Initialize(&FV_PWM_Inst, FV_PWM_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize HV_FREQ
    Status = XGpio_Initialize(&HV_FREQ_Inst, HV_FREQ_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Initialize HV_PWM
    Status = XGpio_Initialize(&HV_PWM_Inst, HV_PWM_ID);
    if(Status != XST_SUCCESS)
    	return XST_FAILURE;

    // Set LEDs direction to outputs
    XGpio_SetDataDirection(&LED_Inst, GPIO_CHANNEL1, 0);
    XGpio_SetDataDirection(&LED_Inst, GPIO_CHANNEL2, 0);
	XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL2, 0x7);

    // Set all buttons direction to inputs
    XGpio_SetDataDirection(&BTN_Inst, GPIO_CHANNEL1, 1); // Button input

    // Set freq direction to outputs
    XGpio_SetDataDirection(&FV_FREQ_Inst, GPIO_CHANNEL1, 0);
    XGpio_SetDataDirection(&FV_FREQ_Inst, GPIO_CHANNEL2, 0);
    XGpio_SetDataDirection(&FV_PWM_Inst, GPIO_CHANNEL1, 0);
    XGpio_SetDataDirection(&FV_PWM_Inst, GPIO_CHANNEL2, 0);

    // Set HV freq direction to outputs
    XGpio_SetDataDirection(&HV_FREQ_Inst, GPIO_CHANNEL1, 0);
    XGpio_SetDataDirection(&HV_FREQ_Inst, GPIO_CHANNEL2, 0);
    XGpio_SetDataDirection(&HV_PWM_Inst, GPIO_CHANNEL1, 0);
    XGpio_SetDataDirection(&HV_PWM_Inst, GPIO_CHANNEL2, 0);

    /*
	 * Initialize the interrupt controller driver so that it's ready to use.
	 * specify the device ID that was generated in xparameters.h
	 */
    Status = XIntc_Initialize(&Intc, INTC_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return Status;
	}

	Status = XIntc_Start(&Intc, XIN_REAL_MODE);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    //----------------------------------------------------
    // SETUP THE BUTTON INTERRUPT
    //----------------------------------------------------

	Status = GpioSetupIntrSystem(&Intc, &BTN_Inst, BTN_ID,
			INTC_GPIO_INTERRUPT_ID, GPIO_CHANNEL1);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

    //----------------------------------------------------
    // SETUP THE UARTLITE 0
    //----------------------------------------------------
	Status = XUartLite_Initialize(&UartLite, UARTLITE_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = SetupInterruptSystem(&UartLite);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Setup the handlers for the UartLite that will be called from the
	 * interrupt context when data has been sent and received, specify a
	 * pointer to the UartLite driver instance as the callback reference so
	 * that the handlers are able to access the instance data.
	 */
	XUartLite_SetSendHandler(&UartLite, SendHandler, &UartLite);
	XUartLite_SetRecvHandler(&UartLite, RecvHandler, &UartLite);

 	/*
	 * Initialize the send buffer bytes with a pattern to send and the
	 * the receive buffer bytes to zero to allow the receive data to be
	 * verified.
	 */
	for (Index = 0; Index < TEST_BUFFER_SIZE; Index++) {
		SendBuffer[Index] = Index;
		ReceiveBuffer[Index] = 0;
	}

	/*
	 * Enable the interrupt of the UartLite so that interrupts will occur.
	 */
	XUartLite_EnableInterrupt(&UartLite);


	//----------------------------------------------------
    // SETUP THE TIMER0 & Interrupt
    //----------------------------------------------------

	Status = XTmrCtr_Initialize(&TMR_Inst, TIMER0_DEVICE_ID);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	Status = TmrCtrSetupIntrSystem(&Intc, &TMR_Inst, TIMER0_DEVICE_ID,
					TMRCTR_INTERRUPT_ID,
					TIMER_CNTR_0);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XTmrCtr_SetHandler(&TMR_Inst, TimerCounterHandler,&TMR_Inst);

	XTmrCtr_SetOptions(&TMR_Inst, TIMER_CNTR_0,
				XTC_INT_MODE_OPTION | XTC_AUTO_RELOAD_OPTION);

	XTmrCtr_SetResetValue(&TMR_Inst, TIMER_CNTR_0, RESET_VALUE);

	flash_hv_freq = HV_DEFINE_FREQ/1000;
	flash_h_duty = HV_DUTY;
	cal_h_duty = flash_h_duty * 10;
	flash_fv_freq = FV_DEFINE_FREQ/1000;
	flash_f_duty = FV_DUTY;
	cal_f_duty = flash_f_duty * 10;

	flash_expose_pause_time = 30;
	set_time_data = 10; 		//100ms
	//flash_preheat_time = 34; 	//340ms
	flash_preheat_time = 100; 	//1000ms
	last_end_time = 10; 		//100ms
	auto_expose_enable = 0;
	production_process = 0; //normal mode
	expose_ready_ok = 1;

	HV_soft_start_enable = 0;
	fast_preheat_enable = 0;
	power_stabilize_flag = 0;

	return XST_SUCCESS;
}

int GpioSetupIntrSystem(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 DeviceId, u16 IntrId, u16 IntrMask)
{
	GlobalIntrMask = IntrMask;

	/* Hook up interrupt service routine */
	XIntc_Connect(IntcInstancePtr, IntrId,
		      (Xil_ExceptionHandler)BTN_Handler, InstancePtr);

	XIntc_Enable(IntcInstancePtr, IntrId);

	/*
	 * Enable the GPIO channel interrupts so that push button can be
	 * detected and enable interrupts for the GPIO device
	 */
	XGpio_InterruptEnable(InstancePtr, IntrMask);
	XGpio_InterruptGlobalEnable(InstancePtr);

	/*
	 * Initialize the exception table and register the interrupt
	 * controller handler with the exception table
	 */
	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)INTC_HANDLER, IntcInstancePtr);

	/* Enable non-critical exceptions */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

void GpioDisableIntr(INTC *IntcInstancePtr, XGpio *InstancePtr,
			u16 IntrId, u16 IntrMask)
{
	XGpio_InterruptDisable(InstancePtr, IntrMask);
	XIntc_Disable(IntcInstancePtr, IntrId);
	return;
}

int TmrCtrSetupIntrSystem(INTC* IntcInstancePtr,
				 XTmrCtr* TmrCtrInstancePtr,
				 u16 DeviceId,
				 u16 IntrId,
				 u8 TmrCtrNumber)
{
	 int Status;

	Status = XIntc_Connect(IntcInstancePtr, IntrId,
				(XInterruptHandler)XTmrCtr_InterruptHandler,
				(void *)TmrCtrInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIntc_Enable(IntcInstancePtr, IntrId);

	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
					(Xil_ExceptionHandler)
					INTC_HANDLER,
					IntcInstancePtr);

	Xil_ExceptionEnable();
	return XST_SUCCESS;
}

void TmrCtrDisableIntr(INTC* IntcInstancePtr, u16 IntrId)
{
	XIntc_Disable(IntcInstancePtr, IntrId);
}

void SendHandler(void *CallBackRef, unsigned int EventData)
{
	TotalSentCount = EventData;
}

void RecvHandler(void *CallBackRef, unsigned int EventData)
{
	u32 command_no;
	TotalReceivedCount = EventData;
	ReceiveBuffer[0]=XUartLite_RecvByte(XPAR_UARTLITE_0_BASEADDR);
	command_no = ReceiveBuffer[0];
	Receive_Control(command_no);
}

int SetupInterruptSystem(XUartLite *UartLitePtr)
{
	int Status;

	Status = XIntc_Connect(&Intc, UARTLITE_INT_IRQ_ID,
			   (XInterruptHandler)XUartLite_InterruptHandler,
			   (void *)UartLitePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	XIntc_Enable(&Intc, UARTLITE_INT_IRQ_ID);

	Xil_ExceptionInit();

	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
			 (Xil_ExceptionHandler)XIntc_InterruptHandler,
			 &Intc);

	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

void HVM_Freq_set(u32 MF, float MD, u32 FF, float FD)
{
	//----------------------------------------------------
	// SETUP MF_FREQ
	//----------------------------------------------------
	if(MF>250000) MF = 250000;
	else if(MF<10000) MF = 10000;
    if(MD>49.0) MD=49.0; //duty < 49.0
	else if(MD<1.0) MD=1.0; //duty < 1.0

	bottom_freq = HV_LOW_FREQ;
	hv_freq = HV_PWM_COUNT * MF;
	hv_pwmf = MD*2.0; // 반주기에 대해 100% 기준으로 2배를 하여야 한 주기에 대한 %값이 됨

	hv_pwm_cmd=(u32)((hv_pwmf/100.0f)*(float)(HV_PWM_COUNT_HALF));
    hv_status=STOP;

    HV_dec_freq = HV_PWM_COUNT * (MF - bottom_freq);

    XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL1, hv_status);
    //XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL2, HV_dec_freq);
    XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL2, hv_freq);
    XGpio_DiscreteWrite(&HV_PWM_Inst, GPIO_CHANNEL1, HV_PWM_COUNT);
    XGpio_DiscreteWrite(&HV_PWM_Inst, GPIO_CHANNEL2, hv_pwm_cmd);

    //----------------------------------------------------
	// SETUP FF_FREQ
	//----------------------------------------------------

	if(FF>250000) FF = 250000;
	else if(FF<10000) FF = 10000;
    if(FD>49.0) FD=49.0; //duty < 49.0
	else if(FD<1.0) FD=1.0; //duty < 1.0

	fv_freq = FV_PWM_COUNT * FF;
	fv_pwmf = FD*2.0; // 반주기에 대해 100% 기준으로 2배를 하여야 한 주기에 대한 %값이 됨

	fv_pwm_cmd=(u32)((fv_pwmf/100.0f)*FV_PWM_COUNT_HALF);
    fv_status=STOP;

    FV_dec_duty = FD - FV_DUTY_DECREASE;
    //FV_dec2_duty = FD - (FV_DUTY_DECREASE + FV_DUTY_DECREASE2);
    FV_dec2_duty = FD - FV_DUTY_DECREASE2;

    XGpio_DiscreteWrite(&FV_FREQ_Inst, GPIO_CHANNEL1, fv_status);
    XGpio_DiscreteWrite(&FV_FREQ_Inst, GPIO_CHANNEL2, fv_freq);
    XGpio_DiscreteWrite(&FV_PWM_Inst, GPIO_CHANNEL1, FV_PWM_COUNT);
    XGpio_DiscreteWrite(&FV_PWM_Inst, GPIO_CHANNEL2, fv_pwm_cmd);

	fv_dec_count = 0;
	hv_dec_count = 0;
	hv_add_count = 0;
}


void BTN_Handler(void *CallbackRef)
{
	XGpio *GpioPtr = (XGpio *)CallbackRef;
	XGpio_InterruptDisable(&BTN_Inst, BUTTON_INTERRUPT);

	intr_btn = XGpio_DiscreteRead(&BTN_Inst, GPIO_CHANNEL1);
	if(intr_btn == 0) BTN_press_time=0;

	if(BeforeInputStatus != intr_btn)
	{
		//xil_printf("\r\nInput G1: %02x\tG2: %02x",intr_btnG1, intr_btnG2);
		btn_Progress = TRUE;
	    finish_check1 = 0;
		PWM_CLK_enable();

		if((intr_btn == 0)&&(btn_expose == 1))
		{
			preheat_time = power_stabilize+flash_preheat_time;
			expose_time = preheat_time+set_time_data;
			Expose_stop();
			remote_flag = 0;
			expose_enable = 0;
			tRemote_expose = 0;
			finish_check2 = 0;
			btn_expose = 0;
		}

		if((intr_btn == EXPOSE_SW) && (expose_ready_ok == 1)) //조사 스위치 루틴
		{
			if(auto_expose_enable > 0){
				if(expose_enable == 0){
					pause_time = 1;
					timer_1s = 0;
					expose_enable = 1;
				}else{
					expose_enable = 0;
				}
			}
		}

		BeforeInputStatus = intr_btn;
		btn_Progress = FALSE;
	}

	/* Clear the Interrupt */
	XGpio_InterruptClear(GpioPtr, GlobalIntrMask);
	// Enable GPIO interrupts
	XGpio_InterruptEnable(&BTN_Inst, BUTTON_INTERRUPT);
}

void Receive_Control(u32 command_no)
{
	switch(command_no){

		case '1' : //HV freq up 1KHz
					if(flash_hv_freq<250){
						flash_hv_freq+=1;
						xil_printf("\r\nMF %d(KHz)",flash_hv_freq);
					}
			break;
		case '2' : //HV freq down 1KHz
					if(flash_hv_freq>2){
						flash_hv_freq-=1;
						xil_printf("\r\nMF %d(KHz)",flash_hv_freq);
					}
			break;
		case '3' : //HV pwm up
					if(cal_h_duty<490){
						cal_h_duty+=1;
						flash_h_duty = cal_h_duty/10.0;
						xil_printf("\r\nMD %d.%03d(%%)",(int)flash_h_duty,FloatToInt(flash_h_duty));
					}
			break;
		case '4' : //HV pwm down
					if(cal_h_duty>1){
						cal_h_duty-=1;
						flash_h_duty = cal_h_duty/10.0;
						xil_printf("\r\nMD %d.%03d(%%)",(int)(flash_h_duty),FloatToInt(flash_h_duty));
					}
			break;
		case '5' : //FV freq up 1KHz
					if(flash_fv_freq<250){ //MAX input clk / 2 , ap_clk : 268MHz
						flash_fv_freq+=1;
						xil_printf("\r\nFF %d(KHz)",flash_fv_freq);
					}
			break;
		case '6' : //FV_freq down 1KHz
					if(flash_fv_freq>2){
						flash_fv_freq-=1;
						xil_printf("\r\nFF %d(KHz)",flash_fv_freq);
					}
			break;
		case '7' : //FV pwm up
					if(cal_f_duty<490){
						cal_f_duty+=1;
						flash_f_duty=cal_f_duty/10.0;
						xil_printf("\r\nFD %d.%03d(%%)",(int)flash_f_duty,FloatToInt(flash_f_duty));
					}
			break;
		case '8' : //FV_pwm down
					if(cal_f_duty>1){
						cal_f_duty-=1;
						flash_f_duty=cal_f_duty/10.0;
						xil_printf("\r\nFD %d.%03d(%%)",(int)flash_f_duty,FloatToInt(flash_f_duty));
					}
			break;
		case '&' : //FV_pwm = 17%
					cal_f_duty = 170;
					flash_f_duty=cal_f_duty/10.0;
					xil_printf("\r\nFD %d.%03d(%%)",(int)flash_f_duty,FloatToInt(flash_f_duty));
			break;

		case '*' : //FV_pwm = 49%
					cal_f_duty = 490;
					flash_f_duty=cal_f_duty/10.0;
					xil_printf("\r\nFD %d.%03d(%%)",(int)flash_f_duty,FloatToInt(flash_f_duty));
			break;

		case '9' : //on time +
					if(set_time_data < 150) set_time_data += 1;
					xil_printf("\r\nExpose %d(ms)",set_time_data*10);
					break;

		case '0' : //on time -
					if(set_time_data >= 2) set_time_data -= 1;
					xil_printf("\r\nExpose %d(ms)",set_time_data*10);
					break;

		case 'p' : //preheat time +
					if(flash_preheat_time < 200) flash_preheat_time += 1;
					xil_printf("\r\nPreheat %d(ms)",flash_preheat_time*10);
			break;
		case 'o' : //preheat time -
					if(flash_preheat_time > 0) flash_preheat_time -= 1;
					xil_printf("\r\nPreheat %d(ms)",flash_preheat_time*10);
					break;
		case 'i' : //방전 time +
					if(last_end_time < 150) last_end_time += 1;
					xil_printf("\r\nDischarge %d(ms)",last_end_time*10);
			break;
		case 'u' : //방전 time -
					if(last_end_time > 0) last_end_time -= 1;
					xil_printf("\r\nDischarge %d(ms)",last_end_time*10);
					break;

		case 13 : //'enter' expose
					if(pause_time == 0){
						remote_flag = 1;
						expose_enable = 1;
						xil_printf("\r\nRemote Expose...");
					}else{
						xil_printf("\r\nCooling. wait");
					}
			break;

		case 'v' : //production process
					if(production_process == 0){
						production_process = 1;
						xil_printf("\r\nSetup");
					}else{
						production_process = 0;
						xil_printf("\r\nNormal");
					}
			break;

		case ',' : //0.1 sec
					set_time_data = 10;
					xil_printf("\r\nExpose %d(ms)",set_time_data*10);
					break;

		case '.' : //0.3 sec
					set_time_data = 30;
					xil_printf("\r\nExpose %d(ms)",set_time_data*10);
					break;

		case '/' : //1sec
					set_time_data = 100;
					xil_printf("\r\nExpose %d(ms)",set_time_data*10);
					break;

		case 'm' : //display memory
					Display_Info();
					xil_printf("\r\nHV Freq %d kHz",flash_hv_freq);
					xil_printf("\tHV Duty %d.%03d(%%)", (int)(flash_h_duty), FloatToInt(flash_h_duty));
					xil_printf("\r\nFV_Freq %d kHz",flash_fv_freq);
					xil_printf("\tFV Duty %d.%03d(%%)",(int)(flash_f_duty), FloatToInt(flash_f_duty));
					xil_printf("\r\nPreheat %d ms",flash_preheat_time*10);
					xil_printf("  Expose %d ms",set_time_data*10);
					xil_printf("  Discharge %d ms",last_end_time*10);
					//Measure_property();//ADC 'system_dc_adc'
					//xil_printf("\r\nTube temperature:%d.%03d Centigrade(H:%d.%03d|L:%d.%03d), SYSTEM POWER:%d.%03d Volt.",
					//			(int)emc_adc_data->tube_temperature,FloatToInt(emc_adc_data->tube_temperature),
					//			(int)(flash_Ht_limit/10.0),FloatToInt(flash_Ht_limit/10.0),(int)(flash_Lt_limit/10.0),FloatToInt(flash_Lt_limit/10.0),
					//			(int)emc_adc_data->syspower_volt_Data,FloatToInt(emc_adc_data->syspower_volt_Data));

					if(production_process == 0) xil_printf("\r\nNormal mode");
					else xil_printf("\r\nSetup mode");
					xil_printf("\tTotal Expose Count : %d",Expose_Count);

					break;

		case '@' : //Auto Expose Test
					if(auto_expose_enable <= 80){
						auto_expose_enable += 30;
						expose_enable = 1;
						timer_1s = 0;
						print("\r\nAging Start...\n\r");
					}else{
						auto_expose_enable = 0;
						expose_enable = 0;
						print("\r\nAging Stop...\n\r");
					}
					break;

		case '?' : //Help
					xil_printf("\r\n\r\n****************** Command List - Jig Controller %d.%03d ********************",(int)FW_VERSION, FloatToInt(FW_VERSION));
					xil_printf("\r\n 1\t: MF up +1khz\t\t\t2\t: MF down -1khz");
					xil_printf("\r\n 3\t: MD up +0.1%%\t\t\t4\t: MD down -0.1%%");
					xil_printf("\r\n 5\t: FF up +1khz\t\t\t6\t: FF down -1khz");
					xil_printf("\r\n 7\t: FD up +0.1%%\t\t\t8\t: FD down -0.1%%");
					xil_printf("\r\n 9\t: Expose time +100ms\t\t0\t: Expose time -100ms");
					xil_printf("\r\n p\t: Preheat time +100ms\t\to\t: Preheat time -100ms");
					xil_printf("\r\n i\t: Discharge time +100ms\t\tu\t: Discharge time -100ms");
					xil_printf("\r\n v\t: Toggle setup mode\t\t,\t: Expose time 100ms");
					xil_printf("\r\n .\t: Expose time 300ms\t\t/\t: Expose time 1sec");
					xil_printf("\r\n &\t: FD 49.0%%\t\t\t*\t: FD 17.0%%");
					xil_printf("\r\n m\t: Display param.\t\t@\t: Auto Expose 30/60/90/stop");
					xil_printf("\r\n ?\t: help");
					xil_printf("\r\n***************************************************************************\r\n");
					break;
		default : break;

	}
}

void TimerCounterHandler(void *CallBackRef, u8 TmrCtrNumber)
{
	XTmrCtr *InstancePtr = (XTmrCtr *)CallBackRef;

	if (XTmrCtr_IsExpired(InstancePtr, TmrCtrNumber))
	{
		Timer_Interrupt++; //100us

		//if(FV_soft_start_enable) FV_Soft_Start_Duty();

		//if(HV_soft_start_enable) HV_Soft_Start_Duty();//2023.10.26 kdh 1~20%(3ms)
		if((HV_soft_start_enable)&&(Timer_Interrupt%2 == 0)) HV_Soft_Start_Duty();//2023.10.26 kdh 1~20%(6ms)

		if((Timer_Interrupt%100) == 0) //10ms : 100us*100
		{
			TimerExpired++;

			if(fast_preheat_enable) Fast_preheat();
			//if(FV_deceleration_enable) FV_deceleration_Duty();		// < 100ms
			//if(HV_add_enable) HV_add_Freq();							// < 100ms
			//if(HV_deceleration_enable) HV_deceleration_Freq();		// > 100ms
			//if(FV_deceleration2_enable) FV_deceleration2_Duty();		// > 100ms

			if(SW_EXPOSE_ON == 0)
			{
				if(TimerExpired%2==0)
				{
					//display_manager();
				}
			}

			if(TimerExpired%100 == 0) //1s : 10ms*100
			{
				// auto expose for test ---------------------------------------------------//
				if(((auto_expose_enable > 0)||(remote_flag == 1)) && (expose_enable == 1))
				{
					if( ((remote_flag==1) || ((timer_1s!=0)&&(timer_1s == auto_expose_enable))) && (finish_check1==0) )
					{
						SW_EXPOSE_ON = 1;
						// Expose time initial
						power_stabilize = 0;
						preheat_time = 0;
						expose_time = 0;
						expose_end_time = 0;
						remote_flag = 0;

						if(production_process == 1) rate_cal = 3;// 테스트 모드일 때는 휴지시간 3sec
						else rate_cal = (int)((float)set_time_data*(float)expose_pause_time*0.01);

						//Measure_property();//ADC 'system_dc_adc'
						/**********************************************************************************************************/
						//if(production_process == 0) bat_remain_check(); // bat level 보상
						//else HVM_Freq_set(flash_hv_freq*1000, flash_h_duty, flash_fv_freq*1000, flash_f_duty); // for test Jig
						/**********************************************************************************************************/

						HVM_Freq_set(flash_hv_freq*1000, flash_h_duty, flash_fv_freq*1000, flash_f_duty); // for test Jig

						PowerOn();
						tRemote_expose = 1; // expose restart after pause time
						if(auto_expose_enable > 0) pause_time = 0;
						pause_process_1();
					}

					if(auto_expose_enable > 0){
						if(timer_1s == auto_expose_enable){
							timer_1s = 0;
							pause_time = 0;
						}
						xil_printf("\r\nAuto expose : %d", auto_expose_enable-timer_1s);
					}
				// auto expose for test end -----------------------------------------------//
				}else if(pause_time != 0){ // pause time -----------------------------------//
					pause_process_2();
					pause_process_3();
					//printf("\r\nDebug point1, timer_1s: %d, rate_cal: %d", timer_1s, rate_cal);
				}// pause time end ---------------------------------------------------------//

				timer_1s++;
				if(accumulate_pause_time > 0){
					accumulate_pause_time--;
					//xil_printf("\r\nDebug1 Cooling flag:%d, Accumulate_Pause_Time:%d",need_cooling_flag,accumulate_pause_time);
				}
			}
		}
	}

	if (TimerExpired != before_TimerExpired) //10ms interval
	{
		before_TimerExpired = TimerExpired;

		if(!btn_Progress) btn_interval++; //10ms interval ++
			//xil_printf("btn_interval %d\r\n",btn_interval);
		if(TimerExpired%20 == 0) // Status LED Display by 0.2s
		{
			blank_time++;
			if(blank_time%2 == 0) led_blink_status = (led_blink_status&0xFE) ;
			else led_blink_status = (led_blink_status|0x1) ;
			XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL1, led_blink_status);
		}

		if((TimerExpired > 10)&&(expose_stop_signal == 1))// power off signal on-> 100ms -> off
		{
			Expose_stop_step2();
			expose_stop_signal = 0;
		}

		// expose routine ---------------------------------------------------------//
		if(tRemote_expose && (expose_ready_ok == 1))// Auto Expose
		{
			if(pause_time == 0)
			{
				FV_HV_RUN_soft_start();
			}
		}else{
			if(SW_EXPOSE_ON == 1)
			{
				Expose_stop();
				SW_EXPOSE_ON = 0;
			}
		}
		// expose routine end -----------------------------------------------------//
	}

	if((btn_interval>25)&&(SW_EXPOSE_ON == 0)) // 250ms
	{
		btn_interval=0;
		timer_btn = XGpio_DiscreteRead(&BTN_Inst, GPIO_CHANNEL1);
		if(enter_btn_time > 0) enter_btn_time-- ;

		if(timer_btn!=0){
			BTN_press_time++;
			//xil_printf("\r\nbutton press time %d",BTN_press_time);

			if((timer_btn == EXPOSE_SW) && (expose_ready_ok == 1))
			{
				if((pause_time == 0)&&(finish_check1 == 0)){
					if(intr_btn != 0){
						if(btn_expose == 0){
							remote_flag = 1;
							expose_enable = 1;
							btn_expose = 1;
							xil_printf("\r\nTbtn Expose.");
						}
					}else{
						remote_flag = 0;
						expose_enable = 0;
						tRemote_expose = 0;
						xil_printf("\r\nNot ready.");
					}
				}else{
					xil_printf("\r\nCooling. wait");
				}

			}

		}else{
			BTN_press_time=0;
		}
	}
}

void pause_process_1(void)
{
	if(production_process == 0){
		rate_cal = (int)(set_time_data*flash_expose_pause_time*0.01);
		accumulate_pause_time += (int)(set_time_data*COOLING_STANDARD*0.01); //누적 조사 시간 - 기본 휴지시간
	}else{
		rate_cal = 3;// 테스트 모드일 때는 휴지시간 3sec
		accumulate_pause_time = (int)(set_time_data*COOLING_STANDARD*0.01);
	}

	timer_1s = 0;
	finish_check1 = 1;
}

void pause_process_2(void)
{
	if(accumulate_pause_time > COOLING_TIME){ //필요한 쿨링의 누적 시간이 5분이상이 되면

#ifndef acc_PT
		need_cooling_flag = 0;  //누적시간 적용하지 않음
#else
		need_cooling_flag = 1;	//1sec 3회 연속 조사 후 5분 휴지시간
		xil_printf("\r\nCooling Activation:%d, Accumulate_Pause_Time:%d",need_cooling_flag,accumulate_pause_time);
#endif

	}else if(accumulate_pause_time < 1){
		need_cooling_flag = 0;
	}
}

void pause_process_3(void)
{
	if(need_cooling_flag == 1)
	{
		if(timer_1s > COOLING_TIME) // 휴지시간
		{
			need_cooling_flag = 0;
			pause_time = 0;
			timer_1s = 0;
			accumulate_pause_time = 0;
		}else{
			xil_printf("\r\nCooling... %d",COOLING_TIME - timer_1s);
		}
	}else{
		if(timer_1s > rate_cal)
		{
			pause_time = 0;
			timer_1s = 0;
		}else{
			xil_printf("\r\nPause... %d",rate_cal-timer_1s);
		}
	}
}

void Expose_stop(void)
{
	hv_status=STOP;
    XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL1, hv_status);

    expose_stop_signal = 1;
    fast_preheat_enable = 0;
}

void Expose_stop_step2(void)
{
	fv_status=STOP;
	XGpio_DiscreteWrite(&FV_FREQ_Inst, GPIO_CHANNEL1, fv_status);

	exposDisplay(0); //expos off Display

	PowerOff();

	pause_time = 1; // 종료시 휴지시간 적용

	// 주파수 초기화
    //XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL2, hv_freq);
    XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL2, HV_dec_freq);
	fv_pwm_cmd=(u32)((fv_pwmf/100.0f)*(float)(FV_PWM_COUNT_HALF));
	XGpio_DiscreteWrite(&FV_PWM_Inst, 2, fv_pwm_cmd);

	// 조사 관련 변수 초기화
	SW_EXPOSE_ON = 0;
	remote_flag = 0;
	power_stabilize = 0;
	preheat_time = 0;
	expose_time = 0;
	expose_end_time = 0;

	power_stabilize_flag = 0;
	preheat_time_flag = 0;
	expose_time_flag = 0;
	expose_end_time_flag = 0;
	HV_soft_start_enable = 0;
	HV_soft_start_duty = HV_START_DUTY;
	fast_preheat_enable = 0;
    fv_dec_count=0;
    hv_dec_count=0;
	hv_add_count = 0;
}

void FV_HV_RUN_soft_start(void)
{
	if(power_stabilize < 1) //power_stabilize //10ms
	{
		power_stabilize++;
	}else{
		if(power_stabilize_flag == 0)
		{
			//fast_preheat_enable = 1 ;
			//fv_dec2_count = 0;
			//temp_preheat = flash_f_duty * 2.0; // start시 필라멘트 듀티의 2배 셋업
			temp_preheat = flash_f_duty; // start시 필라멘트 듀티 셋업

			if(temp_preheat > 49.0) temp_preheat = 49.0;
			fv_pwm_cmd=(u32)((temp_preheat*2.0/100.0f)*FV_PWM_COUNT_HALF);
			XGpio_DiscreteWrite(&FV_PWM_Inst, GPIO_CHANNEL2, fv_pwm_cmd);
			// FV Start Function End

			fv_status=RUN;
			XGpio_DiscreteWrite(&FV_FREQ_Inst, GPIO_CHANNEL1, fv_status);

			power_stabilize_flag = 1;
			//xil_printf("power_stabilize: %d ms\r\n",power_stabilize*10);
		}
	}

	if(preheat_time < (power_stabilize+flash_preheat_time)) //flash_preheat_time
	{
		preheat_time++;
	}else{
		if(preheat_time_flag == 0)
		{
			// Enable HV Soft Start Function
			HV_soft_start_enable = 1;
			HV_soft_start_duty = HV_START_DUTY;
			hv_pwm_cmd=(HV_soft_start_duty/100.0f)*HV_PWM_COUNT_HALF;

		    XGpio_DiscreteWrite(&HV_PWM_Inst, GPIO_CHANNEL2, (int)hv_pwm_cmd);
			// HV Start Function End

		    //--------------------------------------------
		    exposDisplay(1); // Expose Icon On

			hv_status=RUN;
			XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL1, hv_status);

			preheat_time_flag = 1;
			//xil_printf("preheat_time: %d ms\r\n",(preheat_time-power_stabilize)*10);
		}
	}

	if(expose_time<(preheat_time+set_time_data)) // 100ms~1500ms
	{
		expose_time++;
		//xil_printf("\r\nExpose:%d, preheat:%d, FV_deceleration_enable:%d",expose_time, preheat_time, FV_deceleration_enable);
	}else{
		if(expose_time_flag == 0)
		{
			hv_status=STOP;
			exposDisplay(0); // Expose Icon Off
			XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL1, hv_status);

			expose_time_flag = 1;
			//xil_printf("Expose_time: %d ms\r\n",(expose_time-preheat_time)*10);
		}
	}

	//if(expose_end_time<(expose_time+10)) // 100ms
	if(expose_end_time<(expose_time+last_end_time))
	{
		expose_end_time++;

	}else{
		if(expose_end_time_flag == 0)
		{
		    fv_status=STOP;
			XGpio_DiscreteWrite(&FV_FREQ_Inst, GPIO_CHANNEL1, fv_status);

			// HV freq init.
			XGpio_DiscreteWrite(&HV_FREQ_Inst, GPIO_CHANNEL2, HV_dec_freq);
			// FV freq init.
			fv_pwm_cmd=(u32)((fv_pwmf/100.0f)*(float)(FV_PWM_COUNT_HALF));
			XGpio_DiscreteWrite(&FV_PWM_Inst, 2, fv_pwm_cmd);

			PowerOff();

			tRemote_expose = 0; // for auto expose test

			finish_check1=0;
			pause_time = 1; // 정상 종료 일 때만 휴지시간 허용
			expose_end_time_flag = 1;
			fast_preheat_enable = 0;
			Expose_Count = Expose_Count+1 ;
		}
	}
}

void Fast_preheat(void)
{
	float fv_temp1, fv_temp2 ;
	float heat_cmd, out_value ;

	heat_cmd = flash_f_duty*2.0;

	if (heat_cmd > 49.0) heat_cmd = 49.0;
	fv_dec2_count++;

	fv_temp2 = ((heat_cmd - (flash_f_duty))/flash_preheat_time);//step value to divide by interval

	out_value = heat_cmd - (fv_temp2 * fv_dec2_count);

	if( out_value > (flash_f_duty) )
	{
		fv_temp1 = ((out_value*2.0/100.0f)*FV_PWM_COUNT_HALF);
		XGpio_DiscreteWrite(&FV_PWM_Inst, GPIO_CHANNEL2, fv_temp1);
		//xil_printf("fv_temp1:%d,fv_dec_count%d\r\n",(int)fv_temp1,fv_dec2_count);
	}else{
		fv_pwm_cmd=(u32)(((flash_f_duty*2.0)/100.0f)*(float)(FV_PWM_COUNT_HALF));
		XGpio_DiscreteWrite(&FV_PWM_Inst, GPIO_CHANNEL2, fv_pwm_cmd);
		fast_preheat_enable = 0;
		fv_dec2_count=0;
	}
}

void HV_Soft_Start_Duty(void)
{
	//------------------------------------------------------------------
	//HV Duty Soft Start option code
	//------------------------------------------------------------------
	if(HV_soft_start_duty < hv_pwmf)
	{
		if(HV_soft_start_duty < (hv_pwmf-2))
		{
			HV_soft_duty = hv_pwmf -(hv_pwmf - (HV_soft_start_duty+=2));
		}else HV_soft_duty = hv_pwmf -(hv_pwmf - HV_soft_start_duty++);

	    hv_pwm_cmd=(u32)((HV_soft_duty/100.0f)*(float)(HV_PWM_COUNT_HALF));
	    XGpio_DiscreteWrite(&HV_PWM_Inst, GPIO_CHANNEL2, hv_pwm_cmd);
	}
	//setup last HV duty
	if(HV_soft_start_duty >= hv_pwmf)
	{
	    hv_pwm_cmd=(u32)((hv_pwmf/100.0f)*(float)(HV_PWM_COUNT_HALF));
	    XGpio_DiscreteWrite(&HV_PWM_Inst, GPIO_CHANNEL2, hv_pwm_cmd);
	    HV_soft_start_enable = 0;
	}
}

void Expose_flag_init(void)
{
	power_stabilize = 0;
	preheat_time = 0;
	expose_time = 0;
	expose_end_time = 0;
	preheat_end_time = 0;
	pause_time = 0;
	finish_check1=0;
}

void PowerOn(void)
{
	//12V on + display power led
	led_blink_status |= 0x2;
	XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL1, led_blink_status);
	XGpio_DiscreteWrite(&POWER_CTRL_Inst, GPIO_CHANNEL1, 1);
	XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL2, 0); //b:g:r all_on
    //xil_printf("\r\nPower ON");
}

void PowerOff(void)
{
	led_blink_status &= 0xFD;
	XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL1, led_blink_status);
    XGpio_DiscreteWrite(&POWER_CTRL_Inst, GPIO_CHANNEL1, 0);
	XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL2, 0x7); //b:g:r all_off
    //xil_printf("\r\nPower OFF");
}

void exposDisplay(u8 e_status)
{
    if(e_status) XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL2, 0x06); //b:g:r r_on
    else XGpio_DiscreteWrite(&LED_Inst, GPIO_CHANNEL2, 0x0); //b:g:r all_on
}

void PWM_CLK_enable(void)
{
	XGpio_DiscreteWrite(&PWMEN_CTRL_Inst, GPIO_CHANNEL1, 0); // pwm clock enable, clock power up
}

void PWM_CLK_disable(void)
{
	XGpio_DiscreteWrite(&PWMEN_CTRL_Inst, GPIO_CHANNEL1, 1); // pwm clock power down
}

