#include "mbed.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"

InterruptIn button(USER_BUTTON);
BufferedSerial pc(USBTX, USBRX);

void gesture_RPC(Arguments *in, Reply *out);
void angle_detect_RPC(Arguments *in, Reply *out);
void gesture_UI();
void angle_detect();
RPCFunction rpc1(&gesture_RPC, "gesture_RPC");
// RPCFunction rpc2(&angle_detect_RPC, "angle_detect_RPC");
DigitalOut myled1(LED1);	// gesture_UI mode
DigitalOut myled2(LED2);	// angle_detect mode

Thread t_angle_detect;
Thread t_gesture_UI;

int16_t pDataXYZ[3] = {0};
int idR[32] = {0};
int indexR = 0;
int Angle;

void gesture_RPC(Arguments *in, Reply *out){
	char buffer[200];
	t_gesture_UI.start(&gesture_UI);
	sprintf(buffer, "Angle: %d degree", Angle);
	out->putData(buffer);
}

void gesture_UI(){
	myled1 = 1;
	char buffer[200];
	Angle = 30;
	while (1) {
		if (1) {	// receive the command from gesture
			if (Angle == 60) 
				Angle = 30;
			else 
				Angle += 5;
		}
	}
	myled1 = 0;
}
// void angle_detect_RPC(Arguments *in, Reply *out){
// 	t_gesture_UI.start(&angle_detect);
// }

// void angle_detect(){

// }
int main() {
    BSP_ACCELERO_Init();
	char buf[256], outbuf[256];

   	FILE *devin = fdopen(&pc, "r");
   	FILE *devout = fdopen(&pc, "w");

	while (1) {
		memset(buf, 0, 256);      // clear buffer
		for(int i=0; i<255; i++) {
			char recv = fgetc(devin);
			if (recv == '\r' || recv == '\n') {
				printf("\r\n");
				break;
			}
			buf[i] = fputc(recv, devout);
		}
		RPC::call(buf, outbuf);
		printf("%s\r\n", outbuf);
   	}
	
}