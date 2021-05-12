#include "config.h"
#include "magic_wand_model_data.h"

#include "mbed_rpc.h"
#include "mbed.h"
#include "uLCD_4DGL.h"
#include "accelerometer_handler.h"
#include "stm32l475e_iot01_accelero.h"
#include <cmath>

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

using namespace std::chrono;

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.



// Store x, y, z data
int16_t aDataXYZ[3] = {0};
/////
uLCD_4DGL uLCD(D1, D0, D2);
/////

Thread thread1, thread2;
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
int angle = 30;
int mode = 0;
float over_angle[10] = {0};

/////

////////////////////////////////////////mqtt////////////////////////////////
// GLOBAL VARIABLES
WiFiInterface *wifi;
InterruptIn btn2(USER_BUTTON);

volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
bool RPC_function_run = 1;
bool call_mqtt = false;

const char* topic = "Mbed";

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;
EventQueue UI_queue;
EventQueue tile_queue;

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
    if (mode == 1 && arrivedcount == 1)
      closed = true;
    else if (mode == 2 && arrivedcount == 10)
      closed = true;
    RPC_function_run = 0;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
    MQTT::Message message;
    char buff[100];
    if (mode == 1)
      sprintf(buff, "threshold_angle : %d", angle);
    else if (mode == 2) {
      sprintf(buff, "over_angle : %g\n", over_angle[arrivedcount]);
    }
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    int rc = client->publish(topic, message);

    printf("rc:  %d\r\n", rc);
    printf("Puslish message: %s\r\n", buff);
}

void close_mqtt() {
    closed = true;
}

void mqtt() {
    call_mqtt = true;
    ///////// print out the angle in the end
    printf("threshold angle is :%d\r\n", angle);
    printf("please push enter to continue\n");
}

////////////////////////////////////////////mqtt/////////////////////////////////////

constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

/////

void UI_mode(Arguments *in, Reply *out);
void gesture_mode();
void tile_mode(Arguments *in, Reply *out);
void angle_detect();
RPCFunction UI_gesture(&UI_mode, "UI_mode");
RPCFunction Tile_mode(&tile_mode, "tile_mode");

BufferedSerial pc(USBTX, USBRX);

/////

int main() {

    ///////////////////////////////connection part of mqtt//////////////////////////
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.249";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    ///////////////////////////////////////////////connection part of mqtt////////////////////////////////////

  // Init accelerometer
  BSP_ACCELERO_Init();
  //// button interrupt
  mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));    //// mqtt should be in thread otherwise it won't run
  btn2.fall(mqtt_queue.event(mqtt));
  ///////
  uLCD.text_width(4);
  uLCD.text_height(4);
  uLCD.locate(1,2);
  ///////
  char buf[256], outbuf[256];

   FILE *devin = fdopen(&pc, "r");
   FILE *devout = fdopen(&pc, "w");


   while (true) {
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

      if (call_mqtt) {
        arrivedcount = 0;
        closed = false;

        //btn3.rise(&close_mqtt);
        
        while(!closed) {
                publish_message(&client);
                client.yield(500);
                ThisThread::sleep_for(500ms);
        }
        call_mqtt = false;
      }
   }

   //////////////////////////// disconnection part of mqtt////////////////////
   printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0) {
            printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0) {
    printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
  //////////////////////////// disconnection part of mqtt////////////////////

}

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

void UI_mode(Arguments *in, Reply *out){
    RPC_function_run = 1;       /////// reset
    mode = 1;
    angle = 30;
    thread1.start(callback(&UI_queue, &EventQueue::dispatch_forever));
    led1 = 1;
    led2 = 0;
    UI_queue.call(gesture_mode);
}

void gesture_mode() {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return;
  }

  error_reporter->Report("Set up successful...\n");

  while (RPC_function_run) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

    // If there was no new data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }

    // Run inference, and report any error
    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // Produce an output
    if (gesture_index < label_num) {
      // error_reporter->Report(config.output_message[gesture_index]);
      printf("%d\n", gesture_index);
      if (gesture_index == 0) {
        angle += 5;
        uLCD.locate(1,2);
        uLCD.printf("%d\n", angle);
      }
      else if (gesture_index == 1) {
        angle += 10;
        uLCD.locate(1,2);
        uLCD.printf("%d\n", angle);
      }
      else if (gesture_index == 2) {
        uLCD.locate(1,2);
        uLCD.printf("%d\n", angle);
      }
    }
  }
}

void tile_mode(Arguments *in, Reply *out) {
  mode = 2;
  led1 = 0;
  led2 = 1;
  thread2.start(callback(&tile_queue, &EventQueue::dispatch_forever));
  tile_queue.call(angle_detect);
}

void angle_detect() {
  double alen = 0, adot = 0;
  float cos = 0, theta = 0;
  int referenceXYZ[3] = {0};
  int referencelen = 0;
  int over_num = 0;

  for (int i = 0; i < 10; i++) {
    led3 = 0;
    ThisThread::sleep_for(500ms);
    BSP_ACCELERO_AccGetXYZ(aDataXYZ);
    referenceXYZ[0] = aDataXYZ[0];
    referenceXYZ[1] = aDataXYZ[1];
    referenceXYZ[2] = aDataXYZ[2];
    referencelen = sqrt(aDataXYZ[0] * aDataXYZ[0] + aDataXYZ[1] * aDataXYZ[1] + aDataXYZ[2] * aDataXYZ[2]);
    over_angle[i] = 0;
  }
  while(over_num < 10) {
    led3 = 1;
    ThisThread::sleep_for(100ms);
    BSP_ACCELERO_AccGetXYZ(aDataXYZ);
    alen = sqrt(aDataXYZ[0] * aDataXYZ[0] + aDataXYZ[1] * aDataXYZ[1] + aDataXYZ[2] * aDataXYZ[2]);
    adot = 17 * aDataXYZ[0] + (-28) * aDataXYZ[1] + 1020 * aDataXYZ[2];
    cos = adot / (alen * referencelen);
    theta = acos(cos) * 180 / 3.1415926;
    printf("%f\n", theta);
    if (theta > angle) {
      over_angle[over_num] = theta;
      over_num++;
    }
  }
  mqtt();
}