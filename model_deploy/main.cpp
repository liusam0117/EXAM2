#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"
#include "uLCD_4DGL.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"
#include "mbed_events.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
// The gesture index of the prediction
int gesture_index, angle;
int THangle;
int mode = 1; //0 is gestureUI, 1 is RPC.
int16_t pDataXYZ_[3] = {0};
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);

BufferedSerial pc(USBTX, USBRX);
Thread t1;
Thread t2;
InterruptIn user_bt(USER_BUTTON);

uLCD_4DGL uLCD(D1, D0, D2); // serial tx, serial rx, reset pin;

void uLCD_print();
int PredictGesture(float* output);
void Gesture_UI();
RPCFunction rpc1(&Gesture_UI, "Gesture_UI");
void find_gesture();
void find_feature();
int save_feature[100];
bool start_save_feature = true;

void uLCD_print(){

  //print "gesture_index" on uLCD.
  uLCD.background_color(0xFFFFFF);
  uLCD.cls();
  uLCD.text_width(1); //2X size text
  uLCD.text_height(1);
  uLCD.textbackground_color(WHITE);
  uLCD.color(BLUE);
  uLCD.printf("\ngesture ID:\n"); //Default Green on black text    
  uLCD.text_width(4); //4X size text
  uLCD.text_height(4);
  uLCD.color(GREEN);
  uLCD.locate(1,2);
  uLCD.printf("%2d", gesture_index);
  return;
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

void Gesture_UI(){ 
  mode = 0;
  myled1 = 1;
  //start a thread.
  t1.start(find_gesture);// gesture ID
  t2.start(find_feature);// feature
}

void find_gesture(){
  int test_num = 0;
  //keep on finding gesture_index
  while(true){
    //predict gesture.
    if(mode == 0 && test_num < 10){
      // Whether we should clear the buffer next time we fetch data
      bool should_clear_buffer = false;
      bool got_data = false;

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
        return -1;
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
        return -1;
      }

      int input_length = model_input->bytes / sizeof(float);

      TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
      if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
        return -1;
      }

      error_reporter->Report("Set up successful...\n");

      while (true) {
      
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
          error_reporter->Report(config.output_message[gesture_index]);
          uLCD_print(); //call uLCD print.
          start_save_feature = false;
          int cnt_ = 0;
          printf("\n");
          while(save_feature[cnt_] != -1){
            printf("%d", save_feature[cnt_]);
            cnt_++;
          }
          printf("\n");
          start_save_feature = true;
          test_num++;
        }
      }
      if(test_num >= 10){
        mode = 1;
      }
    }
    else ;
  }
  
}

void find_feature(){
  memset(save_feature, -1, 500);
  int index = 0;
  while(true){
    if(mode == 0 && start_save_feature ){
      BSP_ACCELERO_AccGetXYZ(pDataXYZ_);
      if(pDataXYZ_[2]<900){ //high speed go down.
        save_feature[index] = 1;
      }else{
        save_feature[index] = 0;
      }
      if(index < 100) index++;
      else index = index;
    }
    if(!start_save_feature) index = 0;
    ThisThread::sleep_for(50ms);
  }
}



int main() {

  mode = 1; //RPCmode.

  //user_bt.rise(); 

  //RPC mode
  char buf[256], outbuf[256];

  FILE *devin = fdopen(&pc, "r");
  FILE *devout = fdopen(&pc, "w");
  while(1) {
      memset(buf, 0, 256);
      for (int i = 0; ; i++) {
          char recv = fgetc(devin);
          if (recv == '\n') {
              printf("\r\n");
              break;
          }
          buf[i] = fputc(recv, devout);
      }
      //Call the static call method on the RPC class
      RPC::call(buf, outbuf);      
      //printf("%s\r\n", outbuf);
  }
}
