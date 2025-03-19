/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
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
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <Car_detection_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"
#include "edge-impulse-sdk/classifier/ei_run_classifier.h"  
#include "edge-impulse-sdk/dsp/spectral/signal.hpp"  
#include "esp_camera.h"

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

#define CAMERA_MODEL_ESP_EYE // Has PSRAM
//#define CAMERA_MODEL_AI_THINKER // Has PSRAM
#define LEDR_PIN D2   // LED connected to D
#define LEDY_PIN D5  // LED connected to D
#define LEDG_PIN D6   // LED connected to D
#define SWITCH_PIN D7 // Switch connected to D7


#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    10
#define SIOD_GPIO_NUM    40
#define SIOC_GPIO_NUM    39

#define Y9_GPIO_NUM      48
#define Y8_GPIO_NUM      11
#define Y7_GPIO_NUM      12
#define Y6_GPIO_NUM      14
#define Y5_GPIO_NUM      16
#define Y4_GPIO_NUM      18
#define Y3_GPIO_NUM      17
#define Y2_GPIO_NUM      15
#define VSYNC_GPIO_NUM   38
#define HREF_GPIO_NUM    47
#define PCLK_GPIO_NUM    13

#else
#error "Camera model not selected"
#endif

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
uint8_t *snapshot_buf; //points to the output of the capture

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,

    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG, //YUV422,GRAYSCALE,RGB565,JPEG
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality
    .fb_count = 1,       //if more than one, i2s runs in continuous mode. Use only with JPEG
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
bool detectCar(void);
/**
•⁠  ⁠@brief      Arduino setup function
*/
void print_heap_info(const char *message) {  
    Serial.printf("\nHeap Info (%s):\n", message);  
    heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);  
}  
void setup()
{
  pinMode(LEDR_PIN, OUTPUT);            // Set LED as output
  pinMode(LEDY_PIN, OUTPUT);
  pinMode(LEDG_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT_PULLUP);   // Set switch as input with internal pull-up
    // put your setup code here, to run once:
    Serial.begin(115200);
    Serial.println("Starting setup...");  
    print_heap_info("Startup");  

    //comment out the below line to start inference immediately after upload
    // while (!Serial);
  // Serial.println("Edge Impulse Inferencing Demo");
  // if (ei_camera_init() == false) {
  //     ei_printf("Failed to initialize Camera!\r\n");
  //  }
  // else {
  //     ei_printf("Camera initialized\r\n");
  //  }

  // ei_printf("\nStarting continious inference in 10 seconds...\n");
  // ei_sleep(10000);
    //  if (!ei_camera_init()) {  
    //     Serial.println("ERROR: Camera failed to initialize. Halting program.");  
    //     while (1) {  // Halt the program if camera init fails  
    //         delay(1000);  // Keep printing an error message for debugging  
    //         Serial.println("Camera is not initialized. Check hardware and configuration.");  
    //     }  
    // }  

    // Serial.println("Camera initialized successfully.");  
    //   ei_sleep(10000);
}

/**
•⁠  ⁠@brief      Get data and run inferencing
*
•⁠  ⁠@param[in]  debug  Get debug info if true
*/
// Define states  
#define CAR_GO 0  
#define CAR_WAIT 1  
#define PEDESTRIAN_GO 2  
#define PEDESTRIAN_WAIT 3  
int currentState = CAR_GO;  

void loop()
{
  Serial.println("Entering loop function...");  
  unsigned long currentMillis = millis(); // For time tracking  
  Serial.print("Current State: ");  
  Serial.println(currentState);  
    // Handle different states  
    if (currentState == CAR_GO) {  
   // Turn on Green light, turn off others  
    digitalWrite(LEDG_PIN, HIGH);  
    digitalWrite(LEDY_PIN, LOW);  
    digitalWrite(LEDR_PIN, LOW);  

    // Check for state transition conditions  
    Serial.println("Current State: CAR_GO");  
    Serial.println("Checking switch and car detection...");  

    int switchState = digitalRead(SWITCH_PIN);  // Read the switch state  
    Serial.print("Switch State: ");   
    Serial.println(switchState);  // Debugging output for switch  

    // Detect cars or check if the pedestrian button is pressed  
    if (switchState == HIGH || detectCar()) {  
        Serial.println("Transitioning to CAR_WAIT...");  
        currentState = CAR_WAIT; // Move to next state  
    } else {  
        Serial.println("No event detected, staying in CAR_GO.");  
        delay(3000); // Stay in CAR_GO for 3 seconds  
    }

    }  
    // State: CAR_WAIT  
    else if (currentState == CAR_WAIT) {  
        // Turn on Yellow light, turn off others  
        digitalWrite(LEDG_PIN, LOW);   // Green Light OFF  
        digitalWrite(LEDY_PIN, HIGH);  // Yellow Light ON  
        digitalWrite(LEDR_PIN, LOW);   // Red Light OFF  

        // Wait 2 seconds in CAR_WAIT, then transition to PEDESTRIAN_GO  
        delay(2000);  
        currentState = PEDESTRIAN_GO;  
    }  
    // State: PEDESTRIAN_GO  
    else if (currentState == PEDESTRIAN_GO) {  
        // Turn on Red light (Cars stop), pedestrian signal BLINKS  
        digitalWrite(LEDG_PIN, LOW);   // Green Light OFF  
        digitalWrite(LEDY_PIN, LOW);   // Yellow Light OFF  
        digitalWrite(LEDR_PIN, HIGH);  // Red Light ON  

        // Pedestrian signal blinking  
        unsigned long startBlinkTime = millis();  
        while (millis() - startBlinkTime < 5000) {  // Blink for 5 seconds  
            // Simulate blinking with 500ms intervals  
            if ((millis() / 500) % 2 == 0) {  
                digitalWrite(LED_BUILTIN, HIGH);  // Turn ON Red  
            } else {  
                digitalWrite(LED_BUILTIN, LOW);  // Turn OFF Red  
            }  

            // If the pedestrian switch is pressed again, reset the timer to extend crossing time  
            if (digitalRead(SWITCH_PIN) == HIGH) {  
                startBlinkTime = millis();  // Reset blinking timer  
            }  
        }  
        currentState = PEDESTRIAN_WAIT;  // Transition to PEDESTRIAN_WAIT  
    }  
    // State: PEDESTRIAN_WAIT  
    else if (currentState == PEDESTRIAN_WAIT) {  
        // Stay in Red for Cars and Solid pedestrian signal  
        digitalWrite(LEDG_PIN, LOW);   // Green Light OFF  
        digitalWrite(LEDY_PIN, LOW);   // Yellow Light OFF  
        digitalWrite(LEDR_PIN, HIGH);  // Red Light ON  
        digitalWrite(LED_BUILTIN, HIGH);
        // Wait 2 seconds for pedestrians to finish crossing  
        delay(2000);  
        currentState = CAR_GO; // Return to CAR_GO after pedestrian phase  
    }
    
    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    free(snapshot_buf);

}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

   bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_JPEG, snapshot_buf);

   esp_camera_fb_return(fb);

   if(!converted){
       ei_printf("Conversion failed\n");
       return false;
   }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }


    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}
bool detectCar() {  
    Serial.println("Running car detection...");  

    // Create the signal object  
    ei::signal_t signal;  
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;  
    signal.get_data = &ei_camera_get_data; // Function to fetch image data  

    // Create the result structure  
    ei_impulse_result_t result;  

    // Declare the error variable and run the classifier  
    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);  

    if (err != EI_IMPULSE_OK) {  
        Serial.print("Classifier error: ");  
        Serial.println(err);  
        return false; // Return false if classifier fails  
    }  

    // Print the classification results for debugging  
    Serial.println("Inference complete. Checking results...");  
    for (size_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {  
        Serial.print("Label: ");  
        Serial.print(ei_classifier_inferencing_categories[i]);  
        Serial.print(", Value: ");  
        Serial.println(result.classification[i].value);  

        // Check for "car" label with confidence above threshold  
        if (strcmp(ei_classifier_inferencing_categories[i], "car") == 0 &&  
            result.classification[i].value > 0.6f) {  
            Serial.println("Car detected!");  
            return true;  // Car detected  
        }  
    }  

    Serial.println("No car detected.");  
    return false; // No car detected  
}  
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif