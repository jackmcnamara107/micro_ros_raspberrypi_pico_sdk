#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float64.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <time.h>
#include <math.h>
#include <stdio.h>
#include "hardware/timer.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"


const uint LED_PIN = 25;

// Right Motor Encoder pins
const uint ENC1_A = 8; // orange wire
const uint ENC1_B = 9; // white wire

// Left Motor Encoder pins
const uint ENC2_A = 0; // orange wire
const uint ENC2_B = 1; // white wire

const uint ENA = 16; // PWM Signal to motor controller
const uint IN1 = 17; // Motor controller IN1 
const uint IN2 = 18; // Motor controller IN2

const uint ENB = 15; // PWM Signal to motor controller
const uint IN3 = 14; // Motor controller IN1 
const uint IN4 = 13; // Motor controller IN2

// Initialise variables
int right_pos = 0; // Encoder 1 position
int prev_pos = 0;
int left_pos = 0; // Encoder 2 position

double previousTime = 0;
double interval = 0.05;

const int encoder_points = 644;

double ang_velocity_right = 0;
double ang_velocity_left = 0;

double rad_per_point;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher_right;
rcl_publisher_t publisher_left;
rcl_subscription_t subscriber;
std_msgs__msg__Float64 send_right_msg;
std_msgs__msg__Float64 send_left_msg;

clock_t prevT = 0; // Previous time of loop
float eprev_right = 0; // Previous error from control loop
float eintegral = 0; // Previous integral of error (area under curve)
double target = 1; // Control loop target signal
float pwr = 0;
int dir = 1;

// void get_wheel_velocity(int64_t last_call_time);
void get_ang_velocity();
void timer_callback(rcl_timer_t *timer, int64_t last_call_time);
void subscription_callback(const void * msgin);
void actuate_motor(double target);
void setMotor(int dir, int pwmVal, int pwm, int in1, int in2);
uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d);

void subscription_callback(const void * msgin)
{
    const std_msgs__msg__Float64 * msg = (const std_msgs__msg__Float64 *)msgin;
    target = msg->data;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    // send_left_msg.data = pwr;
    // send_left_msg.data = 0.5;
    // pwm_set_freq_duty(pwm_gpio_to_slice_num(ENA), pwm_gpio_to_channel(ENA), 2000, 100);
    // pwm_set_enabled(pwm_gpio_to_slice_num(ENA), true);
    get_ang_velocity();
    actuate_motor(target);
    RCSOFTCHECK(rcl_publish(&publisher_right, &send_right_msg, NULL));
    RCSOFTCHECK(rcl_publish(&publisher_left, &send_left_msg, NULL));
}

clock_t clock_ms()
{
    return (clock_t) time_us_64() / 10000;
}

double delta_time(double startTime, double endTime)
{
    double executionTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    return executionTime;
}

void get_ang_velocity()
{
    double currentTime = clock();
    double deltaTime = delta_time(previousTime, currentTime);
    if (deltaTime > interval){

        ang_velocity_right = right_pos * (2*M_PI/encoder_points);
        // ang_velocity_left = left_pos * (2*M_PI/encoder_points);

        // printf("deltaTime: %f, ang_velocity_left: %f, ang_velocity_right: %f \n", deltaTime, ang_velocity_left, ang_velocity_right);

        right_pos = 0;
        left_pos = 0;
        send_right_msg.data = ang_velocity_right;
        previousTime = clock();
    }
}

void gpio_callback(uint gpio, uint32_t events) {
    int b = gpio_get(ENC1_B);
    if (b > 0){
        right_pos++;
    } else {
        right_pos--;
    }
    // send_right_msg.data = right_pos;
    
}

// void gpio1_callback(void) {
//     if (gpio_get_irq_event_mask(ENC2_A) & GPIO_IRQ_EDGE_RISE){
//         gpio_acknowledge_irq(ENC2_A, GPIO_IRQ_EDGE_RISE);
//         // Read the encoder b wire and compare
//         int b = gpio_get(ENC2_B);
// 		if (b > 0){
// 			left_pos++;
// 		} else {
// 			left_pos--;
// 		}
// 		send_left_msg.data = left_pos;
//     }
// }

void actuate_motor(double target)
{
    // PID Controller
    float kp = 1;
    float ki = 0;
    float kd = 0;

    // Get time difference from last loop
    // clock_t currT = clock();
    // // float deltaT = (currT - prevT)/(1.0e6);
    // double deltaT = (double)(currT - prevT) / 100;
    double currT = clock();
    double deltaT = delta_time(prevT, currT);
    prevT = currT;

    double e = ang_velocity_right - target; // Error between current position and targer

    float dedt = (e-eprev_right)/deltaT; // Derivative of change in error

    eintegral = eintegral + e*deltaT; // Integral of change in error

    float u = kp*e + kd*dedt + ki*eintegral; // Control signal

    float pwr = fabs(u);
    // if (pwr > 100){
    //     pwr = 100;
    // }

    if (pwr > 0){
        pwr = (double) pwr/(1.85/100);
    } else {
		pwr = 0;
	}

    if (pwr > 100){
        pwr = 100;
    }

    int dir = 1;
    if (signbit(u)){
        dir = -1;
    }

    send_left_msg.data = (double) dir;

    setMotor(dir,pwr,ENA,IN1,IN2);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2)
{
    pwm_set_freq_duty(pwm_gpio_to_slice_num(pwm), pwm_gpio_to_channel(pwm), 2000, pwmVal);
    pwm_set_enabled(pwm_gpio_to_slice_num(pwm), true);
    if (dir == 1){
        gpio_put(IN1, 1);
        gpio_put(IN2, 0);
    } else if (dir == -1){
        gpio_put(IN1, 0);
        gpio_put(IN2, 1);
    } 
    // else {
    //     gpio_put(IN1, 0);
    //     gpio_put(IN2, 0);
    // }
}

uint32_t pwm_set_freq_duty(uint slice_num, uint chan,uint32_t f, int d)
{
    uint32_t clock = 125000000;
    uint32_t divider16 = clock / f / 4096 + (clock % (f * 4096) != 0);
    if (divider16 / 16 == 0)
    divider16 = 16;
    uint32_t wrap = clock * 16 / divider16 / f - 1;
    pwm_set_clkdiv_int_frac(slice_num, divider16/16, divider16 & 0xF);
    pwm_set_wrap(slice_num, wrap);
    pwm_set_chan_level(slice_num, chan, wrap * d / 100);
    return wrap;
}

int main()
{	
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

	gpio_init(ENC1_B);
	gpio_init(ENC2_B);
	gpio_set_dir(ENC1_B, GPIO_IN);
	gpio_set_dir(ENC2_B, GPIO_IN);

    gpio_init(ENC1_A);
	gpio_init(ENC2_A);
	gpio_set_dir(ENC1_A, GPIO_IN);
	gpio_set_dir(ENC2_A, GPIO_IN);

    gpio_init(IN1);
    gpio_init(IN2);
	gpio_set_dir(IN1, GPIO_OUT);
    gpio_set_dir(IN2, GPIO_OUT);
    gpio_set_function(ENA, GPIO_FUNC_PWM);

    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;
    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);

	// create publisher_right
    rclc_publisher_init_default(
        &publisher_right,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "ang_velocity_right");

	// create publisher_right
    rclc_publisher_init_default(
        &publisher_left,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "pwm_signal");

    rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
        "cmd_vel_right");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(100),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 2, &allocator);

    // Message object to receive publisher data
    std_msgs__msg__Float64 msg;

    // Add subscription to the executor
    rcl_ret_t rc = rclc_executor_add_subscription(
    &executor, &subscriber, &msg,
    &subscription_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &timer);

    send_right_msg.data = 0;
	// send_left_msg.data = 0;

	// Set interrupt and interrupt handler
    gpio_set_irq_enabled_with_callback(ENC1_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled_with_callback(ENC2_A, GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    // gpio_set_irq_enabled(ENC2_A, GPIO_IRQ_EDGE_RISE, true);
    // gpio_add_raw_irq_handler(ENC2_A, gpio1_callback);

	gpio_put(LED_PIN, 1);

    while (true)
    {
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
    return 0;
}