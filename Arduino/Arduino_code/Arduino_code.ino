#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

const int button_pin = 7;
const int led_pin = 13;
bool is_blinking = false;
bool is_on = false;
bool lightIsOn = false;


void turn_led (const std_msgs::Empty &toggle_msg){
  lightIsOn = !lightIsOn;
}

void blink_led(const std_msgs::Empty &toggle_msg){
  is_blinking = !is_blinking;
}

ros::Subscriber<std_msgs::Empty> sub_led("arduino_toggle_led", &turn_led);
ros::Subscriber<std_msgs::Empty> sub_blink("arduino_blink_led", &blink_led);

std_msgs::Bool pushed_msg;
ros::Publisher start_button("button_pushed", &pushed_msg);

bool last_reading;
long last_debounce_time=0;
long debounce_delay=50;
bool published = false;
bool lamp_value = false;
long last_blink_time = 0;

void setup() {
  pinMode(led_pin,OUTPUT);
  pinMode(button_pin,INPUT);
  nh.initNode();
  nh.advertise(start_button);
  nh.subscribe(sub_led);
  nh.subscribe(sub_blink);
  
  digitalWrite(button_pin, HIGH);
  last_reading =! digitalRead(button_pin);
}

void loop() {
  if(last_blink_time == 0){
    last_blink_time = millis();    
  }
  
  bool reading = digitalRead(button_pin);

  if (last_reading!= reading){
    last_debounce_time = millis();
    published = false;
  }

  if ( !published && (millis() - last_debounce_time) > debounce_delay){
      pushed_msg.data = (reading);
      start_button.publish(&pushed_msg);
      published = true;
  }

  if (is_blinking && millis() - last_blink_time > 500) {
    last_blink_time = millis();
    if (is_on) {
      is_on = false;
      digitalWrite(led_pin, LOW);
    } else{
      is_on = true;
      digitalWrite(led_pin, HIGH);
    }
  }
  else if(is_on && !is_blinking) {
    is_on = false;
    digitalWrite(led_pin, LOW);
  }else if (!is_blinking) {
     digitalWrite(led_pin, lightIsOn);
  }

  
  
  last_reading = reading;

  nh.spinOnce();
}
