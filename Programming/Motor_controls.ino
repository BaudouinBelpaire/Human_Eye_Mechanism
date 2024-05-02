#include <Servo.h>

#define TIME_PER_DEGREE 2
#define MIN_ANGLE 52
#define MAX_ANGLE 133

Servo servo_x;
Servo servo_y;

int x, y, x_tmp, y_tmp = 90;

int compute_delay_time(int x_angle, int y_angle){
  int time_x = x_angle * TIME_PER_DEGREE;
  int time_y = y_angle * TIME_PER_DEGREE;

  if(time_x>time_y){
    return time_x;
  }

  else{
    return time_y;
  }
}

int trim_angle(int angle){
  if (angle>MAX_ANGLE){
    angle = MAX_ANGLE;
  }
  else if (angle<MIN_ANGLE){
    angle = MIN_ANGLE;
  }
  return angle;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.setTimeout(3); //Default value at 1s which cause latency to the code
  servo_x.attach(4, 500, 2500); //D4
  servo_y.attach(3, 500, 2500); //D3
  delay(1000);
  servo_x.write(90);
  servo_y.write(90);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  String myString;
  while(Serial.available()>0){
    myString = Serial.readString(); //Read string received with a structure as "<X,Y>"
    int commaIndex = myString.indexOf(','); //Get the index of the comma
    int startIndex = myString.indexOf('<'); //Get the index of the "<"
    int endIndex = myString.indexOf('>'); //Get the index of the ">"
    x = myString.substring(startIndex + 1, commaIndex).toInt(); //Get the value of X
    y = myString.substring(commaIndex + 1, endIndex).toInt(); //Get the value of Y
    /*Serial.print("X: ");
    Serial.print(x);
    Serial.print(" and ");
    Serial.print("Y: ");
    Serial.println(y);*/
  }

  int x_theta = abs(x-x_tmp);
  int y_theta = abs(y-y_tmp);

  int longest_time = compute_delay_time(x_theta, y_theta);

  x = trim_angle(x);
  y = trim_angle(y);

  servo_x.write(x);
  servo_y.write(y);

  //delay(longest_time);

  x_tmp = x;
  y_tmp = y;
}
