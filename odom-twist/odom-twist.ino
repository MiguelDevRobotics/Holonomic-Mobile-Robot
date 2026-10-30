// para ros
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;


/*void messageCb( const std_msgs::Float32MultiArray& vel);
ros::Subscriber<std_msgs::Float32MultiArray> sub("vel_omni", messageCb );*/

void onTwist(const geometry_msgs::Twist &msg);
ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &onTwist);

double x = 0.0;
double y = 0.0;
double theta = 0;

//constantes del robot
const double r = 0.04;
const double B = 0.127;
const double L = 0.127;

char base_link[] = "/base_link";
char odom[] = "/odom";

//variables del encoder 1
const int A1 = 36, B1e = 39; //pines para los canales del encoder
const int dirA1 = 26, dirB1 = 27, pwmM1 = 14; // pines para el control de velocidad
volatile bool dir1; // bandera para el loop principal
volatile int Aval1, Bval1; // valores leidos de los encoders
volatile int counter1;
float vel1 = 0, vel_filt1 = 0;

//variables del encoder 2
const int A2 = 34, B2 = 35; //pines para los canales del encoder
const int dirA2 = 32, dirB2 = 33, pwmM2 = 25; // pines para el control de velocidad
volatile bool dir2; // bandera para el loop principal
volatile int Aval2, Bval2; // valores leidos de los encoders
volatile int counter2;
float vel2 = 0, vel_filt2 = 0;

//variables del encoder 3
const int A3e = 23, B3 = 19; //pines para los canales del encoder
const int dirA3 = 12, dirB3 = 5, pwmM3 = 17; // pines para el control de velocidad
volatile bool dir3; // bandera para el loop principal
volatile int Aval3, Bval3; // valores leidos de los encoders
volatile int counter3;
float vel3 = 0, vel_filt3 = 0;

//variables del encoder 4
const int A4e = 16, B4 = 4; //pines para los canales del encoder
const int dirA4 = 2, dirB4 = 18, pwmM4 = 15; // pines para el control de velocidad
volatile bool dir4; // bandera para el loop principal
volatile int Aval4, Bval4; // valores leidos de los encoders
volatile int counter4;
float vel4 = 0, vel_filt4 = 0;

unsigned long timeold = 0;
float conv = 60*1000/5850, alpha = 0.05;

// variables para el control (PD)

struct PD{
  float kp;
  float kd;
  float Ts;
  float ea;
  float Aka;
  float refe;
  unsigned long timeold;
  };
  
PD M1{100,2,20,0,0,0,0};
PD M2{100,2,20,0,0,0,0};
PD M3{100,2,20,0,0,0,0};
PD M4{100,2,20,0,0,0,0};

float IRAM_ATTR control(PD obj, float y){
float e=obj.refe-y;
obj.Ts = (millis()-obj.timeold)/1000;
obj.Aka=obj.Aka+e*obj.Ts;
float de=(e-obj.ea)/obj.Ts;
float u = obj.kp*e+de*obj.kd;
obj.timeold = millis();
obj.ea = e;
if (u >255){
  u = 255;
  }else if(u < -255){
    u = -255;
    }
return u;
}

void IRAM_ATTR encoder1(){
  
  int ANow = digitalRead(A1);
  int BNow = digitalRead(B1e);

  if(dir1 && (counter1<0)){
    counter1 = 0;
    }else if(!dir1 && counter1>0){
      counter1 = 0;
      }
    
  
  if(ANow == 0 && BNow == 0){
    if(Aval1 == 0 && Bval1 == 1){
      counter1++;
      dir1 = true;
      }else if(Aval1 == 1 && Bval1 == 0){
        counter1--;
        dir1 = false;
        }
  }else if(ANow == 1 && BNow == 0){
    if(Aval1 == 0 && Bval1 == 0){
      counter1++;
      dir1 = true;
      }else if(Aval1 == 1 && Bval1 == 1){
        counter1--;
        dir1 = false;
        }
    }else if(ANow == 1 && BNow == 1){
      if(Aval1 == 1 && Bval1 == 0){
        counter1++;
        dir1 = true;
      }else if(Aval1 == 0 && Bval1 == 1){
        counter1--;
        dir1 = false;
        }
      }else if(ANow == 0 && BNow == 1){
        if(Aval1 == 1 && Bval1 == 1){
          counter1++;
          dir1 = true;
        }else if(Aval1 == 0 && Bval1 == 0){
        counter1--;
        dir1 = false;
        }
      }

  Aval1 = ANow;
  Bval1 = BNow;  
  }

void IRAM_ATTR encoder2(){
  
  int ANow = digitalRead(A2);
  int BNow = digitalRead(B2);

  if(dir2 && (counter2<0)){
    counter2 = 0;
    }else if(!dir2 && counter2>0){
      counter2 = 0;
      }
    
  
  if(ANow == 0 && BNow == 0){
    if(Aval2 == 0 && Bval2 == 1){
      counter2++;
      dir2 = true;
      }else if(Aval2 == 1 && Bval2 == 0){
        counter2--;
        dir2 = false;
        }
  }else if(ANow == 1 && BNow == 0){
    if(Aval2 == 0 && Bval2 == 0){
      counter2++;
      dir2 = true;
      }else if(Aval2 == 1 && Bval2 == 1){
        counter2--;
        dir2 = false;
        }
    }else if(ANow == 1 && BNow == 1){
      if(Aval2 == 1 && Bval2 == 0){
        counter2++;
        dir2 = true;
      }else if(Aval2 == 0 && Bval2 == 1){
        counter2--;
        dir2 = false;
        }
      }else if(ANow == 0 && BNow == 1){
        if(Aval2 == 1 && Bval2 == 1){
          counter2++;
          dir2 = true;
        }else if(Aval2 == 0 && Bval2 == 0){
        counter2--;
        dir2 = false;
        }
      }

  Aval2 = ANow;
  Bval2 = BNow;  
  }

void IRAM_ATTR encoder3(){
  
  int ANow = digitalRead(A3e);
  int BNow = digitalRead(B3);

  if(dir3 && (counter3<0)){
    counter3 = 0;
    }else if(!dir3 && counter3>0){
      counter3 = 0;
      }
    
  
  if(ANow == 0 && BNow == 0){
    if(Aval3 == 0 && Bval3 == 1){
      counter3++;
      dir3 = true;
      }else if(Aval3 == 1 && Bval3 == 0){
        counter3--;
        dir3 = false;
        }
  }else if(ANow == 1 && BNow == 0){
    if(Aval3 == 0 && Bval3 == 0){
      counter3++;
      dir3 = true;
      }else if(Aval3 == 1 && Bval3 == 1){
        counter3--;
        dir3 = false;
        }
    }else if(ANow == 1 && BNow == 1){
      if(Aval3 == 1 && Bval3 == 0){
        counter3++;
        dir3 = true;
      }else if(Aval3 == 0 && Bval3 == 1){
        counter3--;
        dir3 = false;
        }
      }else if(ANow == 0 && BNow == 1){
        if(Aval3 == 1 && Bval3 == 1){
          counter3++;
          dir3 = true;
        }else if(Aval3 == 0 && Bval3 == 0){
        counter3--;
        dir3 = false;
        }
      }

  Aval3 = ANow;
  Bval3 = BNow;  
  }

void IRAM_ATTR encoder4(){
  
  int ANow = digitalRead(A4e);
  int BNow = digitalRead(B4);

  if(dir4 && (counter4<0)){
    counter4 = 0;
    }else if(!dir4 && counter4>0){
      counter4 = 0;
      }
    
  
  if(ANow == 0 && BNow == 0){
    if(Aval4 == 0 && Bval4 == 1){
      counter4++;
      dir4 = true;
      }else if(Aval4 == 1 && Bval4 == 0){
        counter4--;
        dir4 = false;
        }
  }else if(ANow == 1 && BNow == 0){
    if(Aval4 == 0 && Bval4 == 0){
      counter4++;
      dir4 = true;
      }else if(Aval4 == 1 && Bval4 == 1){
        counter4--;
        dir4 = false;
        }
    }else if(ANow == 1 && BNow == 1){
      if(Aval4 == 1 && Bval4 == 0){
        counter4++;
        dir4 = true;
      }else if(Aval4 == 0 && Bval4 == 1){
        counter4--;
        dir4 = false;
        }
      }else if(ANow == 0 && BNow == 1){
        if(Aval4 == 1 && Bval4 == 1){
          counter4++;
          dir4 = true;
        }else if(Aval4 == 0 && Bval4 == 0){
        counter4--;
        dir4 = false;
        }
      }

  Aval4 = ANow;
  Bval4 = BNow;  
  }


//void messageCb( const std_msgs::Float32MultiArray& vel){
void onTwist(const geometry_msgs::Twist &msg){
  double Vf = msg.linear.x;
  double Vl = msg.linear.y;
  double W = msg.angular.z;

  double V1,V2,V3,V4;
  V1 = Vf-Vl-W*(L+B);
  V2 = Vf+Vl+W*(L+B);
  V3 = Vf+Vl-W*(L+B);
  V4 = Vf-Vl+W*(L+B);

  M1.refe = V1/(2*PI)*60/r;
  M2.refe = -V2/(2*PI)*60/r;
  M3.refe = V3/(2*PI)*60/r;
  M4.refe = -V4/(2*PI)*60/r;
}

void odo(void *parametros) {
    
    nh.initNode();
    broadcaster.init(nh);
    nh.subscribe(sub);

    delay(2000);

    // para el calculo de la odormetria
    
    double xcp=0, ycp=0,Vf=0,Vl=0,omega=0;
    unsigned long time_odom = millis();
    double dt = 0;
    double V1,V2,V3,V4;
  while (true){
  
    // odometria
    V1 = (vel_filt1*2*PI/60)*r;
    V2 = -(vel_filt2*2*PI/60)*r;
    V3 = (vel_filt3*2*PI/60)*r;
    V4 = -(vel_filt4*2*PI/60)*r;
    
    Vf = (V1+V2+V3+V4)/4;
    Vl = (-V1+V2+V3-V4)/4;
    omega = (-V1+V2-V3+V4)/(4*(L+B));

    dt = millis()-time_odom;
        // modelo cinematico
    
    xcp = Vf*cos(theta)-Vl*sin(theta);
    ycp = Vf*sin(theta)+Vl*cos(theta);
    theta += omega*dt/1000;
    

    x += xcp*dt/1000;
    y += ycp*dt/1000;
    // integrando el modelo cinematico para obtener x,y y theta
    
    if(theta > PI)
      theta=-PI;
    else if(theta < -PI){
      theta = PI;
      }
    time_odom = millis();
    
    // tf odom->base_link
    t.header.frame_id = odom;
    t.child_frame_id = base_link;
  
    t.transform.translation.x = x;
    t.transform.translation.y = y;
  
    t.transform.rotation = tf::createQuaternionFromYaw(theta);
    t.header.stamp = nh.now();
  
    broadcaster.sendTransform(t);
    nh.spinOnce();
  
    delay(10);
    
    }
    // Realiza aquí tu tarea
    vTaskDelete(NULL); // Elimina la tarea después de ejecutarla
}
void setup() {
  xTaskCreatePinnedToCore(odo, "odo", 2048, NULL, 1, NULL, 0); // Crea la tarea y la asigna al núcleo 0

  
  // configuracion de pwm
  ledcSetup(0,5000,8);
  ledcAttachPin(pwmM1,0);
  pinMode(dirA1,OUTPUT);
  pinMode(dirB1,OUTPUT);
  M1.refe=0;

  ledcSetup(1,5000,8);
  ledcAttachPin(pwmM2,1);
  pinMode(dirA2,OUTPUT);
  pinMode(dirB2,OUTPUT);
  M2.refe=0;

  ledcSetup(2,5000,8);
  ledcAttachPin(pwmM3,2);
  pinMode(dirA3,OUTPUT);
  pinMode(dirB3,OUTPUT);
  M3.refe=0;

  ledcSetup(3,5000,8);
  ledcAttachPin(pwmM4,3);
  pinMode(dirA4,OUTPUT);
  pinMode(dirB4,OUTPUT);
  M4.refe=0;

  // inicializacion de encoders
  pinMode(A1,INPUT);
  pinMode(B1e,INPUT);
  attachInterrupt(digitalPinToInterrupt(A1),encoder1,CHANGE);
  attachInterrupt(digitalPinToInterrupt(B1e),encoder1,CHANGE);
  Aval1 = digitalRead(A1);
  Bval1 = digitalRead(B1e);

  pinMode(A2,INPUT);
  pinMode(B2,INPUT);
  attachInterrupt(digitalPinToInterrupt(A2),encoder2,CHANGE);
  attachInterrupt(digitalPinToInterrupt(B2),encoder2,CHANGE);
  Aval2 = digitalRead(A2);
  Bval2 = digitalRead(B2);

  pinMode(A3e,INPUT);
  pinMode(B3,INPUT);
  attachInterrupt(digitalPinToInterrupt(A3e),encoder3,CHANGE);
  attachInterrupt(digitalPinToInterrupt(B3),encoder3,CHANGE);
  Aval3 = digitalRead(A3e);
  Bval3 = digitalRead(B3);

  pinMode(A4e,INPUT);
  pinMode(B4,INPUT);
  attachInterrupt(digitalPinToInterrupt(A4e),encoder4,CHANGE);
  attachInterrupt(digitalPinToInterrupt(B4),encoder4,CHANGE);
  Aval4 = digitalRead(A4e);
  Bval4 = digitalRead(B4);

  
}

void loop() {
  if(millis() - timeold >=3){
    // obtencion de la velocidad del motor
    float elapsed = millis()-timeold;
    vel1 = conv*counter1/elapsed;
    vel_filt1 = (alpha*vel1)+((1-alpha)*vel_filt1);
    counter1 = 0;

    vel2 = conv*counter2/elapsed;
    vel_filt2 = (alpha*vel2)+((1-alpha)*vel_filt2);
    counter2 = 0;

    vel3 = conv*counter3/elapsed;
    vel_filt3 = (alpha*vel3)+((1-alpha)*vel_filt3);
    counter3 = 0;

    vel4 = conv*counter4/elapsed;
    vel_filt4 = (alpha*vel4)+((1-alpha)*vel_filt4);
    counter4 = 0;

    timeold = millis();
    
    //control de velocidad del motor
    float pwmOut1 = control(M1,vel_filt1);
    if(pwmOut1 > 0){
      digitalWrite(dirA1,HIGH);
      digitalWrite(dirB1,LOW);
      }else{
        digitalWrite(dirA1,LOW);
        digitalWrite(dirB1,HIGH);
        pwmOut1 = pwmOut1*-1;
        }
    ledcWrite(0,pwmOut1);

    float pwmOut2 = control(M2,vel_filt2);
    if(pwmOut2 > 0){
      digitalWrite(dirA2,HIGH);
      digitalWrite(dirB2,LOW);
      }else{
        digitalWrite(dirA2,LOW);
        digitalWrite(dirB2,HIGH);
        pwmOut2 = pwmOut2*-1;
        }
    ledcWrite(1,pwmOut2);

    float pwmOut3 = control(M3,vel_filt3);
    if(pwmOut3 > 0){
      digitalWrite(dirA3,HIGH);
      digitalWrite(dirB3,LOW);
      }else{
        digitalWrite(dirA3,LOW);
        digitalWrite(dirB3,HIGH);
        pwmOut3 = pwmOut3*-1;
        }
    ledcWrite(2,pwmOut3);

    float pwmOut4 = control(M4,vel_filt4);
    if(pwmOut4 > 0){
      digitalWrite(dirA4,HIGH);
      digitalWrite(dirB4,LOW);
      }else{
        digitalWrite(dirA4,LOW);
        digitalWrite(dirB4,HIGH);
        pwmOut4 = pwmOut4*-1;
        }
    ledcWrite(3,pwmOut4);

  
    //Serial.println(vel_filt4);
  }

}
