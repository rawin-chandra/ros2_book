long time_cal = 0;
long time_cal2 = 0;

long EncoderVal[2] = {0,0};
double Vels[4] = {0,0,0,0};

float WheelDiameter = 0.067;      //set to your wheel of robot
int TPR = 210;			//set to your motor value

volatile int counterL_forward = 0;
volatile int counterR_forward = 0;
volatile int counterL_backward = 0;
volatile int counterR_backward = 0;


void countL_forward() {
    counterL_forward++;
}


void countR_forward() {
     counterR_forward++;
}

void countL_backward() {
     counterL_backward++;
}

void countR_backward() {
    counterR_backward++;
}

void speed_cal_forward(){
  if(time_cal == 0) {
    time_cal = millis();
    return;
  }

    EncoderVal[0] = counterL_forward;
    counterL_forward = 0;

    EncoderVal[1] = counterR_forward;
    counterR_forward = 0;

  long aTime = millis();
  int DTime = aTime-time_cal;
  time_cal = aTime;

  //calculate short term measured velocity
  Vels[0] = (TicksToMeters(EncoderVal[0])/DTime)*1000;
  Vels[1] = (TicksToMeters(EncoderVal[1])/DTime)*1000;
}
void speed_cal_backward(){
    if(time_cal2 == 0) {
    time_cal2 = millis();
    return;
  }

    EncoderVal[0] = counterL_backward;
    counterL_backward = 0;

    EncoderVal[1] = counterR_backward;
    counterR_backward = 0;

  long aTime = millis();
  int DTime = aTime-time_cal2;
  time_cal2 = aTime;

  Vels[2] = (TicksToMeters(EncoderVal[0])/DTime)*1000;
  Vels[3] = (TicksToMeters(EncoderVal[1])/DTime)*1000;
  }

double TicksToMeters(int Ticks){
    return (Ticks*3.14*WheelDiameter)/TPR;
}

void setup() {
    attachInterrupt(digitalPinToInterrupt(2), countL_forward, RISING); //channel A
    attachInterrupt(digitalPinToInterrupt(3), countR_backward, RISING); //channel B
    attachInterrupt(digitalPinToInterrupt(18), countL_backward, RISING); //channel B
    attachInterrupt(digitalPinToInterrupt(19), countR_forward, RISING);  //channel A
    Serial.begin(9600);
}

void loop() {
  speed_cal_forward();
  speed_cal_backward();
  Serial.print(Vels[0]);
  Serial.print(",");
  Serial.print(Vels[1]);
  Serial.print(",");
  Serial.print(Vels[2]);
  Serial.print(",");
  Serial.println(Vels[3]);

  delay(3);
}
