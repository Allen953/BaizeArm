

#define LF_Motor1 32
#define LF_Motor2 33
#define LB_Motor1 26
#define LB_Motor2 25
#define RF_Motor1 17
#define RF_Motor2 5
#define RB_Motor1 18
#define RB_Motor2 19


#define Step_L1 LF_Motor1
#define Step_L2 LF_Motor2
#define Step_L3 LB_Motor1
#define Step_L4 LB_Motor2

#define Step_R1 RF_Motor1
#define Step_R2 RF_Motor2
#define Step_R3 RB_Motor1
#define Step_R4 RB_Motor2

#define StepVelo 5

void init_pin(){
  pinMode(LF_Motor1,OUTPUT);
  pinMode(LF_Motor2,OUTPUT);
  pinMode(LB_Motor1,OUTPUT);
  pinMode(LB_Motor2,OUTPUT);
  pinMode(RF_Motor1,OUTPUT);
  pinMode(RF_Motor2,OUTPUT);
  pinMode(RB_Motor1,OUTPUT);
  pinMode(RB_Motor2,OUTPUT);

  digitalWrite(LF_Motor1,LOW);
  digitalWrite(LF_Motor2,LOW);
  digitalWrite(LB_Motor1,LOW);
  digitalWrite(LB_Motor2,LOW);
  digitalWrite(RF_Motor1,LOW);
  digitalWrite(RF_Motor2,LOW);
  digitalWrite(RB_Motor1,LOW);
  digitalWrite(RB_Motor2,LOW);
}
//50个周期为1圈
void StepL_position(int Goal_pos){
  static long current_pos = 0;

  if(current_pos<Goal_pos)
  {
    while(current_pos<Goal_pos)
    {
      digitalWrite(Step_L1,HIGH);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,HIGH);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,HIGH);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,HIGH);
      delay(StepVelo);    
      current_pos++;
    }   
    Serial.print("current_pos:");
    Serial.print(current_pos);
    Serial.print("  ");
    Serial.print("Goal_pos:");
    Serial.println(Goal_pos);
     
  }
  else if(current_pos>Goal_pos)
  {
    while(current_pos>Goal_pos)
    {
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,HIGH);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,HIGH);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,HIGH);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,HIGH);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);    
      current_pos--;
    }    
  }
  
  Serial.println(current_pos);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  init_pin();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  while(!Serial.available()>0);
  long goal_pos = Serial.parseInt();
  StepL_position(goal_pos);

}
