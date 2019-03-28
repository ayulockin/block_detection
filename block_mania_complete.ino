#define lc 0 // black 
#define sc 1 //white

/////////////motor pinout/////////////////

#define lm1 3 
#define lm2 5
#define rm1 6
#define rm2 9 

int s1,s2,s3,s4,s5,s6,s7,s8;

/////////////pid parameters////////////////

float error = 0;
float prev_error = 0;
float Kp = 37.5;
float Kd = 75;  //100/150 
float P = 0;
float D = 0;
float pd = 0;

int lms,rms;
int bs = 130;  

int left_max = 200;
int right_max = 202; //based on motor speed diff
int left_min = 0;
int right_min = 0;

///////////////sharp turn parameters///////////////

int left=0,right=0;
int leftthres = 300;    //150
int rightthres = 300;   //150 

int leftacutethres = 700;
int rightacutethres = 700;


/////////////brake parameters///////////////

int brake_count = 0;
int brake_thres = 400;

/////////////block mania parameters/////////////

int b;
int len;
int len_small;
int len_big;
int i=0;
bool flag = false;

double block_counter=0;
double block_threshold = 2400;

void setup() {
  pinMode(lm1, OUTPUT);
  pinMode(lm2, OUTPUT);
  pinMode(rm1, OUTPUT);
  pinMode(rm2, OUTPUT);
  pinMode(4,OUTPUT);
  digitalWrite(4,LOW);

  left = 1000;
  right = 1000;

}

void loop() {
  readsensors();
  error_map();

  if (s1==lc){
    left = 0;
  }
  if (s8==lc){
    right = 0;
  }

   if (b==HIGH){
    flag = true;
    while(flag==true){
      block_counter = block_counter + 1;
      readsensors();
      if(b==LOW){
        flag= false; 
        if(block_counter>block_threshold){
           len_big++;
           tone(10,5000,20);
        }
        else if(block_counter < block_threshold){
          len_small++;
          int a = 0;
          if(a==0){digitalWrite(4,HIGH);a = 1;}
          if(a==1){digitalWrite(4,LOW);a=0;}
        }
      }
    }
  }
  block_counter = 0;

  if((s2==lc)&&(s7==lc)){     //intersection blink
     digitalWrite(LED_BUILTIN,HIGH);
  }
  else{
     digitalWrite(LED_BUILTIN,LOW);
  } 

  if (left==0 && right ==0){ //for braking
    if (brake_count>15000){
      brake_count=15000;
    }
    brake_count++;
  }
  else{
    brake_count = 0;
  }

  if ((s1!=lc)&&(s2!=lc)&&(s3!=lc)&&(s4!=lc)&&(s5!=lc)&&(s6!=lc)&&(s7!=lc)&&(s8!=lc)){
    if ((left>leftthres)&&(right<rightthres)){//right sharp turns
    brake();
    //delay(5);
    readsensors();
    while((s4!=lc)&&(s5!=lc)){
      sharp_right_turn();
      readsensors();
    }
    brake();
    //delay(10);
    //left = 1000;
    //right = 1000;
    }
    else if ((left<leftthres)&&(right>rightthres)){//left sharp turn
      brake();
      //delay(5);
      readsensors();
      while((s4!=lc)&&(s5!=lc)){
        sharp_left_turn();
        readsensors();
      }
      brake();
      //left = 1000;
      //right = 1000;
     
    }

  }

  else if((s1==lc)&&(s2==lc)&&(s3==lc)&&(s4==lc)&&(s5==lc)&&(s6==lc)&&(s7==lc)&&(s8==lc)){
    if (brake_count>brake_thres){  //end of maze
       stop_end();
       for(int j=0;j<len_big;j++){
        digitalWrite(13,HIGH);
        delay(1000);
        digitalWrite(13,LOW);
        delay(1000);
       }
       delay(2000);
       for(int j=0;j<len_small;j++){
        digitalWrite(4,HIGH);
        delay(1000);
        digitalWrite(4,LOW);
        delay(1000);
       }
       brake();
       delay(10000);
  }
  }

  pid();
  left=left+1;
  right=right+1;
  if(left>20000){//20000
    left=1000;
  }
  if(right>20000){
    right=1000;
  }
}

void afterstop(){

}

//////////////////////sensor get value//////////////////////

void readsensors(){
  s1 = digitalRead(A0);
  s2 = digitalRead(A1);
  s3 = digitalRead(A2);
  s4 = digitalRead(A3);
  s5 = digitalRead(A4);
  s6 = digitalRead(A5);
  s7 = digitalRead(2);
  s8 = digitalRead(7);
  b = digitalRead(12);
}

///////////////////////error definition/////////////////////
 
void error_map(){ 
  if((s3==sc)&&(s4==lc)&&(s5==lc)&&(s6==sc))
   error=0;
   else if((s3==lc)&&(s4==lc)&&(s5==lc)&&(s6==sc))
   error=-0.5;
  else if((s3==sc)&&(s4==lc)&&(s5==sc)&&(s6==sc))
   error=-0.5;
  else if((s3==sc)&&(s4==lc)&&(s5==lc)&&(s6==lc))
   error=0.5;
  else if((s3==sc)&&(s4==sc)&&(s5==lc)&&(s6==sc))
   error=0.5;
  
  else if((s2==sc)&&(s3==lc)&&(s4==lc)&&(s5==sc))
   error=-1; 
  else if((s4==sc)&&(s5==lc)&&(s6==lc)&&(s7==sc))
   error=1;
  
  else if((s2==lc)&&(s3==lc)&&(s4==lc)&&(s5==sc))
   error=-1.5;
  else if((s2==sc)&&(s3==lc)&&(s4==sc)&&(s5==sc))
   error=-1.5; 
  else if((s4==sc)&&(s5==lc)&&(s6==lc)&&(s7==lc))
   error=1.5;
  else if((s4==sc)&&(s5==sc)&&(s6==lc)&&(s7==sc))
   error=1.5;
   
  else if((s1==sc)&&(s2==lc)&&(s3==lc)&&(s4==sc))
   error=-2;
  else if((s5==sc)&&(s6==lc)&&(s7==lc)&&(s8==sc))
   error=2;
  
  else if((s1==lc)&&(s2==lc)&&(s3==lc)&&(s4==sc))
   error=-2.5;
  else if((s1==sc)&&(s2==lc)&&(s3==sc)&&(s4==sc))
   error=-2.5;
  else if((s5==sc)&&(s6==lc)&&(s7==lc)&&(s8==lc))
   error=2.5;
  else if((s5==sc)&&(s6==sc)&&(s7==lc)&&(s8==sc))
   error=2.5;
   
  else if((s1==lc)&&(s2==lc)&&(s3==sc))
   error=-3;
  else if((s6==sc)&&(s7==lc)&&(s8==lc))
   error=3;
      
  if((s1==lc)&&(s2==sc)&&(s8==sc))
   error=-3.5; 
  if((s1==sc)&&(s7==sc)&&(s8==lc))
   error=3.5;
}

///////////////////////pid computation/////////////////////

void pid(){
  readsensors();
  error_map();

  pd = Kp*error + Kd*(error - prev_error);
 
  lms = bs + pd;
  rms = bs - pd;

  if(lms>=left_max)
  {
   lms=left_max;
  }
 
  else if(lms<=left_min)
  {
   lms=left_min;
  }
   
  if(rms>=right_max)
  {
   rms=right_max;
  }
  
  else if(rms<=right_min)
  {
   rms=right_min;
  }

  analogWrite(lm1,lms+2);
  analogWrite(lm2,0);
  analogWrite(rm1,rms);
  analogWrite(rm2,0);

  prev_error = error;

}

/////////////////////////intersection handling////////////////////

void stop_end(){
  brake();
  delay(500);
}

////////////////////////////turn functions////////////////////////

void straight(){
  analogWrite(lm1,110);
  analogWrite(lm2,0);
  analogWrite(rm1,110);
  analogWrite(rm2,0);
}

void sharp_left_turn(){ 
  analogWrite(lm1,0);
  analogWrite(lm2,110);
  analogWrite(rm1,110);
  analogWrite(rm2,0);  
}

void sharp_right_turn(){
  analogWrite(lm1,110);//110->110->140
  analogWrite(lm2,0);
  analogWrite(rm1,0);
  analogWrite(rm2,110);  
}

void sharp_accute_left_turn(){
  analogWrite(lm1,0);
  analogWrite(lm2,110);
  analogWrite(rm1,110);
  analogWrite(rm2,0);  
}

void sharp_accute_right_turn(){
  analogWrite(lm1,110);
  analogWrite(lm2,0);
  analogWrite(rm1,0);
  analogWrite(rm2,110);  
}

void about_turn(){
  analogWrite(lm1,0);
  analogWrite(lm2,110);
  analogWrite(rm1,110);
  analogWrite(rm2,0);  
}

void brake(){
  analogWrite(lm1,255);
  analogWrite(lm2,255);
  analogWrite(rm1,255);
  analogWrite(rm2,255); 
}
