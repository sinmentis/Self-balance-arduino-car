
int xPin=A1;
int yPin=A0;

int out1=8;     //output1 for HT12E IC
int out2=9;     //output1 for HT12E IC
int out3=10;    //output1 for HT12E IC
int out4=11;    //output1 for HT12E IC

void setup(){
  Serial.begin(9600);
  pinMode(xPin,INPUT);
  pinMode(yPin,INPUT);

  pinMode(out1,OUTPUT);
  pinMode(out2,OUTPUT);
  pinMode(out3,OUTPUT);
  pinMode(out4,OUTPUT);

}


void loop() 
{
  int xval=analogRead(xPin);
  int yval=analogRead(yPin);
  Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
  Serial.println();
  

 

  if ((xval>340 && xval<380) && (yval>320 && yval<350)) //stop
  {
    digitalWrite(out1,LOW);  
    digitalWrite(out2,LOW);   
    digitalWrite(out3,LOW);   
    digitalWrite(out4,LOW);
    Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
  Serial.print("stop");
  Serial.println();

  } 

  else 
  { 
    if ((xval>320 && xval<380) && (yval>250 && yval<310)) //forward
   {
     digitalWrite(out1,LOW);  
     digitalWrite(out2,LOW);   
     digitalWrite(out3,LOW);  
     digitalWrite(out4,HIGH);
     Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
  Serial.print("forward");
  Serial.println();

      
    }
    if ((xval>335 && xval<380) && (yval>350 && yval<400)) //backward
  {
   digitalWrite(out1,LOW);   
   digitalWrite(out2,LOW);  
   digitalWrite(out3,HIGH);   
   digitalWrite(out4,LOW);
   Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
  Serial.print("backward");
  Serial.println();

      
    }   

    if ((xval>380 && xval<420) && (yval>330 && yval<370)) //left
    {
      digitalWrite(out1,LOW);  
      digitalWrite(out2,HIGH);   
      digitalWrite(out3,LOW);   
      digitalWrite(out4,LOW);
      Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
 Serial.print("left");
  Serial.println();

     }


    if ((xval>280 && xval<320) && (yval>320 && yval<360))//right
    {
      digitalWrite(out1,HIGH);  
      digitalWrite(out2,LOW);   
      digitalWrite(out3,LOW);   
      digitalWrite(out4,LOW);
      Serial.print(xval);
  Serial.print(", ");
  Serial.println(yval);
  Serial.print(", ");
  Serial.print("right");
  Serial.println();

      
    }
  }
  

}
