
/*
 * Rentang Servo --> 400 = 0 derajat; 2400 = 180 derajat; 1360 = 90 derajat
 * yaw = 0 - 180 (kiri - kanan)
 * end effector = 0-40 (tutup-buka)
 * pitch = 0-130 (bawah-atas)
 * front = 90 - 180(tengah - depan)
 * *yaw,end,pitch,front, pid, locked# 
 * *90,0,65,90,150,1#   --> Posisi Awal
 * 
 */

float init_pixel;
float init_p;
float init_mea;
float error_mea;
float dPID = 90;

int detected = 0;
int yawPin = PA0;
int endPin = PA1;
int pitchPin = PB4;
int frontPin = PB5;
int pulseTime;
int dtt;
int sementara = 90;
int counter = 0;
int locked = 0;
int millis_state = 1;

float yawSementara = 90;
float endSementara = 0;
float pitchSementara = 65;
float frontSementara = 90;

char incomingByte = 0;

String dataIn;
String dt[10];

boolean parsing;

unsigned long int servo_delay;

void move_multiple_servo(float yaw, float endf, float pitch, float front){
  move_servo(yawPin, yaw);
  move_servo(endPin, endf);
  move_servo(pitchPin, pitch);
  move_servo(frontPin, front);
}

int pick_place(float locked_yaw = 90){
  /*
   * *90,0,65,90#   --> Posisi Awal
   * *90,30,65,160# --> End terbuka dan front maju
   * *90,0,65,160#  --> End tertutup (pick)
   * *90,0,90,90#   --> Pitch naik dan front mundur
   * *150,0,90,90#  --> Ganti YAW ke arah lokasi box
   * *150,0,90,160# --> front maju
   * *150,30,90,160# --> end terbuka (place)
   */
  if(millis_state){
    servo_delay = millis();
    millis_state = 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 0){
    Serial1.println("Servo di Posisi Awal");
    move_multiple_servo(locked_yaw, 0, 65, 90);
  }else if(counter == 0){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1000 && counter == 1){
    Serial1.println("Servo Membuka end");
    move_multiple_servo(locked_yaw, 30, 65, 90);
  }else if(counter == 1){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 2){
    Serial1.println("Servo maju");
    move_multiple_servo(locked_yaw, 30, 65, 160);
  }else if(counter == 2){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 3){
    Serial1.println("Servo Mengambil");
    move_multiple_servo(locked_yaw, 0, 65, 160);
  }else if(counter == 3){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 4){
    Serial1.println("Servo Kembali");
    move_multiple_servo(locked_yaw, 0, 65, 90);
  }else if(counter == 4){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1000 && counter == 5){
    Serial1.println("Servo ke atas");
    move_multiple_servo(locked_yaw, 0, 90, 90); //Ganti 150 Menjadi Lokasi Box
  }else  if(counter == 5){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 6){
    Serial1.println("Servo ke Lokasi");
    move_multiple_servo(150, 0, 90, 90); //Ganti 150 Menjadi Lokasi Box
  }else  if(counter == 6){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 7){
    Serial1.println("Servo Maju");
    move_multiple_servo(150, 0, 90, 160);
  }else  if(counter == 7){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 8){
    Serial1.println("Servo Melepaskan");
    move_multiple_servo(150, 30, 90, 160);
  }else  if(counter == 8){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1000 && counter == 9){
    Serial1.println("Servo Melepaskan");
    move_multiple_servo(150, 30, 90, 90);
  }else  if(counter == 9){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 10){
    Serial1.println("Servo kembali ke posisi");
    move_multiple_servo(90, 0, 65, 90);
  }else if(counter == 10){
    counter = 0;
    millis_state = 1;
    locked = false;
    return 0;
  }

  return 0;
}

void kalman(float data, float *p, float *pixel){
  float kg, est, e_est;

  kg = *p/(*p + error_mea);
  est = *pixel + kg*(data-*pixel);
  e_est = (1-kg)*(*p);

  *p = e_est;
  *pixel = est;
}

int parsingData(String data){
  int j = 0;

  for(int i = 0; i<data.length(); ++i){
    if(data[0] != '*'){
      return 0;
    }else if(data[i] == '*'){
      dt[j] = "";
      continue;
    }
    
    if(data[i] == '#' || data[i] == ','){
      j++;
      dt[j] = "";
    }
    else{
      dt[j] = dt[j] + data[i];
    }
  }
  return j;
}


void move_servo(int pin, int degree){
  if(degree==90){
    degree = 1360;
  }else if(degree<90){
    degree = 400 + (96/9)*degree;
  }else{
    degree -= 90;
    degree = 1360 + ((104/9)*degree);
  }
  
  pulseTime = degree;
  digitalWrite(pin, HIGH);
  delayMicroseconds(pulseTime);
  digitalWrite(pin, LOW);
  delayMicroseconds(25);
}

void setup()
{
  Serial1.begin(9600);
  pinMode(yawPin,OUTPUT);
  pinMode(endPin,OUTPUT);
  pinMode(pitchPin,OUTPUT);
  pinMode(frontPin,OUTPUT);
  
  Serial1.println("Init Servo");

  init_pixel = 150;
  init_p = 100;
  error_mea = 10;
}

void loop()
{
  if(Serial1.available()>0){
    char inChar = (char)Serial1.read();
    dataIn += inChar;
    if (inChar == '\n'){
      parsing = true;
    }
  }

  if(parsing){
    dtt = parsingData(dataIn);
    float sData[dtt];
    for(int x = 0; x<dtt; ++x){
      sData[x] = dt[x].toFloat();
    }
    Serial1.print("yaw : ");
    Serial1.print(sData[0]);
    Serial1.print(" end : ");
    Serial1.print(sData[1]);
    Serial1.print(" pitch : ");
    Serial1.print(sData[2]);
    Serial1.print(" front : ");
    Serial1.print(sData[3]);
    Serial1.println("");
    

    yawSementara = sData[0];
    endSementara = sData[1];
    pitchSementara = sData[2];
    frontSementara = sData[3];
    locked = sData[5];
    
    //if(!locked)
//    sementara = sData[4];
//    Serial1.print(sData[4]);
//    dPID += sData[4];
//    if(dPID<0) dPID = 0;
//    else if(dPID>180) dPID = 180;
//    move_servo(yawPin, dPID);
    
    parsing = false;
    dataIn = "";
  }

  if(locked){
    Serial1.println("Jalan");
    pick_place(90);
  }else if (detected){
    move_multiple_servo(dPID, 0, 65, 90);
  }else{
    move_multiple_servo(90, 0, 65, 90);
  }
  
}
