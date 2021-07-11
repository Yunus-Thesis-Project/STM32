/*
 * Rentang Servo --> 400 = 0 derajat; 2400 = 180 derajat; 1360 = 90 derajat
 * yaw = 0 - 180 (kiri - kanan)
 * end effector = 0-40 (tutup-buka)
 * pitch = 0-130 (bawah-atas)
 * front = 90 - 180(tengah - depan)
 * *yaw,end,pitch,front, pid, locked, base, kelas, konveyor# 
 * *90,20,30,85,0,1,15,1,1#   --> Posisi Awal
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
int basePin = PB6;
int konveyorPin = PB7;
int pulseTime;
int dtt;
int sementara = 90;
int counter = 0;
int millis_state = 1;
int konveyorCount = 0;

float yawSementara = 95;
float endSementara = 20;
float pitchSementara = 30;
float frontSementara = 85;
float baseSementara = 15;
float PIDSementara = 0;
float kelasSementara = 1;
float sudutKelas = 30;
float konveyorSementara = 1;
float locked = 0;

char incomingByte = 0;

String dataIn;
String dt[10];

boolean parsing;
boolean finish_place = false;
boolean konveyorState = false;

unsigned long int servo_delay;

void move_multiple_servo(float yaw, float endf, float pitch, float front, float base){
  move_servo(yawPin, yaw);
  move_servo(endPin, endf);
  move_servo(pitchPin, pitch);
  move_servo(frontPin, front);
  move_servo(basePin, base);
}

int pick_place(float locked_yaw = 90, float kelas = 1){
  
  if(kelas == 1){
    sudutKelas = 30;
  }else if(kelas == 2){
    sudutKelas = 60;
  }
  else if(kelas == 3){
    sudutKelas = 90;
  }
  else if(kelas == 4){
    sudutKelas = 135;
  }
  else{
    sudutKelas = 170;
  }
   
  if(millis_state){
    servo_delay = millis();
    millis_state = 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 0){
    Serial1.println("Servo di Posisi Awal");
    move_multiple_servo(locked_yaw, 20, 55, 85, 15);
  }else if(counter == 0){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1000 && counter == 1){
    Serial1.println("Servo Membuka end");
    move_multiple_servo(locked_yaw, 50, 55, 85, 15);
  }else if(counter == 1){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 2){
    Serial1.println("Servo maju");
    move_multiple_servo(locked_yaw, 50, 55, 160, 15);
  }else if(counter == 2){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 3){
    Serial1.println("Servo Mengambil");
    move_multiple_servo(locked_yaw, 5, 55, 160, 15);
  }else if(counter == 3){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=2500 && counter == 4){
    Serial1.println("Servo Kembali");
    move_multiple_servo(locked_yaw, 5, 65, 85, 15);
  }else if(counter == 4){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1500 && counter == 5){
    Serial1.println("Servo Berputar 180 derajat");
    move_multiple_servo(locked_yaw, 5, 65, 85, 190); 
  }else  if(counter == 5){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 6){
    Serial1.println("Servo ke Lokasi Kelas");
    move_multiple_servo(sudutKelas, 5, 65, 85, 190); 
  }else  if(counter == 6){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 7){
    Serial1.println("Servo Maju");
    move_multiple_servo(sudutKelas, 5, 65, 165, 190);
  }else  if(counter == 7){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=1000 && counter == 8){
    Serial1.println("Servo Melepaskan");
    move_multiple_servo(sudutKelas, 60, 65, 165, 190);
  }else  if(counter == 8){
    counter++;
    millis_state = 1;
    return 0;
  }

  if(millis() - servo_delay <=1000 && counter == 9){
    Serial1.println("Servo Kembali");
    move_multiple_servo(sudutKelas, 60, 65, 85, 190);
  }else  if(counter == 9){
    counter++;
    millis_state = 1;
    return 0;
  }
  
  if(millis() - servo_delay <=3000 && counter == 10){
    Serial1.println("Servo kembali ke posisi");
    move_multiple_servo(90, 20, 30, 85, 15);
  }else if(counter == 10){
    counter = 0;
    millis_state = 1;
    locked = 0;
    finish_place = true;
    parsing = false;
    dataIn = "";
    Serial1.println("Wait a second ...");
    delay(2000);
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
  Serial1.begin(57600);
  pinMode(yawPin,OUTPUT);
  pinMode(endPin,OUTPUT);
  pinMode(pitchPin,OUTPUT);
  pinMode(frontPin,OUTPUT);
  pinMode(basePin, OUTPUT);
  pinMode(konveyorPin, OUTPUT);
  
  Serial1.println("Init Servo");

  init_pixel = 150;
  init_p = 100;
  error_mea = 10;
}

void loop()
{
  if(Serial1.available()>0 && locked == 0){
    char inChar = (char)Serial1.read();
    dataIn += inChar;
    if (inChar == '\n'){
      Serial1.println(dataIn);
      if(dataIn.length()<20){
        parsing = false; 
        dataIn = ""; 
      }else{
        parsing = true;
      }
      
    }
  }

  if(parsing && locked == 0){
    dtt = parsingData(dataIn);
    float sData[dtt];
    for(int x = 0; x<dtt; ++x){
      sData[x] = dt[x].toFloat();
    }
    Serial1.print("yaw : ");
    Serial1.print(sData[0],1);
    Serial1.print(" end : ");
    Serial1.print(sData[1],1);
    Serial1.print(" pitch : ");
    Serial1.print(sData[2],1);
    Serial1.print(" front : ");
    Serial1.print(sData[3],1);
    Serial1.print(" PID : ");
    Serial1.print(sData[4],1);
    Serial1.print(" Lock : ");
    Serial1.print(sData[5],1);
    Serial1.print(" Base : ");
    Serial1.print(sData[6],1);
    Serial1.print(" Kelas : ");
    Serial1.print(sData[7],1);
    Serial1.print(" Konveyor : ");
    Serial1.print(sData[8],1);
    Serial1.println("");
    

    yawSementara = sData[0];
    endSementara = sData[1];
    pitchSementara = sData[2];
    frontSementara = sData[3];
    PIDSementara = sData[4];
    locked = sData[5];
    baseSementara = sData[6];
    kelasSementara = sData[7];
    konveyorSementara = sData[8];

    if(konveyorState){
      if(konveyorSementara == 1){
        konveyorCount++;
      }else{
        konveyorCount = 0;
      }

      if(konveyorCount>50){
        konveyorState = false;
        konveyorCount = 0;
        dPID = 90;
      }
    }else{
      if(konveyorSementara == 1){
        digitalWrite(konveyorPin, HIGH);
      }else{
        digitalWrite(konveyorPin, LOW);
        konveyorState = true;
      }  
    }
    
    if(finish_place){
      PIDSementara = 0;
      dPID = 90;
      finish_place = false;
    }
    
    if(locked == 1){
      Serial1.println("Target Locked");
      digitalWrite(konveyorPin, LOW);
    }
    else{
      dPID += PIDSementara;
      if(dPID<0) dPID = 0;
      else if(dPID>180) dPID = 180;
      move_multiple_servo(dPID, 20, 30, 85, 15);
    }
      
    parsing = false;
    dataIn = "";
  }

  if(locked == 1){
    pick_place(dPID, kelasSementara);
  }
//  else if (detected){
//    Serial1.println("Tracking");
//    move_multiple_servo(dPID, 0, 30, 90);
//    detected = 0;
//  }
//  else{
////    Serial1.println("Move Multi Servo");
//    move_multiple_servo(yawSementara, endSementara, pitchSementara, frontSementara, baseSementara);
//  }
}
