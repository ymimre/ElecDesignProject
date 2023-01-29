#define front_echo 10
#define front_trig 11
#define left_echo 12
#define left_trig 2
#define right_echo 8
#define right_trig 13

#define mot_L1 4
#define mot_L2 5
#define mot_LE 3
#define mot_R1 6
#define mot_R2 7
#define mot_RE 9

#define magnetic A0

#define reset A2
int state;
int count, temp;
long df, dr, dl, tf, tr, tl;


#include <MPU6050_tockn.h>
#include <Wire.h>
#include <Ultrasonic.h>

MPU6050 mpu6050(Wire);
unsigned long time_now = 0;
int direction_state = 0;

Ultrasonic leftSensor(left_trig, left_echo);
Ultrasonic rightSensor(right_trig, right_echo);
Ultrasonic frontSensor(front_trig, front_echo);

void setup() {
  pinMode(front_echo, INPUT);
  pinMode(front_trig, OUTPUT);
  pinMode(left_echo, INPUT);
  pinMode(left_trig, OUTPUT);
  pinMode(right_echo, INPUT);
  pinMode(right_trig, OUTPUT);

  pinMode(mot_L1, OUTPUT);
  pinMode(mot_L2, OUTPUT);
  pinMode(mot_LE, OUTPUT);
  pinMode(mot_R1, OUTPUT);
  pinMode(mot_R2, OUTPUT);
  pinMode(mot_RE, OUTPUT);

  pinMode(magnetic, INPUT);
  //pinMode(reset, OUTPUT);

  Serial.begin(9600);
  count = 0;
  temp = 100;
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //digitalWrite(reset, HIGH);

}

void loop() {
  mpu6050.update();

  update_ultra();

  read_magnet();

  if (dr > 30 && dl > 30 && df > 30) {

    if (count >= 1) {
      stop_moving();
      exit(0);

    } else {
      mpu6050.update();
      update_ultra();

      time_now = millis();
      while (millis() < time_now + 1000) {
        turn_right();
        update_ultra();
      }


      time_now = millis();

      while (millis() < time_now + 150) {
        stop_moving();
      }

      mpu6050.update();
      direction_state = int(floor(mpu6050.getAngleZ())) + 10;
      update_ultra();

      while (dr + dl > 26 && df > 13) {
        go_forward();
        update_ultra();
        read_magnet();
      }


      while (count < 1) {
        right_wall_follower();
      }

      time_now = millis();
      while (millis() < time_now + 1000) {
        turn_right();
        update_ultra();
        read_magnet();
      }
      time_now = millis();

      while (millis() < time_now + 150) {
        stop_moving();
      }

      mpu6050.update();
      direction_state = int(floor(mpu6050.getAngleZ())) + 10;
      update_ultra();
      while (dr < 30 && dl < 30 && df < 30) {
        left_wall_follower();
      }
    }

  } else {
    right_wall_follower();
  }
}

void left_wall_follower() {
  mpu6050.update();

  update_ultra(); //Ultrasonik sensörleri çalıştırıyor

  mpu6050.update();

  read_magnet();

  if (dl > 18 && dl > 0) {

    mpu6050.update();

    time_now = millis();
    while (df > 10 && millis() < time_now + 300) { //Öndeki duvara 12cm'e kadar yaklaş
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();
    }

    time_now = millis();
    while (millis() < time_now + 150) {
      stop_moving();
    }
    update_ultra();
    read_magnet();
    /*
        if (df < 15) {
          while (df < 50) {
            turn_right();
            update_ultra();
          }

          while (dl > 8) {
            turn_right();
            dl = leftSensor.read();
          }
        } else {

    */
    time_now = millis(); //Delay kullanma gyro'yu bozuyor
    while (millis() < time_now + 950) { //Bu sayı pil durumuna göre değişebilir deneyip ayarlamak lazım
      turn_left();
    }

    //   }


    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ())) + 10;
    update_ultra();
    read_magnet();


    while (dl > 15) { //Sağda duvar görene kadar devam et. Bu arka arkaya iki kere sağa dönmesi gerektiğinde çalışmayabilir denemedim.
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();
    }

  } else if (df > 18 && df > 0) {

    //while (df > 12 && dr < 18 ) {
    mpu6050.update();
    go_forward();
    update_ultra();
    read_magnet();
    //}


  } else if (dr > 18 && dr > 0) {

    mpu6050.update();

    while (df > 10 ) {
      mpu6050.update();;
      go_forward();
      update_ultra();
      read_magnet();
    }

    time_now = millis();
    while (millis() < time_now + 150) {
      stop_moving();
    }

    update_ultra();
    read_magnet();
    /*
        time_now = millis();

        float current_df = df;
        float current_dr = dr;
        float current_dl = dl;

        time_now = millis();

         time_now = millis(); //Delay kullanma gyro'yu bozuyor
        while (millis() < time_now + 850) { //Bu sayı pil durumuna göre değişebilir deneyip ayarlamak lazım
          turn_left();
        }
    */
    while (df < 40) {
      turn_right();
      df = frontSensor.read();
      read_magnet();
    }

    while (dl > 10) {
      turn_right();
      update_ultra();
      read_magnet();
    }

    // digitalWrite(reset, LOW);
    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ())) + 10;
    update_ultra();
    read_magnet();

    while (dr > 18 && dl > 10) {
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();

    }
  } else if (dr < 15 && dl < 15 && df < 15 && dr > 0 && dl > 0 && df > 0) {

    mpu6050.update();
    update_ultra();
    read_magnet();

    while (df < 40 || dr + dl > 21) {
      turn_right();
      update_ultra();
      read_magnet();
    }
    /*
      time_now = millis();
      while (millis() < time_now + 1200) {
      mpu6050.update();
      turn_right();
      }
    */
    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ())) + 10;
    update_ultra();
    read_magnet();
  }

}

void right_wall_follower() {
  mpu6050.update();

  update_ultra(); //Ultrasonik sensörleri çalıştırıyor
  read_magnet();
  mpu6050.update();

  if (dr > 18 && dr > 0 && (dl < 18 || df < 18)) {

    mpu6050.update();

    time_now = millis();
    while (df > 10 && millis() < time_now + 300) { //Öndeki duvara 12cm'e kadar yaklaş
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();
    }

    time_now = millis();
    while (millis() < time_now + 150) {
      stop_moving();
    }
    update_ultra();
    read_magnet();
    /*
        if (df < 15) {
          while (df < 50) {
            turn_right();
            update_ultra();
          }

          while (dl > 8) {
            turn_right();
            dl = leftSensor.read();
          }
        } else {

    */
    time_now = millis(); //Delay kullanma gyro'yu bozuyor
    while (millis() < time_now + 562.5) { //Bu sayı pil durumuna göre değişebilir deneyip ayarlamak lazım
      turn_right();
    }

    //   }


    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ())) + 5;
    update_ultra();
    read_magnet();


    while (dr > 15) { //Sağda duvar görene kadar devam et. Bu arka arkaya iki kere sağa dönmesi gerektiğinde çalışmayabilir denemedim.
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();
    }

  } else if (df > 18 && df > 0) {

    //while (df > 12 && dr < 18 ) {
    mpu6050.update();
    go_forward();
    update_ultra();
    read_magnet();
    //}


  } else if (dl > 18 && dl > 0) {

    mpu6050.update();

    while (df > 10 ) {
      mpu6050.update();;
      go_forward();
      update_ultra();
      read_magnet();
    }

    time_now = millis();
    while (millis() < time_now + 150) {
      stop_moving();
    }

    update_ultra();
    read_magnet();
    /*
        time_now = millis();

        float current_df = df;
        float current_dr = dr;
        float current_dl = dl;

        time_now = millis();

         time_now = millis(); //Delay kullanma gyro'yu bozuyor
        while (millis() < time_now + 850) { //Bu sayı pil durumuna göre değişebilir deneyip ayarlamak lazım
          turn_left();
        }
    */
    while (df < 40) {
      turn_left();
      update_ultra();
      read_magnet();
    }

    while (dr > 9.75) {
      turn_left();
      update_ultra();
      read_magnet();
    }

    // digitalWrite(reset, LOW);
    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ()));
    update_ultra();
    read_magnet();

    while (dl > 18 && dr > 10) {
      mpu6050.update();
      go_forward();
      update_ultra();
      read_magnet();
    }
  } else if (dr < 15 && dl < 15 && df < 15 && dr > 0 && dl > 0 && df > 0) {

    mpu6050.update();
    update_ultra();
    read_magnet();

    while (df < 40 || dr + dl > 21) {
      turn_right();
      update_ultra();
      read_magnet();
    }
    /*
      time_now = millis();
      while (millis() < time_now + 1200) {
      mpu6050.update();
      turn_right();
      }
    */
    time_now = millis();

    while (millis() < time_now + 150) {
      stop_moving();
    }

    mpu6050.update();
    direction_state = int(floor(mpu6050.getAngleZ())) + 10;
    update_ultra();
    read_magnet();
  }

}
/*


  state = digitalRead(magnetic);
  if(state == 0) {
   if(temp > 80 || count !=1){
      count = count +1;
      temp = 0;
   }
  }

  if(count == 2) {
    stop_moving();
    exit(0);
  }
  if(df < 30 && df > 0){
    turn_right();
    delay(300);
    }else{
    go_forward();
    delay(10);
    }

    temp = temp+1;
*/

void turn_right() {
  digitalWrite(mot_R1, LOW);
  digitalWrite(mot_R2, HIGH);
  analogWrite(mot_RE, 125); //Hızları değiştirme bence yavaş olması daha iyi

  digitalWrite(mot_L1, HIGH);
  digitalWrite(mot_L2, LOW);
  analogWrite(mot_LE, 125);
}
void go_forward() {
  mpu6050.update();
  int z = int(floor(mpu6050.getAngleZ())) - direction_state;
  //int q  = z % 90;

  digitalWrite(mot_R1, HIGH);
  digitalWrite(mot_R2, LOW);
  analogWrite(mot_RE, min(max(150 - 2 * z , 100), 255));
  //analogWrite(mot_RE, 175 - 2 * q); //Açıyı floor'la ayarlıyor direction state kullanmadım çünkü gyro bozulabiliyor dönüşlerde. Floor kullanmak zorunda değilsin ama ben bunu kullandım.

  digitalWrite(mot_L1, HIGH);
  digitalWrite(mot_L2, LOW);
  analogWrite(mot_LE, min(max(150 + 2 * z , 100), 255));
  //analogWrite(mot_LE, 175 + 2 * q);

}

void stop_moving() {
  digitalWrite(mot_R1, LOW);
  digitalWrite(mot_R2, LOW);
  analogWrite(mot_RE, 0);

  digitalWrite(mot_L1, LOW);
  digitalWrite(mot_L2, LOW);
  analogWrite(mot_LE, 0);
}

void turn_left() {
  digitalWrite(mot_R1, HIGH);
  digitalWrite(mot_R2, LOW);
  analogWrite(mot_RE, 125);

  digitalWrite(mot_L1, LOW);
  digitalWrite(mot_L2, HIGH);
  analogWrite(mot_LE, 125);
}

void update_ultra() {
  dr = rightSensor.read();
  dl = leftSensor.read();
  df = frontSensor.read();
  /*
    time_now = micros();
    while (micros() < time_now + 5) {
    digitalWrite(right_trig, LOW);
    }
    time_now = micros();
    while (micros() < time_now + 10) {
    digitalWrite(right_trig, HIGH);
    }
    digitalWrite(right_trig, LOW);

    tr = pulseIn(right_echo, HIGH);
    dr = tr / 29.1 / 2;

    time_now = micros();
    while (micros() < time_now + 5) {
    digitalWrite(left_trig, LOW);
    }
    time_now = micros();
    while (micros() < time_now + 10) {
    digitalWrite(left_trig, HIGH);
    }
    digitalWrite(left_trig, LOW);

    tl = pulseIn(left_echo, HIGH);
    dl = tl / 29.1 / 2;

    time_now = micros();
    while (micros() < time_now + 5) {
    digitalWrite(front_trig, LOW);
    }
    time_now = micros();
    while (micros() < time_now + 10) {
    digitalWrite(front_trig, HIGH);
    }
    digitalWrite(front_trig, LOW);

    tf = pulseIn(front_echo, HIGH);
    df = tf / 29.1 / 2;
  */
}

void read_magnet() {
  state = digitalRead(magnetic);
  if (state == 0) {
    if (temp > 20 || count != 1) {
      count = count + 1;
      temp = 0;
    }
  }
  temp = temp + 1;
}
