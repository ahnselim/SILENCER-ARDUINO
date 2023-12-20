#include "arduinoFFT.h"
#define SAMPLES 128             // SAMPLES-pt FFT. 2의 제곱수여야함. 최대 128까지
#define SAMPLING_FREQUENCY 2048  // Ts = Based on Nyquist, 가장 높을 것으로 예상한 주파수의 2배.
int echo = 7; //echo 핀
int trig = 12; // trig 핀
#define ENABLE_A 10 //모터 A 제어
#define IN1_A 9
#define IN2_A 8
#define SoundSensorPin1 A2
#define SoundSensorPin2 A3
#define VREF 5.0
float soundValue1, soundValue2; // 각 마이크에서 읽은 아날로그 값
int mode = 0; // 0은 자동 모드, 1은 수동 모드



arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;

double vReal[SAMPLES]; // create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; // create vector of size SAMPLES to hold imaginary values
int soundSensorPin = A0; // 사운드 센서 A0
int tempSensorPin = A1;  // 온도 센서 A1
float temp;

void motor_dir(int dir, int speed=85)
{
  if(dir==0) //순방향
  {
    digitalWrite(IN1_A,HIGH);
    digitalWrite(IN2_A,LOW);
    analogWrite(ENABLE_A, speed);
    delay(100);
  }
  else if(dir==1) //역방향
  {
    digitalWrite(IN1_A,LOW);
    digitalWrite(IN2_A,HIGH);
    analogWrite(ENABLE_A, speed);
    delay(100);
  }
  else if(dir==2) //멈춤
  { 
    digitalWrite(IN1_A,LOW);
    digitalWrite(IN2_A,LOW);
    digitalWrite(ENABLE_A,0);
  }
 
}

void setup()
{
    pinMode(ENABLE_A, OUTPUT);
    pinMode(IN1_A, OUTPUT);
    pinMode(IN2_A, OUTPUT);
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
    Serial.begin(115200); // 시리얼 모니터
    samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); // microsecond 주기
    if (Serial.available() > 0) {
    int dir = Serial.parseInt();
    motor_dir(dir);
  }
}



void loop()
{
  /*데시벨 측정하는 코드*/
  soundValue1 = analogRead(SoundSensorPin1)/1024.0*VREF; // 첫 번째 마이크에서 아날로그 값 읽기
  soundValue2 = analogRead(SoundSensorPin2)/1024.0*VREF; // 두 번째 마이크에서 아날로그 값 읽기

  float decibel1 = soundValue1*50.0/2; // 첫 번째 마이크 값을 데시벨로 변환
  float decibel2 = soundValue2*50.0/2; // 두 번째 마이크 값을 데시벨로 변환
  float decibel3 =decibel1-decibel2;

  Serial.print("Sound Level Mic1 (dB): ");
  Serial.println(decibel1,1);
  
  Serial.print("Sound Level Mic2 (dB): ");
  Serial.println(decibel2,1);

  Serial.print("Decibel reduction (dB) : ");
  Serial.println(decibel3,1);


  /* 초음파 센서로 거리재는 코드 */
  float cycletime;
  float distance;
  
  digitalWrite(trig, HIGH);
  delay(10);
  digitalWrite(trig, LOW);
  
  cycletime = pulseIn(echo, HIGH); 
  
  distance = ((340 * cycletime) / 10000) / 2;  
  
       /* Sample SAMPLES times */
    for (int i = 0; i < SAMPLES; i++)
    {
        microSeconds = micros(); // 현재의 마이크로초(microseconds)를 microSeconds 변수에 저장

        vReal[i] = analogRead(soundSensorPin); // 아날로그 사운드 센서 핀에서 값을 읽어와서 vReal[i]에 저장
        vImag[i] = 0;                          // vImag[i] 값을 0으로 설정. 복소수의 허수 부분을 나타냄

        /* 가능하다면 sample 사이 남는 시간 */
        while (micros() < (microSeconds + samplingPeriod))
        {
            // 샘플링 주기가 끝날 때까지 아무 것도 하지 않고 대기
        }
    }

     int val = analogRead(tempSensorPin);
     temp = (5.0*val*100)/1024; //온도 센서에서 아날로그 값을 읽어와서 온도를 계산


    /* Perform FFT on samples */
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD); // Fast Fourier Transform을 수행하기 전에 윈도잉(Windowing) 과정을 진행
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD); //윈도잉이 적용된 신호에 대해 실제 FFT를 수행하는 코드. FFT는 시간 도메인의 신호를 주파수 도메인으로 변환하는데 사용됨.
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES); //FFT를 통해 얻은 복소수 결과를 크기로 변환하는 코드. 각 주파수 성분의 크기를 얻을 수 있음. 


    /* peak 주파수를 찾고 파장의 길이를 계산하고 그에 따른 디스크 위치 계산*/
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY)-5;// FFT를 통해 계산된 주파수에서 가장 높은 피크 찾음
    double speedd = 331.3 * sqrt((temp+273.15) / 273.15);// 온도에 따른 소리의 속도를 계산
    double wavelength = (speedd / peak) * 100; // 피크 주파수에 따른 파장을 계산
    double wavelength4=wavelength/4;
    double diskdis= 55-(55-(wavelength4+5))-2; // 파장의 1/4 값을 계산하고, 이를 기반으로 디스크의 위치를 계산
    
    Serial.print("Frequency : ");
    Serial.print(peak);
    Serial.println(" Hz"); // Peak 주파수 출력


    Serial.print("Celsius : ");
    Serial.println(temp); // 온도 출력


    Serial.print("Wavelength : ");
    Serial.println(wavelength); // 파장 출력


    Serial.print("distance : ");
    Serial.println(distance); // 거리 출력

    
    Serial.print("diskdistance : ");
    Serial.println(diskdis);

    Serial.println("=======================================================================");
    delay(500);
    
  if (Serial.available()) {
    char command = Serial.read();
    if (command == '0' || command == '1' || command == '2') {
        motor_dir(command - '0');
    } else if (command == '3') {
        mode = 1 - mode; // 모드 변경. 자동 모드면 수동 모드로, 수동 모드면 자동 모드로 변경.
    }
  }

 if (mode == 0) { // 자동 모드일 때
    if(distance>=diskdis+1) {
        motor_dir(1);
    } else if(distance<diskdis-1) {
        motor_dir(0);
    } else {
        motor_dir(2);
        Serial.println("Motor Stopped");
        delay(1000);
    }
}



}
