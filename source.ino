#include <TM1637.h> //Led 4
#include <EEPROM.h> // EEPROM ngoài
#include <ShiftRegister74HC595.h> //Led 2
#include <avr/io.h>
#include <avr/interrupt.h>

//Khai báo chân led 2
#define Clockpin 3 //SH_CP (chân 11)
#define Datapin 2 //DS (chân 14)
#define Latchpin 4 //ST_CP (chân 12)

//Khai báo chân điện áp
const int analogPin = A0;
const float referenceVoltage = 5.0; // Điện áp tham chiếu của Arduino
const float voltageThreshold = 4.0; // Ngưỡng điện áp để xác định sự giảm điện áp

const int a[10] ={0b00111111,0b00000110,0b01011011,0b01001111,0b01100110,0b01101101,0b01111101,0b00000111,0b01111111,0b01101111}; //mảng số
byte digit1 = 0;
byte digit2 = 0;

volatile unsigned long counter;

// Nút nhấn
#define buttonPinUp 8   // Nút nhấn tăng kết nối chân D8
#define buttonPinDown 9 // Nút nhấn giảm kết nối chân D9

int lastButtonStateUp = LOW;    // Trạng thái nút tăng trước đó
int lastButtonStateDown = LOW;  // Trạng thái nút giảm trước đó

unsigned long lastDebounceTimeUp = 0;   // Thời gian lần nhấn trước của nút tăng
unsigned long lastDebounceTimeDown = 0; // Thời gian lần nhấn trước của nút giảm
const unsigned long debounceDelay = 100; // Thời gian chống rung (ms)

//EEPROM
int address = 1;
int ReadValue;
int tempRead = 0;

void setup() {
  Serial.begin(9600); // baud 9600
  pinMode(buttonPinUp, INPUT);
  pinMode(buttonPinDown, INPUT);
  analogReference(DEFAULT);
// output led 2
  pinMode (Clockpin, OUTPUT);
  pinMode (Datapin, OUTPUT);
  pinMode (Latchpin, OUTPUT);
// Timer1 và counter
  EEPROM.get(address, tempRead);
    if (tempRead == 0) {
      counter = 0;
    } else {
      counter = tempRead * 4;
      EEPROM.write(address, 0);
    }
  Timer1setup();
}

void loop() {
  // put your main code here, to run repeatedly:
  UpdateDisplay(); // cập nhật hiển thị
  float averageVoltage = analogRead(analogPin) * (referenceVoltage / 1023.0);
  delay(0.1);
  if (averageVoltage < voltageThreshold) {
    EEPROM.put(address, counter / 4);
  }
}
//Check
int checkVoltageThreshold(float voltage) {
  return (voltage < voltageThreshold) ? 1 : 0;
}
//Display
void UpdateDisplay() {
  digit1 = (counter / 10) % 10;
  digit2 = (counter / 100) % 10;
  digitalWrite(Latchpin, LOW);
  shiftOut(Datapin, Clockpin, MSBFIRST, ~a[digit1]);
  shiftOut(Datapin, Clockpin, MSBFIRST, ~a[digit2]);
  digitalWrite(Latchpin, HIGH);
  //Serial.println(DisplayLED);

  //Button

    // Đọc trạng thái nút nhấn tăng
  int readingUp = digitalRead(buttonPinUp);

  // Nếu trạng thái nút thay đổi (tăng hoặc giảm)
  if (readingUp != lastButtonStateUp) {
    lastDebounceTimeUp = millis();
  }

  // Kiểm tra nếu thời gian chống rung đã hết
  if ((millis() - lastDebounceTimeUp) > debounceDelay) {
    // Nếu trạng thái nút là HIGH (nút được nhấn)
    if (readingUp == HIGH && lastButtonStateUp == LOW) {
      counter++;
    }
  }

  // Đọc trạng thái nút nhấn giảm
  int readingDown = digitalRead(buttonPinDown);

  // Nếu trạng thái nút thay đổi (tăng hoặc giảm)
  if (readingDown != lastButtonStateDown) {
    lastDebounceTimeDown = millis();
  }

  // Kiểm tra nếu thời gian chống rung đã hết
  if ((millis() - lastDebounceTimeDown) > debounceDelay) {
    // Nếu trạng thái nút là HIGH (nút được nhấn)
    if (readingDown == HIGH && lastButtonStateDown == LOW) {
      counter--;
    }
  }

  // Lưu lại trạng thái nút hiện tại
  lastButtonStateUp = readingUp;
  lastButtonStateDown = readingDown;
}
//Timer1
void Timer1setup() {  // Khởi tạo Timer1
  cli();  // Tắt ngắt toàn cầu
  TCCR1A = 0;  // Đặt TCCR1A về 0
  TCCR1B = 0;  // Đặt TCCR1B về 0
  TIMSK1 = 0;
  // Đặt bộ chia xung nhịp cho Timer1 là 1024
  TCCR1B |= (1 << CS11) | (1 << CS10);
  // Đặt giá trị so sánh cho Timer1 để tạo ra ngắt mỗi 1 giây
  TCNT1 = 40536;
  // Bật ngắt so sánh cho Timer1
  //TIMSK1 |= (1 << OCIE1A);
  TIMSK1 = (1 << TOIE1);                  // Overflow interrupt enable 
  sei();  // Bật ngắt toàn cầu
}

ISR(TIMER1_OVF_vect) {
  TCNT1 = 40536;
counter++;// Tăng biến đếm mỗi giây*/
  if (counter > 999) {
    counter = 0;  // Trở về 0 khi counter đạt 99
  }
}
void ButtonPG() {
    // Đọc trạng thái nút nhấn tăng
  int readingUp = digitalRead(buttonPinUp);

  // Nếu trạng thái nút thay đổi (tăng hoặc giảm)
  if (readingUp != lastButtonStateUp) {
    lastDebounceTimeUp = millis();
  }

  // Kiểm tra nếu thời gian chống rung đã hết
  if ((millis() - lastDebounceTimeUp) > debounceDelay) {
    // Nếu trạng thái nút là HIGH (nút được nhấn)
    if (readingUp == HIGH && lastButtonStateUp == LOW) {
      counter++;
    }
  }

  // Đọc trạng thái nút nhấn giảm
  int readingDown = digitalRead(buttonPinDown);

  // Nếu trạng thái nút thay đổi (tăng hoặc giảm)
  if (readingDown != lastButtonStateDown) {
    lastDebounceTimeDown = millis();
  }

  // Kiểm tra nếu thời gian chống rung đã hết
  if ((millis() - lastDebounceTimeDown) > debounceDelay) {
    // Nếu trạng thái nút là HIGH (nút được nhấn)
    if (readingDown == HIGH && lastButtonStateDown == LOW) {
      counter--;
    }
  }

  // Lưu lại trạng thái nút hiện tại
  lastButtonStateUp = readingUp;
  lastButtonStateDown = readingDown;
}
