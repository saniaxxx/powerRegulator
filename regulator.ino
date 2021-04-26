#include <LCD_1602_RUS.h>                                          // Библиотека для работы с LCD 1602 с русскими символами
#include <EEPROM.h>                                                // Библиотека для работы с EEPROM
#include <Wire.h>                                                  // Библиотека для работы с шиной I2C

//------------------------------------ Настройка подключения пинов к ардуино

#define detect 3                                                   // Детектор нуля
#define triac 5                                                    // Управление симистором
#define SW 7                                                       // энкодер SW кнопка
#define CLK 8                                                      // энкодер CLK
#define DT 9                                                       // энкодер DT
#define STP 10                                                     // кнопка стоп
#define RZG 11                                                     // кнопка разгон

LCD_1602_RUS lcd(0x27, 16, 2);                                     // Адрес для подключения LCD экрана через I2C

//------------------------------------ Константы

double resist_ten_max = 1050.00;                                    // Максимальное сопротивление ТЭНа, Ом
double resist_ten_min = 5.00;                                     // Минимальное сопротивление ТЭНа, Ом
double I_max_max = 30.00;                                         // Максимальный ограничивающй ток, А
double I_max_min = 5.00;                                          // Миниимальный ограничивающй ток, А

//------------------------------------ Переменные настроек

double resist_ten;                                                // Сопротивление ТЭНа
int power_ten;                                                    // Расчётная мощность ТЭНа
int power_ten_max = 220.00 * 220.00 / resist_ten_min;             // Максимальная расчетная мощность ТЭНа
int power_ten_min = 220.00 * 220.00 / resist_ten_max;             // Минимальная расчетная мощность ТЭНа
double I_max;                                                     // Ограничивающий ток
int P_max;                                                        // Ограничивающая мощность
int P_max_max;                                                    // Максимальная ограничивающая мощность
int P_max_min;                                                    // Минимальная ограничивающая мощность
int ACS_type;                                                     // Тип датчика ACS712
double ACS_coeff;                                                 // Коэффициент датчика ACS712 |5А - 0.024414063 | 20А - 0.048828125 | 30A - 0.073242188 |
int P_ust1;                                                       // Предустановка мощности 1
int P_ust2;                                                       // Предустановка мощности 2
int P_ust3;                                                       // Предустановка мощности 3
int P_ust4;                                                       // Предустановка мощности 4
byte setup_sw = false;                                            // Флаг режима настройки
int setup_step = 1;                                               // Флаг настройки текущего параметра
int menu_step = 0;                                                // Флаг меню предустановок
int ust_W = 0;                                                    // Установленная мощность
int angle = 2250;                                                 // Угол открытия симистора

//------------------------------------ Переменные для расчета RMS тока и мощности

volatile unsigned long Iism = 0;                                  // Мгновенное значение тока
volatile int cntr = 0;                                            // Счетчик в обработчике прерывания
volatile unsigned long Isumm = 0;                                 // Сумма квадратов тока
double real_I = 0;                                                // Расчетный RMS ток
int real_W = 0;                                                   // Расчетная мощность
double sqrt_I_sum = 0;                                            // Переменная для расчета квадратного корня
int zero = 0;                                                     // Детект нуля

//------------------------------------ Переменные для таймингов

unsigned long previousMillis = 0;                                 // Момент времени для регулирования мощности
unsigned long displayMillis = 0;                                  // Момент времени для обновления LCD
unsigned long encoderMillis = 0;                                  // Момент времени для энкодера

//------------------------------------ Переменные для энкодера

unsigned long encoderMillis_speed = 0;                            // Скорость вращения энкодера
int encoder_CLK = 0;                                              // Обработка вращения
int encoder_DT = 0;                                               // Обработка вращения
int encoder_CLK_prev = 0;                                         // Обработка вращения
byte set_CLK = false;                                             // Обработка нажатия
unsigned long SW_time_press = 0;                                  // Момент времени нажатия
unsigned long SW_press_time = 0;                                  // Счетчик времени нажатия

//------------------------------------ Процедура инициализации

void setup(void) {

  //------------------------------------ Инициализация вводов-выводов

  pinMode(detect, INPUT);                                         // Инициализация входа детекта нуля
  pinMode(triac, OUTPUT);                                         // Инициализация выхода управления симистором
  pinMode(CLK, INPUT_PULLUP);                                     // Подтягивание резистора входа энкодера CLK
  pinMode(DT, INPUT_PULLUP);                                      // Подтягивание резистора входа энкодера DT
  pinMode(SW, INPUT_PULLUP);                                      // Подтягивание резистора входа энкодера SW
  pinMode(STP, INPUT);                                            // Инициализация кнопки стоп
  pinMode(RZG, INPUT);                                            // Инициализация кнопки разгон

  //------------------------------------ Инициализация LCD

  lcd.init();
  lcd.backlight();
  lcd.setCursor(1, 0);
  lcd.print("Регулятор");
  lcd.setCursor(7, 1);
  lcd.print("Мощности");
  delay(2000);
  lcd.clear();

  //------------------------------------ Считывание настроек из EEPROM

  EEPROM.get(0, resist_ten);
  if (isnan(resist_ten) || resist_ten > resist_ten_max || resist_ten < resist_ten_min)
  {
    resist_ten = resist_ten_min;
    setup_sw = true;
  }
  power_ten = 220.00 * 220.00 / resist_ten;

  EEPROM.get(4, I_max);
  if (isnan(I_max) || I_max > I_max_max || I_max < I_max_min)
  {
    I_max = I_max_min;
    setup_sw = true;
  }
  P_max = round(I_max * I_max * resist_ten / 5) * 5;
  P_max_max = round(I_max_max * I_max_max * resist_ten / 5) * 5;
  P_max_min = round(I_max_min * I_max_min * resist_ten / 5) * 5;

  EEPROM.get(8, ACS_type);
  if (isnan(ACS_type) || ACS_type > 2 || ACS_type < 0)
  {
    ACS_type = 0;
    setup_sw = true;
  }
  switch (ACS_type)
  {
    case 0:
      ACS_coeff = 0.024414063;
      break;
    case 1:
      ACS_coeff = 0.048828125;
      break;
    case 2:
      ACS_coeff = 0.073242188;
      break;
  }

  EEPROM.get(10, P_ust1);
  if (isnan(P_ust1) || P_ust1 < 0)
  {
    P_ust1 = 0;
    setup_sw = true;
  }
  EEPROM.get(12, P_ust2);
  if (isnan(P_ust2) || P_ust2 < 0)
  {
    P_ust2 = 0;
    setup_sw = true;
  }
  EEPROM.get(14, P_ust3);
  if (isnan(P_ust3) || P_ust3 < 0)
  {
    P_ust3 = 0;
    setup_sw = true;
  }
  EEPROM.get(16, P_ust4);
  if (isnan(P_ust4) || P_ust4 < 0)
  {
    P_ust4 = 0;
    setup_sw = true;
  }

  if (setup_sw)
  {
    lcd.clear();
    lcd.setCursor(6, 0);
    lcd.print("Меню");
    lcd.setCursor(4, 1);
    lcd.print("настроек");
    delay(1000);
    lcd.clear();
  }

  //------------------------------------ Настройка АЦП

  ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << MUX2) | (0 << MUX1) | (1 << MUX0);
  ADCSRA = B11101111;
  TCCR1A = 0x00;
  TCCR1B = 0x00;
  TCCR1B = (0 << CS12) | (1 << CS11) | (1 << CS10);
  OCR1A = 0;
  TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);

  //------------------------------------ Вызов прерывания при детектировании нуля

  attachInterrupt(1, zero_crosss_int, RISING);

}

//------------------------------------  Прерывание по нулю

void zero_crosss_int()
{
  TCNT1 = 0;
  OCR1A = angle;
  zero++;
}

//------------------------------------ Обработка таймера по совпадению нуля

ISR (TIMER1_COMPA_vect)
{
  PORTD |=  (1 << PORTD5);
  TCNT1 = 65535 - 200;
}

ISR (TIMER1_OVF_vect)
{
  PORTD &=  ~(1 << PORTD5);
  TCNT1 = OCR1A + 1;
}

//------------------------------------ Обработка прерывания АЦП для расчета среднеквадратичного тока

ISR(ADC_vect)
{
  byte An_pin = ADCL;
  byte An = ADCH;
  Iism = ((An << 8) + An_pin) - 512;
  Iism *= Iism;
  Isumm += Iism;
  cntr++;
}

void loop(void) {

  //------------------------------------ Расчет RMS тока и мощности

  if (zero >= 8)                                               //zero - количество полупериодов для рассчета среднеквадратичного
  {
    sqrt_I_sum = Isumm / cntr; //
    real_I = sqrt(sqrt_I_sum) * ACS_coeff; //
    real_W = real_I * real_I * resist_ten;
    cntr = 0;
    Isumm = 0;
    zero = 0;
  }

  //------------------------------------ Обработка кнопки энкодера

  if (digitalRead(SW) == LOW) {
    if (millis() - SW_time_press > 1) {
      SW_press_time++;
      SW_time_press = millis();
    }
  }

  if (digitalRead(SW) == HIGH) {

    if ( SW_press_time >= 50 & !set_CLK)
    {
      if (setup_sw)
      {
        switch (setup_step) {
          case 1:
            setup_step = 2;
            set_CLK = true;
            break;
          case 2:
            setup_step = 3;
            set_CLK = true;
            P_max = round(I_max * I_max * resist_ten / 5) * 5;
            P_max_max = round(I_max_max * I_max_max * resist_ten / 5) * 5;
            P_max_min = round(I_max_min * I_max_min * resist_ten / 5) * 5;
            lcd.clear();
            break;
          case 3:
            setup_step = 4;
            set_CLK = true;
            break;
          case 4:
            setup_step = 5;
            set_CLK = true;
            lcd.clear();
            if (P_ust1 > P_max ) P_ust1 = P_max;
            if (P_ust2 > P_max ) P_ust2 = P_max;
            if (P_ust3 > P_max ) P_ust3 = P_max;
            if (P_ust4 > P_max ) P_ust4 = P_max;
            break;
          case 5:
            setup_step = 6;
            set_CLK = true;
            break;
          case 6:
            setup_step = 7;
            set_CLK = true;
            break;
          case 7:
            setup_step = 8;
            set_CLK = true;
            break;
          case 8:
            setup_step = 9;
            set_CLK = true;
            lcd.clear();
            break;
          case 9:
            setup_step = 1;
            set_CLK = true;
            lcd.clear();
            break;
        }
      }
      else
      {
        switch (menu_step)
        {
          case 0:
            menu_step = 1;
            set_CLK = true;
            lcd.clear();
            break;
          case 1:
            ust_W = P_ust1;
            menu_step = 0;
            set_CLK = true;
            lcd.clear();
            break;
          case 2:
            ust_W = P_ust2;
            menu_step = 0;
            set_CLK = true;
            lcd.clear();
            break;
          case 3:
            ust_W = P_ust3;
            menu_step = 0;
            set_CLK = true;
            lcd.clear();
            break;
          case 4:
            ust_W = P_ust4;
            menu_step = 0;
            set_CLK = true;
            lcd.clear();
            break;
        }
      }
    }

    SW_press_time = 0;
    set_CLK = false;
  }

  if (SW_press_time > 1000) {
    if (setup_sw)
    {
      EEPROM.put(0, resist_ten);
      EEPROM.put(4, I_max);
      EEPROM.put(8, ACS_type);
      EEPROM.put(10, P_ust1);
      EEPROM.put(12, P_ust2);
      EEPROM.put(14, P_ust3);
      EEPROM.put(16, P_ust4);
      setup_sw = false;
      lcd.clear();
      lcd.setCursor(1, 0);
      lcd.print("Настройки");
      lcd.setCursor(6, 1);
      lcd.print("сохранены");
      delay(1000);
      lcd.clear();
    }
    else
    {
      angle = 2250;
      ust_W = 0;
      setup_sw = true;
      setup_step = 1;
      lcd.clear();
      lcd.setCursor(6, 0);
      lcd.print("Меню");
      lcd.setCursor(4, 1);
      lcd.print("настроек");
      delay(1000);
      lcd.clear();
    }
    SW_press_time = 0;
    set_CLK = true;
  }


  //------------------------------------ Обработка вращения энкодера

  if (millis() >= (encoderMillis + 5))
  {
    encoder_CLK = digitalRead(CLK);
    encoder_DT = digitalRead(DT);
    if ((!encoder_CLK) && (encoder_CLK_prev))
    {
      if (encoder_DT)
      {
        if (setup_sw)
        {
          switch (setup_step)
          {
            case 1:                                   // Настройка сопротивления ТЭНа
              resist_ten += (millis() <= (encoderMillis_speed + 30)) ? 2.00 : (millis() <= (encoderMillis_speed + 70)) ? 1.00 : (millis() <= (encoderMillis_speed + 270)) ? 0.10 : 0.01;
              if (resist_ten > resist_ten_max ) resist_ten = resist_ten_max;
              power_ten = 220.00 * 220.00 / resist_ten;
              break;
            case 2:                                   // Настройка мощности ТЭНа
              power_ten += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 10 : 1;
              if (power_ten > power_ten_max ) power_ten = power_ten_max;
              resist_ten = 220.00 * 220.00 / (double)power_ten;
              break;
            case 3:                                   // Настройка ограничивающего тока
              I_max += (millis() <= (encoderMillis_speed + 30)) ? 2.00 : (millis() <= (encoderMillis_speed + 70)) ? 1.00 : (millis() <= (encoderMillis_speed + 270)) ? 0.10 : 0.01;
              if (I_max > I_max_max ) I_max = I_max_max;
              P_max = round(I_max * I_max * resist_ten / 5) * 5;
              break;
            case 4:                                   // Настройка ограничивающей мощности
              P_max += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_max > P_max_max ) P_max = P_max_max;
              I_max = sqrt((double)P_max / resist_ten);
              break;
            case 5:                                   // Настройка предустановок мощности
              P_ust1 += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust1 > P_max ) P_ust1 = P_max;
              break;
            case 6:
              P_ust2 += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust2 > P_max ) P_ust2 = P_max;
              break;
            case 7:
              P_ust3 += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust3 > P_max ) P_ust3 = P_max;
              break;
            case 8:
              P_ust4 += (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust4 > P_max ) P_ust4 = P_max;
              break;
            case 9:                                   // Настройка типа датчика
              switch (ACS_type)
              {
                case 0:
                  ACS_type = 1;
                  ACS_coeff = 0.048828125;
                  break;
                case 1:
                  ACS_type = 2;
                  ACS_coeff = 0.073242188;
                  break;
                case 2:
                  ACS_type = 0;
                  ACS_coeff = 0.024414063;
                  break;
              }
              break;
          }
        }
        else
        { // Настройка подаваемой мощности
          switch (menu_step)
          {
            case 0:
              ust_W +=  (millis() <= (encoderMillis_speed + 30)) ? 50 :  (millis() <= (encoderMillis_speed + 200)) ? 10 : 5;
              if (ust_W > P_max) ust_W = P_max;
              break;
            case 1:
              menu_step = 2;
              break;
            case 2:
              menu_step = 3;
              break;
            case 3:
              menu_step = 4;
              break;
            case 4:
              menu_step = 0;
              lcd.clear();
              break;
          }
        }
        encoderMillis_speed = millis();
      }
      else {
        if (setup_sw)
        {
          switch (setup_step)
          {
            case 1:                                   // Настройка сопротивления ТЭНа
              resist_ten -= (millis() <= (encoderMillis_speed + 30)) ? 2.00 : (millis() <= (encoderMillis_speed + 70)) ? 1.00 : (millis() <= (encoderMillis_speed + 270)) ? 0.10 : 0.01;
              if (resist_ten < resist_ten_min ) resist_ten = resist_ten_min;
              power_ten = 220.00 * 220.00 / resist_ten;
              break;
            case 2:                                   // Настройка мощности ТЭНа
              power_ten -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 10 : 1;
              if (power_ten < power_ten_min ) power_ten = power_ten_min;
              resist_ten = 220.00 * 220.00 / (double)power_ten;
              break;
            case 3:                                   // Настройка ограничивающего тока
              I_max -= (millis() <= (encoderMillis_speed + 30)) ? 2.00 : (millis() <= (encoderMillis_speed + 70)) ? 1.00 : (millis() <= (encoderMillis_speed + 270)) ? 0.10 : 0.01;
              if (I_max < I_max_min ) I_max = I_max_min;
              P_max = round(I_max * I_max * resist_ten / 5) * 5;
              break;
            case 4:                                   // Настройка ограничивающей мощности
              P_max -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_max < P_max_min ) P_max = P_max_min;
              I_max = sqrt((double)P_max / resist_ten);
              break;
            case 5:                                   // Настройка предустановок мощности
              P_ust1 -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust1 < 0 ) P_ust1 = 0;
              break;
            case 6:
              P_ust2 -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust2 < 0 ) P_ust2 = 0;
              break;
            case 7:
              P_ust3 -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust3 < 0 ) P_ust3 = 0;
              break;
            case 8:
              P_ust4 -= (millis() <= (encoderMillis_speed + 30)) ? 200 : (millis() <= (encoderMillis_speed + 70)) ? 100 : (millis() <= (encoderMillis_speed + 270)) ? 20 : 5;
              if (P_ust4 < 0 ) P_ust4 = 0;
              break;
            case 9:                                   // Настройка типа датчика
              switch (ACS_type)
              {
                case 0:
                  ACS_type = 2;
                  ACS_coeff = 0.073242188;
                  break;
                case 1:
                  ACS_type = 0;
                  ACS_coeff = 0.024414063;
                  break;
                case 2:
                  ACS_type = 1;
                  ACS_coeff = 0.048828125;
                  break;
              }
              break;
          }
        }
        else
        { // Настройка подаваемой мощности

          switch (menu_step)
          {
            case 0:
              ust_W -=  (millis() <= (encoderMillis_speed + 30)) ? 50 :  (millis() <= (encoderMillis_speed + 200)) ? 10 : 5;
              if (ust_W < 0) ust_W = 0;
              break;
            case 1:
              menu_step = 0;
              lcd.clear();
              break;
            case 2:
              menu_step = 1;
              break;
            case 3:
              menu_step = 2;
              break;
            case 4:
              menu_step = 3;
              break;
          }

        }
        encoderMillis_speed = millis();
      }

    }
    encoder_CLK_prev = encoder_CLK;

    encoderMillis = millis();
  }

  //------------------------------------ Регулятор мощности

  if (millis() - previousMillis >= 15) {              //Если регулировка долгая то уменьшить значение, если видно частые изменения регулировки то увеличить значение
    previousMillis = millis();
    if (real_W > ust_W + 5) angle++;
    if (real_W < ust_W - 5) angle--;
    if (real_W < ust_W + 5 && real_W > ust_W - 5) angle = angle;
    if (angle > 2250) angle = 2250;
    if (angle <= 1) angle = 1;
    if (ust_W == 0) angle = 2250;
  }

  if (ust_W == 0) {
    TIMSK1 = 0x00;
    PORTD &=  ~(1 << PORTD5);
  }

  else
  {
    TIMSK1 |= (1 << OCIE1A) | (1 << TOIE1);
  }

  //------------------------------------ Вывод информации на LCD

  if (millis() - displayMillis >= 300) {
    displayMillis = millis();

    if (setup_sw)                           // Режим настройки
    {
      switch (setup_step)
      {
        case 1:
          lcd.setCursor(0, 0);
          lcd.print(">R тэн ");
          lcd.print(resist_ten);
          lcd.print(" Ом  ");
          lcd.setCursor(0, 1);
          lcd.print(" W тэн ");
          lcd.print(power_ten);
          lcd.print(" Вт  ");
          break;
        case 2:
          lcd.setCursor(0, 0);
          lcd.print(" R тэн ");
          lcd.print(resist_ten);
          lcd.print(" Ом  ");
          lcd.setCursor(0, 1);
          lcd.print(">W тэн ");
          lcd.print(power_ten);
          lcd.print(" Вт  ");
          break;
        case 3:
          lcd.setCursor(0, 0);
          lcd.print(">I макс ");
          lcd.print(I_max);
          lcd.print(" А  ");
          lcd.setCursor(0, 1);
          lcd.print(" W макс ");
          lcd.print(P_max);
          lcd.print(" Вт  ");
          break;
        case 4:
          lcd.setCursor(0, 0);
          lcd.print(" I макс ");
          lcd.print(I_max);
          lcd.print(" А  ");
          lcd.setCursor(0, 1);
          lcd.print(">W макс ");
          lcd.print(P_max);
          lcd.print(" Вт  ");
          break;
        case 5:
          lcd.setCursor(0, 0);
          lcd.print(">");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 6:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(">");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 7:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(">");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 8:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(">");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 9:
          lcd.setCursor(0, 0);
          lcd.print("Тип датчика тока ");
          lcd.setCursor(0, 1);
          switch (ACS_type)
          {
            case 0:
              lcd.print("   ACS712 5A    ");
              break;
            case 1:
              lcd.print("   ACS712 20A   ");
              break;
            case 2:
              lcd.print("   ACS712 30A   ");
              break;
          }
          break;
      }
    }
    else {                           // Режим работы
      switch (menu_step)
      {
        case 0:
          lcd.setCursor(0, 0);
          lcd.print(ust_W);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(map(angle, 2250, 1, 0, 100));
          lcd.print("% ");
          lcd.print(angle);
          lcd.print("  ");
          
          lcd.setCursor(0, 1);
          lcd.print(real_W);
          lcd.print(" Вт  ");

          lcd.setCursor(8, 1);
          lcd.print(real_I);
          lcd.print(" А  ");
          break;
        case 1:
          lcd.setCursor(0, 0);
          lcd.print(">");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 2:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(">");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 3:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(">");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(" ");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
        case 4:
          lcd.setCursor(0, 0);
          lcd.print(" ");
          lcd.print(P_ust1);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 0);
          lcd.print(" ");
          lcd.print(P_ust3);
          lcd.print(" Вт  ");
          lcd.setCursor(0, 1);
          lcd.print(" ");
          lcd.print(P_ust2);
          lcd.print(" Вт  ");
          lcd.setCursor(8, 1);
          lcd.print(">");
          lcd.print(P_ust4);
          lcd.print(" Вт  ");
          break;
      }
    }
  }
}
