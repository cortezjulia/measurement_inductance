
void referencias();
#define serial
#define charge_cap 4
#define charge_rele 5
#define charge_calibracao 12
#define freqIn 2
#define charge_terra 3
#define maximo 6
#define minimo 7
#define saida 8
#define led_erro 9
#define led_acerto 10
#define buzer 11
#define divres A2
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//Inicializa o display no endereco 0x27
//LiquidCrystal_I2C lcd(0x27, 16, 2);
LiquidCrystal_I2C lcd(0x3f, 16, 2);
double offset;
double inductance_offset;
unsigned int cont = 0;
int media = 0;
double offset_final;
double vetor[5];
int calibrar;
int primeira = 0;
int calibrar_fora = 0;
double capacitance;
double inductance;
double indu_final;
double cap_final;

volatile boolean first;
volatile boolean triggered;
volatile unsigned long overflowCount;
volatile unsigned long startTime;
volatile unsigned long finishTime;
unsigned long tempo_ini;
// here on rising edge
void isr ()
{
  unsigned int counter = TCNT1;  // quickly save it

  // wait until we noticed last one
  if (triggered)
    return;

  if (first)
  {
    startTime = (overflowCount << 16) + counter;
    first = false;
    return;
  }

  finishTime = (overflowCount << 16) + counter;
  triggered = true;
  detachInterrupt(0);
}  // end of isr

// timer overflows (every 65536 counts)
ISR (TIMER1_OVF_vect)
{
  overflowCount++;
}  // end of TIMER1_OVF_vect


void prepareForInterrupts ()
{
  // get ready for next time
  EIFR = bit (INTF0);  // clear flag for interrupt 0
  first = true;
  triggered = false;  // re-arm for next time
  attachInterrupt(0, isr, RISING);
}  // end of prepareForInterrupts


void setup ()
{
  lcd.init();
  lcd.setBacklight(HIGH);
  lcd.setCursor(0, 0);
  lcd.print("Seletor de");
  lcd.setCursor(4, 1);
  lcd.print("Indutancia");
 // pinMode(maximo, INPUT);
 // pinMode(minimo, INPUT);
  //pinMode(saida, OUTPUT);
  pinMode(buzer, OUTPUT);
  pinMode(led_acerto, OUTPUT);
  pinMode(led_erro, OUTPUT);
  //digitalWrite(saida, HIGH);
  digitalWrite(buzer, LOW);
  digitalWrite(led_acerto, LOW);
  digitalWrite(led_erro, LOW);
  // reset Timer 1
  TCCR1A = 0;
  TCCR1B = 0;
  // Timer 1 - interrupt on overflow
  TIMSK1 = bit (TOIE1);   // enable Timer1 Interrupt
  // zero it
  TCNT1 = 0;
  overflowCount = 0;
  // start Timer 1
  TCCR1B =  bit (CS10);  //  no prescaling

  pinMode(freqIn, INPUT);
  pinMode(charge_cap, OUTPUT);
  // pinMode(charge_capalternativo, OUTPUT);
  pinMode(charge_rele, OUTPUT);
  pinMode(charge_calibracao, OUTPUT);
  pinMode(charge_terra, OUTPUT);
  digitalWrite(charge_cap, LOW);
  digitalWrite(charge_rele, LOW);
  digitalWrite(charge_calibracao, LOW);
  digitalWrite(charge_terra, HIGH);

  delay(1000);
  lcd.clear();


  delay(2000);
  // set up for interrupts
  prepareForInterrupts ();
  lcd.clear();
} // end of setup

void loop ()
{
  while (1)
  {
    if (((millis() - tempo_ini) >= 1800000) || (primeira == 0))
    {
      digitalWrite(charge_rele, LOW);
      digitalWrite(charge_cap, LOW);
      digitalWrite(charge_calibracao, LOW);
      digitalWrite(charge_terra, HIGH);
      calibrar = 1;
      calibrar_fora = 0;
      tempo_ini = millis();
      primeira = 1;
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("**Retire a peca**");
      digitalWrite(buzer, LOW);
      digitalWrite(led_acerto, HIGH);
      digitalWrite(led_erro, HIGH);
      delay(6000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Aguarde...");
      digitalWrite(buzer, LOW);
    }
    if (calibrar != 0 && calibrar_fora == 0)
    {
      digitalWrite(charge_rele, LOW);
      digitalWrite(charge_cap, LOW);
      digitalWrite(charge_calibracao, LOW);
      digitalWrite(charge_terra, HIGH);
      delay(400);
      digitalWrite(charge_cap, HIGH);
      delay(200);
      digitalWrite(charge_cap, LOW);
      //delay(5);
      digitalWrite(charge_calibracao, HIGH);
      calibrar_fora = 1;
    }
    else if (calibrar == 0)
    {
      delay(10);
      digitalWrite(charge_rele, LOW);
      digitalWrite(charge_cap, HIGH);
      delay(200);
      digitalWrite(charge_cap, LOW);
      // delay(5);
      //delay(2);
      digitalWrite(charge_rele, HIGH);
   
    }
    if (!triggered)
      return;

    unsigned long elapsedTime = finishTime - startTime;
    float freq = F_CPU / float (elapsedTime);  // each tick is 62.5 ns at 16 MHz




    if (calibrar != 0)
    {
      capacitance = 220E-6;
      inductance = ((1. / (capacitance * (double) freq * (double) freq * 4.*3.14159 * 3.14159)) * 1.E6);
      offset = (4.63 - inductance);

    }
    else
    {
      
      capacitance = 220E-6;
      inductance = ((1. / (capacitance * (double) freq * (double) freq * 4.*3.14159 * 3.14159)) * 1.E6);
      inductance_offset = ((inductance) + offset);
    }
    digitalWrite(charge_cap, LOW);
    digitalWrite(charge_rele, LOW);
    digitalWrite(charge_calibracao, LOW);
    delay(5);

    if (calibrar != 0)
    {


      calibrar = 1;
      calibrar_fora = 0;
      if ((offset < 1) && (offset > -1))
      {
        vetor[media] = offset;  media++;
      }

      if (media == 5)
      {
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("***CALIBRADO**");
        digitalWrite(led_acerto, HIGH);
        digitalWrite(led_erro, HIGH);
        digitalWrite(charge_terra, LOW);
        delay(5000);

        offset_final = (vetor[0]+vetor[1]+vetor[2]+vetor[3]+vetor[4])/5;
        lcd.clear();
        lcd.setCursor(0, 0);

        lcd.print(offset_final);

        // lcd.setCursor(8, 0);
        // lcd.print("uF");
        delay(5000);


        calibrar = 0;
        calibrar_fora = 0;
        media=0;
      }
    }
    else
    {
     
      inductance_offset = (inductance_offset);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print( inductance_offset);
      lcd.setCursor(6, 0);
      lcd.print("uH");

     

          if ( inductance_offset > 4.45 &&  inductance_offset < 5.00)
          {

            lcd.setCursor(0, 1);
            lcd.print("peca aceita");
            delay(50);
            digitalWrite(buzer, LOW);
            digitalWrite(led_acerto, HIGH);
            digitalWrite(led_erro, LOW);

          }
          else
          {
            lcd.setCursor(0, 1);
            lcd.print("peca rejeitada");
            delay(50);
            digitalWrite(buzer, HIGH);
            digitalWrite(led_acerto, LOW);
            digitalWrite(led_erro, HIGH);
          }
          
        }
       

      

      delay(500);
    


    // set up for interrupts
    prepareForInterrupts ();

    lcd.clear();

    digitalWrite(charge_terra, LOW);
    digitalWrite(buzer, LOW);
    digitalWrite(led_acerto, LOW);
    digitalWrite(led_erro, LOW);
    digitalWrite(charge_cap, LOW);
    digitalWrite(charge_rele, LOW);
    digitalWrite(charge_calibracao, LOW);

  }
}
