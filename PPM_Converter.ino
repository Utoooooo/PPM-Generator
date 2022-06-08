/*Editted by Rui Zhou 2021 for PPM drone control
 * 
 * PPM generator originally written by David Hasko
 * on https://code.google.com/p/generate-ppm-signal/ 
 */

//////////////////////CONFIGURATION///////////////////////////////


#define CHANNEL_NUMBER 3  //set the number of channels, if doesn't work change back to 7
#define CHANNEL_DEFAULT_VALUE 1500  //set the default servo value
#define FRAME_LENGTH 22500  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PULSE_LENGTH 300  //set the pulse length
#define onState 0  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino

//Define ppm Values
int ppm[CHANNEL_NUMBER];
int duration;
int throttle;
//Channels to hold parsed data:
int thrust = 1000;
int roll = 1500;
int pitch = 1500;
int yaw = 1500;

//for receiving throttle inputs
const byte numChars = 32;
char recvChars[numChars];
char tempChars[numChars];

boolean newData = false;


void setup(){  
  Serial.begin(115200);
  Serial.setTimeout(1);
  //initiallize default ppm values
  for(int i=1; i<CHANNEL_NUMBER; i++){
      ppm[i] = CHANNEL_DEFAULT_VALUE;
  }
  
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 44000;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();

}

void loop(){
  recvThrottle();
  if (newData == true){
//    Serial.print("entered");
    strcpy(tempChars, recvChars);
    parseData();
    newData = false;
    Serial.print(ppm[0]);
    }
//  updatePPM();
}

//receive throttle input from python
void recvThrottle(){
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMrk = '<';
  char endMrk = '>';
  char c;

  while (Serial.available()>0 && newData == false){
    c = Serial.read();

    if (recvInProgress == true){
      if (c != endMrk){
        recvChars[ndx] = c;
        ndx++;
        if (ndx >= numChars){
          ndx = numChars -1;
          }
        }
      else {
        recvChars[ndx] = '\0';
        recvInProgress = false;
        ndx = 0;
        newData = true;
        }
      }
    else if (c == startMrk){
      recvInProgress = true;
      }
    }
}

//parse received string into throttle variables
void parseData(){
  char * tokIdx; //token index
  tokIdx = strtok(tempChars, ",");
  ppm[0] = atoi(tokIdx);
  
  tokIdx = strtok(NULL, ",");
  ppm[1] = atoi(tokIdx);
  
  tokIdx = strtok(NULL, ",");
  ppm[2] = atoi(tokIdx);
  
  tokIdx = strtok(NULL, ",");
  ppm[3] = atoi(tokIdx);
}

//updates value in ppm array
void updatePPM(){
//  Serial.println(thrust);
  ppm[0] = thrust;
  ppm[1] = roll;
  ppm[2] = pitch;
  ppm[3] = yaw;
  }

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if (state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PULSE_LENGTH * 2;
    state = false;
  } else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= CHANNEL_NUMBER){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PULSE_LENGTH;// 
      OCR1A = (FRAME_LENGTH - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PULSE_LENGTH) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
