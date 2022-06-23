/*
bool leftOn = false;
bool rightOn = false;

bool lastLeftOn = false;
bool lastRightOn = false;

int leftOnTotal = 1;
int rightOnTotal = 1;

int leftOffTotal = 1;
int rightOffTotal = 1;

float leftOnFloat = .5;
float rightOnFloat = .5;

int leftOnValue = 2;
int rightOnValue = 6;

int lastLeftOnValue = 2;
int lastRightOnValue = 6;

int leftCurrentCounter = 0;
int rightCurrentCounter = 0;

int leftCurrentTotal = 2;
int rightCurrentTotal = 2;


int variation = 12;




// off counter keeps going up until it reaches close total
// on counter keeps going up until it reaches open total




void setup() {
  Serial.begin(9600);
  while(!Serial);

  Serial.println("starting program");


  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(7, INPUT);
  pinMode(8, INPUT);
}




void loop() {

  // ------------------- // calculate left on value with leftOnFloat

  leftOnValue = (int)((leftOnFloat * variation) + .5);

  // ------------------- // calculate left on and left off counter totals based on left on value

  leftOnTotal = leftOnValue;
  leftOffTotal = variation - leftOnValue;

  // ---- // simplify the ratio (6:6 turns to 1:1, 3:9 turns to 1:3, etc.)

  for (int i = 1; i <= variation; i++) {
    if (leftOffTotal % i == 0 && leftOnTotal % i == 0) { // if i is a factor of leftOnTotal and leftOffTotal
      //leftOffTotal /= i;
      //leftOnTotal /= i;
    }
  }

  // ------------------- // update counters
  
  if (leftCurrentCounter < leftCurrentTotal) {
    leftCurrentCounter++;
  } else { // if end of cycle
    leftCurrentCounter = 1;
    if (leftOn) {
      if (leftOffTotal > 0) {
        leftOn = false;
        leftCurrentTotal = leftOffTotal;
      }
    } else {
      if (leftOnTotal > 0) {
        leftOn = true;
        leftCurrentTotal = leftOnTotal;
      }
    }
  }

  
  digitalWrite(3, leftOn * 255);

  // ------------------- // button functionality


  float brightenSpeed = .001;


  if (digitalRead(7)) {
    leftOnFloat += brightenSpeed;
    //Serial.println("yellow button pushed");
  }
  
  if (digitalRead(8)) {
    leftOnFloat -= brightenSpeed;
    //Serial.println("green button pushed");
  }

  if (leftOnFloat < 0) leftOnFloat = 0;
  if (leftOnFloat > 1) leftOnFloat = 1;


  if (leftOnValue != lastLeftOnValue) {
    
    Serial.print(leftOnTotal);
    Serial.print(":");
    Serial.println(leftOffTotal);

    lastLeftOnValue = leftOnValue;
  }

  delay(10);
  
}*/
