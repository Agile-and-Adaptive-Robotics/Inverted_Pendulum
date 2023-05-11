



// off counter keeps going up until it reaches close total
// on counter keeps going up until it reaches open total


int minimumOffTotal = 4;
int minimumOnTotal = 2;



bool leftCalculateOn(float leftValveOpen) {
  
  static bool on = false;
  static bool lastOn = false;
  static int onTotal = 10;
  static int offTotal = 10;
  static int currentCounter = 0;
  static int currentTotal = 2;
  
  int variation = 12;
  

  // ------------------- // calculate left on value with leftValveOpen

  int onValue = (int)((leftValveOpen * variation) + .5);


  // ------------------- // calculate left on and left off counter totals based on left on value

  onTotal = onValue;
  offTotal = variation - onValue;

  // ---- // simplify the ratio (6:6 turns to 1:1, 3:9 turns to 1:3, etc.)

  for (int i = 1; i <= variation; i++) {
    if (offTotal % i == 0 && onTotal % i == 0) { // if i is a factor of leftOnTotal and leftOffTotal
      offTotal /= i;
      onTotal /= i;

      
      if (offTotal < minimumOffTotal) {
        float changeBy = minimumOffTotal / offTotal;
        offTotal = (int)((offTotal * changeBy) + .5);
        onTotal = (int)((onTotal * changeBy) + .5);
      }
  
      
      if (onTotal < minimumOnTotal) {
        float changeBy = minimumOnTotal / onTotal;
        offTotal = (int)((offTotal * changeBy) + .5);
        onTotal = (int)((onTotal * changeBy) + .5);

      }
    }
  }

  
  if (currentCounter < currentTotal) {
    currentCounter++;
  } else { // if end of cycle
    currentCounter = 1;
    if (on) {
      if (offTotal > 0) {
        on = false;
        currentTotal = offTotal;
      }
    } else {
      if (onTotal > 0) {
        on = true;
        currentTotal = onTotal;
      }
    }
  }


  return on;
  
}




bool rightCalculateOn(float rightValveOpen) {
  
  static bool on = false;
  static bool lastOn = false;
  static int onTotal = 1;
  static int offTotal = 1;
  static int currentCounter = 0;
  static int currentTotal = 2;
  
  int variation = 12;
  

  // ------------------- // calculate right on value with rightValveOpen

  int onValue = (int)((rightValveOpen * variation) + .5);


  // ------------------- // calculate right on and right off counter totals based on right on value

  onTotal = onValue;
  offTotal = variation - onValue;

  // ---- // simplify the ratio (6:6 turns to 1:1, 3:9 turns to 1:3, etc.)

  for (int i = 1; i <= variation; i++) {
    if (offTotal % i == 0 && onTotal % i == 0) { // if i is a factor of rightOnTotal and rightOffTotal
      offTotal /= i;
      onTotal /= i;

      // 0 and 12 are being set to 0 : 0, must be fixed to avoid bugs
      
      if (offTotal < minimumOffTotal) {
        float changeBy = minimumOffTotal / offTotal;
        offTotal = (int)((offTotal * changeBy) + .5);
        onTotal = (int)((onTotal * changeBy) + .5);
      }
  
      
      if (onTotal < minimumOnTotal) {
        float changeBy = minimumOnTotal / onTotal;
        offTotal = (int)((offTotal * changeBy) + .5);
        onTotal = (int)((onTotal * changeBy) + .5);

      }
    }

  }
  
  if (currentCounter < currentTotal) {
    currentCounter++;
  } else { // if end of cycle
    currentCounter = 1;
    if (on) {
      if (offTotal > 0) {
        on = false;
        currentTotal = offTotal;
      }
    } else {
      if (onTotal > 0) {
        on = true;
        currentTotal = onTotal;
      }
    }
  }



        //Serial.print(minimumOnTotal); Serial.print(" : "); Serial.print(onTotal); Serial.println();
  //Serial.print(offTotal); Serial.print(" : "); Serial.print(onTotal); Serial.println();

  return on;
  
}
