



// off counter keeps going up until it reaches close total
// on counter keeps going up until it reaches open total


int minimumOffTotal = 5;
int minimumOnTotal = 5;



bool leftCalculateOn() {
  
  static bool on = false;
  static bool lastOn = false;
  static int onTotal = 1;
  static int offTotal = 1;
  static int currentCounter = 0;
  static int currentTotal = 2;
  
  int variation = 12;
  

  // ------------------- // calculate left on value with leftValveOpen

  int onValue = (int)((leftValveOpen * variation) + .5);

  
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



  // ------------------- // calculate left on and left off counter totals based on left on value

  onTotal = onValue;
  offTotal = variation - onValue;

  // ---- // simplify the ratio (6:6 turns to 1:1, 3:9 turns to 1:3, etc.)

  for (int i = 1; i <= variation; i++) {
    if (offTotal % i == 0 && onTotal % i == 0) { // if i is a factor of leftOnTotal and leftOffTotal
      offTotal /= i;
      onTotal /= i;

      if (minimumOffTotal < 5) {
        int changeBy = (int)((5 / offTotal) + .5);
        offTotal * changeBy;
        onTotal * changeBy;
      }
  
      
      if (minimumOnTotal < 5) {
        int changeBy = (int)((5 / onTotal) + .5);
        offTotal * changeBy;
        onTotal * changeBy;
      }
    }
  }

  return on;
  
}




bool rightCalculateOn() {
  
  static bool on = false;
  static bool lastOn = false;
  static int onTotal = 1;
  static int offTotal = 1;
  static int currentCounter = 0;
  static int currentTotal = 2;
  
  int variation = 12;
  

  // ------------------- // calculate right on value with rightValveOpen

  int onValue = (int)((rightValveOpen * variation) + .5);

  
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



  // ------------------- // calculate right on and right off counter totals based on right on value

  onTotal = onValue;
  offTotal = variation - onValue;

  // ---- // simplify the ratio (6:6 turns to 1:1, 3:9 turns to 1:3, etc.)

  for (int i = 1; i <= variation; i++) {
    if (offTotal % i == 0 && onTotal % i == 0) { // if i is a factor of rightOnTotal and rightOffTotal
      offTotal /= i;
      onTotal /= i;

      
      if (minimumOffTotal < 5) {
        int changeBy = (int)((5 / offTotal) + .5);
        offTotal * changeBy;
        onTotal * changeBy;
      }
  
      
      if (minimumOnTotal < 5) {
        int changeBy = (int)((5 / onTotal) + .5);
        offTotal * changeBy;
        onTotal * changeBy;
      }
    }

  }

  return on;
  
}
