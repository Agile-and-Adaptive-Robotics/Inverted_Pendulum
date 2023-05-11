
void loop() {

  

  
  // ------ decide what to set BPAs to ------ //
/*
  sensors_event_t event;
  gyro.getEvent(&event);

  float angleXVelocity = event.gyro.x - .01;
  float angleYVelocity = event.gyro.y;
  float angleZVelocity = event.gyro.z - .01;

  static int lastAngleTime = 0;
  float deltaAngleTime = millis() - lastAngleTime;
  lastAngleTime = millis();

  static float angleX = 0;
  static float angleY = 0;
  static float angleZ = 0;
  
  angleX += angleXVelocity * (deltaAngleTime / 1000000);
  angleY += angleYVelocity * (deltaAngleTime / 1000000);
  angleZ += angleZVelocity * (deltaAngleTime / 1000000);


  float targetAngleX = PI / 2;
  float targetAngleY = 0;
  float targetAngleZ = PI / 2;

  float deltaAngleX = targetAngleX - angleX;
  float deltaAngleY = targetAngleY - angleY;
  float deltaAngleZ = targetAngleZ - angleZ;
  */

  acceleration.calculateAcceleration();

  // ---- calculate angle ----//

  float totalAcceleration = abs(acceleration.x) + abs(acceleration.y) + abs(acceleration.z);

  float angleX = acceleration.x / totalAcceleration;
  float angleZ = acceleration.y / totalAcceleration;
    

  float moveSpeed = .001;

  static float desiredAngle = 0;

  if (digitalRead(8)) {
    desiredAngle += moveSpeed;
  }
  if (digitalRead(9)) {
    desiredAngle -= moveSpeed;
  }

  if (desiredAngle < -1) desiredAngle = -1;
  if (desiredAngle >  1) desiredAngle =  1;
  


  // -- calculate x joint PWMs -- /

  static float xJointAngle = 0;

  
  
  posterior.jointPWMs[0] = ;
  anterior. jointPWMs[0] = ;
  peroneus. jointPWMs[0] = ;
  halongus. jointPWMs[0] = null;
  dilongus. jointPWMs[0] = null;
  haflexor. jointPWMs[0] = null;
  diflexor. jointPWMs[0] = null;

  
  // -- calculate z joint PWMs -- /


  static float zJointAngle = 0;

  float zAngleOffset = .1 // angle from foot to rotation point

  float zTopPWM = .5 + (sin(zAngleOffset + zJointAngle) / 2);
  float zBotPWM = 1 - zTopPWM;
  
  posterior.jointPWMs[1] = zTopPWM;
  anterior. jointPWMs[1] = zBotPWM;
  peroneus. jointPWMs[1] = zBotPWM;
  halongus. jointPWMs[1] = null;
  dilongus. jointPWMs[1] = null;
  haflexor. jointPWMs[1] = null;
  diflexor. jointPWMs[1] = null;
  
  // -- calculate c joint PWMs -- /


  static float cJointAngle = 0;

  

  float cTopPWM;
  float cBotPWM;

  float cCosPos = cos( cJointAngle); // this is used by top BPA to calculate PWM -- if angle is positive, top    BPA needs to act
  float cCosNeg = cos(-cJointAngle); // this is used by bot BPA to calculate PWM -- if angle is negative, bottom BPA needs to act

  float cPowerAt0 = .1;

  // calculate for top BPA
  if (cJointAngle > 0) { // PWM is inverse to cosine of angle
    cTopPWM = (1 / cCosPos) - 1 + cPowerAt0;
  }
  else { // BPA has a ton of power (if model was perfect, it would have infinite power and need infinite energy)
    cTopPWM = ((cPowerAt0 / (PI / 2)) * cJointAngle) + cPowerAt0;
  }
  
  // calculate for bot BPA
  if (cJointAngle < 0) { // PWM is inverse to cosine of -angle
    cBotPWM = (1 / cNegPos) - 1 + cPowerAt0;
  }
  else { // BPA has a ton of power (if model was perfect, it would have infinite power and need infinite energy)
    cBotPWM = ((cPowerAt0 / (PI / 2)) * -cJointAngle) + cPowerAt0;
  }
  
  posterior.jointPWMs[2] = null;
  anterior. jointPWMs[2] = null;
  peroneus. jointPWMs[2] = null;
  halongus. jointPWMs[2] = null;
  dilongus. jointPWMs[2] = cTopPWM;
  haflexor. jointPWMs[2] = null;
  diflexor. jointPWMs[2] = cBotPWM;
  


  // -- calculate m joint PWMs -- /

  
  static float mJointAngle = 0;

  float mTopPWM;
  float mBotPWM;

  float mCosPos = cos( mJointAngle); // this is used by top BPA to calculate PWM -- if angle is positive, top    BPA needs to act
  float mCosNeg = cos(-mJointAngle); // this is used by bot BPA to calculate PWM -- if angle is negative, bottom BPA needs to act

  float mPowerAt0 = .1;

  // calculate for top BPA
  if (mJointAngle > 0) { // PWM is inverse to cosine of angle
    mTopPWM = (1 / mCosPos) - 1 + mPowerAt0;
  }
  else { // BPA has a ton of power (if model was perfect, it would have infinite power and need infinite energy)
    mTopPWM = ((mPowerAt0 / (PI / 2)) * mJointAngle) + mPowerAt0;
  }
  
  // calculate for bot BPA
  if (mJointAngle < 0) { // PWM is inverse to cosine of -angle
    mBotPWM = (1 / mNegPos) - 1 + mPowerAt0;
  }
  else { // BPA has a ton of power (if model was perfect, it would have infinite power and need infinite energy)
    mBotPWM = ((mPowerAt0 / (PI / 2)) * -mJointAngle) + mPowerAt0;
  }
  
  
  posterior.jointPWMs[3] = null;
  anterior. jointPWMs[3] = null;
  peroneus. jointPWMs[3] = null;
  halongus. jointPWMs[3] = mTopPWM;
  dilongus. jointPWMs[3] = null;
  haflexor. jointPWMs[3] = mBotPWM;
  diflexor. jointPWMs[3] = null;
  
  
  
  
  
  // ------ set BPAs to desired PWMs ------ //
  
  posterior.calculatePWM();
  anterior.calculatePWM();
  peroneus.calculatePWM();
  halongus.calculatePWM();
  dilongus.calculatePWM();
  haflexor.calculatePWM();
  diflexor.calculatePWM();
  
  posterior.setValve();
  anterior.setValve();
  peroneus.setValve();
  halongus.setValve();
  dilongus.setValve();
  haflexor.setValve();
  diflexor.setValve();

  


  // ------ debug ------ //

  //printDebug("posterior", posterior.on + 3);
  //printDebug("anterior", anterior.on + 5);
  //printDebug("peroneus", peroneus.on + 7);
  //printDebug("x", angleX * PI);
  //printDebug("z", angleZ * PI);
  //Serial.println();
  

  

  // ------ delay till next ------ //

  static int lastTime = 0;
  int currentTime = millis();
  int deltaTime = currentTime - lastTime;
  if (delayTime - deltaTime > 0) delay(delayTime - deltaTime);
  lastTime = millis();
}
