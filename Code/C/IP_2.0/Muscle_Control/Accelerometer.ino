
class AccelerationData {
  public:
  float x;
  float y;
  float z;

  void calculateAcceleration() {
    
    sensors_event_t aevent, mevent;
    accelmag.getEvent(&aevent, &mevent);

    x = aevent.acceleration.x;
    y = aevent.acceleration.y;
    z = aevent.acceleration.z;
  }
  
};

AccelerationData acceleration;
