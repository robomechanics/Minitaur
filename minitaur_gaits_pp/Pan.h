#ifndef Pan_h
#define Pan_h

class Pan : public Behavior {
  public:
    int dir;
    int beginTime;
    float ext;
    float phi;
    float ext_back;
    float ext_front;
   // Pan() {}
    // From base class
    void begin() {
      // runs at 1 KHz
      for (int i = 0; i < 4; ++i) {
        leg[i].setGain(EXTENSION, 0.7);
        leg[i].setGain(ANGLE, 0.7);
        leg[i].setPosition(EXTENSION, 1.57);
        leg[i].setPosition(ANGLE, 0);
      }
    }
    Pan() { // assigns variable begin values
      dir = 1;
      beginTime = X.t;
      ext = 1.57;
      phi = 0;
      ext_back = 1.57;
      ext_front = 1.57;
    }
    
    void getPositions_flatPlane() {
      if (ext > (3.14159 - 0.4) || (ext < 0.4)) {
        dir = dir * -1;
      }
      ext += ((0.000029728 * (5.88636)) * dir); // increases ext by small amt
    }
     
    void getPositions_tiltPlane(uint8_t is_back) {
      if (ext > (3.14159 - 0.4) || (ext < 0.4)) {
        dir = dir * -1;
      }
      if (is_back) {
        ext_back += ((0.000029728 * 5.8836) * dir);
        ext = ext_back;
    } else {
        ext_front -= ((0.000029728 * 5.8836) * dir);
        ext = ext_front;
      }
    }

    
    void update() {
        for (int i = 0; i < 4; ++i) {
          leg[i].setGain(EXTENSION, 0.7);
          leg[i].setGain(ANGLE, 0.7);
      }
        int currTime = X.t - beginTime;
      // slowly change extension
      const uint32_t tstart = 5000;
      if (currTime > tstart) { 
        for (int i=0; i<4; ++i) {
          uint8_t is_back = (i == 1) || (i == 3);
//         if (i == 3) { // Testing purposes
          if (currTime < (17200 + tstart)) { // time to finish translating plane 
            getPositions_flatPlane();
          }
          else if (currTime < (18200 + tstart)) { // time to finish tilting plane
            ext = 1.57;
            phi = 0;
          }
          else if (currTime < (52600 + tstart)) { // time to finish tilting plane
            getPositions_tiltPlane(is_back);
          }
          
          leg[i].setPosition(EXTENSION, ext);
          leg[i].setPosition(ANGLE, phi);
          
          if (i == 3) {
          Serial1.print(currTime, 1);
          Serial1.print('\t');
          Serial1.print("ext ");
          Serial1.print(ext, 2);
          Serial1.print('\t');
          Serial1.print("phi ");
          Serial1.print(phi, 2);
          Serial1.print('\t');
          Serial1.print("leg 3");  
          Serial1.print('\n');
          }
//        }
       }
      } else {
          for (int i=0; i<4; ++i) {
            leg[i].setPosition(EXTENSION, 1.57);
            leg[i].setPosition(ANGLE, 0);
        }
    }
  };
    bool running() {
      return false;
    }
    void end() {
    }
   void signal() {
    }
    

};
#endif
