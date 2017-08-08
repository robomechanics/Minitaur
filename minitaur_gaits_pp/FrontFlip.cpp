#include "HAL.h"
#include "FrontFlip.h"

FrontFlip frontFlip;







//void FrontFlip::begin() {
//  tStart = X.t;
//  frontFlipPhase = FFWAIT;
//}
void FrontFlip::begin(){
  mode = FF_WAIT;
  tstart = X.t;

}
void FrontFlip::update() {
  X.mode = mode;

  float kAng = 0.5, kAngD = 0.02;
  
  switch (mode) {
    default:
      case FF_STAND:
        leg[0].setGain(ANGLE, 0.7, .002);
        leg[0].setGain(EXTENSION, standStiff);
        leg[2].setGain(ANGLE, 0.7, .002);
        leg[2].setGain(EXTENSION, standStiff);
        leg[1].setGain(ANGLE, 0.7, .002);
        leg[1].setGain(EXTENSION, standStiff);
        leg[3].setGain(ANGLE, 0.7, .002);
        leg[3].setGain(EXTENSION, standStiff);

        leg[0].setPosition(ANGLE, -X.pitch);
        leg[0].setPosition(EXTENSION, startingHeight);
        leg[2].setPosition(ANGLE, -X.pitch);
        leg[2].setPosition(EXTENSION, startingHeight);
        leg[1].setPosition(ANGLE, -X.pitch+0.3);
        leg[1].setPosition(EXTENSION, startingHeight);
        leg[3].setPosition(ANGLE, -X.pitch+0.3);
        leg[3].setPosition(EXTENSION, startingHeight);
      break;
      
      case FF_WAIT:
        leg[0].setGain(ANGLE, 0.7, .002);
        leg[0].setGain(EXTENSION, standStiff);
        leg[2].setGain(ANGLE, 0.7, .002);
        leg[2].setGain(EXTENSION, standStiff);
        leg[1].setGain(ANGLE, 0.7, .002);
        leg[1].setGain(EXTENSION, standStiff);
        leg[3].setGain(ANGLE, 0.7, .002);
        leg[3].setGain(EXTENSION, standStiff);

        leg[0].setPosition(ANGLE, -X.pitch);
        leg[0].setPosition(EXTENSION, startingHeight);
        leg[2].setPosition(ANGLE, -X.pitch);
        leg[2].setPosition(EXTENSION, startingHeight);
        leg[1].setPosition(ANGLE, -X.pitch+0.3);
        leg[1].setPosition(EXTENSION, startingHeight);
        leg[3].setPosition(ANGLE, -X.pitch+0.3);
        leg[3].setPosition(EXTENSION, startingHeight);
        if(X.t-tstart>100){
          mode = FF_JUMP;
        }
      break;

      case FF_JUMP : {
          leg[1].setOpenLoop(EXTENSION, g00s3);
          leg[3].setOpenLoop(EXTENSION, g00s3);
          //track down with front paws

          leg[0].setGain(EXTENSION, jumpStiff);
          leg[2].setGain(EXTENSION, jumpStiff);

          leg[0].setPosition(EXTENSION, .86);
          leg[2].setPosition(EXTENSION, .86); // .12 m

          leg[0].setPosition(ANGLE, -X.pitch + .3);
          leg[2].setPosition(ANGLE, -X.pitch + .3);

          if (leg[1].getPosition(EXTENSION) > 2.77|| leg[3].getPosition(EXTENSION) > 2.77) {
            //Once legs have extended, end jump
            mode = FF_INVERT;
          }
        } break;

      case FF_INVERT : {

          leg[1].setGain(EXTENSION, standStiff);
          leg[3].setGain(EXTENSION, standStiff);
          leg[1].setPosition(EXTENSION, startingHeight);
          leg[3].setPosition(EXTENSION, startingHeight);


          leg[0].setPosition(EXTENSION, 1.32); // .15m
          leg[2].setPosition(EXTENSION, 1.32);
          leg[0].setPosition(ANGLE, -X.pitch);
          leg[2].setPosition(ANGLE, -X.pitch);

          leg[1].setPosition(ANGLE, PI / 2);
          leg[3].setPosition(ANGLE, PI / 2);

          if (fabsf(X.pitch) > 1.7) {
            mode = FF_LAND;
          }

        } break;

      case FF_LAND : {
          for (int i = 0; i < 4; ++i) {
            leg[i].setGain(ANGLE, kAng, kAngD);
            leg[i].setGain(EXTENSION, standStiff);
          }

          leg[0].setPosition(EXTENSION, 1.32);
          leg[2].setPosition(EXTENSION, 1.32);
          leg[0].setPosition(ANGLE, -X.pitch+.2);
          leg[2].setPosition(ANGLE, -X.pitch+.2);

          leg[1].setPosition(ANGLE, -X.pitch-.3);
          leg[3].setPosition(ANGLE, -X.pitch-.3);
          leg[1].setPosition(EXTENSION, landingHeight);
          leg[3].setPosition(EXTENSION, landingHeight);

          if (fabsf(X.pitch)>2.8) {
            mode = FF_POSTLAND;
          }


        } break;

      case FF_POSTLAND : {

          for (int i = 0; i < 4; ++i) {
            leg[i].setGain(ANGLE, kAng, kAngD);
            leg[i].setGain(EXTENSION, .5);
          }

          leg[0].setPosition(EXTENSION, 1.43);
          leg[1].setPosition(EXTENSION, 1.43);
          leg[2].setPosition(EXTENSION, 1.43);
          leg[3].setPosition(EXTENSION, 1.43);

          leg[0].setPosition(ANGLE, -X.pitch+.2);
          leg[2].setPosition(ANGLE, -X.pitch+.2);
          leg[1].setPosition(ANGLE, -X.pitch-.2);
          leg[3].setPosition(ANGLE, -X.pitch-.2);

        } break;
      



    }




  }

