#ifndef __RGBLED_H__
#define __RGBLED_H__

#include <Arduino.h>

class RGBLED
{
public:
  RGBLED( int rpin, int gpin, int bpin, boolean commonanode );
  RGBLED( int rpin, int gpin, int bpin, boolean commonanode, int scaleR, int scaleG, int scaleB );
  
  void ScaleFactor( int scaleR, int scaleG, int scaleB );
  void Color( int r, int g, int b );
  void Color( byte* values );
private:
  int mScaleR, mScaleG, mScaleB;
  int mRPin, mGPin, mBPin;
  boolean mCommonAnode;

};


#endif

