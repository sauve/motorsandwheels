#include "rgbled.h"

RGBLED::RGBLED( int rpin, int gpin, int bpin, boolean commonanode )
{
  mScaleR = 1;
  mScaleG = 1;
  mScaleB = 1;
  
  mRPin = rpin;
  mGPin = rpin;
  mBPin = rpin;
  mCommonAnode = commonanode;
}

RGBLED::RGBLED( int rpin, int gpin, int bpin, boolean commonanode, int scaleR, int scaleG, int scaleB )
{
  mScaleR = scaleR;
  mScaleG = scaleG;
  mScaleB = scaleB;
  
  mRPin = rpin;
  mGPin = rpin;
  mBPin = rpin;
  mCommonAnode = commonanode;
}

void ScaleFactor( int scaleR, int scaleG, int scaleB )
{
  mScaleR = scaleR;
  mScaleG = scaleG;
  mScaleB = scaleB;
}


void RGBLED::Color( int r, int g, int b )
{
  if (mCommonAnode)
  {
    analogWrite(mRPin, 255-r);
    analogWrite(mGPin, 255-g);
    analogWrite(mBPin, 255-b);
    return;
  }
  analogWrite(mRPin, r);
  analogWrite(mGPin, g);
  analogWrite(mBPin, b);
}

void Color( byte* values )
{
  Color( values[0], values[1], values[2] );
}
