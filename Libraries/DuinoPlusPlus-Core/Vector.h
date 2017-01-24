/*
 * MIT License
 *
 * Copyright (c) 2016 Antoine de Maleprade
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef __VECTOR_H__
#define __VECTOR_H__

#include <Arduino.h>

#define STATIC_ASSERT(cond, msg) (1/(!!cond))

template<size_t Size, typename Precision=float> class Vector
{
public:
  // data
  Precision values[Size];

  // constructors
  Vector() {
        memset(values, 0, sizeof(Precision)*Size); //memset only works with '0'
  };

  // Vector static_cast
  template<typename T> Vector(Vector<Size, T> vector_right)
  {
    for(size_t i=0; i<Size; i++)
    {
      values[i] = (Precision)(vector_right.values[i]);
    }
  };


  Vector(const Precision defaultValue)
  {
    for(size_t i=0; i<Size; i++)
      values[i] = defaultValue;
  }

// default constructors
  Vector(const Precision v0,
                 const Precision v1):
                 values{v0, v1}
  {
        STATIC_ASSERT(Size==2, "Wrong number of arguments");
  }

  Vector(const Precision v0,
                 const Precision v1,
                 const Precision v2):
                 values{v0, v1, v2}
  {
        STATIC_ASSERT(Size==3, "Wrong number of arguments");
  }

  Vector(const Precision v0,
                 const Precision v1,
                 const Precision v2,
                 const Precision v3):
                 values{v0, v1, v2, v3}
  {
        STATIC_ASSERT(Size==4, "Wrong number of arguments");
  }

  void reset(Precision defaultValue)
  {
    for(size_t i=0; i<Size; i++)
      values[i] = defaultValue;
  }

  void print(const String &separator) const
  {
    for(size_t i=0; i<Size; i++)
    {
      Serial.print(values[i]);
      if((i+1)<Size)
        Serial.print(separator);
    }
  }
  void print() const
  {
        print(F(" \t"));
  };
  void println() const {print();Serial.println();};
  void println(const String &separator) const {print(separator);Serial.println();};

  String toString(const String& separator, unsigned char stringOption = 0x7F) const {
    String returnString;
    for(size_t i=0; i<Size; i++)
    {
      if(stringOption == 0x7F) {
        returnString += String(values[i]);
      } else {
        returnString += String(values[i], stringOption);
      }
      if((i+1)<Size)
        returnString += separator;
    }
    return returnString;
  };

  // operations
  Vector<Size, Precision>& constrainIn(Precision minVal, Precision maxVal)
  {
        for(size_t i=0; i<Size; i++)
        {
                values[i] = max(min(values[i],maxVal),minVal);
        }
        return *this;
  }

  template<typename T> Vector<Size, Precision>& operator= (const Vector<Size, T>& vector_right)
  {
    for(size_t i=0; i<Size; i++)
    {
      values[i] = (Precision)vector_right.values[i];
    }
    return *this;
  }

  Precision sqNorme() const
  {
    Precision sqSum = (Precision)0;
    for(size_t i=0; i<Size; i++)
      sqSum += values[i]*values[i];
    return sqSum;
  }

  Precision norme() const
  {
    return sqrt(sqNorme());
  }

  Vector<Size, Precision>& operator*= (Precision right_value)
  {
    for(size_t i=0; i<Size; i++)
      values[i] *= right_value;
    return *this;
  }

  Precision normalize()
  {
    Precision normeVal = norme();
    if(normeVal!=0){
      *this *= 1/normeVal;
    }
    return normeVal;
  }

  Vector<Size, Precision> normalized() const
  {
    Vector<Size, Precision> normalizedVector(*this);
    normalizedVector.normalize();
    return normalizedVector;
  }

//  void rotate(const Quaternion& rotQ)
//  {
//    Vector<3, Precision> rotatedVector;
//    float t2 =   rotQ[0]*rotQ[1];
//    float t3 =   rotQ[0]*rotQ[2];
//    float t4 =   rotQ[0]*rotQ[3];
//    float t5 =  -rotQ[1]*rotQ[1];
//    float t6 =   rotQ[1]*rotQ[2];
//    float t7 =   rotQ[1]*rotQ[3];
//    float t8 =  -rotQ[2]*rotQ[2];
//    float t9 =   rotQ[2]*rotQ[3];
//    float t10 = -rotQ[3]*rotQ[3];
//    rotatedVector[0] = 2*( (t8 + t10)*(*this)[0] + (t6 -  t4)*(*this)[1] + (t3 + t7)*(*this)[2] ) + (*this)[0];
//    rotatedVector[1] = 2*( (t4 +  t6)*(*this)[0] + (t5 + t10)*(*this)[1] + (t9 - t2)*(*this)[2] ) + (*this)[1];
//    rotatedVector[2] = 2*( (t7 -  t3)*(*this)[0] + (t2 +  t9)*(*this)[1] + (t5 + t8)*(*this)[2] ) + (*this)[2];
//    (*this) = rotatedVector;
//  }

  void rotate(Vector<3, Precision> rotation)
  {
    float phi = rotation.normalize();
    float cosphi = cos(phi);
    float sinphi = sin(phi);
    Vector<3, Precision> rotatedVector;

    for(size_t i=0;i<3;i++){
      rotatedVector.values[i] = cosphi*values[i];
      for(size_t j=0;j<3;j++){
        rotatedVector.values[i] += (1-cosphi)*rotation.values[i]*rotation.values[j]*values[j];
        if(i!=j)
          rotatedVector.values[i] += (((i+j+(i<j))%2==0)?1:-1)*rotation.values[3-(i+j)]*sinphi*values[j];
      }
    }

    *this = rotatedVector;
  }

  void gain(const  Vector<Size, Precision>& gainVector)
  {
    for(size_t i=0; i<Size; i++)
      values[i] *= gainVector[i];
  }

  Vector<Size, Precision>& operator+=(const Vector<Size, Precision>& right_vector)
  {
    for(size_t i=0; i<Size; i++)
      values[i] += right_vector.values[i];
    return *this;
  }

  Vector<Size, Precision>& operator-=(const Vector<Size, Precision>& right_vector)
  {
    for(size_t i=0; i<Size; i++)
      values[i] -= right_vector.values[i];
    return *this;
  }

  Vector<Size, Precision> operator-() const
  {
    Vector<Size, Precision> newVector;
    for(size_t i=0; i<Size; i++)
      newVector.values[i] = -values[i];
    return newVector;
  }

  Precision operator*(const Vector<Size, Precision>& right_vector) const
  {
    Precision finalValue = (Precision)0;
    for(size_t i=0; i<Size; i++)
      finalValue += values[i]*right_vector.values[i];
    return finalValue;
  }

  inline Precision& operator[](unsigned int index)
  {
    return values[index];
  }

  inline const Precision operator[](unsigned int index) const
  {
    return values[index];
  }

};

template<typename Precision> inline Vector<3, Precision> operator^(const Vector<3, Precision>& left_vector, const Vector<3, Precision>& right_vector)
{
  Vector<3, Precision> result;
  for(size_t i=0; i<3; i++)
    result.values[i] = left_vector.values[(i+1)%3]*right_vector.values[(i+2)%3]-left_vector.values[(i+2)%3]*right_vector.values[(i+1)%3];
  return result;
}

template<size_t Size, typename Precision> inline Vector<Size, Precision> operator+(const Vector<Size, Precision>& left_vector, const Vector<Size, Precision>& right_vector)
{
  Vector<Size, Precision> result;
  result = left_vector;
  result += right_vector;
  return result;
}

template<size_t Size, typename Precision> inline Vector<Size, Precision> operator-(const Vector<Size, Precision>& left_vector, const Vector<Size, Precision>& right_vector)
{
  Vector<Size, Precision> result;
  result = left_vector;
  result -= right_vector;
  return result;
}

//template<size_t Size, typename Precision> inline Vector<Size, Precision> operator*(Precision value, const Vector<Size, Precision>& vector)
//{
//  Vector<Size, Precision> newVector(vector);
//  newVector *= value;
//  return newVector;
//}

template<size_t Size, typename PrecisionValue, typename PrecisionVector> inline Vector<Size, PrecisionVector> operator*(PrecisionValue value, const Vector<Size, PrecisionVector>& vector)
{
  Vector<Size, PrecisionVector> newVector(vector);
  newVector *= (PrecisionVector)value;
  return newVector;
};

inline float fconstrain(float val, float min, float max)
{
  float newValue = val;
  if(newValue>max) newValue=max;
  if(newValue<min) newValue=min;
  return newValue;
};

#endif //__VECTOR_H__
