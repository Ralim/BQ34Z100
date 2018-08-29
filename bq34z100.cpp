/*
 * BQ34Z100 library for reading / writing / settting up
 * This allows you to use this ic in your own projects :)
 * By Ben V. Brown (Ralim@Ralimtek.com)
 */

#include "bq34z100.h"

extern "C" {
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
}
#include "Arduino.h"
#define BQ34Z100 0x55
uint8_t flashbytes[32] = {0};
bq34z100::bq34z100()
{

    Wire.begin();//init I2C (making sure wire starts)
}
//read a standard normal acces value (the common ones you need)
uint32_t bq34z100::Read(int add, uint8_t length)
{
    uint32_t returnVal = 0;

    for (int i = 0; i < length; i++)
    {
        Wire.beginTransmission(BQ34Z100);
        Wire.write(add + i);
        Wire.endTransmission();
        Wire.requestFrom(BQ34Z100, 1);
        returnVal = returnVal + (Wire.read() << (8 * i));
    }
    return returnVal;
}

int bq34z100::enableCal()
{
    return readControl(0x2D,0x00);//reading this will enable the IT algorithum
}

int bq34z100::enterCal()
{
    return readControl(0x2D,0x00);//reading this will enable the IT algorithum
}


int bq34z100::exitCal()
{
    return readControl(0x2D,0x00);//reading this will enable the IT algorithum
}

int bq34z100::enableIT()
{
    return readControl(0x21,0x00);//reading this will enable the IT algorithum
}
//Read a regester set from the control register (used to read the instant current
//  or enable IT)
int bq34z100::readControl(uint8_t add,uint8_t add2)
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(add2);
    Wire.write(add);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100, 2);
    int16_t temp = Wire.read();
    temp = temp | (Wire.read() << 8);
    return temp;

}
//lets you read the instantainious current
int bq34z100::readInstantCurrent()
{
    return readControl(0x18,0x00);
}
//returns the current temp
int bq34z100::getTemp()
{
    return Read(0x0c, 2) - 2731; //return temp in x10 format
}

uint8_t bq34z100::getSOC()
{
    return Read(0x02, 2) ; //return temp in x10 format
}

int bq34z100::getVoltage()
{
    return Read(0x08, 2) ; //return temp in x10 format
}

int bq34z100::getCapacity()
{
    return Read(0x06, 2) ; //return temp in x10 format
}
int bq34z100::getRemaining()
{
    return Read(0x04, 2) ; //return temp in x10 format
}
int bq34z100::getCurrent()
{
    return Read(0x0a, 2) ; //return temp in x10 format
}
int bq34z100::getStatus()
{
    return readControl(0x00,0x00);
}
int bq34z100::getFlags()
{
    return readControl(0x0E,0x0F);
}
void bq34z100::reset()
{
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x00);Wire.write(0x41);
    Wire.endTransmission();
    Wire.requestFrom(BQ34Z100, 1); 
    Wire.read();
}

/**    FLASH CONFIGURATION SETTINGS  **/
int bq34z100::setup(uint8_t BatteryChemistry,uint8_t SeriesCells,uint16_t CellCapacity,uint16_t PackCurrentVoltage,uint16_t AppliedCurrent)
{
    //so we set the following:
    /*  Battery Chemistry
    *   Number of Series Cells
    *   Cell capacity (mAh)
    *   Set to use internal Temp Sensor
    *   Disable built in voltage divider
    *   Calibrate voltage Divider
    *   Calibrate Current Shunt
    */
//Set chemistry

    //Set Series Cells
    //Set cell capacity
    chg48Table(CellCapacity,CellCapacity,4200,4200,4200,4200);
    //set pack configuration
    chg64Table(0x29c0,0,0,SeriesCells);
    if(PackCurrentVoltage>5000)
    {
        //calibrate VDivider
        CalibrateVoltageDivider(PackCurrentVoltage);
        CalibrateVoltageDivider(PackCurrentVoltage);//calling 3 times due to rounding issues
        CalibrateVoltageDivider(PackCurrentVoltage);//it gets closer each time
    }
   //we now need to calibrate thecurrent
CalibrateCurrentShunt(AppliedCurrent);

}
void bq34z100::CalibrateCurrentShunt(int16_t current)
{
    if(current>-200 && current<200)
        return;//too small to use to calibrate
    //current is in milliamps
    if(current<0)
        current=-current;
    int16_t currentReading = getCurrent();
    if(currentReading<0)
        currentReading = -currentReading;
    if(currentReading==0)
        currentReading=20;
    Serial.println(currentReading);
    readFlash(0x68, 15);
    delay(30);
  
    uint32_t curentGain = ((uint32_t)flashbytes[0])<<24 | ((uint32_t)flashbytes[1])<<16|((uint32_t)flashbytes[2])<<8|(uint32_t)flashbytes[3];
    Serial.println(curentGain,DEC);
    float currentGainResistance = (4.768/XemicsTofloat(curentGain));
    Serial.println(currentGainResistance);
    float newGain = (((float)currentReading)/((float)current)) * currentGainResistance;
    Serial.println(newGain);
    //we now have the new resistance calculated
Serial.println("--");
    chg104Table(0,newGain,newGain);
    //chg104Table(0,5,5);

    delay(30);
}
uint16_t bq34z100::CalibrateVoltageDivider(uint16_t currentVoltage)
{
    if(currentVoltage<5000)
        return 0;
//So do this we set the voltage divider to 1.5 Times the current pack voltage (to start with)
 float setVoltage =   ((float)readVDivider());
 //chg104Table((uint16_t)setVoltage);//set voltage divider
 float readVoltage = (float)getVoltage();
float newSetting = (currentVoltage/readVoltage)*setVoltage;
chg104Table((uint16_t)newSetting,0,0);//set new divider ratio
return (uint16_t)newSetting;
}

// Read 2 bytes of subclass and position offset
//subClass = subclass id
//offset = the value you want to change in case we need
// a different page
bool bq34z100::readFlash(uint16_t subclass, uint8_t offset)
{
delay(10);
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x61);//blocckdatacontrol
    Wire.write(0x00);
    Wire.endTransmission();
delay(30);//mimic bq2300
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3e);//DataflashClass()
    Wire.write(subclass);//sets the class to load
    Wire.endTransmission();
delay(30);//mimic bq2300
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3f);//DataFlashBlock()
    Wire.write((uint8_t)(offset / 32)); // change this to 0x01 if offset is >31
    Wire.endTransmission();
    delay(30);//mimic bq2300
    bool changed = false;
    Wire.beginTransmission(BQ34Z100);
        Wire.write(0x40);
        Wire.endTransmission();

       Wire.requestFrom(BQ34Z100, 33);
    for (int i = 0; i < 32; i++) //loop through all bytes in the page
    {
    flashbytes[i] = Wire.read();//store it
        
    }
     Wire.endTransmission();
     delay(10);
    return false;
}
void bq34z100::writeTable(uint16_t subclass, uint8_t offset)
{
Wire.beginTransmission(BQ34Z100);
    Wire.write(0x61);//blocckdatacontrol
    Wire.write(0x00);
    Wire.endTransmission();
delay(30);//mimic bq2300
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3e);//DataflashClass()
    Wire.write(subclass);//sets the class to load
    Wire.endTransmission();
delay(30);//mimic bq2300
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x3f);//DataFlashBlock()
    Wire.write((uint8_t)(offset / 32)); // change this to 0x01 if offset is >31
    Wire.endTransmission();
    delay(30);//mimic bq2300
    bool changed = false;
    Wire.beginTransmission(BQ34Z100);
        Wire.write(0x40);
        for(int i=0;i<32;i++)
        {
        Wire.write(flashbytes[i]);
    }
     Wire.endTransmission();
   
}
//This calcs the checksum and then writes it to the device
//This then causes the device to check it, and if correct
//It will then store the new data in flash
void bq34z100::checkSum(uint16_t subclass, uint8_t offset)
{
    //calc checksum
    int chkSum = 0;
    for (int i = 0; i < 32; i++)
    {
        chkSum += flashbytes[i];
    }
    int chkSumTemp = chkSum / 256;
    chkSum = chkSum - (chkSumTemp * 256);
    chkSum = 255 - chkSum;
    //We Write the whole table first
//writeTable(subclass,offset);
delay(35);
    //Now
    //write checksum
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x60);//send chksum command
    Wire.write(chkSum);
    Wire.endTransmission();
delay(150);//bq neeeds time here!!!

}

void bq34z100::chgFlash(uint8_t index, int value)
{
    if (index > 31)
        index = index % 32;

    //  change flashbyte first
    flashbytes[index] = value;

    // write flashbyte
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index);
    Wire.write(flashbytes[index]);
    Wire.endTransmission();
}



void bq34z100::chgFlashPair(uint8_t index, int value)
{
    if (index > 31)
        index = index % 32;

    //  change flashbyte first
    flashbytes[index] = value >> 8; //high byte
    flashbytes[index + 1] = value & 0xFF; //lower byte
    // write flashbyte
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index);
    Wire.write(flashbytes[index]);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index + 1);
    Wire.write(flashbytes[index + 1]);
    Wire.endTransmission();
}
void bq34z100::chgFlashQuad(uint8_t index, uint32_t value)
{
    if (index > 31)
        index = index % 32;

    //  change flashbyte first
    flashbytes[index] = value >> 24; //high byte
    flashbytes[index + 1] = value >>16; //lower byte
    flashbytes[index + 2] = value >>8; //lower byte
    flashbytes[index + 3] = value & 0xFF; //lower byte

    // write flashbyte
    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index);
    Wire.write(flashbytes[index]);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index + 1);
    Wire.write(flashbytes[index + 1]);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index + 2);
    Wire.write(flashbytes[index + 2]);
    Wire.endTransmission();

    Wire.beginTransmission(BQ34Z100);
    Wire.write(0x40 + index + 3);
    Wire.write(flashbytes[index + 3]);
    Wire.endTransmission();
}

void bq34z100::chg48Table(uint16_t designCap, uint16_t designEnergy, uint16_t CellVT1T2, uint16_t CellVT2T3, uint16_t CellVT3T4, uint16_t CellVT4T5)
{
    readFlash(48, 24);
    chgFlashPair(21, designCap);
    chgFlashPair(23, designEnergy);
    chgFlashPair(28, CellVT1T2);//0x0E10 = 3600
    chgFlashPair(30, CellVT2T3);//0x0E10 = 3600
    checkSum(48, 24);
    delay(300);
    readFlash(48, 35);
    chgFlashPair(32, CellVT3T4);//0x0E10 = 3600
    chgFlashPair(34, CellVT4T5);//0x0E10 = 3600
    checkSum(48, 35);
}
void bq34z100::chg49Table(uint16_t UVLo, uint16_t UVLoR, uint16_t OVLo, uint16_t OVLoR)
{
    readFlash(49, 17);
    chgFlashPair(9 , UVLo);//2500

    chgFlashPair(12, UVLoR);//2900

    chgFlashPair(14, OVLo); //3900

    chgFlashPair(17, OVLoR); //3700

    checkSum(49, 17);
}
void bq34z100::chg64Table(uint16_t packConfigReg, uint16_t alertConfigReg, uint8_t ledChgReg, uint8_t CellsS)
{
    readFlash(64, 6);
    chgFlashPair(0, packConfigReg);
    chgFlash(4, ledChgReg);
    chgFlashPair(5, alertConfigReg);
    chgFlash(7, CellsS); //cell count
    checkSum(64, 6);
}

void bq34z100::chg68Table(uint16_t MinCellVoltage)
{
    readFlash(68, 0);
    chgFlashPair(0, MinCellVoltage);
    checkSum(68, 0);
}
void bq34z100::chg80Table(uint16_t CellTermVoltage)
{
    readFlash(80, 67);
    chgFlashPair(67, CellTermVoltage);//0A28
    checkSum(80, 67);
}
void bq34z100::chg82Table(uint16_t QMaxCell, uint16_t CellVAtMaxChargeTerm)
{
    readFlash(82, 5);
    chgFlashPair(0, QMaxCell);//0x4B00
    chgFlashPair(5, CellVAtMaxChargeTerm);//0x0E10
    checkSum(82, 5);
}
void bq34z100::chg83Table(uint16_t cellChemistry)
{
    readFlash(83, 1);
    chgFlashPair(0, cellChemistry); //LiFepo4 = 0401
    checkSum(83, 1);
}
//If vDiv is <500 dont change vdiv
//if CCG && CCD ==0 then dont change them
void bq34z100::chg104Table(uint16_t Vdivider,float CCGain,float CCDelta)
{
    if(Vdivider>32768)
        return;
    
        
    readFlash(0x68, 15);
    delay(30);
    if(Vdivider>500)
    chgFlashPair(14, Vdivider);
    if(!(CCGain==0 && CCDelta==0))
    {
        float GainDF = 4.768/CCGain;
        float DeltaDF = 5677445/CCDelta;
        chgFlashQuad(0,floatToXemics(GainDF));
        chgFlashQuad(4,floatToXemics(DeltaDF));
    }
    checkSum(0x68, 15);

}
uint16_t bq34z100::readVDivider()
{
readFlash(0x68, 15);
    uint16_t val = (((uint16_t)flashbytes[14]) <<8) | flashbytes[15];
    return val;

}
uint32_t bq34z100::floatToXemics(float X)
{
	int iByte1, iByte2, iByte3, iByte4, iExp;
            bool bNegative = false;
            float fMantissa;
            // Don't blow up with logs of zero
            if (X == 0) X = 0.00001F;
            if (X < 0)
            {
                bNegative = true;
                X = -X;
            }
            // find the correct exponent
            iExp = (int)((log(X) / log(2)) + 1);// remember - log of any base is ln(x)/ln(base)

            // MS byte is the exponent + 0x80
            iByte1 = iExp + 128;
           
            // Divide input by this exponent to get mantissa
            fMantissa = X / (pow(2, iExp));
           
            // Scale it up
            fMantissa = fMantissa / (pow(2, -24));
           
            // Split the mantissa into 3 bytes
            iByte2 = (int)(fMantissa / (pow(2, 16)));
            
            iByte3 = (int)((fMantissa - (iByte2 * (pow(2, 16)))) / (pow(2, 8)));
           
            iByte4 = (int)(fMantissa - (iByte2 * (pow(2, 16))) - (iByte3 * (pow(2, 8))));
           
            // subtract the sign bit if number is positive
            if (bNegative == false)
            {
                iByte2 = iByte2 & 0x7F;
            }
            return (uint32_t)((uint32_t)iByte1 << 24 | (uint32_t)iByte2 << 16 | (uint32_t)iByte3 << 8 | (uint32_t)iByte4);
}
float bq34z100::XemicsTofloat(uint32_t X)
{
	 bool bIsPositive = false;
            float fExponent, fResult;
            byte vMSByte = (byte)(X >> 24);
            byte vMidHiByte = (byte)(X >> 16);
            byte vMidLoByte = (byte)(X >> 8);
            byte vLSByte = (byte)X;
            // Get the sign, its in the 0x00 80 00 00 bit
            if ((vMidHiByte & 128) == 0)
            { bIsPositive = true; }

            // Get the exponent, it's 2^(MSbyte - 0x80)
            fExponent = pow(2, (vMSByte - 128));
            // Or in 0x80 to the MidHiByte
            vMidHiByte = (byte)(vMidHiByte | 128);
            // get value out of midhi byte
            fResult = (vMidHiByte) * 65536;
            // add in midlow byte
            fResult = fResult + (vMidLoByte * 256);
            // add in LS byte
            fResult = fResult + vLSByte;
            // multiply by 2^-24 to get the actual fraction
            fResult = fResult * pow(2, -24);
            // multiply fraction by the ‘exponent’ part
            fResult = fResult * fExponent;
            // Make negative if necessary
            if (bIsPositive)
                return fResult;
            else
                return -fResult;
}
float bq34z100::readCurrentShunt()
{
    readFlash(0x68, 15);
    delay(30);
    uint32_t curentGain = ((uint32_t)flashbytes[0])<<24 | ((uint32_t)flashbytes[1])<<16|((uint32_t)flashbytes[2])<<8|(uint32_t)flashbytes[3];
    return (float) (4.768/XemicsTofloat(curentGain));
    
}
