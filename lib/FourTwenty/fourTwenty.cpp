#include "fourTwenty.h"

MCP4921 out1(SPI_2_MOSI, SPI_2_CLOCK, &hspi);
MCP4921 out2(SPI_2_MOSI, SPI_2_CLOCK, &hspi);  
MCP4921 out3(SPI_2_MOSI, SPI_2_CLOCK, &hspi);  
MCP4921 out4(SPI_3_MOSI, SPI_3_CLOCK, &fspi);
MCP4921 out5(SPI_3_MOSI, SPI_3_CLOCK, &fspi);

TwoWire i2cPort = TwoWire(0);

INA237 ina237_ch1(&i2cPort, 0x40);
INA237 ina237_ch2(&i2cPort, 0x41);
INA237 ina237_ch3(&i2cPort, 0x44);
INA237 ina237_ch4(&i2cPort, 0x45);


void initFourTwenty(){
    
    out1.begin(OUTPUT_CS_1);
    out2.begin(OUTPUT_CS_2);
    out3.begin(OUTPUT_CS_3);
    out4.begin(OUTPUT_CS_4);
	out5.begin(OUTPUT_CS_5);
	
    //4-20 Input section
    ina237_ch1.begin();
    ina237_ch2.begin();
    ina237_ch3.begin();
    ina237_ch4.begin();

}

uint16_t getFourTwentyInputCurrent(int channel){
    switch(channel){
        case 1:
            return ina237_ch1.getCurrent();
            break;
        case 2:
            return ina237_ch2.getCurrent();
            break;
        case 3:
            return ina237_ch3.getCurrent();
            break;
        case 4:
            return ina237_ch4.getCurrent();
            break;
        default:
            return 0;
            break;
    }
}

void setFourTwentyDAC(int channel, int value){
    switch(channel){
        case 1:
            out1.analogWrite(value, 0);
            break;
        case 2:
            out2.analogWrite(value, 0);
            break;
        case 3:
            out3.analogWrite(value, 0);
            break;
        case 4:
            out4.analogWrite(value, 0);
            break;
		case 5:
            out5.analogWrite(value, 0);
            break;
            
        default:
            break;
    }
}