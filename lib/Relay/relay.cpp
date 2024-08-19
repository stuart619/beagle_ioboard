#include "relay.h"

void initRelays(){
    pinMode(RELAY_1, OUTPUT);
    pinMode(RELAY_2, OUTPUT);
    pinMode(RELAY_3, OUTPUT);
	pinMode(RELAY_4, OUTPUT);
    pinMode(RELAY_5, OUTPUT);

}

void setRelay(uint8_t relay_channel, uint8_t state){
    switch(relay_channel){
        case 1:
            digitalWrite(RELAY_1, state);
            break;
        case 2:
            digitalWrite(RELAY_2, state);
            break;
        case 3:
            digitalWrite(RELAY_3, state);
            break;
		case 4:
            digitalWrite(RELAY_4, state);
            break;
        case 5:
            digitalWrite(RELAY_5, state);
            break;
			
        default:
            break;
    }
}