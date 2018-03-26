#include <Arduino.h>

//DECODER-------------------IR_FREQUENCY-----------------DECODER

int frequency_history[5];
int current_frequency_index=0;
int count =0;
int time;
int frequency=0;

uint32_t Counter (uint32_t currentTime) {
        frequency_history [current_frequency_index]= 10*count;
        current_frequency_index++;

        // Fills index with 4 values from decoder and once the 4th array is Fills
        // the decoder functions fill the array again.
        if (current_frequency_index >4) {
                current_frequency_index=0;
        }
        int i;
        for (i=1; i<5; ++i) {
                if (frequency_history[i-1] != frequency_history[1]) {
                        break;
                }
        }


        if (i==5) {
                frequency= frequency_history[3];
                // can choose any value in the index 1-4 thay should all be the same.
        }
        else {
                frequency=0;

        }

        count=0;
        return (currentTime+ CORE_TICK_RATE*100);
}

// IR FREQUENCY ------------------------------------------------------------

void Decoder(){ //Frequency Sensor DECODER
        count++;
}

int is_valid_frequency(int frequency)
{
    return 1;
}
void setup_frequency (){
        attachInterrupt (4,Decoder,FALLING);
        attachCoreTimerService (Counter);
}
