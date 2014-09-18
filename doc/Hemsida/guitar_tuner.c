#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

// LED 0-6 DEFINITIONS
#define led0 0b00000001
#define led1 0b00000010
#define led2 0b00000100
#define led3 0b00001000
#define led4 0b00010000
#define led5 0b00100000
#define led6 0b01000000

#define SIGNAL_CUTOFF_THRESHOLD 0x97
#define SAMPLES 37
#define	BURN 10       // how many edges to skip to allow signal to stabilize
#define BURN_RESULT 3 // results to skip once a string has been detected


#define	T1_PRESCALER 256
#define FREQUENCY	       	8000000		             // CPU CLOCK IN Hz
#define BASE_FREQUENCY 	(FREQUENCY/T1_PRESCALER) // TIMER/COUNTER1 COUNTS PER SECOND

// ACTUAL FREQUENCIES
#define ACOUSTIC_E	82.4				// low string (1)
#define ACOUSTIC_A	110
#define ACOUSTIC_D	146.72
#define ACOUSTIC_G	195.92
#define ACOUSTIC_B	246.92				
#define ACOUSTIC_EH	329.8				// high string (6)

/*
STRING OFFSET CALCULATION TABLE
			KEY:
			"STRING" = OFFSET_STRING
			"------" = TRANSITION

			//------12804,777140335392762577228596646
			// E	11377,42718446601941747572815534
			//------9950,077228596646072374227714034
			// A	8522,7272727272727272727272727273
			//------7456,224596014672350550213145633
			// D	6389,7219193020719738276990185387
			//------5587,419146666144194345454245896
			// G	4785,1163740302164148632094732544
			//------4290,9463289234185913616225561635
			// B	3796,7762838166207678600356390734
			//------3319,7040909683467696183137564675
			// EH	2842,6318981200727713765918738629
			//------2365,5597052717987731348699912565
*/

// FREQUENCIES AS COUNTER VALUES
#define OFFSET_E	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_E)		  // Low E
#define OFFSET_A	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_A)			// A
#define OFFSET_D	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_D)			// D
#define OFFSET_G	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_G)			// G	
#define OFFSET_B	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_B)			// B
#define OFFSET_EH	  ((BASE_FREQUENCY*SAMPLES)/ACOUSTIC_EH)  	// High E

// STRING TRANSITIONS AS COUNTER VALUES
#define TRANSITION_E_ETOP (OFFSET_E + (OFFSET_E - OFFSET_A)/2)	
#define TRANSITION_E_A (OFFSET_E - (OFFSET_E - OFFSET_A)/2)
#define TRANSITION_A_D (OFFSET_A - (OFFSET_A - OFFSET_D)/2)
#define TRANSITION_D_G (OFFSET_D - (OFFSET_D - OFFSET_G)/2)
#define TRANSITION_G_B (OFFSET_G - (OFFSET_G - OFFSET_B)/2)
#define TRANSITION_B_EH (OFFSET_B - (OFFSET_B - OFFSET_EH)/2)
#define TRANSITION_EH_BOTTOM (OFFSET_EH - (OFFSET_B - OFFSET_EH)/2 )

// SMALLEST TIMER INCREMENT FOR A LED WITHIN A TRANSITION
#define LED_OFFSET_E  ((TRANSITION_E_ETOP - TRANSITION_E_A) / 6)
#define LED_OFFSET_A  ((TRANSITION_E_A - TRANSITION_A_D) / 6)
#define LED_OFFSET_D  ((TRANSITION_A_D - TRANSITION_D_G) / 6)
#define LED_OFFSET_G  ((TRANSITION_D_G - TRANSITION_G_B) / 6)
#define LED_OFFSET_B  ((TRANSITION_G_B - TRANSITION_B_EH) / 6)
#define LED_OFFSET_EH  ((TRANSITION_B_EH - TRANSITION_EH_BOTTOM) / 6)

volatile uint8_t signal;
volatile uint16_t timer;

uint16_t topCount;  	// HOW MANY PERIODS
uint16_t timer_prev;	// OLD TCNT1
uint16_t result;		  // SMALLEST SIGNAL PERIOD


// SIGNAL STABILIZING VARIABLES
unsigned int freqCount;
unsigned int burnCount;
short int burned;

unsigned short burnResultCount;
short int burned_result_mode;

// STRING TARGET INFORMATION VARIABLES
short int current_string;
short int current_string_offset;
short int prev_string_offset;
short int current_string_target;

void initCounter1() {
 
	// Counter register is 8-bit, and 256 is max value
	// so from above 0,032 ms * 256 increments = 8,192 ms
	// at 8,192 ms timer will overflow

 	// TIMER/COUNTER1 PRESCALER
 	// TCCR1B|=(1<<CS12)|(1<<CS10);  // 1024
   	 TCCR1B|=(1<<CS12);			       // 256
	// TCCR1B|= (1<<CS11)|(1<<CS10); // 64

   // Enable Overflow Interrupt Enable
   TIMSK|=(1<<TOIE0);

   // INITIALIZE VARIABLES
   topCount = 0;
   burnCount = 0;
   freqCount = 0;
   burned_result_mode = 0;

   signal = 0;
   result = 0;
   burned = 0;
   current_string = 0;
   current_string_offset = 0;
   current_string_target = 0;
   prev_string_offset = 0;
}

void initCounter0() {
 
	// Counter register is 8-bit, and 256 is max value
	// so from above 0,032 ms * 256 increments = 8,192 ms
	// at 8,192 ms timer will overflow

 	// TIMER/COUNTER0 PRESCALER
 	// TCCR0|=(1<<CS02)|(1<<CS01)|(1<<CS00); // 1024
		 TCCR0|=(1<<CS02);                     // 256
	// TCCR0|= (1<<CS01)|(1<<CS00);          // 64

	//Enable Overflow Interrupt Enable
	TIMSK|=(1<<TOIE0);
}

void initLEDS() {
    // PORTD SHALL SEND OUTPUT TO LEDS
	  DDRD = 0xff;
}


void initADC() {

	/* 
	ADC PRESCALER CALCULATION TABLE
	
	Prescaler / MHz
	256 / 8 000 000 = 0,000032 s = 0,032 ms
 
	MHz / Prescaler
	8 000 000 / 256 = 31 250 Hz = 31,250 kHz 
	8 000 000 / 128 = 62 500 Hz = 62,500 kHz
	8 000 000 / 64 = 125 000 Hz = 125,000 kHz
	*/

	// ADC PRESCALER
	   ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128
	// ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (0 << ADPS0); // 64
	// ADCSRA |= (1 << ADPS2) | (0 << ADPS1) | (1 << ADPS0); // 32
	// ADCSRA |= (1 << ADPS2) | (0 << ADPS1) | (0 << ADPS0); // 16
	
	ADMUX |= (1 << REFS0); // Set ADC reference to AVCC
	ADMUX |= (1 << ADLAR); // Left adjust ADC result to allow easy 8 bit reading

  // ADC0 USED AS STANDARD, MUX SETTING IS SKIPPED

 	ADCSRA |= (1 << ADATE);  // Set ADC to Free-Running Mode

	// ADC NOISE CANCELER - OPTIONAL
	// MCUCR |= (1 << SM0); // Setting sleep mode to "ADC Noise Reduction"
	// MCUCR |= (1 << SE);  // Sleep enable 

	ADCSRA |= (1 << ADEN);  // Enable ADC
	ADCSRA |= (1 << ADIE);  // Enable ADC Interrupt
}

int main (void) {
 
   initLEDS();
   initADC();
   initCounter1();
   initCounter0();

   sei();   // Enable Global Interrupts

   ADCSRA |= (1 << ADSC);  // Start A2D Conversions

   while(1) { // Loop Forever
   
		// ADC NOISE CANCELER - EXPERIMENTAL
		/*
		MCUCR |= (1<<SE);         // Set enable sleep
    ADCSRA |= (1<<ADSC);      // Start converting
    __asm volatile ("sleep"); // go to sleep
    MCUCR &= ~(1<<SE);        // Clear enable sleep
		*/
   }
}

ISR(TIMER1_OVF_vect) {

	// This is the interrupt service routine for TIMER0 OVERFLOW Interrupt.
	// CPU automatically call this when TIMER0 overflows.

	// WHEN TIMER1-COUNTER REACHES OVERFLOW ALL ALGORITHM VARIABLES WILL BE RESET
	// RESET BURN
	burnCount = 0;
	burnResultCount = 0;
	burned = 0;
	
	// RESET TOP COUNT
	topCount = 0;
	result = UINT16_MAX;
	timer_prev = 0;
	timer = 0;
	TCNT1 = 0;
}

ISR(ADC_vect) {

	// SAVE ADC' VALUE, AND THE TIMER1 VALUE
	signal = ADCH;
  timer = TCNT1;

	// IF SIGNAL IS LOWER THAN result = 0xFFFF -> SIGNAL IS SET TO RESULT
	// RESULT WILL BE DECREMENTED PROPORTIONALLY TO SIGNAL
	if(signal < result){
		result = signal;
	}
	
	// IF NO TOPS AND WE ARE AT BEGINNING OF COUNT
	if(topCount == 0) {
		TCNT1 = 0;
		timer = 0;
	}
	
	// WHEN SIGNAL RISES AGAIN, IS LARGER THAN result, THEN A PERIOD IS REGISTERED
	// CONDITION HAS A NOISE THRESHOLD
	if(signal > result && signal < SIGNAL_CUTOFF_THRESHOLD) {
		
		// ONCE SIGNAL IS LARGER THAN SIGNAL_CUTOFF_THRESHOLD, THEN INITIATE BURN SEQUENCE
		// BURN NOTIFIES HOW MANY PERIODS SHALL BE SKIPPED
		if(burnCount < BURN && burned == 0) {
			++burnCount;
			return;
		} else {
			
			// WHEN BURN SEQUENCE IS FINISHED SIGNAL IS "STABLE"
			// PERIOD COUNTER IS INITIATED, BURN SEQUENCE VARIABLES ARE RESET
			if(burnCount >= BURN && burned == 0) {
				burnCount = 0;
				burned = 1;
				TCNT1 = 0;
			}
			
			// IF TIME INTERVAL BETWEEN PERIODS IS TOO BIG, THEN RESET ALGORITHM
			if((timer - timer_prev) > 2000) {
				topCount = 0; // SUBSEQUENTLY RESETS TCNT1 AND timer
				result = UINT16_MAX;
				TCNT1 = 0;    // JUST IN CASE
				timer_prev = 0;
				return;
			}
			
			// A TOP IS FOUND
			topCount++;
				
		}
		
		// SAVE TCNT1 FOR DETECTION OF ERRONEOUS TIME INTERVALS BETWEEN PERIODS
		timer_prev = timer;
		
		// RESET RESULT
		result = UINT16_MAX;
	}
	
	// IF BURN SEQUENCE, AND, PERIOD COUNT IS FINISHED, THEN FREQUENCY IS FOUND
	if(topCount == SAMPLES) {
	
			// TIMER1 REPRESENTS THE FREQUENCY, SEE DECLARATIONS
			result = TCNT1;

			// E
			if( TRANSITION_E_ETOP > result && result > TRANSITION_E_A) {
				current_string = 0;

				if(result >= (TRANSITION_E_A)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_E_A + LED_OFFSET_E)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_E_A + (LED_OFFSET_E*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_E_A + (LED_OFFSET_E*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_E_A + (LED_OFFSET_E*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_E_A + (LED_OFFSET_E*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_E_A + (LED_OFFSET_E*6))) {
					current_string_offset = 0;
				}
			}
			
			// A
			if( TRANSITION_E_A > result && result > TRANSITION_A_D) {
				current_string = 1;

				if(result >= (TRANSITION_A_D)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_A_D + LED_OFFSET_A)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_A_D + (LED_OFFSET_A*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_A_D + (LED_OFFSET_A*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_A_D + (LED_OFFSET_A*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_A_D + (LED_OFFSET_A*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_A_D + (LED_OFFSET_A*6))) {
					current_string_offset = 0;
				}
			}

			// D
			if( TRANSITION_A_D > result && result > TRANSITION_D_G) {
				current_string = 2;

				if(result >= (TRANSITION_D_G)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_D_G + LED_OFFSET_D)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_D_G + (LED_OFFSET_D*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_D_G + (LED_OFFSET_D*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_D_G + (LED_OFFSET_D*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_D_G + (LED_OFFSET_D*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_D_G + (LED_OFFSET_D*6))) {
					current_string_offset = 0;
				}
			}

			// G
			if( TRANSITION_D_G > result && result > TRANSITION_G_B) {
				current_string = 3;

				if(result >= (TRANSITION_G_B)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_G_B + LED_OFFSET_G)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_G_B + (LED_OFFSET_G*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_G_B + (LED_OFFSET_G*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_G_B + (LED_OFFSET_G*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_G_B + (LED_OFFSET_G*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_G_B + (LED_OFFSET_G*6))) {
					current_string_offset = 0;
				}
			}

			// B
			if( TRANSITION_G_B > result && result > TRANSITION_B_EH) {
				current_string = 4;

				if(result >= (TRANSITION_B_EH)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_B_EH + LED_OFFSET_B)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_B_EH + (LED_OFFSET_B*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_B_EH + (LED_OFFSET_B*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_B_EH + (LED_OFFSET_B*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_B_EH + (LED_OFFSET_B*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_B_EH + (LED_OFFSET_B*6))) {
					current_string_offset = 0;
				}
			}

			// EH
			if( TRANSITION_B_EH > result && result > TRANSITION_EH_BOTTOM) {
				current_string = 5;

				if(result >= (TRANSITION_EH_BOTTOM)) {
					current_string_offset = 6;
				}
				if(result >= (TRANSITION_EH_BOTTOM + LED_OFFSET_EH)) {
					current_string_offset = 5;
				}
				if(result >= (TRANSITION_EH_BOTTOM + (LED_OFFSET_EH*2))) {
					current_string_offset = 4;
				}
				if(result >= (TRANSITION_EH_BOTTOM + (LED_OFFSET_EH*3))) {
					current_string_offset = 3;
				}
				if(result >= (TRANSITION_EH_BOTTOM + (LED_OFFSET_EH*4))) {
					current_string_offset = 2;
				}
				if(result >= (TRANSITION_EH_BOTTOM + (LED_OFFSET_EH*5))) {
					current_string_offset = 1;
				}
				if(result >= (TRANSITION_EH_BOTTOM + (LED_OFFSET_EH*6))) {
					current_string_offset = 0;
				}
			}
			
			// DETECT TARGET STRING, SAVE IT, AND ENTER BURN RESULT MODE
			if(burned_result_mode == 0) {
				
				// INITIATE BURN RESULT VARIABLES
				burned_result_mode = 1;
				burnResultCount = 0;
				
				// STRING TARGET CHANGE
				current_string_target = current_string; 

				// INDICATE STRING CHANGE ON LEDS (NEW STRING TARGET)
				for(int i = 0; i < 7; i++) {
					PORTD ^= (1 << current_string_target);
					_delay_ms(150);
				}

				// SHOW FIRST VALUE AS LED INDICATION WILL OTHERWISE BE SKIPPED
				PORTD = 0;
				PORTD |= (1 << current_string_offset);

			} else if(burned_result_mode == 1) {

				// IN BURN RESULT MODE DO THE FOLLOWING
				// BURN NOT COMPLETE, AND, STRING IS NOT TARGET
				
				// CHECK IF WE ARE ON TARGET, IF SO, INDICATE STRING WITH LEDS
				if(current_string == current_string_target) {
					
					// OUTPUT TO LEDS, CORRECT STRING
					if(current_string_offset < prev_string_offset) {
						for(int i = prev_string_offset ; i > (current_string_offset - 1); --i) {
							PORTD = (1 << i);
							_delay_ms(70);
						}
					} else if(current_string_offset > prev_string_offset) {
						for(int i = prev_string_offset; i < (current_string_offset + 1); ++i) {
							PORTD = (1 << i);
							_delay_ms(70);
						}
					}
					PORTD = 0;					
					PORTD |= (1 << current_string_offset);

					prev_string_offset = current_string_offset;

					// RESET OUR BURN COUNTER TO PROCEED WITH THE BURN ALGORITHM
					burnResultCount = 0;
				} else {
					// IF OFF TARGET, INCREMENT BURN COUNT
					burnResultCount++;
					PORTD = 0;
				}
				
				// CHECK IF BURN SEQUENCE HAS COMPLETED
				if(burnResultCount == BURN_RESULT){
					
					// RESET VARIABLES 
					burned_result_mode = 0;					
				}
			}
			
			// RESET VARIABLES FOR NEXT PERIOD COUNT
			result = UINT16_MAX;
			topCount = 0;
			TCNT1 = 0;
			
			// RESET BURN SEQUENCE
			burned = 0;
		}
}
