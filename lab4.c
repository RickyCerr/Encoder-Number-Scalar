///////////////////////////////////////////////////////////////////////////////
// encoder_skel.c
// Tests functionality of both the encoders using the bar graph 
// Name: Richard Cerrato
// Date: 11/9/2023
///////////////////////////////////////////////////////////////////////////////
// pg 276 spi
//PORT D connections
//   PORT D:               7  6  5  4  3  2  1  0
//    SRCLK     SCK (SCK)--|  |  |  |  |  |  |  |--unused         
//     enc_Qh  MISO (MISO)----|  |  |  |  |  |-----unused
//     SER_bar MOSI (MOSI)-------|  |  |  |--------unused  
//           (unused) SS_n----------|  |-----------unused
// *SS_n is unused but must be configred as an output in master mode
//
//PORT (E) connections
//   PORT E:         7  6  5  4  3  2  1  0
//      not present--|  |  |  |  |  |  |  |--RCLK bargraph only
//      not present-----|  |  |  |  |  |-----SH/LD_n (encoder bd)
//      not present--------|  |  |  |--------unused
//      not present-----------|  |-----------unused
//
//PORT (C) connections
//   PORT C:         7  6  5  4  3  2  1  0
//      not present--|  |  |  |  |  |  |  |--OE_n (PWM dimming) DONT USE ANYMORE, WE DONT NEED TO DIM
//      not present-----|  |  |  |  |  |-----unused
//      not present--------|  |  |  |--------unused
//      not present-----------|  |-----------unused
//

// On the Encoder board:  
//                         8  7  6  5  4  3  2  1  0--vss GND
//               Vdd 3.3v--|  |  |  |  |  |  |  |--Qh PORTD6 MISO       
//    PORTE1       sh/ld_n----|  |  |  |  |  |-----clk_inhibit gnd
//      srclk to bar_graph-------|  |  |  |--------sd_in SER bargraph xxxxnope
//   output from main board SCK-----|  |-----------sd_in MOSI PORTD5   xxxxnope

// PIN lAYOUT FOR A 2X5 HEADER (EXAMPLE PORT N)
//            
//      PN0 - [o] [o] - PN1
//      PN2 - [o] [o] - PN3
//      PN4 - [o] [o] - PN5
//      PN6 - [o] [o] - PN7
//      GND - [o] [o] - 3.3V
//
// (on main board)

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#define TRUE 1
#define FALSE 0
#define LOOP_DELAY 500  //delay in ms for the loop
#define BLINK_DELAY_MS 100

uint8_t num_to_seg[12] = {    //each index is a (corresponds to the decimal value) number 0-11 in the correct binary configuration activate the appropriate 7seg segments, starting at 0
	0b11000000, // 0
	0b11111001, // 1
	0b10100100, // 2
	0b10110000, // 3
	0b10011001, // 4
	0b10010010, // 5
	0b10000010, // 6
	0b11111000, // 7
  0b10000000, // 8
	0b10011000, // 9
	0b11111111, //some weird garbage
	0b01111111 //some more weird garbage
};

  //PORT C corresponding digit selection with bit outs
  uint8_t digit_select[4] = {
  0b01100000, /* digit 4 (ones place) [0] */
  0b01001010, /* digit 3 (tens place) [1] */
  /* 0b01001000,  : (semicolon) [2] */
  0b01000010, /* digit 2 (100s place) [3] */
  0b01000000 /* digit 1 (1000s place) [4] */
  };


uint8_t segment_val[5]; //stores the next value of each segment so port A can be set to it


void segsum(uint16_t sum) {

  segment_val[3] = 0xFF;//num_to_seg[(sum/1000) % 10]; //1000s place

  segment_val[2] = num_to_seg[(sum/100) % 10]; //100s place

  segment_val[1] = num_to_seg[(sum/10) % 10]; //10s place
 
  segment_val[0] = num_to_seg[(sum) % 10]; //1s place

  

  //if(sum<1000) segment_val[1] = 0xFF; //these if statements cutoff the leading 0s, by checking if the count is under a certain number
  if(sum<100) segment_val[2] = 0xFF;  //example: if sum is < 1000, the 1000s place digit should be turned off (set to 0xFF)
  if(sum<10) segment_val[1] = 0xFF;


}

  
///////////////////////////////////////////////////////////////////////////////
//                      Initalize Port D SPI 
void init_SPID(void){
  PORTD.DIRSET  |= 0b10110000; //configure SPI pins on port D bit 6 is 0 because it is MISO (which is an input)
  SPID.CTRL     |= 0b01010000; //setup SPI   
  SPID.INTCTRL  = 0x00; //no interrupts 
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
void init_32Mhz_RC_clk(void){
//setup 32Mhz internal RC clock
  OSC.CTRL = OSC_RC32MEN_bm; //enable 32MHz clock
  while (!(OSC.STATUS & OSC_RC32MRDY_bm)); //wait for clock to be ready
  CCP = CCP_IOREG_gc; //enable protected register change
  CLK.CTRL = CLK_SCLKSEL_RC32M_gc; //switch to 32MHz clock
}
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************
//                               encoder_chk                                        
//******************************************************************************
//
//Checks for encoder movement using the 4-state state machine. 
//Tested with Bournes 24 pulse encoder PEC12R-3217F-S0024.
//Clockwise        movement returns  1 
//Counterclockwise movement returns  0
//No encoder       movement returns -1
//Detent position is expected to assert A and B inputs both to logic true.
//
//expects encoder pin states to be in the arg "encoder_var" as follows:
//     encoder S2     encoder S1
//   array index 1   array index 0
//    b_pin a_pin     b_pin a_pin
//     bit6  bit5      bit1  bit0
// bits 3,4 not used; bit2 is switch for S1, bit7 is for switch S2

//uint8_t encoder_var[2] = {0x00, 0x00};
uint8_t old_A[2] = {0,0};
uint8_t old_B[2] = {0,0};
int count = 0;
int display_val = 2;

int8_t encoder_chk(uint8_t encoder_var, uint8_t encoder_sel){
  
  uint8_t new_A[2] = {0,0};
  uint8_t new_B[2] = {0,0};
  
  new_A[0] = ((encoder_var &0x01) == 0) ? 0 : 1;
  new_B[0] = ((encoder_var &0x02) == 0) ? 0 : 1;

  new_A[1] = ((encoder_var &0x20) == 0) ? 0 : 1;
  new_B[1] = ((encoder_var &0x40) == 0) ? 0 : 1;

  uint8_t return_val;

  return_val = -1; // default return value , no change
  if (( new_A[encoder_sel] != old_A[encoder_sel] ) || ( new_B[encoder_sel] != old_B[encoder_sel] )){ // if change occurred
    if (( new_A[encoder_sel] == 0) && ( new_B[encoder_sel] == 0)) {
      if ( old_A[encoder_sel] == 1){ count ++;}
      else { count --;}
    }
  else if (( new_A[encoder_sel] == 0) && ( new_B[encoder_sel] == 1)) {
    if ( old_A[encoder_sel] == 0){ count ++;}
    else { count --;}
    }
  else if (( new_A[encoder_sel] == 1) && ( new_B[encoder_sel] == 1)) { // detent position
    if ( old_A[encoder_sel] == 0){ if ( count == 3){ return_val =0;}} // one direction
    else { if ( count == -3){ return_val =1;}} // or the other
    count = 0; // count is always reset in detent position
    }
  else if (( new_A[encoder_sel] == 1) && ( new_B[encoder_sel] == 0)) {
    if ( old_A[encoder_sel] == 1) { count ++;}
    else { count --;}
    }
    old_A[encoder_sel] = new_A[encoder_sel] ; // save what are now old values
    old_B[encoder_sel] = new_B[encoder_sel] ;
  } // if change occurred
return ( return_val ); // return encoder state
} 

///////////////////////////////////////////////////////////////////////////////
//ISR for TC0 

uint8_t spi_data_out = 0x01; //initalize output data (bar graph) to have only one LED lit
int8_t encoder_out = 0;
uint8_t encoder_sel = 0;
int scale = 0;
int delay = 0;
ISR(TCC0_OVF_vect){
  
  PORTC.OUT = 0b01000000; 

  PORTE.OUT = 0b00000001;  //set SH/LD (PORTE Bit 1) to low AND set rclk (PORTE Bit 0) to high
  PORTE.OUT = 0b00000010;  //set SH/LD to high (loads values onto SPID) AND set rclk (PORTE Bit 0) to low
  
  SPID.DATA = 0x00;
  while(bit_is_clear(SPID.STATUS,7));
  uint8_t encoder_var = SPID.DATA; // load SPI data from MISO (encoder Qh)

  SPID_DATA = spi_data_out; // Send data to SPI port _ is for sending output, . is for receiving input
  while(!(SPID_STATUS)); // Wait for SPI completion

  encoder_out = encoder_chk(encoder_var, encoder_sel); // check encoder with the loaded MISO SPID data and the encoder selector (either S1 or S2)
  if (encoder_sel == 0){
    encoder_sel ++; // increment encoder select to so the next check is for encoder S2
  }
  else if (encoder_sel == 1){
    encoder_sel --; // decrement encoder select to so the next check is for encoder S1
  }
  
  if(encoder_out == 1){
    scale += 1; //if turned right, increment count by 1
  }
  else if(encoder_out == 0){
    scale -= 1; //if turned left, decrement count by 1
  }

  if(scale > 255 ) scale = 0; // reset count if its in upper or lower bound (0 - 255)
  if(scale < 0)  scale = 255;

  segsum(scale); // calls segsum

  for(int j = 0; j < 4; j++){
      PORTA.OUT = 0xFF;
      PORTC.OUT = digit_select[j];  //Sets port C to be the correct digit to output 
      PORTA.OUT = segment_val[j];   //sets Port A to the array for its corresponding digit and number value
      _delay_ms(1);
		}

  int num_leds = (scale / 32) + 1; // how many LED bars should be lit, gives a number 1 - 8
  spi_data_out = (1 << num_leds) - 1; // set MOSI (bargraph LEDs to this value, will be displayed later)

//capture encoder pin states into shift register
//send data in thermomether code to bar graph while shifing in bits from encoder
//update bar graph display
//see if the encoders moved
//adjust count based on encoder movement
}//ISR 
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//setup timer for normal mode overflows every 1mS
void init_TC0_normal(void){
//setup timer for normal mode and overflow interrupt
//
TCC0_CTRLA |= TC_CLKSEL_DIV64_gc; //TC0 runs on 500Khz
TCC0_CTRLB |= TC_WGMODE_NORMAL_gc;
TCC0_PER    = 0x0200;             //512 decimal
TCC0_INTCTRLA = TC0_OVFINTLVL0_bm; //interrupt level bit zero set
}
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
//                                    main()
int main(void){
  init_32Mhz_RC_clk(); //set clock to 32Mhz
  init_SPID();         //setup SPI on PORT D
  init_TC0_normal();   //setup timer TC0

//configure all PORT A pins as WIREDANDPULL OPC[2:0]=111
	PORTA.PIN0CTRL = 0b00111000; //sets port a pins as logic AND
	PORTA.PIN1CTRL = 0b00111000;
	PORTA.PIN2CTRL = 0b00111000;
	PORTA.PIN3CTRL = 0b00111000;
	PORTA.PIN4CTRL = 0b00111000;
	PORTA.PIN5CTRL = 0b00111000;
	PORTA.PIN6CTRL = 0b00111000;
	PORTA.PIN7CTRL = 0b00111000;
//configure Port C upper nibble as all totem pole outputs
  PORTC.DIR = 0xFF; //initialize port C as outputs
  PORTCFG.MPCMASK = 0xF0; 
  PORTC.PIN4CTRL = PORT_OPC_TOTEM_gc;
//configure other PORT pins as needed
  PORTA.DIR = 0b11111111 ;

  PORTE.DIRSET  |= 0b00000011;  //RCLK and SH/LD pin as output   
  PORTE.OUT     = 0b00000010; //RCLK pin starts logic low

//enable low level interrupts and global interrupt enable
  PMIC.CTRL |= 0x01;
  sei();

  while(1){
//Turn on each digit by incrementing the count to PORTC bits 4-6.
   // PORTE.OUT = 0b00000001; // Toggle RCLK high
   // PORTE.OUT = 0b00000000; // RCLK goes back low
  //_delay_ms(250); 
  _delay_loop_2(LOOP_DELAY); //loop to make LEDs visible
  }//while
}//main


