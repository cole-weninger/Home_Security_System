#define TOTAL_KEYS     12   // Number of Buttons on Keypad
#define TOTAL_COLS     3    // Number of Columns on Keypad
#define TOTAL_ROWS     4    // Number of Rows on Keypad

/* Define keys */
char hexaKeys[4][3] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};

/*
//           MSP430FR2xxx                          Keypad Matrix
//        -----------------                >-/ -o  >-/ -o  >-/ -o
//        |               |                |    |  |    |  |    |
//     /|\|           P8.0|<---o--ROW1-----^----|--^----|--^----|
//      | |               |    |                |       |       |
//      --|RST            |   4.7M              |       |       |
//        |               |    |                |       |       |
//        |               |   GND          >-/ -o  >-/ -o  >-/ -o
//        |               |                |    |  |    |  |    |
//        |           P8.1|<---o--ROW2-----^----|--^----|--^----|
//        |               |    |                |       |       |
//        |               |   4.7M              |       |       |
//        |               |    |                |       |       |
//        |               |   GND          >-/ -o  >-/ -o  >-/ -o
//        |               |                |    |  |    |  |    |
//        |           P8.2|<---o--ROW3-----^----|--^----|--^----|
//        |               |    |                |       |       |
//        |               |   4.7M              |       |       |
//        |               |    |                |       |       |
//        |               |   GND          >-/ -o  >-/ -o  >-/ -o
//        |               |                |    |  |    |  |    |
//        |           P8.3|<---o--ROW4-----^----|--^----|--^----|
//        |               |    |                |       |       |
//        |               |   4.7M              |       |       |
//        |               |    |                |       |       |
//        |               |   GND               |       |       |
//        |           P5.0|-->----COL1----------^       |       |
//        |           P5.2|-->----COL2------------------^       |
//        |           P5.3|-->----COL3--------------------------^
*/

#define ROW_PINS	GPIO_PIN0|GPIO_PIN1|GPIO_PIN2|GPIO_PIN3
#define ROW_PORT	GPIO_PORT_P8
#define COL_PINS	GPIO_PIN0|GPIO_PIN2|GPIO_PIN3
#define COL_PORT	GPIO_PORT_P5

void Init_Keypad()
{
	//GPIO_setAsInputPinWithPullDownResistor(ROW_PORT, ROW_PINS);
	P8DIR &= 0xF0;
	P8REN |= 0x0F;

	//GPIO_setAsOutputPin(COL_PORT, COL_PINS);
	//GPIO_setOutputHighOnPin(COL_PORT, COL_PINS);
	P5DIR |= 0b1101;
	P5OUT |= 0b1101;
}

static int KeypressDetected(){
	if ((P8IN & 0x0F) != 0)
		return 1;
	else
		return 0;
}

static char Keypad_performScan ()
{

    if (!KeypressDetected())
		return '\0';

    __delay_cycles(100);

    int kb = 0;
	int row = -1;
	int col = -1;
	char keyNum = 0;
	
	if (P8IN & 0b1)
		row = 0;
	else if (P8IN & 0b10)
		row = 1;
	else if (P8IN & 0b100)
		row = 2;
	else if (P8IN & 0b1000)
		row = 3;

	P5OUT &= 0xF2;

	for (kb = 0; kb < TOTAL_COLS; kb++){

		switch(kb){
			case 0:
				P5OUT |= 0b1;
				break;
			case 1:
				P5OUT |= 0b100;
				break;
			case 2:
				P5OUT |= 0b1000;
				break;
			default:
				P1OUT |= 0b1;
				break;
		}
		
		if (KeypressDetected()){
			col = kb;
			break;
		}
			
	}
	
	keyNum = hexaKeys[row][col]; // Calculate keypad entry

    //toggle power to keypad
	P5OUT |= 0b1101;
	
    return(keyNum);
}
