/*
 * FinalBoard.cpp
 *
 * Created: 4/15/2026 09:47:28
 * Author : Groups 4 and 12
 */ 

// ----- INCLUDES ----- //

#include <avr/io.h>
#include <avr/interrupt.h>



// ----- DEFINES ----- //

#define FLAG_ON							(0xFFU)
#define FLAG_OFF						(0U)

#define IO_BOARD_SPI_SCK				(PORTB5)	// Digital 13
#define IO_BOARD_SPI_MISO				(PORTB4)	// Digital 12
#define IO_BOARD_SPI_MOSI				(PORTB3)	// Digital 11
#define IO_BOARD_SPI_NSS				(PORTB2)	// Digital 10

#define IO_BOARD_SR_LATCH_IN			(PORTB1)	// Digital 9
#define IO_BOARD_SR_LATCH_OUT			(PORTB0)	// Digital 8
#define IO_STROBE_LOOP_DELAY			(10U)
#define IO_UPDATE_MS					(20U)

#define DEBOUNCE_CIRCULAR_BUFFER_SIZE	(3U)

#define FLIPPER_PICK_OCRx				(255U)
#define FLIPPER_HOLD_OCRx				(25U)
#define FLIPPER_PICK_HIGH_COUNT			(2U)		// Pick time: FLIPPER_PICK_HIGH_COUNT * IO_UPDATE_MS
#define FLIPPER_LEFT_OC0A				(PORTD6)	// Digital 6
#define FLIPPER_RIGHT_OC0B				(PORTD5)	// Digital 5

#define DROP_TARGET_RESET_PIN			(PORTD2)	// Digital 2
#define DROP_TARGET_PICK_PULSE_MS		(60U)
#define DROP_TARGET_RESET_DELAY_MS		(1000U)

#define BALL_HOLD_PIN					(PORTD4)	// Digital 4 (Amy Change :D)
#define BALL_HOLD_PICK_PULSE_MS			(40U)

#define BALL_LAUNCH_PIN					(PORTD3)	// Digital 3
#define BALL_LAUNCH_PICK_PULSE_MS		(80U)

#define DEBUG_PIN						(PORTD0)	// Digital 0

#define ADC_ANALOG_INPUT_0				(PORTC0)	// A0
#define ADC_UPDATE_MS					(10U)

#define UART_TX_PIN						(PORTD1)	// Digital 1
#define UART_RTS_PIN					(PORTD7)	// Digital 7

#define SCORE_UPDATE_MS					(100U)

#define ROLLOVER_1_SCORE				(100U)
#define ROLLOVER_2_SCORE				(100U)
#define ROLLOVER_3_SCORE				(100U)
#define ROLLOVER_4_SCORE				(100U)
#define ROLLOVER_5_SCORE				(100U)
#define ROLLOVER_6_SCORE				(100U)

#define RAMP_ENTRANCE_SCORE				(100U) 

#define BONUS_MODE_TIMEOUT				(10000U)

#define SCORE_FLASH_DELAY				(2000U)
#define SCORE_IDLE_VALUE				(0U)
// ----- STRUCTURES ----- //

struct IO_Input_Struct {
	// Input Port C
	uint8_t Rollover4     : 1;
	uint8_t Rollover3     : 1;
	uint8_t Rollover2     : 1;
	uint8_t Rollover1     : 1;
	uint8_t StandUpTarget : 1;
	uint8_t Spinner       : 1;
	uint8_t               : 1;
	uint8_t ProgramADC    : 1; // PROGRAMMING: Overwrite LEDMeasurements_LUT[IO_Output.EntryIndex] with ADCresult
	
	// Input Port B
	uint8_t BallReturnSensor : 1;
	uint8_t DropTarget3      : 1;
	uint8_t DropTarget2      : 1;
	uint8_t DropTarget1      : 1;
	uint8_t CycleEntry       : 1; // PROGRAMMING: Cycle through LEDMeasurements_LUT entries (With rollover)
	uint8_t CycleIncDecDelta : 1; // PROGRAMMING: Toggle IO_Output.DeltaSelection bit
	uint8_t DecrementEntry   : 1; // PROGRAMMING: Add delta to current entry
	uint8_t IncrementEntry   : 1; // PROGRAMMING: Subtract delta from current entry
	
	// Input Port A
	uint8_t RFlipperPB   : 1;
	uint8_t RFlipperEOS  : 1;
	uint8_t LFlipperPB   : 1;
	uint8_t LFlipperEOS  : 1;
	uint8_t LaunchPB     : 1;
	uint8_t RampEntrance : 1;
	uint8_t Rollover6    : 1;
	uint8_t Rollover5    : 1;
	
	// Unused byte
	uint8_t : 8;
};
volatile IO_Input_Struct IO_Input = {0U};

struct IO_Output_Struct {
	// Output Port C
	uint8_t LED0 : 1;
	uint8_t LED1 : 1;
	uint8_t LED2 : 1;
	uint8_t LED3 : 1;
	uint8_t LED4 : 1;
	uint8_t LED5 : 1;
	uint8_t LED6 : 1;
	uint8_t LED7 : 1;
	
	// Output Port B
	//uint8_t LED8  : 1;
	//uint8_t PortB : 7;
	
	// Output Port A
	//uint8_t PortA;
	
	// ADC Table Tuning
	uint16_t LED8           : 1;
	uint16_t CurrentEntry   : 10;
	uint16_t EntryIndex     : 4;
	uint16_t DeltaSelection : 1;
	
	// Control Byte
	uint8_t BallLaunchState      : 2;
	uint8_t DropTargetState      : 2;
	uint8_t BallHoldState        : 2;
	uint8_t DropTargetsDown      : 1;
	uint8_t                      : 1;
	
};
volatile IO_Output_Struct IO_Output = {0U};



// ----- GLOBAL VARIABLES ----- //

volatile uint32_t TIME_clockTick_ms = 0U;

volatile uint8_t SPI_IN_PortA, SPI_IN_PortB, SPI_IN_PortC;
volatile uint8_t IO_FSM_state;
volatile uint8_t IO_DEBOUNCE_flag = 0U;
volatile uint32_t rawSwitchStates[DEBOUNCE_CIRCULAR_BUFFER_SIZE] = {0U};
volatile uint8_t writeIndex = 0U;
volatile uint32_t stableHigh, stableLow;
volatile uint8_t i;

volatile uint8_t LFlipper_highCount = 0U;
volatile uint8_t LFlipper_state = 0U;
volatile uint8_t RFlipper_highCount = 0U;
volatile uint8_t RFlipper_state = 0U;
volatile uint8_t FLIPPER_updateFlag = 0U;

uint32_t ballLaunchStart = 0U;

uint32_t IO_lastUpdate = 0U;

uint32_t dropTargetPulseStart = 0U;
uint8_t dropTarget1Count = 0U;
uint8_t dropTarget2Count = 0U;
uint8_t dropTarget3Count = 0U;
uint8_t dropTarget1State = 0U;
uint8_t dropTarget2State = 0U;
uint8_t dropTarget3State = 0U;
uint8_t dropTargetBonusCount = 0U;
uint8_t dropTargetResetDelayStart = 0U;

uint32_t ballHoldStart = 0U;
uint8_t ballHoldCount = 0U;

uint16_t LEDMeasurements_LUT[9U] = {0U};
uint16_t LEDFill_LUT[9U];
uint8_t deltaList[2U];
uint8_t LED_incrementDone = 0U;
uint8_t LED_decrementDone = 0U;
uint8_t LED_cycleDeltaDone = 0U;
uint8_t LED_incrementEntryDone = 0U;
uint8_t LEDProgramming_state = 0U;

uint32_t ADC_lastConversion = 0U;
volatile uint16_t movingAvg_circBuffer[8U] = {0U};
volatile uint8_t movingAvg_bufferIndex = 0U;
volatile uint16_t movingAvg_sum = 0U;
volatile uint16_t ADCresult = 0U;
uint8_t ADC_ResultPresentFlag = 0U;

uint8_t RolloverSwitchState = 0U;
uint8_t Rollover1Count = 0U;
uint8_t Rollover2Count = 0U;
uint8_t Rollover3Count = 0U;
uint8_t Rollover4Count = 0U;
uint8_t Rollover5Count = 0U;
uint8_t Rollover6Count = 0U;

uint8_t RampState = 0U;
uint8_t RampCount = 0U;



uint32_t scoreboard_lastUpdate = 0U;

// UART Transmission Variables:
volatile uint8_t dataBuffer[6U] = {
	0x10U,
	0x01U,
	0x00U,
	0x02U,
	0x00U,
	0xFFU
};

volatile uint8_t bufferIndex = 0;

volatile uint32_t outputScore = 0U;
volatile uint32_t pastGameScore = 1000U;
volatile uint32_t scoreBuffer = 0U;

volatile uint8_t gameModeState = 0U;
volatile uint32_t bonusTimeStart = 0U;

volatile uint32_t lastIdleFlash = 0U;

volatile uint8_t scoreFlasherState = 0;
// ----- HELPER FUNCTIONS ----- //

/*
 * Performs timestampA - timestampB, accounting for clock tick overflow
 */
uint32_t TIME_tickDiff(uint32_t timestampA, uint32_t timestampB)
{
	uint32_t returnValue = 0U;
	
	if (timestampA < timestampB)
	{
		// Clock tick overflow occurred, do the workaround:
		returnValue = (0xFFFFFFFFU - timestampB) + 1U + timestampA;
	}
	else
	{
		// No clock tick overflow occurred, normal subtraction OK
		returnValue = timestampA - timestampB;
	}
	
	return returnValue;
}


uint32_t TIME_getTick(void)
{
	return TIME_clockTick_ms;
}



void debounceFSM(void)
{
	// Pointers are fun :D
	uint8_t *byteArray = (uint8_t*)(&(rawSwitchStates[writeIndex]));
	byteArray[0U] = SPI_IN_PortC;
	byteArray[1U] = SPI_IN_PortB;
	byteArray[2U] = SPI_IN_PortA;
	
	// Update write index as circular buffer
	writeIndex++;
	if (writeIndex == DEBOUNCE_CIRCULAR_BUFFER_SIZE)
		writeIndex = 0U;
	
	// Compute stableHigh, stableLow
	stableHigh = 0xFFFFFFFFU;
	stableLow = 0U;
	for (i = 0U; i < DEBOUNCE_CIRCULAR_BUFFER_SIZE; i++)
	{
		stableHigh &= rawSwitchStates[i];
		stableLow |= rawSwitchStates[i];
	}
	
	// Determine new debounced states
	uint32_t *debouncedStates = (uint32_t*)(&IO_Input);
	*debouncedStates = ((*debouncedStates) & stableLow) | stableHigh;
}


void FlipperFSMs(void)
{
	// Left Flipper
	if (IO_Input.LFlipperPB)
	{
		LFlipper_state = 0U;
		OCR0A = 0U;
	}
	else
	{
		switch (LFlipper_state)
		{
			case 0U:
				LFlipper_state = 1U;
				OCR0A = FLIPPER_PICK_OCRx;
				LFlipper_highCount = 0U;
				break;
			case 1U:
				if (LFlipper_highCount <= FLIPPER_PICK_HIGH_COUNT)
				{
					LFlipper_highCount++;
				}
				else
				{
					LFlipper_state = 2U;
					OCR0A = FLIPPER_HOLD_OCRx;
				}
				break;
			case 2U:
				if (!(IO_Input.LFlipperEOS))
				{
					LFlipper_state = 1U;
					OCR0A = FLIPPER_PICK_OCRx;
					LFlipper_highCount = 0U;
				}
				break;
			default:
				LFlipper_state = 0U;
				OCR0A = 0U;
				LFlipper_highCount = 0U;
				break;
		}
	}
	
	// Right Flipper
	if (IO_Input.RFlipperPB)
	{
		RFlipper_state = 0U;
		OCR0B = 0U;
	}
	else
	{
		switch (RFlipper_state)
		{
			case 0U:
				RFlipper_state = 1U;
				OCR0B = FLIPPER_PICK_OCRx;
				RFlipper_highCount = 0U;
				break;
			case 1U:
				if (RFlipper_highCount <= FLIPPER_PICK_HIGH_COUNT)
				{
					RFlipper_highCount++;
				}
				else
				{
					RFlipper_state = 2U;
					OCR0B = FLIPPER_HOLD_OCRx;
				}
				break;
			case 2U:
				if (!(IO_Input.RFlipperEOS))
				{
					RFlipper_state = 1U;
					OCR0B = FLIPPER_PICK_OCRx;
					RFlipper_highCount = 0U;
				}
				break;
			default:
				RFlipper_state = 0U;
				OCR0B = 0U;
				RFlipper_highCount = 0U;
				break;
		}
	}
}


void BallLaunch_FSM(uint32_t now)
{
	switch (IO_Output.BallLaunchState)
	{
		case 0U:
			// Idle state
			if (!(IO_Input.LaunchPB))
			{
				IO_Output.BallLaunchState = 1U;
			}
			break;
		case 1U:
			// Running state
			if (ballLaunchStart == 0U)
			{
				ballLaunchStart = now;
			
				// Set pin
				PORTD |= (1U << BALL_LAUNCH_PIN);
			}
			else if (TIME_tickDiff(now, ballLaunchStart) >= BALL_LAUNCH_PICK_PULSE_MS)
			{
				ballLaunchStart = 0U;
				IO_Output.BallLaunchState = 2U; // Goto done state
			
				// Reset pin
				PORTD &= ~(1U << BALL_LAUNCH_PIN);
			}
			break;
		case 2U:
			// Done state
			if (IO_Input.LaunchPB)
			{
				IO_Output.BallLaunchState = 0U;
			}
			break;
		default:
			break;
	}
}


void IO_Update(uint32_t now)
{
	IO_lastUpdate = now;
	FLIPPER_updateFlag = FLAG_ON;
	
	//uint8_t *inputArray = (uint8_t*)(&IO_Input);
	uint8_t *outputArray = (uint8_t*)(&IO_Output);
	//outputArray[0] = inputArray[0];
	//outputArray[1] = inputArray[1];
	//outputArray[2] = inputArray[2];
	//outputArray[2] = IO_Output.BallLaunchState;
	
	// Strobe L_I to latch in inputs
	PORTB &= ~(1U << IO_BOARD_SR_LATCH_IN);
	for (uint8_t i = 0U; i < IO_STROBE_LOOP_DELAY; i++) {}
	PORTB |= (1U << IO_BOARD_SR_LATCH_IN);
	
	// Send first output, prep for reading first input
	SPDR = outputArray[2U];
	IO_FSM_state = 0U;
}


void DropTargets_FSM(uint32_t now)
{
	switch (IO_Output.DropTargetState)
	{
		case 0U:
			/*
			// Waiting for reset condition / Idle
			if (IO_Output.DropTargetReset)
			{
				dropTargetPulseStart = now;
				//IO_Output.DropTargetResetState = 1;
			}
			
			if (dropTargetPulseStart != 0U)
			{
				if (TIME_tickDiff(now, dropTargetPulseStart) >= DROP_TARGET_RESET_DELAY_MS)
				{
					dropTargetPulseStart = 0U;
					IO_Output.DropTargetResetState = 1U;
				}
			}
			*/
			
			if (!(IO_Input.DropTarget1) && dropTarget1State == 0U)
			{
				dropTarget1State = 1U;
				dropTarget1Count = 10U;
			}
			
			if (!(IO_Input.DropTarget2) && dropTarget2State == 0U)
			{
				dropTarget2State = 1U;
				dropTarget2Count = 10U;
			}
			
			if (!(IO_Input.DropTarget3) && dropTarget3State == 0U)
			{
				dropTarget3State = 1U;
				dropTarget3Count = 10U;
			}
			
			if (dropTarget1State && dropTarget2State && dropTarget3State)
			{
				IO_Output.DropTargetState = 1U;
				IO_Output.DropTargetsDown = 1U; // CHLOE
				dropTargetPulseStart = 0U;
				dropTargetResetDelayStart = 0U;
				dropTargetBonusCount = 50U;
			}
			break;
		case 1U:
			// Pick
			if (dropTargetResetDelayStart == 0U)
			{
				dropTargetResetDelayStart = now;
			}
			else if (TIME_tickDiff(now, dropTargetResetDelayStart) >= 3000U)
			{
				if (dropTargetPulseStart == 0U)
				{
					dropTargetPulseStart = now;
					
					// Set pin
					PORTD |= (1U << DROP_TARGET_RESET_PIN);
				}
				else if (TIME_tickDiff(now, dropTargetPulseStart) >= DROP_TARGET_PICK_PULSE_MS)
				{
					dropTargetPulseStart = 0U;
					dropTargetResetDelayStart = 0U;
					IO_Output.DropTargetState = 2U;
					
					dropTarget1State = 0U;
					dropTarget2State = 0U;
					dropTarget3State = 0U;
					
					// Reset pin
					PORTD &= ~(1U << DROP_TARGET_RESET_PIN);
				}
			}
			break;
		case 2U:
			// Pulse complete, wait for switches reset
			if (IO_Input.DropTarget1 && IO_Input.DropTarget2 && IO_Input.DropTarget3)
			{
				IO_Output.DropTargetState = 0U;
				IO_Output.DropTargetsDown = 0U; // CHLOE
			}
			break;
		default:
			break;
	}
}


void BallHold_FSM(uint32_t now)
{
	switch (IO_Output.BallHoldState)
	{
		case 0U:
			// Idle state
			if (!(IO_Input.Rollover2))
			{
				IO_Output.BallHoldState = 1U;
				ballHoldStart = now;
				
				// Set pin for pick
				PORTD |= (1U << BALL_HOLD_PIN);
			}
			break;
		case 1U:
			// Pick state
			if (TIME_tickDiff(now, ballHoldStart) >= BALL_HOLD_PICK_PULSE_MS)
			{
				IO_Output.BallHoldState = 2U;
				ballHoldStart = now;
				
				ballHoldCount = 0U;
				PORTD &= ~(1U << BALL_HOLD_PIN);
			}
			break;
		case 2U:
			// Hold state
			if (IO_Input.Rollover2)
			{
				IO_Output.BallHoldState = 0U;
				PORTD &= ~(1U << BALL_HOLD_PIN);
			}
			
			if (TIME_tickDiff(now, ballHoldStart) >= 2U)
			{
				ballHoldStart = now;
				ballHoldCount++;
				if (ballHoldCount == 3U)
				{
					PORTD |= (1U << BALL_HOLD_PIN);
				}
				else if (ballHoldCount >= 4U)
				{
					ballHoldCount = 0U;
					PORTD &= ~(1U << BALL_HOLD_PIN);
				}
			}
			break;
		default:
			break;
	}
}

void updateGamemode(void)
{
	switch(gameModeState)
	{
		// Idle
		case 0U:
			if(!(IO_Input.LaunchPB))
			{
				gameModeState = 1U;
				outputScore = 42069U;
			}
			break;
		// Normal Gameplay Scoring
		case 1U:
			
			/*
			if((!(IO_Input.DropTarget1) && 
				!(IO_Input.DropTarget2) &&
				!(IO_Input.DropTarget3) && 
				!(IO_Output.DropTargetReset)))
			{
			*/ // CHLOE
			if (IO_Output.DropTargetsDown)
			{
				bonusTimeStart = TIME_getTick();
				gameModeState = 2U;
			}
			break;
		
		//Bonus mode Gameplay Scoring
		case 2U:			
			if(TIME_tickDiff(TIME_getTick(), bonusTimeStart) >= BONUS_MODE_TIMEOUT)
			{
				gameModeState = 1U;
				//IO_Output.DropTargetReset = 1U; // CHLOE
			}
			break;
		
		//Game Over
		case 3U:
			pastGameScore = outputScore;
			gameModeState = 0U;
			break;
		default:
			gameModeState = 0U;
	}
	
}

void updateInputCounters(void)
{
	//--Spinner--//
	
	//--Ramp--//
	switch(RampState)
	{
		case 0U:
			if(!(IO_Input.RampEntrance))
			{
				RampState = 1U;
			}
			break;
		case 1:
			if(!(IO_Input.RampEntrance))
			{
				RampCount++;
				RampState = 2U;				
			}
			break;
		case 2U:
			if(IO_Input.RampEntrance)
			{
				RampState = 0U;
			}
			break;
		default:
			
			break;
	}
	
	//---Drop Targets---//
	//TODO Talk with doug about his progress
	
	//---Rollover Switches---//
	switch(RolloverSwitchState)
	{
		// Check if any rollover switch is non zero to go into count state
		case 0U:
			if (!(IO_Input.Rollover1) |
				!(IO_Input.Rollover2) |
				!(IO_Input.Rollover3) |
				!(IO_Input.Rollover4) |
				!(IO_Input.Rollover5) |
				!(IO_Input.Rollover6) )
			{
				RolloverSwitchState = 1U;
			}
			
			break;
		
		case 1U:
			if(!(IO_Input.Rollover1))
			{
				Rollover1Count++;
			}
			if(!(IO_Input.Rollover2))
			{
				Rollover2Count++;
			}
			if(!(IO_Input.Rollover3))
			{
				Rollover3Count++;
			}
			if(!(IO_Input.Rollover4))
			{
				Rollover4Count++;
			}
			if(!(IO_Input.Rollover5))
			{
				Rollover5Count++;
			}
			if(!(IO_Input.Rollover6))
			{
				Rollover6Count++;
			}
			
			RolloverSwitchState = 2U;
			
			break;
		case 2U:
			if ((IO_Input.Rollover1) &
				(IO_Input.Rollover2) &
				(IO_Input.Rollover3) &
				(IO_Input.Rollover4) &
				(IO_Input.Rollover5) &
				(IO_Input.Rollover6) )
			{
				RolloverSwitchState = 0U;
			}
			
			break;
		default:
			RolloverSwitchState = 0U;
			
			break;
	}
}


void beginScoreboardUpdate()
{
	
	//---Rollover Switches---//
	
	if(Rollover1Count)
	{
		scoreBuffer += (Rollover1Count * ROLLOVER_1_SCORE);
		Rollover1Count = 0U;
	}
	if(Rollover2Count)
	{
		scoreBuffer += (Rollover2Count * ROLLOVER_2_SCORE);
		Rollover2Count = 0U;
	}
	if(Rollover3Count)
	{
		scoreBuffer += (Rollover3Count * ROLLOVER_3_SCORE);
		Rollover3Count = 0U;
	}
	if(Rollover4Count)
	{
		scoreBuffer += (Rollover4Count * ROLLOVER_4_SCORE);
		Rollover4Count = 0U;
	}
	if(Rollover5Count)
	{
		scoreBuffer += (Rollover5Count * ROLLOVER_5_SCORE);
		Rollover5Count = 0U;
	}
	if(Rollover6Count)
	{
		scoreBuffer += (Rollover6Count * ROLLOVER_6_SCORE);
		Rollover6Count = 0U;
	}
	
	// ----- DROP TARGETS ----- //
	if (dropTarget1Count)
	{
		scoreBuffer += dropTarget1Count;
		dropTarget1Count = 0U;
	}
	if (dropTarget2Count)
	{
		scoreBuffer += dropTarget2Count;
		dropTarget2Count = 0U;
	}
	if (dropTarget3Count)
	{
		scoreBuffer += dropTarget3Count;
		dropTarget3Count = 0U;
	}
	if (dropTargetBonusCount)
	{
		scoreBuffer += dropTargetBonusCount;
		dropTargetBonusCount = 0U;
	}
	
	//--Ramp Scoring--//
	if(RampCount)
	{
		scoreBuffer += (RampCount * RAMP_ENTRANCE_SCORE);
		RampCount = 0U;
	}	
	
	switch(gameModeState)
	{
		case 0U:
			//define logic to switch back and forth between 0s and previous scores
			
			if(TIME_tickDiff(TIME_getTick(), lastIdleFlash) >= SCORE_FLASH_DELAY)
			{
				lastIdleFlash = TIME_getTick();
				if(!scoreFlasherState)
				{
					outputScore = pastGameScore;
					scoreFlasherState = 1U;
				}
				else
				{
					outputScore = SCORE_IDLE_VALUE;
					scoreFlasherState = 0U;
				}
			}
			
			
			break;
		//Normal Gameplay Scoring
		case 1U:
			outputScore += scoreBuffer;
			scoreBuffer = 0U;
			break;
		
		//Bonus Mode
		case 2U:
			outputScore += (scoreBuffer << 1U);
			scoreBuffer = 0U;
			break;
		//Game Over
		case 3U:
			scoreBuffer = 0U;
			break;
		default:
			scoreBuffer = 0U;
			break;
		
	}
	
	
	//outputScore = 0x5555U;
	dataBuffer[2] = static_cast<uint8_t>(outputScore & 0xFF);
	dataBuffer[4] = static_cast<uint8_t>(outputScore >> 8);
	
	// Enable TX complete ISR
	UCSR0B |= (1 << TXCIE0);
	
	// Begin TX cycle
	UCSR0B |= (1 << TXB80);			// 9th bit = 1 for node address
	UDR0 = dataBuffer[0];			// TX node address
	bufferIndex = 1;				// Setup for subsequent TXs
	
}

// ----- SETUP / CONFIG FUNCTION ----- //
void setup(void)
{
	/*
	 * INPUT STRUCTURE:
	 */
	
	uint32_t *magicPointer = (uint32_t*)(&IO_Input);
	*magicPointer = 0xFFFFFFFFU;//CK this code
	
	/*
	 * TIMER2, 1 ms Heartbeat Config:
	 */
	
	// Registers
	TCCR2A = (1U << WGM21); // CTC mode
	TCCR2B = (1U << CS22) | (0U << CS21) | (0U << CS20); // Divide by 64
	TIMSK2 = (1U << OCIE2A);
	OCR2A = 249U; // OCR2A = (1e-3 * 16e6) / 64 - 1 = 249
	
    /*
	 * SPI, I/O Board Config:
	 */
	
	// Pins
	DDRB &= ~(1U << IO_BOARD_SPI_MISO);
	DDRB |= ((1U << IO_BOARD_SPI_SCK) | (1U << IO_BOARD_SPI_MOSI) | (1U << IO_BOARD_SPI_NSS) | 
		(1U << IO_BOARD_SR_LATCH_IN) | (1U << IO_BOARD_SR_LATCH_OUT));
	
	// Setting initial values
	PORTB |= ((1U << IO_BOARD_SPI_NSS) | (1U << IO_BOARD_SR_LATCH_IN));
	PORTB &= ~(1U << IO_BOARD_SR_LATCH_OUT);
	
	// Registers
	SPCR = (1U << SPIE) | (1U << SPE) | (0U << DORD) | (1U << MSTR) |
		(0U << CPOL) | (0U << CPHA) | (1U << SPR1) | (0U << SPR0);
	SPSR = 0U;
	
	/*
	 * TIMER0, Flipper PWM Config:
	 */
	
	// Pins
	
	DDRD |= (1U << FLIPPER_LEFT_OC0A) | (1U << FLIPPER_RIGHT_OC0B);
	
	// Registers
	
	TCCR0A = (1U << COM0A1) | (1U << COM0B1) | (1U << WGM01) | (1U << WGM00);
	TCCR0B = (0U << CS02) | (1U << CS01) | (1U << CS00);
	TIMSK0 = 0U;
	OCR0A = 0U;
	OCR0B = 0U;
	
	/*
	 * Ball Launch Config:
	 */
	
	DDRD |= (1U << BALL_LAUNCH_PIN);
	PORTD &= ~(1U << BALL_LAUNCH_PIN);
	
	/*
	 * Drop Target Reset Config:
	 */
	
	DDRD |= (1U << DROP_TARGET_RESET_PIN);
	PORTD &= ~(1U << DROP_TARGET_RESET_PIN);
	
	/*
	 * Ball Hold Config:
	 */
	
	DDRD |= (1U << BALL_HOLD_PIN);
	PORTD &= ~(1U << BALL_HOLD_PIN);
	
	/*
	 * Debug Pin:
	 */
	
	DDRD |= (1U << DEBUG_PIN);
	PORTD &= ~(1U << DEBUG_PIN);
	
	/*
	 * ADC Config:
	 */
	
	// Pins
	DDRC &= ~(1U << ADC_ANALOG_INPUT_0);
	
	// Registers
	ADMUX = ((0U << REFS1) | (1U << REFS0) | (0U << ADLAR) | (0U << MUX3) |
		(0U << MUX2) | (0U << MUX1) | (0U << MUX0));
	ADCSRA = ((1U << ADEN) | (0U << ADSC) | (0U << ADATE) | (0U << ADIF) |
		(1U << ADIE) | (1U << ADPS2) | (1U << ADPS1) | (1U << ADPS0));
	ADCSRB = 0U;
	DIDR0 = (1U << ADC0D);	// Disable digital buffer for A0
	
	// Fill lookup tables
	LEDFill_LUT[0U] = 0x01FFU;
	LEDFill_LUT[1U] = 0x00FFU;
	LEDFill_LUT[2U] = 0x007FU;
	LEDFill_LUT[3U] = 0x003FU;
	LEDFill_LUT[4U] = 0x001FU;
	LEDFill_LUT[5U] = 0x000FU;
	LEDFill_LUT[6U] = 0x0007U;
	LEDFill_LUT[7U] = 0x0003U;
	LEDFill_LUT[8U] = 0x0001U;
	
	deltaList[0U] = 10U;
	deltaList[1U] = 1U;
	
	LEDMeasurements_LUT[0U] = 0x01A7U;
	LEDMeasurements_LUT[1U] = 0x0194U;
	LEDMeasurements_LUT[2U] = 0x015CU;
	LEDMeasurements_LUT[3U] = 0x012EU;
	LEDMeasurements_LUT[4U] = 0x011EU;
	LEDMeasurements_LUT[5U] = 0x0110U; // 0x0119U
	LEDMeasurements_LUT[6U] = 0x0101U;
	LEDMeasurements_LUT[7U] = 0x00EEU;
	LEDMeasurements_LUT[8U] = 0x00EBU;
	
	IO_Output.DeltaSelection = 0U;
	IO_Output.EntryIndex = 0U;
	IO_Output.CurrentEntry = LEDMeasurements_LUT[0U];
	/*
	 * UART TX Config:
	 */
	
	// Registers
	
	// N/A
	UCSR0A = 0U;
	
	// Transmit enable, 9-bit character size
	UCSR0B = (1U << TXEN0) | (1U << UCSZ02) | (1U << TXCIE0);
	UCSR0C = (1U << UCSZ01) | (1U << UCSZ00);
	
	// 250k baud rate
	UBRR0 = 3U;
	
	// Pins
	DDRD |= (1U << UART_TX_PIN) | (1U << UART_RTS_PIN);
	PORTD |= (1U << UART_RTS_PIN);
	
	// Enable global interrupts
	sei();
}



// ----- MAIN ENTRYPOINT ----- //
int main(void)
{
	// Do setup tasks
	setup();
	
	uint32_t now;
	uint32_t startupTime = TIME_getTick();
	
	// Power-on delay
	while (TIME_tickDiff(TIME_getTick(), startupTime) < 2000U);
	
    while (1U)
	{
		/*
		 * I/O Update
		 */
		now = TIME_getTick();
		if (TIME_tickDiff(now, IO_lastUpdate) >= IO_UPDATE_MS)
		{
			IO_Update(now);
		}
		
		/*
		 * DEBOUNCING
		 */
		if (IO_DEBOUNCE_flag)
		{
			IO_DEBOUNCE_flag = FLAG_OFF;
			debounceFSM();
		}
		
		/*
		 * FLIPPER CONTROL
		 */
		if (FLIPPER_updateFlag)
		{
			FLIPPER_updateFlag = FLAG_OFF;
			FlipperFSMs();
		}
		
		/*
		 * BALL LAUNCH CONTROL
		 */
		now = TIME_getTick();
		BallLaunch_FSM(now);
		
		/*
		 * DROP TARGET CONTROL
		 */
		now = TIME_getTick();
		DropTargets_FSM(now);
		
		/*
		 * BALL HOLD CONTROL
		 */
		now = TIME_getTick();
		BallHold_FSM(now);
		
		/*
		 * ADC CONTROL
		 */
		now = TIME_getTick();
		if (TIME_tickDiff(now, ADC_lastConversion) >= ADC_UPDATE_MS)
		{
			ADC_lastConversion = now;
			ADCSRA |= (1U << ADSC);
		}
		
		if (ADC_ResultPresentFlag)
		{
			ADC_ResultPresentFlag = FLAG_OFF;
			
			// Update LED bar with result given LUT
			for (i = 0U; i < 9U; i++)
			{
				if (ADCresult >= LEDMeasurements_LUT[i])
				{
					// Set the LED bar pins accordingly
					uint8_t *bytePointer = (uint8_t*)(&IO_Output);
					
					// Write data to LED0 - LED7
					bytePointer[0U] = (uint8_t)(LEDFill_LUT[i] & 0x00FFU);
					
					// Write to LED8, guarantee bit clear and then or in data
					bytePointer[1U] &= ~(0x01U);
					bytePointer[1U] |= (uint8_t)((LEDFill_LUT[i] & 0x0100U) >> 8U);
					
					break;
				}
			}
		}
		
		// Update Game Rule Mode
		updateGamemode();
		
		// Counter Update
		updateInputCounters();
		
		// Scoreboard Update
		now = TIME_getTick();
		if(TIME_tickDiff(now, scoreboard_lastUpdate) >= SCORE_UPDATE_MS)
		{
			scoreboard_lastUpdate = now;
			beginScoreboardUpdate();
		}
		
		

		
		// Debug output
		PORTD ^= (1U << DEBUG_PIN);
	}
}



// ----- TIMER2 ISR: 1 ms Heartbeat ----- //
ISR(TIMER2_COMPA_vect)
{
	TIME_clockTick_ms++;
}



// ----- SPI (IO BOARD) ISR ----- //
ISR(SPI_STC_vect)
{	
	uint8_t *outputArray = (uint8_t*)(&IO_Output);
	switch (IO_FSM_state)
	{
		case 0U:
			// Read first input, send second output
			SPI_IN_PortA = SPDR;
			SPDR = outputArray[1U];
			IO_FSM_state = 1U;
			break;
		case 1U:
			// Read second input, send third output
			SPI_IN_PortB = SPDR;
			SPDR = outputArray[0U];
			
			IO_FSM_state = 2U;
			break;
		case 2U:
			// Read third input
			SPI_IN_PortC = SPDR;
			
			// Strobe L_O to latch outputs
			PORTB |= (1U << IO_BOARD_SR_LATCH_OUT);
			for (uint8_t i = 0U; i < IO_STROBE_LOOP_DELAY; i++) {}
			PORTB &= ~(1U << IO_BOARD_SR_LATCH_OUT);
			
			IO_DEBOUNCE_flag = FLAG_ON;
			break;
		default:
			// Fail safe catch :D
			break;
	}
}



// ----- ADC ISR, Conversion Complete ----- //
ISR(ADC_vect)
{
	// ----- ADC Conversion Result And Moving Average Filter ----- //
	
	// Remove old entry from sum
	movingAvg_sum -= movingAvg_circBuffer[movingAvg_bufferIndex];
	
	// Add new entry to buffer
	movingAvg_circBuffer[movingAvg_bufferIndex] = (ADC & 0x03FFU);
	
	// Add new entry to sum
	movingAvg_sum += movingAvg_circBuffer[movingAvg_bufferIndex];
	
	// Compute new average (8 entries, divide by 8 is r-shift by 3)
	ADCresult = movingAvg_sum >> 3U;
	
	// Update buffer index
	movingAvg_bufferIndex++;
	if (movingAvg_bufferIndex >= 8U)
		movingAvg_bufferIndex = 0U;
	
	ADC_ResultPresentFlag = FLAG_ON;
}

ISR(USART_TX_vect)
{
	if (bufferIndex < 5U)
	{
		UCSR0B &= ~(1U << TXB80);	// Clear 9th bit
	}
	else
	{
		UCSR0B |= (1U << TXB80);		// Set 9th bit for stop
		UCSR0B &= ~(1U << TXCIE0);	// Disable TX complete ISR
	}
	
	UDR0 = dataBuffer[bufferIndex];
	bufferIndex++;
}


