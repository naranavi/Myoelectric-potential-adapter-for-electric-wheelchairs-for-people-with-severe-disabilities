#ifndef ADS1x9x_H_
#define ADS1x9x_H_

/****************************************************************/
/* Global functions*/
/****************************************************************/
void Init_ADS1x9x_Resource(void);
void Init_ADS1x9x(void);
void ADS1x9x_Default_Reg_Init(void);
void ADS1x9x_Reg_Write(unsigned char Reg_address, unsigned char Reg_data);
unsigned char ADS1x9x_Reg_Read(unsigned char Reg_address);
//unsigned char Status ADS1x9x_Packet_Read(unsigned char Channel_address, long *data);

void ADS1x9x_Clock_Select(unsigned char clock_in);
void Init_ADS1x9x_Resource(void);
void Start_Read_Data_Continuous(void);
void Power_Up_ADS1x9x(void);
void ADS1x9x_Reset(void);
void Init_ADS1x9x_DRDY_Interrupt(void);
void ADS1x9x_Read_All_Regs(unsigned char ADS1x9xeg_buf[]);
void Soft_Stop_ADS1x9x(void);
void Stop_Read_Data_Continuous(void);
void Soft_Start_ADS1x9x(void);
void ADS1x9x_Default_Reg_Init(void);
void Disable_ADS1x9x_DRDY_Interrupt (void);
void Clear_ADS1x9x_Chip_Enable (void);
void Enable_ADS1x9x_DRDY_Interrupt (void);
void Set_ADS1x9x_Chip_Enable (void);
void ADS1x9x_PowerOn_Init(void);
void ADS1x9x_Enable_Start(void);
void ADS1x9x_Disable_Start(void);
void Set_Device_out_bytes(void);

/****************************************************************/
/* ADS1x9x COMMAND DESCRIPTION and definations */
/****************************************************************/
 // System Commands
 #define WAKEUP		0x02		//Wake-up from standby mode
 #define STANDBY	0x04		//Enter standby mode
 #define RESET		0x06		//Reset the device
 #define START		0x08		//Start/restart (synchronize) conversions
 #define STOP		0x0A		//Stop conversion
 #define OFFSETCAL  0x1A		//Channel offset calibration

// Data Read Commands
 #define RDATAC		0x10		//Enable Read Data Continuous mode.
 								//This mode is the default mode at power-up.
 #define SDATAC		0x11		//Stop Read Data Continuously mode
 #define RDATA		0x12		//Read data by command; supports multiple read back.
 
 // Register Read Commands
 #define RREG		0x20		//Read n nnnn registers starting at address r rrrr
 								//first byte 001r rrrr (2xh)(2) - second byte 000n nnnn(2)
 #define WREG		0x40		//Write n nnnn registers starting at address r rrrr
								//first byte 010r rrrr (2xh)(2) - second byte 000n nnnn(2)
 
/****************************************************************/
/* ADS1x9x register addresses                                  */
/****************************************************************/

#define ADS1x9x_REG_DEVID         (0x0000u)
/* 
 * ID: ID Control Register (Factory-Programmed, Read-Only)
 * Address = 00h
 * ----------------------------------------------------------------------
 * |BIT 7  |BIT 6    |BIT 5   |BIT 4 |BIT 3   |BIT 2   |BIT 1   |BIT 0   |
 * |-------|---------|--------|------|--------|--------|--------|------- |
 * |REV_ID4| REV_ID3 |REV_ID2 |  N/A |   N/A  |   N/A  |REV_ID1 |REV_ID0 |
 * ----------------------------------------------------------------------
 * 
 * The ID Control Register is programmed during device manufacture to indicate device characteristics.
 * 
 * Bits[7:5] : Revision identification These bits identify the device revision.
 * 000 = TBD
 * 001 = TBD
 * 010 = TBD
 * 100 = TBD
 * 101 = TBD
 * 110 = TBD
 * 111 = TBD
 * 
 * Bits[4:2] N/A
 * 
 * BIT[1:0] : Revision identification These bits identify the device revision.
 * 00 = ADS1191
 * 01 = ADS1192
 * 10 = ADS1291
 * 11 = ADS1292/2R
 *  			
*/

#define ADS1x9x_REG_CONFIG1       (0x0001u)
/*
 * 
 * CONFIG1: Configuration Register 1
 * Address = 01h
 *------------------------------------------------------------------
 * |BIT 7       |BIT 6 |BIT 5  |BIT 4 |BIT 3 |BIT 2 |BIT 1  |BIT 0 |
 * |------------|------|-------|------|------|------|-------|------|
 * |SINGLE_SHOT |   0  |   0   | 0    |  0   | OSR2 | OSR1  | OSR0 |
 *------------------------------------------------------------------
 * 
 * Bit 7 SINGLE_SHOT: Single-shot conversion
 * 	This bit sets the conversion mode
 * 		0 = Continuous conversion mode (default)
 * 		1 = Single-shot mode
 * 	NOTE: Pulse mode is disabled when individual data rate control is selected.
 * 
 * Bits[6:3] Must be set to '0'
 * 
 *  Bits[2:0] OSR[2:0]: Channel oversampling ratio
 *  These bits determine the oversampling ratio of both channel 1 and channel 2.
 * --------------------------------------------------------------
 * 			BIT 	|	OVERSAMPLING RATIO 	|	SAMPLE RATE		|
 * 			000 	|	fMOD/1024 			|	125SPS			|
 * 			001 	|	fMOD/512 			|	250SPS			|
 * 			010 	|	fMOD/256 			|	500SPS			|
 * 			011 	|	fMOD/128 			|	1024SPS			|
 * 			100 	|	fMOD/128 			|	2048SPS			|
 * 			101		| Do Not Use			|	Do Not Use		|
 * 			110		| Do Not Use			|	Do Not Use		|
 * 			111		| Do Not Use			|	Do Not Use		|
 * --------------------------------------------------------------
 */
#define ADS1x9x_REG_CONFIG2       (0x0002u)

/*
 * CONFIG2: Configuration Register 2
 * Address = 02h
 *-----------------------------------------------------------------------------------
 * |BIT 7  |BIT 6    	  |BIT 5  	 | BIT 4   |  BIT 3 |BIT 2  |BIT 1    |BIT 0 	|
 * |-------|--------------|----------|---------|--------|-------|---------|---------|
 * |   1   |PDB_LOFF_COMP |PDB_REFBUF| VREF_4V | CLK_EN |   0   |INT_TEST |TEST_FREQ|
 *-----------------------------------------------------------------------------------
 * 
 * Configuration Register 2 configures the test signal generation. See the Input Multiplexer section for more details.
 * 	Bit 7 Must always be set to '1'
 * 
 *  Bit 6 PDB_LOFF_COMP : Lead-off comparator power-down
 *  	This bit powers down the lead-off comparators.
 *  	0 = Lead-off comparators disabled (default)
 *  	1 = Lead-off comparators enabled
 * 	
 *  Bit 5 PDB_REFBUF : Reference buffer power-down
 * 		This bit powers down the internal reference buffer so that the external reference can be used.
 *  	0 = Reference buffer is powered down (default)
 * 		1 = Reference buffer is enabled
 *  
 *	Bit 4 VREF_4V: Enables 4-V reference
 * 		This bit chooses between 2.4V and 4V reference.
 *		0 = 2.4-V reference (default)
 *		1 = 4-V reference
 *  
 * Bit 3 CLKOUT_EN: CLK connection
 *		This bit determines if the internal oscillator signal is connected to the CLK pin when an internal oscillator is used.
 *		0 = Oscillator clock output disabled (default)
 *		1 = Oscillator clock output enabled
 *	
 * 	Bit 2 Must be set to '0'
 *	
 *  Bit 1 TEST_AMP: Test signal amplitude
 *		This bit determines the test signal amplitude.
 * 		0 = No test signal (default)
 *		1 = ?(VREFP ?EVREFN)/2400
 *	
 *  Bit 0 TEST_FREQ: Test signal frequency.
 *		This bit determines the test signal frequency.
 *		0 = At dc (default)
 *		1 = Square wave at 1 Hz
 */

#define ADS1x9x_REG_LOFF       (0x0003u)

/*
 * LOFF: Lead-Off Control Register
 * Address = 03h
 *-----------------------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5    |BIT 4     | BIT 3      |BIT 2       |BIT 1         | BIT 0 	 |
 * |----------|-------  |-------  |----------|----------- |---------   |--------------|----------|
 * |COMP_TH2  |COMP_TH1 |COMP_TH0 | 0        | ILEAD_OFF1 | ILEAD_OFF0 |0             | FLEAD_OFF|
 *------------------------------------------------------------------------------------------------
 * 
 * The Lead-Off Control Register configures the Lead-Off detection operation.
 *	
 * Bits[7:5] Lead-off comparator threshold
 *		These bits determine the lead-off comparator threshold. 
 *		Comparator positive side in %
 *		000 = 95 (default)
 *		001 = 92.5
 *		010 = 90
 *		011 = 87.5
 *		100 = 85
 *		101 = 80
 *		110 = 75
 *		111 = 70
 *		Comparator negative side in %
 *		000 = 5 (default)
 *		001 = 7.5
 *		010 = 10
 *		011 = 12.5
 *		100 = 15
 *		101 = 20
 *		110 = 25
 *		111 = 30
 *	
 * Bit 4 Must be set to 1
 * 
 * Bits[3:2] ILEAD_OFF[1:0]: Lead-off current magnitude
 *		These bits determine the magnitude of current for the current lead-off mode.
 *		00 = 6 nA (default)
 *		01 = 24 nA
 *		10 = 6 microA
 *		11 = 24 microA
 *		
 * Bit 1 Must be set to '0'
 *	
 * Bit 0 FLEAD_OFF: Lead-off frequency
 *		This bit generates the LEAD_OFF_CLK signal. It also generates AC_LEAD_OFF, which is '1' when FLEAD_OFF is '1'.
 *		0 = At dc lead-off detect (default)
 *		1 = At ac lead-off detect at DECICLK/4 (500 Hz for an 2-kHz output rate)
 * 
 */
 
#define ADS1x9x_REG_CH1SET        (0x0004u)
/*	CHnSET: Individual Channel Settings
 * 	Address = 04h
 *--------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5   |  BIT 4  | BIT 3  | BIT 2  |  BIT 1  | BIT 0 	 |
 * |----------|---------|--------|---------|--------|--------|---------|---------|
 * |   PD1    |	GAIN1_1 |GAIN1_0 |  0      |  MUX1_3| MUX1_2 |  MUX1_1 | MUX1_0  |
 *--------------------------------------------------------------------------------
 * 
 * The CH1SET Control Register configures the power mode, PGA gain, and multiplexer settings channels. 
 *	
 * Bit 7 PD1: Channel 1 power-down
 *		0 = Normal operation (default)
 *		1 = Channel 1 power-down
 *	
 * Bits[6:4 ] GAIN1[2:0]: Channel 1 PGA gain setting
 *		These bits determine the PGA gain setting for channel 1.
 *		000 = 6 (default)
 *		001 = 1
 *		010 = 2
 *		011 = 3
 *		100 = 4
 *		101 = 8
 *		110 = 12
 * 
 * Bits[3:0] MUX1[3:0]: Channel 1 input selection
 *		These bits determine the channel 1 input selection.
 *		0000 = Normal electrode input (default)
 *		0001 = Input shorted (for offset measurements)
 *		0010 = RLD_MEASURE
 *		0011 = VDD/2 for supply measurement
 *		0100 = Temperature sensor
 *		0101 = Cal signal
 *		0110 = RLD_DRP (positive electrode is the driver)
 *		0111 = RLD_DRM (negative electrode is the driver)
 *		1000 = Reserved
 *		1001 = MUX RESPP/RESPN to INP/INM
 *		1010 = Reserved
 * 
 */

#define ADS1x9x_REG_CH2SET        (0x0005u)
/*	CHnSET: Individual Channel Settings
 * 	Address = 05h
 *--------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5   |  BIT 4  | BIT 3  | BIT 2  |  BIT 1  | BIT 0 	 |
 * |----------|---------|--------|---------|--------|--------|---------|---------|
 * |   PD2    |	GAIN2_1 |GAIN2_0 |  0      |  MUX2_3| MUX2_2 |  MUX2_1 | MUX2_0  |
 *--------------------------------------------------------------------------------
 * 
 * The CH1SET Control Register configures the power mode, PGA gain, and multiplexer settings channels. 
 *	
 * Bit 7 PD2: Channel 1 power-down
 *		0 = Normal operation (default)
 *		1 = Channel 1 power-down
 *	
 * Bits[6:4 ] GAIN2[2:0]: Channel 1 PGA gain setting
 *		These bits determine the PGA gain setting for channel 1.
 *		000 = 6 (default)
 *		001 = 1
 *		010 = 2
 *		011 = 3
 *		100 = 4
 *		101 = 8
 *		110 = 12
 * 
 * Bits[3:0] MUX2[3:0]: Channel 1 input selection
 *		These bits determine the channel 1 input selection.
 *		0000 = Normal electrode input (default)
 *		0001 = Input shorted (for offset measurements)
 *		0010 = RLD_MEASURE
 *		0011 = VDD/2 for supply measurement
 *		0100 = Temperature sensor
 *		0101 = Cal signal
 *		0110 = RLD_DRP (positive electrode is the driver)
 *		0111 = RLD_DRM (negative electrode is the driver)
 *		1000 = Reserved
 *		1001 = MUX RESPP/RESPN to INP/INM
 *		1010 = Reserved
 * 
 */

#define ADS1x9x_REG_RLD_SENS      (0x0006u)
/*
 * RLD_SENSP
 * Address = 06h
 *---------------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5   |  BIT 4          | BIT 3  | BIT 2  |  BIT 1  | BIT 0 	 |
 * |----------|---------|--------|-----------------|--------|--------|---------|---------|
 * |  CHOP1   |	CHOP0   |PD_RLD  |  RLD_LOFF_SENS  | RLDN2  | RLDP2  |  RLDN1  | RLDP1   |
 *---------------------------------------------------------------------------------------
 * This register controls the selection of the positive and negative signals from each channel for right leg drive
 * derivation. See the Right Leg Drive (RLD DC Bias Circuit) subsection of the ECG-Specific Functions section for
 * details.
 * 
 * Bits[7:6] CHOP[1:0]: Chop frequency
 *		These bits determine PGA chop frequency
 *		00 = fMOD/16
 *		01 = fMOD/32
 *		10 = fMOD/2
 *		11 = fMOD/4
 *
 * Bit 5 PDB_RLD: RLD buffer power
 *		This bit determines the RLD buffer power state.
 *		0 = RLD buffer is powered down (default)
 *		1 = RLD buffer is enabled
 *	
 * Bit 4 RLD_LOFF_SENSE: RLD lead-off sense function
 *		This bit enables the RLD lead-off sense function.
 *		0 = RLD lead-off sense is disabled (default)
 *		1 = RLD lead-off sense is enabled
 *	
 * Bit 3 RLDN2: Channel 2 RLD negative inputs
 *		This bit controls the selection of negative inputs from channel 2 for right leg drive derivation.
 *		0 = Not connected (default)
 *		1 = RLD connected to IN2N
 *	
 * Bit 2 RLDP2: Channel 2 RLD positive inputs
 *		This bit controls the selection of positive inputs from channel 2 for right leg drive derivation.
 *		0 = Not connected (default)
 *		1 = RLD connected to IN2P
 *	
 * Bit 1 RLDN1: Channel 1 RLD negative inputs
 *		This bit controls the selection of negative inputs from channel 1 for right leg drive derivation.
 *		0 = Not connected (default)
 *		1 = RLD connected to IN1N
 *	
 * Bit 0 RLDP1: Channel 1 RLD positive inputs
 *		This bit controls the selection of positive inputs from channel 1 for right leg drive derivation.
 *		0 = Not connected (default)
 *		1 = RLD connected to IN1P 

 */
 
#define ADS1x9x_REG_LOFF_SENS     (0x0007u)
/*
 * LOFF_SENS
 * Address = 07h
 *--------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5   |  BIT 4  | BIT 3  | BIT 2  |  BIT 1  | BIT 0 	 |
 * |----------|---------|--------|---------|--------|--------|---------|---------|
 * | 0        |	0       |FLIP2   |  FLIP1  | LOFFN2 | LOFFP2 |  LOFFN1 | LOFFP1  |
 *--------------------------------------------------------------------------------
 *	This register selects the positive and negative side from each channel for lead-off detection. See the Lead-Off
 *	Detection subsection of the ECG-Specific Functions section for details. Note that the LOFF_STAT register bits
 *	should be ignored if the corresponding LOFF_SENS bits are set to '1'.
 * 
 *	Bits[7:6] Must be set to '0'
 * 
 *	Bit 5 FLIP2: Current direction selection
 *		This bit controls the direction of the current used for lead off derivation.
 *		0 = Disabled (default)
 *		1 = Enabled
 * 
 *	Bit 4 FLIP1: Current direction selection
 *		This bit controls the direction of the current used for lead off derivation.
 *		0 = Disabled (default)
 *		1 = Enabled
 * 
 *	Bit 3 LOFFN2: Channel 2 lead-off detection negative inputs
 *		This bit controls the selection of negative input from channel 2 for lead off detection.
 *		0 = Disabled (default)
 *		1 = Enabled
 * 
 *	Bit 2 LOFFP2: Channel 2 lead-off detection positive inputs
 *		This bit controls the selection of positive input from channel 2 for lead off detection.
 *		0 = Disabled (default)
 *		1 = Enabled
 * 
 *	Bit 1 LOFFN1: Channel 1 lead-off detection negative inputs
 *		This bit controls the selection of negative input from channel 1 for lead off detection.
 *		0 = Disabled (default)
 *		1 = Enabled
 * 
 *	Bit 0 LOFFP1: Channel 1 lead-off detection positive inputs
 *		This bit controls the selection of positive input from channel 1 for lead off detection.
 *		0 = Disabled (default)
 *		1 = Enabled  
 *	
 */
 
#define ADS1x9x_REG_LOFF_STAT      (0x0008u)
/*
 * LOFF_STAT
 * Address = 08h
 *--------------------------------------------------------------------------------------------
 * | BIT 7    |BIT 6    |BIT 5   |  BIT 4     | BIT 3    | BIT 2    |  BIT 1    | BIT 0 	 |
 * |----------|---------|--------|------------|----------|----------|-----------|------------|
 * |  0    	  |MOD_FREQ |0       |  RLD_STAT  | IN2N_OFF | IN2P_OFF | IN1N_OFF  | IN1P_OFF   |
 *--------------------------------------------------------------------------------------------
 *	This register stores the status of whether the positive or negative electrode on each channel is on or off. See the
 *	Lead-Off Detection subsection of the ECG-Specific Functions section for details. Ignore the LOFF_STAT values
 *	if the corresponding LOFF_SENS bits are not set to '1'.
 *	'0' is lead-on (default) and '1' is lead-off. When the LOFF_SENS bits are '0', the LOFF_STAT bits should be
 *	ignored.
 *	
 * 	Bit 7 Must be set to '0'
 *	
 * 	Bit 6 MOD_FREQ: System frequency selection
 *		This bit sets the modultar frequency. Two external clock values are supported: 512 kHz and 2.048 MHz. This bit must be set
 *		so that MOD_FREQ = 128 kHz.
 *		0 = External_CLK/4 (default)
 *		1 = External_CLK/16
 *	
 * 	Bit 5 Must be set to '0'
 *	
 * 	Bit 4 RLD_STAT: RLD lead-off status
 *		This bit determines the status of RLD.
 *		0 = RLD is connected (default)
 *		1 = RLD is not connected
 *	
 * 	Bit 3 IN2N_OFF: Channel 2 negative electrode status
 *		This bit determines if the channel 2 negative electrode is connected or not.
 *		0 = Connected (default)
 *		1 = Not connected
 *	
 * 	Bit 2 IN2P_OFF: Channel 2 positive electrode status
 *		This bit determines if the channel 2 positive electrode is connected or not.
 *		0 = Connected (default)
 *		1 = Not connected
 *	
 * 	Bit 1 IN1N_OFF: Channel 1 negative electrode status
 *		This bit determines if the channel 1 negative electrode is connected or not.
 *		0 = Connected (default)
 *		1 = Not connected
 *	
 * 	Bit 0 IN1P_OFF: Channel 1 positive electrode status
 *		This bit determines if the channel 1 positive electrode is connected or not.
 *		0 = Connected (default)
 *		1 = Not connected
 *  
 */
#define ADS1x9x_REG_RESP1     (0x009u)
/*
 * RESP1: Respiration Control Register 1
 * Address = 09h
    *------------------------------------------------------------------------------------------
 * | BIT 7       |  BIT 6       |BIT 5    |  BIT 4  | BIT 3    | BIT 2    |  BIT 1  | BIT 0   |
 * |-------------|--------------|---------|---------|----------|----------|---------|---------|
 * | RESP_MOD_EN |	RESP_MOD_EN |RESP_PH3 |RESP_PH2 | RESP_PH1 | RESP_PH0 |  1      | REP_CTL |
 *--------------------------------------------------------------------------------------------
 *    This register controls the respiration functionality.
 *    Bit 7 RESP_DEMOD_EN1: Enables respiration demodulation circuitry
 *       This bit enables/disables the demodulation circuitry on channel 1.
 *       0 = RESP demodulation circuitry turned off (default)
 *       1 = RESP demodulation circuitry turned on
 *    
 *    Bit 6 RESP_MOD_EN: Enables respiration modulation circuitry
 *       This bit enables/disables the modulation circuitry on channel 1.
 *       0 = RESP modulation circuitry turned off (default)
 *       1 = RESP modulation circuitry turned on
 *    
 *    Bits[5:2] RESP_PH[3:0]: Respiration phase(1)
 *       These bits control the phase of the respiration demodulation control signal.
 *       RESP_PH[3:0] RESP_CLK = 32kHz RESP_CLK = 64kHz
 *       0000 0? (default) 0? (default)
 *       0001 11.25? 22.5?
 *       0010 22.5? 45?
 *       0011 33.75? 67.5?
 *       0100 45? 90?
 *       0101 56.25? 112.5?
 *       0110 67.5? 135?
 *       0111 78.75? 157.5?
 *       1000 90? Not available
 *       1001 101.25? Not available
 *       1010 112.5? Not available
 *       1011 123.75? Not available
 *       1100 135? Not available
 *       1101 146.25? Not available
 *       1110 157.5? Not available
 *       1111 168.75? Not available
 *
 *    (1) The RESP_PH3 bit is ignored when RESP_CLK = 64kHz.
 *    Bit 1 Must be set to '1'
 *    
 *    Bit 0 RESP_CTRL: Respiration control
 *       This bit sets the mode of the respiration circuitry.
 *       0 = Internal respiration with internal clock
 *       1 = Internal respiration with external clock  
 */

#define ADS1x9x_REG_RESP2      (0x00Au)
#define CALIB_ON 7				//bit
/*
 * RESP2: Respiration Control Register 2
 * Address = 0Ah
 *--------------------------------------------------------------------------------------------
 * | BIT 7    |BIT 6     | BIT 5    |   BIT 4  |  BIT 3   |  BIT 2   |   BIT 1    | BIT 0    |
 * |----------|----------|----------|----------|----------|----------|------------|----------|
 * |CALIB_ON  |0         |   0      |     0    |     0    | RESP_FREQ| RLDREF_INT |  1       |
 *--------------------------------------------------------------------------------------------
 *    This register controls the respiration functionality.
 *    Bit 7 CALIB_ON: Calibration on
 *       This bit is used to enable offset calibration.
 *       0 = Off (default)
 *       1 = On
 *    
 *    Bits[6:3] Must be '0'
 *    
 *    Bit 2 RESP_FREQ: Respiration control frequency
 *       This bit controls the respiration control frequency when RESP_CTRL[1:0] = 10.
 *       0 = 32 kHz (default)
 *       1 = 64 kHz
 *    
 *    Bit 1 RLDREF_INT: RLDREF signal
 *       This bit determines the RLDREF signal source.
 *       0 = RLDREF signal fed externally
 *       1 = RLDREF signal (AVDD ?EAVSS)/2 generated internally (default)
 *    
 *    Bit 0 Must be set to '1;
 */
#define ADS1x9x_REG_GPIO     (0x00Cu)
/*
 * GPIO: General-Purpose I/O Register
 * Address = 0Ch
 *-----------------------------------------------------------------------------------------
 * | BIT 7    |BIT 6     | BIT 5    |   BIT 4  |  BIT 3   |  BIT 2   |   BIT 1  | BIT 0    |
 * |----------|----------|----------|----------|----------|----------|----------|----------|
 * | 0        | 0        |      0   |   0      | GPIOC2   | GPIOC1   | GPIOD2   | GPIOD1   |
 *-----------------------------------------------------------------------------------------
 *    This register controls the GPIO pins.
 *    Bits[7:4] Must be '0'
 *
 *    Bits[3:2] GPIOC[2:1]: GPIO 1 and 2 control
 *       These bits determine if the corresponding GPIOD pin is an input or output.
 *       0 = Output
 *       1 = Input (default)
 *
 *    Bits[1:0] GPIOD[2:1]: GPIO 1 and 2 data
 *    These bits are used to read and write data to the GPIO ports.
 *    When reading the register, the data returned correspond to the state of the GPIO external pins, whether they are
 *    programmed as inputs or as outputs. As outputs, a write to the GPIOD sets the output value. As inputs, a write to the
 *    GPIOD has no effect. GPIO is not available in certain respiration modes.
*/
 

enum ADS129x
{
		ADS1191_16BIT	 = 0,
		ADS1192_16BIT,
		ADS1291_24BIT,
		ADS1292_24BIT
};

enum PORT2_ADC_CONTROL
{
		TP_18	 = 1,
		POW_CE	 = 2,
		ADC_CLK	 = 4,
		ADC_CLK_SEL = 8, 
		ADC_GPIO3 = 16,
		ADC_GPIO2 = 32,
		ADC_GPIO1 = 64
};
enum PORT8_ADC_CONTROL
{
		ADC_RESET 	= 1,
		ADC_START 	= 2,
		ADC_CS	 	= 4
};

#endif /*ADS1x9x_H_*/

