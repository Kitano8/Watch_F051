
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include "MAX17055.h"
#include "LCD.h"
#include "ste2007.h"
#include "itoa.h"


uint16_t MAX_SERIAL_BUF[8];

uint16_t ve_mv = 3100;
uint16_t vr_mv = 3880;
extern uint16_t max_param_check;

#define BAT_FULLCAP 200


#define BAT_R_SENSE                     0.1f
#define capacity_multiplier_mAH (0.005f/BAT_R_SENSE)  // 0,5 мАч на 10 мОм      // 1мАч на 5мОм
#define current_multiplier_mV (0.0000015625f/BAT_R_SENSE)       // 156,25 мкА на 10 мОм         // 312.5 мкА на 5 мОм
#define voltage_multiplier_V 0.000078125f       // Совпадает
#define time_multiplier_Hours (5.625f/3600.0f)
#define time_multiplier_Mins (5.625f/60.0f)
#define percentage_multiplier (1.0f/256.0f)
#define temperature_multiplier (1.0f/256.0f)    // Signed 2's complement format. 1°C LSb when reading only the upper byte
#define term_charge_current_A          0.007f    // Ichrg 7,5%
#define cycles_multiplier (1.0f/100.0f)
#define res_multiplier_mohm          (1.0f/4.096f)

uint16_t ChargeVoltage = 4200;
uint16_t max17055_Status = 0;
float currentTime = 0;
float currentSOC = 0;

uint16_t Saved_RCOMP0=0,Saved_TempCo=0,Saved_FullCapRep=0,Saved_Cycles=0,Saved_FullCapNom=0;


extern I2C_HandleTypeDef hi2c1;
extern tFont Font12,Font18,Font24;
extern uint8_t BACK_COLOR, MENU_COLOR;
extern char TxBuffer [20];
extern uint8_t pressure_buf[96];
extern uint8_t p_count;
uint32_t A2,B2,C2,D2,E2,F2,G2,I2,K2,L2;

static uint32_t FlashFree_Address = 0;
static __IO uint32_t NbrOfPage = 0x00;
static __IO TSC_FLASH_Status TSC_FlashStatus;
static __IO TSC_Flash_TestStatus TSC_MemoryProgramStatus;

uint32_t EndAddr = 0;
uint32_t CalibrationAddr = 0;

uint16_t save_full_cap=0,save_cycles=0,save_rcomp0=0,save_tempco=0,save_QResidual_0=0,save_QResidual_1=0,save_QResidual_2=0,save_QResidual_3=0,save_dqacc=0,save_dpacc=0;

void Set_LastFlashMemoryAddress( uint32_t address);
TSC_FLASH_Status TSC_WriteDataToNVM(uint32_t FlashFree_Address, uint32_t *Data, uint32_t Size);
HAL_StatusTypeDef TSC_FLASH_Unlock(void);
void TSC_FLASH_ClearFlag(uint32_t FLASH_FLAG);
TSC_FLASH_Status TSC_FLASH_ErasePage(uint32_t Page_Address);
TSC_FLASH_Status TSC_FLASH_ProgramWord(uint32_t Address, uint32_t Data);
void restore_regs(void);

#define MAX17055_VEMPTY_REG(ve_mv, vr_mv)\
  (((ve_mv / 10) << 7) | (vr_mv / 40))
    
void max_write_reg (uint8_t reg, uint16_t value);
uint16_t max_read_reg(uint8_t reg);

uint16_t id=0;
uint8_t max17055_check(void){

	id=max_read_reg(MAX17055_DevName);
        
        if(id!=MAX17055_DEV_ID){
          return 0;
        }

	return 1;
}


void max_write_reg (uint8_t reg, uint16_t value){
        uint8_t buf[2];
        buf[0]=(uint8_t)value;
        buf[1]=(uint8_t)(value>>8);
        
        HAL_I2C_Mem_Write(&hi2c1,  MAX17055_ADRESS, (uint16_t)reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 0x10000);     //(u8 *)&value
}

uint16_t max_read_reg(uint8_t reg){
        uint8_t buf[2];
        uint16_t data=0;
        HAL_I2C_Mem_Read(&hi2c1, MAX17055_ADRESS, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 0x10000);
	data=buf[1];
        data=data<<8;
        data|=buf[0];
	return data;
}

void max_write_and_verify (uint8_t reg, uint16_t value){
      uint8_t count=0;
      uint16_t data=0;
      do{
	max_write_reg(reg, value);
        HAL_Delay(1);
        data=max_read_reg(reg);
      }while(value!=data && count++<3);
  
}

uint8_t max17055Init_CM4 (uint8_t loading, uint16_t reconfig){     // 1-loading saved history
  uint16_t StatusPOR = max_read_reg(0x00) & 0x0002;
  uint32_t currentCapacity = getCapacity();
  uint8_t count=0;
  if (currentCapacity != BAT_FULLCAP && currentCapacity != BAT_FULLCAP-1) {
    StatusPOR = 1; //Force re-config
  }
  if(reconfig){
     StatusPOR=1;
  }
  if (StatusPOR == 0) { //If no re-config neccessary
    getCapacity();
    getSOC();
  } 
  else { //Initial Config
    while (max_read_reg( 0x3D) & 1) {//do not continue until FSTAT.DNR == 0
      _delay_ms(10);
      if(count++>50){
        return 1;
      }
    }
    max_write_reg(0x18, (uint16_t)(BAT_FULLCAP / capacity_multiplier_mAH)); //Write DesignCap
    max_write_reg(0x45, (uint16_t)(BAT_FULLCAP / capacity_multiplier_mAH) / 32); //Write dQAcc
    max_write_reg(0x1E,(uint16_t)(term_charge_current_A/current_multiplier_mV) ); //50mA); //Write IchgTerm
    max_write_reg(0x3A, MAX17055_VEMPTY_REG(3100, 3880)); //3.1V //Write VEmpty, same as low voltage cutoff value for software shut init
    
    uint16_t HibCFG = max_read_reg(0xBA); //Store original HibCFG value
    max_write_reg(0x60, 0x90); //Exit Hibernate Mode Step 1
    max_write_reg(0xBA, 0x0); //Exit Hibernate Mode Step 2
    max_write_reg(0x60, 0x0); //Exit Hibernate Mode Step 3

    //Only use integer portion of dQAcc=int(DesignCap/32) in the calculation of dPAcc to avoid quantization of FullCapNom
    if (ChargeVoltage > 4275) {
      max_write_reg(0x46, (uint16_t)(((BAT_FULLCAP / capacity_multiplier_mAH) / 32.0f) * 51200.0f / (BAT_FULLCAP / capacity_multiplier_mAH)));
      max_write_reg(0xDB, 0x8400);
    } else {
      max_write_reg(0x46, (uint16_t)(((BAT_FULLCAP / capacity_multiplier_mAH) / 32.0f) * 44138.0f / (BAT_FULLCAP / capacity_multiplier_mAH)));
      max_write_reg(0xDB, 0x8000);
    }

    while (max_read_reg(0xDB) & 0x8000) {//10ms wait loop. Do not continue until ModelCFG.Refresh == 0
      _delay_ms(10);
    }
    max_write_reg(0xBA, HibCFG);// Restore Original HibCFG value

    // 4 Initialization complete
    max17055_Status = max_read_reg(0x00);
    max_write_reg(0x00, max17055_Status & 0xFFFD);
    
    // 4.1 Identify the battery
    if(loading){
      restore_saved_parameters();
      //Загрузка истории
    }
    else{
      //История не обнаружена
    }
  }
  return 0;
}

#if 0

void max_17055_init (uint8_t restored_config){  // if 1 its load saved settings of batt
  uint16_t temp_data=0,
  HibCFG=0,
  DesignCap=BATT_CAPACITY_MAH,
  IchgTerm=CHARGE_TERMINATION_CURRENT,
  VEmpty=BATT_EMPTY_VOLTAGE,
  ChargeVoltage=BATT_FULL_VOLTAGE_MV,
  dQAcc=BATT_CAPACITY_MAH/32,
  Status=0,
  FullCapNom=0,
  MixCap=0;
  
  __IO uint16_t RepCap=0,RepSOC=0,TTE=0;

  
STEP_1: 
  // 1. Check POR
  temp_data=max_read_reg(0x00)&0x0002;
  if(temp_data==0){
    goto STEP_4_3;
  }
  
  // 2. Delay until FSTAT.DNR bit == 0
  // After power-up, wait for the MAX17055 to complete its startup operations.
  while(max_read_reg(0x3d)&0x0001){
    _delay_ms(10);                       //do not continue until FSTAT.DNR == 0
  }
  
  // 3. Initialize configuration
  
  HibCFG=max_read_reg(0x00); //Store original HibCFG value
  max_write_reg(0x60,0x90);// Exit Hibernate Mode step 1
  max_write_reg(0xba,0x0);// Exit Hibernate Mode step 2
  max_write_reg(0x60,0x0);// Exit Hibernate Mode step 3

 
  // 3.1 OPTION 1 EZ Config (no INI file is needed):
  
  max_write_reg(0x18 , DesignCap) ; // Write DesignCap
  max_write_reg(0x45 , dQAcc) ; //Write dQAcc
  max_write_reg(0x1E , IchgTerm) ; // Write IchgTerm
  max_write_reg(0x3A , VEmpty) ; // Write VEmpt
  
  //Only use integer portion of dQAcc=int(DesignCap/32) in the calculation of dPAcc to avoid quantization of FullCapNom
  if (ChargeVoltage>4275){
      max_write_reg (0x46,(uint16_t)(dQAcc*51200.0f/DesignCap)); // Write dPAcc
      max_write_reg (0xDB , 0x8400) ; // Write ModelCFG
    }
    else{
      max_write_reg (0x46 , (uint16_t)(dQAcc*44138.0f/DesignCap)); //Write dPAcc
      max_write_reg (0xDB , 0x8000) ; // Write ModelCFG
    }
  //Poll ModelCFG.Refresh(highest bit), proceed to Step 4 when ModelCFG.Refresh = 0.
  while (max_read_reg(0xDB)&0x8000){
    _delay_ms(10);//10ms wait loop. Do not continue until ModelCFG.Refresh == 0.
  }
  max_write_reg (0xBA , HibCFG) ; // Restore Original HibCFG value
  
// 4. Initialization complete
  
  Status = max_read_reg(0x00); //Read Status
  max_write_and_verify (0x00, Status & 0xFFFD) ; //Write and Verify Status with POR bit cleared
  
  // 4.1 Identify the battery
  /*
  If the host recognizes the battery pack as one with a saved history, go to Step 4.5 to restore all of the
  saved parameters, otherwise, continue to Step 4.1.
  Monitor the battery.
  Once the MAX17055 is initialized and customized, the host can simply read the desired information
  from the MAX17055 and display that information to the user.
  */
  if(restored_config){
    goto STEP_4_5;
  }
  // Добавил
  RepCap = max_read_reg(0x05) ; //Read RepCap
  RepSOC = max_read_reg(0x06) ; //Read RepSOC
  
  // 4.2 Check for MAX17055 reset
  temp_data = max_read_reg(0x00) & 0x0002; //Read POR bit in Status register
  if(temp_data == 0){ 
      goto STEP_4_3;
  }
  else{
      goto STEP_1;
  }
  
  // Read the fuel gauge results
  
STEP_4_3:  
  // 4.3 Read the reported capacity and state of charge (SOC)

  /*
The MAX17055 automatically calculates and reports the state of charge of the cell in terms of a
percentage and the mAhrs remaining. The reported state of charge (RepSOC), as a percent, is read from
memory location 0x06 and the reported capacity (RepCap), in mAHrs, is read from memory location
0x05
*/
  
  RepCap = max_read_reg(0x05) ; //Read RepCap
  RepSOC = max_read_reg(0x06) ; //Read RepSOC
  
  /*
The RepSOC_HiByte has a unit of 1%, so the RepSOC_HiByte can be directly displayed to the user for
1% resolution. It is recommended to save the RepSOC value so that it can be used to restore the capacity
information in the event of a power loss.
*/
  
  // 4.4 Read the remaining time to empty (TTE)
  
  /*
The MAX17055 also calculates the time to empty (TTE). TTE is in memory location 0x11h. The LSB of
the TTE register is 5.625s
*/
  TTE = max_read_reg(0x11) ; //Read TTE
  
STEP_4_5:
  // 4.5 Save learned parameters
  
  /*
It is recommended to save the learned capacity parameters every time bit 6 of the Cycles register toggles
(so that it is saved every 64% change in the battery) so that if power is lost the values can easily be
restored
*/
  Saved_RCOMP0 = max_read_reg(0x38) ; //Read RCOMP0
  Saved_TempCo = max_read_reg(0x39) ; //Read TempCo
  Saved_FullCapRep = max_read_reg(0x10) ; //Read FullCapRep
  Saved_Cycles = max_read_reg(0x17) ; //Read Cycles
  Saved_FullCapNom = max_read_reg(0x23) ; //Read FullCapNom
  
  // 4.6 Restoring capacity parameters
  
  /*    If power is lost, then the capacity information can be easily restored with the following procedure.*/
  
  max_write_and_verify(0x38, Saved_RCOMP0) ; //WriteAndVerify RCOMP0
  max_write_and_verify(0x39, Saved_TempCo) ; //WriteAndVerify TempCo
  max_write_and_verify(0x23, Saved_FullCapNom) ; //WriteAndVerify FullCapNom
  
  // 4.7 Wait 350ms
  _delay_ms(350);
  
  // 4.8 Restore FullCap
  FullCapNom= max_read_reg(0x23) ; //Read FullCapNom
  MixCap=(max_read_reg(0x0D)*FullCapNom)/25600 ;
  max_write_and_verify(0x0F, MixCap) ; //WriteAndVerify MixCap
  max_write_and_verify(0x10, Saved_FullCapRep) ; //WriteAndVerify FullCapRep
  //Write dQacc to 200% of Capacity and dPacc to 200%
  dQAcc = (Saved_FullCapNom/ 16) ;
  max_write_and_verify (0x46, 0x0C80) ; //Write and Verify dPacc
  max_write_and_verify (0x45, dQAcc) ; //Write and Verify dQacc
  
  // 4.10 Wait 350ms
   _delay_ms(350);
   
   // 4.11 Restore Cycles register
   max_write_and_verify(0x17, Saved_Cycles) ; //WriteAndVerify Cycles
}
/*
If the IC is being production tested, the following procedure updates the fuel gauge outputs to a known
state to verify proper operation of the IC. The sequence for configuring the MAX17055, as outlined in
the MAX17055 Software Implementation Guide, should be followed prior to sending the Quickstart
command to verify that the device was set up correctly. Set the power supply to the desired voltage and
issue the Quickstart command as described below
*/
uint16_t testing_max17055 (void){
  uint16_t Data=0,
  Capacity=BATT_CAPACITY_MAH;   // FullCAPRep
  
    __IO uint16_t RepCap=0,RepSOC=0,TTE=0;

STEP_T1:
  // Step T1. Set the Quickstart and Verify bits
  
/*   A second bit must also be set to 1 to test for possible memory leak since the IC also updates this register internally
*/
  Data= max_read_reg(0x2B) ; //Read MiscCFG
  Data |= 0x1400 ; //Set bits 10 and 12
  max_write_reg (0x2B, Data) ; //Write MiscCFG
  
  // Step T2. Verify there are no memory leaks during Quickstart writing
  
/* If the IC was writing the MiscCFG register internally at the same time as host software, bit 12 does not
remain set. Check bit 12 to confirm Quickstart was correctly received by the IC.
*/
  Data= max_read_reg(0x2B) ; //Read MiscCFG
  Data &= 0x1000 ;
  if (Data==0x1000) { //Quickstart Success
      goto STEP_T3;
  }
  else{
    goto STEP_T1;
  }
  
STEP_T3:  
  // Step T3. Clear the Verify bit
/* After a successful Quickstart, the verify bit must now be cleared.
*/
    Data= max_read_reg(0x2B) ; //Read MiscCFG
    Data &= 0xEFFF ; //Clear bit 12
    max_write_reg (0x2B, Data) ; //Write MiscCFG

    // Step T4. Verify there are no memory leaks during Verify bit clearing
    /*If the IC was writing the MiscCFG register internally at the same time as host software, bit 12 does not
clear. Check bit 12 to confirm.
*/
    
    Data= max_read_reg(0x2B) ; //Read MiscCFG
    Data &= 0x1000 ;
    if (Data==0x0000){ 
      goto STEP_T5;     //Verify clear success
      }
    else{
      goto STEP_T3;
    }
      
STEP_T5:
  // Step T5. Delay 500ms
  /* After Quickstart, the MAX17055 requires 500ms to perform signal debouncing and initial calculations.
*/
  _delay_ms(500);
  
  // Step T6. Write and verify FullCAP Register value
  /* The FullCAP register value must be rewritten after a Quickstart event
*/
  max_write_and_verify(0x10, Capacity) ; //WriteAndVerify FullCapRep
  
  // Step T7. Delay 500ms
  /* After updating FullCAP value, the MAX17055 requires 500ms to calculate output register results.
*/
  _delay_ms(500);
  
  // Step T8. Read and Verify Outputs
  /* The RepCap and RepSOC register locations should now contain accurate fuel gauge results based on a
battery voltage of 3.900V. The reported state of charge (RepSOC), as a percent, is read from memory
location 0x06h and the reported capacity (RepCap), in mAHrs, is read from memory location 0x05h.
*/
  
  RepCap = max_read_reg(0x05) ; //Read RepCap
  RepSOC = max_read_reg(0x06) ; //Read RepSOC
  
  /* Fail the unit if RepCap and RepSOC do not report expected results to within ±1% not including power
supply tolerance. Note that any error in the voltage forced on VBATT for testing creates a much larger
error in the output results from the fuel gauge.
*/
  return 0;
}

/*
Each MAX17055 provides a unique serial number ID. To read this serial-number, clear AtRateEn and DPEn in Config2. After the read
operation is completed, the MAX17055 sets Status2.SNReady flag to indicate that serial number read operation is completed.
128-bit serial information read from MAX17055 overwrites Dynamic Power and AtRate output registers. To continue Dynamic Power and
AtRate operations after reading serial number, the host should set Config2.AtRateEn and Config2.DPEn to 1.
*/               
void max_read_serial_number(void){
           
  max_write_reg(0xbb,0x3658&0xcfff);     // clear AtRateEn and DPEn
  
  MAX_SERIAL_BUF[0]=max_read_reg(0xd4);
  MAX_SERIAL_BUF[1]=max_read_reg(0xd5);
  MAX_SERIAL_BUF[2]=max_read_reg(0xd9);
  MAX_SERIAL_BUF[3]=max_read_reg(0xda);
  
  MAX_SERIAL_BUF[4]=max_read_reg(0xdc);
  MAX_SERIAL_BUF[5]=max_read_reg(0xdd);
  MAX_SERIAL_BUF[6]=max_read_reg(0xde);
  MAX_SERIAL_BUF[7]=max_read_reg(0xdf);
  
  max_write_reg(0xbb,0x3658);     // set Config2.AtRateEn and Config2.DPEn to 1
}
#endif

float getSOC() {
  uint16_t SOC_raw = max_read_reg(0x06);//RepSOC
  return SOC_raw * percentage_multiplier;
}

uint16_t getCapacity() {
  uint16_t capacity_raw = max_read_reg(0x18);//DesignCap
  capacity_raw = (uint16_t)(capacity_raw * capacity_multiplier_mAH);
  return capacity_raw;
}

uint16_t getFullCapRep() {              // полная замеренная ёмкость
  uint16_t capacity_raw = max_read_reg(0x10);//FullCapRep
  capacity_raw = (uint16_t)(capacity_raw * capacity_multiplier_mAH);
  return capacity_raw;
}

uint16_t getRemainingCapacity() {       // оставшаяся ёмкость
  uint16_t remainingCapacity_raw = max_read_reg(0x05);//RepCap
  remainingCapacity_raw = (uint16_t)(remainingCapacity_raw * capacity_multiplier_mAH);
  return remainingCapacity_raw;
}

float getInstantaneousCurrent() {
  int16_t current_raw = max_read_reg(0x0a);//Current
  return current_raw * current_multiplier_mV*1000.0f;
}

float getInstantaneousVoltage() {
  uint16_t voltage_raw = max_read_reg(0x09);//VCell
  return voltage_raw * voltage_multiplier_V*1000.0f;
}

float getTimeToEmpty() {
  uint16_t TTE_raw = max_read_reg(0x11);//TimeToEpmty
  return TTE_raw * time_multiplier_Mins;
}

float getTimeToFull() {
  uint16_t TTE_raw = max_read_reg(0x20);//TimeToFull
  return TTE_raw * time_multiplier_Mins;
}

uint16_t getVEmpty() {
  uint16_t VEmpty_raw = max_read_reg(0x3A);// VEmpty
  return (((VEmpty_raw >> 7) & 0x1FF) * 10);
}

float getCycles() {
  static uint16_t old_cycles=0;
  uint16_t cycle_raw = max_read_reg(0x17);//Cycle reg
  if(old_cycles==0){    // при включении первый раз
    old_cycles=cycle_raw;
  }
  
  if((cycle_raw&0x40) != (old_cycles&0x40)){        // сохранение истории каждый раз при смене 6-го бита регистра.
    old_cycles=cycle_raw;
    SaveCalibrationVariables(); 
  }

  return cycle_raw * cycles_multiplier;
}

uint16_t getHealth(){    
  uint16_t raw=0;
  raw=max_read_reg(0x07);//Health reg
  return raw>>8;
}

float getBat_Res_mOhm(){
  uint16_t raw = max_read_reg(0x14);//Res reg
  return raw * res_multiplier_mohm;
}

float get_temperature (void){
  uint16_t t_raw = max_read_reg(MAX17055_TEMP);//0x08
  return t_raw * temperature_multiplier;
}

uint16_t max17055_get_fstat (void){
  return max_read_reg(MAX17055_FSTAT);
}

uint16_t max17055_get_status (void){
  return max_read_reg(MAX17055_STATUS);
}

uint16_t max17055_get_status2 (void){
  return max_read_reg(MAX17055_Status2);
}

void read_parameters_for_save (void){
  
    /*
It is recommended to save the learned capacity parameters every time bit 6 of the Cycles register toggles
(so that it is saved every 64% change in the battery) so that if power is lost the values can easily be
restored
*/
  Saved_RCOMP0 = max_read_reg(0x38) ; //Read RCOMP0
  Saved_TempCo = max_read_reg(0x39) ; //Read TempCo
  Saved_FullCapRep = max_read_reg(0x10) ; //Read FullCapRep
  Saved_Cycles = max_read_reg(0x17) ; //Read Cycles
  Saved_FullCapNom = max_read_reg(0x23) ; //Read FullCapNom
}

  /*    If power is lost, then the capacity information can be easily restored with the following procedure.*/
void restore_saved_parameters (void){
  
  uint16_t FullCapNom=0,MixCap=0,dQAcc=0;
  
  max_write_and_verify(0x38, Saved_RCOMP0) ; //WriteAndVerify RCOMP0
  max_write_and_verify(0x39, Saved_TempCo) ; //WriteAndVerify TempCo
  max_write_and_verify(0x23, Saved_FullCapNom) ; //WriteAndVerify FullCapNom
  
  // 4.7 Wait 350ms
  _delay_ms(350);
  
  // 4.8 Restore FullCap
  FullCapNom= max_read_reg(0x23) ; //Read FullCapNom
  MixCap=(max_read_reg(0x0D)*FullCapNom)/25600 ;
  max_write_and_verify(0x0F, MixCap) ; //WriteAndVerify MixCap
  max_write_and_verify(0x10, Saved_FullCapRep) ; //WriteAndVerify FullCapRep
  //Write dQacc to 200% of Capacity and dPacc to 200%
  dQAcc = (Saved_FullCapNom/ 16) ;
  max_write_and_verify (0x46, 0x0C80) ; //Write and Verify dPacc
  max_write_and_verify (0x45, dQAcc) ; //Write and Verify dQacc
  
  // 4.10 Wait 350ms
   _delay_ms(350);
   
   // 4.11 Restore Cycles register
   max_write_and_verify(0x17, Saved_Cycles) ; //WriteAndVerify Cycles
}

  /*
    The start address of the system memory is 0x1FFF EC00 for STM32F030x4, STM32F030x6 and STM32F030x8 devices,
    and 0x1FFF D800 for STM32F030xC devices.
    The memory organization of STM32F030x4, STM32F030x6, STM32F070x6 and 
    STM32F030x8 devices is based on a main Flash memory block containing up to 64 pages 
    of 1 Kbyte or up to 16 sectors of 4 Kbytes (4 pages).
    STM32F030F4 считывает 32 страницы по 1кБ

  */

uint16_t check_variables (void){
  uint16_t check = 0;

  Set_LastFlashMemoryAddress( LAST_FLASH_MEMORY_ADDRESS );
  if ((ReadCalibrationVaraible(CalibrationDone_Offset) & 0x000000FF) != 1)
  {
    // Нет значений в памяти
  }
  else
  {
    // есть сохраненные значения
  check = 1;
  A2 =  ( ReadCalibrationVaraible(A2_Offset) );
  B2 =  ( ReadCalibrationVaraible(B2_Offset) );
  C2 =  ( ReadCalibrationVaraible(C2_Offset) );
  D2 =  ( ReadCalibrationVaraible(D2_Offset) );
  E2 =  ( ReadCalibrationVaraible(E2_Offset) );
  F2 =  ( ReadCalibrationVaraible(F2_Offset) );
  G2 =  ( ReadCalibrationVaraible(G2_Offset) );
  I2 =  ( ReadCalibrationVaraible(I2_Offset) );
  K2 =  ( ReadCalibrationVaraible(K2_Offset) );
  L2 =  ( ReadCalibrationVaraible(L2_Offset) );


  Saved_FullCapRep=(uint16_t)A2;
  Saved_Cycles=(uint16_t)B2;
  Saved_RCOMP0=(uint16_t)C2;
  Saved_TempCo=(uint16_t)D2;
  Saved_FullCapNom=(uint16_t)E2;
  
  for(uint8_t i=0;i<26;i++){
    A2=ReadCalibrationVaraible((uint8_t)((11+i)*4));
    pressure_buf[0+i*4]=(uint8_t)(A2>>24);
    pressure_buf[1+i*4]=(uint8_t)(A2>>16);
    pressure_buf[2+i*4]=(uint8_t)(A2>>8);
    pressure_buf[3+i*4]=(uint8_t)(A2>>0);
  }
  
  A2=ReadCalibrationVaraible(Offset(37));
  
  p_count=(uint8_t)A2;
  
}
  return check;
}

uint16_t SaveCalibrationVariables(void){
  uint16_t save_check = 0;
  uint32_t Data[40];
  Set_LastFlashMemoryAddress( LAST_FLASH_MEMORY_ADDRESS );
    /********************* FLASH PROGRAMMING FOR SAVING variable **********************/
  TSC_FlashStatus = HAL_OK;
  TSC_MemoryProgramStatus = PASSED;
  FlashFree_Address = CalibrationAddr;
  
  read_parameters_for_save();
    
  Data[0] = 0x01;
  Data[1] = Saved_FullCapRep;
  Data[2] = Saved_Cycles;
  Data[3] = Saved_RCOMP0;
  Data[4] = Saved_TempCo;
  Data[5] = Saved_FullCapNom;

  
  for(uint8_t i=0;i<26;i++){
    Data[11+i] = (uint32_t)pressure_buf[0+i*4]<<24|(uint32_t)pressure_buf[1+i*4]<<16|(uint32_t)pressure_buf[2+i*4]<<8|(uint32_t)pressure_buf[3+i*4];
  }
  
  Data[37] = p_count;

  TSC_FlashStatus = TSC_WriteDataToNVM(FlashFree_Address, Data, sizeof(Data));

  save_check = (*(__IO uint32_t*) FlashFree_Address) & 0x000000FF;

  return save_check;
}

/**
  * @brief  Write Data in data buffer to Flash.
  * @param  FlashFree_Address: Page address
  * @param  Data: pointer to data buffer
  * @param  Size: data buffer size in bytes
  * @retval FLASH programming status
  */
TSC_FLASH_Status TSC_WriteDataToNVM(uint32_t FlashFree_Address, uint32_t *Data, uint32_t Size)
{
  TSC_FLASH_Status TSC_FlashStatus = HAL_OK;

  uint32_t words = (Size/sizeof(uint32_t)) + ((Size%sizeof(uint32_t))?1:0);
  uint32_t index = 0;


  /* Unlock the Flash Program Erase controller */
  TSC_FLASH_Unlock();

  /* Erase Flash sectors ******************************************************/

  /* Clear All pending flags */
  TSC_FLASH_ClearFlag( TSC_FLASH_FLAG_BSY | TSC_FLASH_FLAG_EOP | TSC_FLASH_FLAG_PGERR | TSC_FLASH_FLAG_WRPERR);

  /* Erase Last Flash Page */
  TSC_FlashStatus = TSC_FLASH_ErasePage( FlashFree_Address );

  for(index = 0; index < words; index++)
  {
     /* Writing calibration parameters to the Flash Memory */
    TSC_FlashStatus = TSC_FLASH_ProgramWord( FlashFree_Address, Data[index]);
    /* Increasing Flash Memory Page Address */
    FlashFree_Address = FlashFree_Address + 4;
  }
  return TSC_FlashStatus;
}
/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
HAL_StatusTypeDef TSC_FLASH_Unlock(void)
{
  return HAL_FLASH_Unlock();
}

/**
  * @brief  Clears the FLASH’s pending flags.
  * @param  LASH_FLAG: specifies the FLASH flags to clear.
  *         This parameter can be any combination of the following values:
  *     @arg  TSC_FLASH_FLAG_PGERR: FLASH Program error flag
  *     @arg  TSC_FLASH_FLAG_WRPRTERR: FLASH Write protected error flag
  *     @arg  TSC_FLASH_FLAG_EOP: FLASH End of Operation flag
  * @retval None
  */
void TSC_FLASH_ClearFlag(uint32_t FLASH_FLAG)
{
  __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG);
}

/**
  * @brief  Erases a specified FLASH page.
  * @param  Page_Address: The page address to be erased.
  * @retval None
  */
TSC_FLASH_Status TSC_FLASH_ErasePage(uint32_t Page_Address)
{
  TSC_FLASH_Status TSC_FlashStatus = HAL_OK;
  FLASH_EraseInitTypeDef EraseInitTypeDef;
  uint32_t page_error=0;
  
  EraseInitTypeDef.PageAddress=Page_Address;
  EraseInitTypeDef.NbPages=1;
  EraseInitTypeDef.TypeErase=FLASH_TYPEERASE_PAGES;
  
   TSC_FlashStatus = HAL_FLASHEx_Erase(&EraseInitTypeDef, &page_error);
   return TSC_FlashStatus;
}

/**
  * @brief  Programs a word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed
  * @retval None
  */
TSC_FLASH_Status TSC_FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  return HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address,Data);
}

/**
  * @brief  Set the last Flash Memory address
  * @param  address: Pointer to the penultimate memory page
  * @retval None
  */
void Set_LastFlashMemoryAddress( uint32_t address)
{
  EndAddr = address;

  /* Calculate the address of the Penultimate Flash Memory Page, where the calibration parameters will be saved. */
  CalibrationAddr = (uint32_t)(EndAddr - FLASH_PAGE_SIZE);      // (7с00+400)-400
  // FLASH_BANK1_END       0x08007FFF
}
