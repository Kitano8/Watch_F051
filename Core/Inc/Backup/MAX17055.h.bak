



#define MAX17055_ADRESS         0x6c //0x36<<1
#define MAX17055_DEV_ID         0x4010

#define MAX_CONFIG_EX_TERM             0xA210

#if 0
#define BATT_CAPACITY_MAH       2500
#define CHARGE_TERMINATION_CURRENT     0xa0 // 0.05A на 0,005 Ом
#define BATT_EMPTY_VOLTAGE      275             // resolution 10mV
#define BATT_FULL_VOLTAGE_MV    4200
#define BATT_SHUNT_OHM         0.005f
#endif

uint16_t SaveCalibrationVariables(void);
uint16_t check_variables (void);
typedef HAL_StatusTypeDef      TSC_FLASH_Status;
typedef enum {FAILED = 0, PASSED = !FAILED} TSC_Flash_TestStatus;

uint8_t max17055_check(void);

void max_write_reg (uint8_t reg, uint16_t value);
uint16_t max_read_reg(uint8_t reg);
void max_write_and_verify (uint8_t reg, uint16_t value);
#if 0
void max_17055_init (uint8_t restored_config);
uint16_t testing_max17055 (void);
void max_read_serial_number(void);
#endif
uint8_t max17055Init_CM4 (uint8_t loading, uint16_t reconfig);
float getSOC();
uint16_t getCapacity();
uint16_t getFullCapRep();
uint16_t getRemainingCapacity();
float getInstantaneousCurrent();
float getInstantaneousVoltage();
float getTimeToEmpty();
float getTimeToFull();
uint16_t getVEmpty();
float getCycles();
uint16_t getHealth();
float getBat_Res_mOhm();
float get_temperature (void);

uint16_t max17055_get_fstat (void);
uint16_t max17055_get_status (void);
uint16_t max17055_get_status2 (void);

void read_parameters_for_save (void);
void restore_saved_parameters (void);


/*
Determining Fuel Gauge Accuracy
To determine the true accuracy of a fuel gauge, as experienced by end users, the battery should be exercised in a dynamic manner. The enduser accuracy cannot be completely understood with only simple cycles. To challenge a correction-based fuel gauge, such as a coulomb
counter, test the battery with partial loading sessions. For example, a typical user can operate the device for 10 minutes and then stop use for
an hour or more. A robust test method includes these kinds of sessions many times at various loads, temperatures, and duration. Refer to the
Application Note 4799: Cell Characterization Procedure for a ModelGauge m3/ModelGauge m5 Fuel Gauge.

Initial Accuracy
The IC uses the first voltage reading after power-up or after cell insertion to determine the starting output of the fuel gauge. It is assumed
that the cell is fully relaxed prior to this reading; however, this is not always the case. If there is a load or charge current at this time, the initial
reading is compensated using a default battery resistance of 40mΩ to estimate the relaxed cell voltage. If the cell was recently charged or
discharged, the voltage measured by the IC may not represent the true state of charge of the cell, resulting in initial error in the fuel gauge
outputs. In most cases, this error is minor and is quickly removed by the fuel gauge algorithm during the first hour of normal operation

The IC detects the end of charge when the application current falls into the band set by the IChgTerm and VFSOC is above the FullSOCThr.
By monitoring both the Current and AvgCurrent registers, the device can reject false end-of-charge events such as application load spikes or
early charge-source removal. See Figure 7. When a proper end-of-charge event is detected, the device learns the FullCapRep register based
on the RepCap register. If the old FullCapRep value is too high, it is adjusted on a controlled downward slope near the end of charge as
defined by the MiscCfg.FUS setting until it reaches RepCap. If the old FullCapRep is too low, it is adjusted upward to match RepCap. This
prevents the calculated state of charge from reporting greater than 100%. See Figure 8.
Charge termination is detected by the IC when the following conditions are met:
• VFSOC register > FullSOCThr register
• AND IChgTerm x 0.125 < Current register < IChgTerm x 1.25
• AND IChgTerm x 0.125 < AvgCurrent register < IChgTerm x 1.25

Fuel Gauge Learning
The IC periodically makes internal adjustments to cell characterization and application information to remove initial error and maintain
accuracy as the cell ages. These adjustments always occur as small undercorrections to prevent instability of the learning process and
prevent any noticeable jumps in the fuel-gauge outputs. Learning occurs automatically without any input from the host. In addition to
estimating the state of charge, the IC observes the battery’s relaxation response and adjusts the dynamics of the voltage fuel gauge. Registers
used by the algorithm include:
Application Capacity (FullCapRep Register): This is the total capacity available to the application at full, set through the IChgTerm and
FullSOCThr registers as described in the End-of-Charge Detection section. See the FullCapRep register description.
Cell Capacity (FullCapNom Register): This is the total cell capacity at full, including some capacity that sometimes is not available to the
application due to high loads and/or low temperature. The IC periodically compares percent change based on an open-circuit voltage
measurement vs. coulomb-count change as the cell charges and discharges, maintaining an accurate estimation of the pack capacity in mAh
as the pack ages. See Figure 9.
Voltage Fuel-Gauge Adaptation: The IC observes the battery’s relaxation response and adjusts the dynamics of the voltage fuel gauge. This
adaptation adjusts the RComp0 register during qualified cell relaxation events.
Empty Compensation Adaptation: The IC updates internal data whenever cell empty is detected (VCell < VEmpty) to account for cell age
or other cell deviations from the characterization information.

*/


/* Status register bits */
// Resered                      (1 << 0)
#define STATUS_POR_BIT          (1 << 1)
#define STATUS_IMN_BIT          (1 << 2) // new         Minimum Current Alert Threshold Exceeded
#define STATUS_BST_BIT          (1 << 3)
// Resered                      (1 << 4)
// Resered                      (1 << 5)
#define STATUS_IMX_BIT          (1 << 6) // new         Maximum Current Alert Threshold Exceeded
#define STATUS_dSOCi_BIT        (1 << 7) // new         State of Charge 1% Change Alert. This is set to 1 whenever the RepSOC register
                                                        // crosses an integer percentage boundary such as 50.0%, 51.0%, etc. Must be cleared by host software
#define STATUS_VMN_BIT          (1 << 8)
#define STATUS_TMN_BIT          (1 << 9)
#define STATUS_SMN_BIT          (1 << 10)
#define STATUS_BI_BIT           (1 << 11)
#define STATUS_VMX_BIT          (1 << 12)
#define STATUS_TMX_BIT          (1 << 13)
#define STATUS_SMX_BIT          (1 << 14)
#define STATUS_BR_BIT           (1 << 15)

/* Status2 register bits */
#define STATUS2_Hib_BIT         (1 << 1)        // Hibernate Status. This bit is set to a 1 when the device is in hibernate mode or 0 when the device is in active mode. Hib is set to 0 at power-up.
#define STATUS2_FullDet_BIT     (1 << 5)        // Full detected
#define STATUS2_SNReady_BIT     (1 << 8)        // If SNReady = 1, the unique serial number is available over the I2C. This bit is set to 1 by firmware after serial number is read
                                                // internally and placed into RAM. The serial number overwrites the Dynamic Power and AtRate output registers as described in the Serial Number Feature section
#define STATUS2_DPReady_BIT     (1 << 12)       // If DPReady = 1, Dynamic Power output registers are filled by the firmware and ready to be read by the host.
#define STATUS2_AtRateReady_BIT (1 << 13)       // If AtRateReady = 1, AtRate output registers are filled by the firmware and ready to be read by the host
/* Fstat register bits */
#define FSTAT_DNR_BIT           (1 << 0)        // Data Not Ready
#define FSTAT_RELDT2_BIT        (1 << 6)        // Long Relaxation
#define FSTAT_FQ_BIT            (1 << 7)        // new Full Qualified. This bit is set when all charge termination conditions have been met. See the End-of-Charge Detection section for details
#define FSTAT_EDET_BIT          (1 << 8)        // Empty Detection
#define FSTAT_RELDT_BIT         (1 << 9)        // Relaxed Cell Detection

/* Config register bits */
#define CONFIG_Ber_BIT          (1 << 0)        // Enable alert on battery removal when the IC is mounted on the host side.
                                                // When Ber = 1, a battery-removal condition, as detected by the AIN pin voltage, triggers an alert
#define CONFIG_Bei_BIT          (1 << 1)        // Enable alert on battery insertion when the IC is mounted on the host side.
                                                // When Bei = 1, a battery-insertion condition, as detected by the AIN pin voltage, triggers an alert.
#define CONFIG_Aen_BIT          (1 << 2)        // When Aen = 1, violation of any of the alert threshold register values by temperature, voltage, current, or SOC triggers an alert. 
                                                // This bit affects the ALRT pin operation only. The Smx, Smn, Tmx, Tmn, Vmx, Vmn, Imx, and Imn bits of the Status register (00h) are not disabled
#define CONFIG_FTHRM_BIT        (1 << 3)        // Force Thermistor Bias Switch. This allows the host to control the bias of the thermistor switch or enable fast detection of battery removal.
                                                // Set FTHRM = 1 to always enable the thermistor bias switch. With a standard 10kΩ thermistor, this adds an additional 200μA to the current drain of the circuit.
#define CONFIG_ETHRM_BIT        (1 << 4)        // Enable Thermistor Automatic Bias. Set to logic 1 to enable the automatic THRM output bias and AIN measurement
// Reserved                     (1 << 5)
#define CONFIG_COMMSH_BIT       (1 << 6)        // Communication Shutdown. : Set to logic 1 to force the device to enter shutdown mode if both SDA and SCL are held low for more than timeout
                                                // of the ShdnTimer register. This also configures the device to wake up on a rising edge of any communication.
                                                // Note that if COMMSH and AINSH are both set to 0, the device wakes up on any edge of the SDA or SCL pins. See the Modes of Operation section
#define CONFIG_SHDN_BIT         (1 << 7)        // Shutdown. Write this bit to logic 1 to force a shutdown of the device after timeout of the ShdnTimer register (default 45s delay).
                                                // SHDN is reset to 0 at power-up and upon exiting shutdown mode. To command shutdown within 22.5s, write ShdnTimer = 0x001E
#define CONFIG_Tex_BIT          (1 << 8)        // Temperature External. Set to 1 to allow external temperature measurements to be written to Temp from the host.
                                                // When set to 0, the IC's own measurements as used as selected by Config.TSEL
#define CONFIG_Ten_BIT          (1 << 9)        // Enable Temperature Channel. Set to 1 and set ETHRM or FTHRM to 1 to enable temperature measurements selected by Config.TSel
#define CONFIG_AINSH_BIT        (1 << 10)
#define CONFIG_IS_BIT           (1 << 11)       // Current ALRT Sticky. When IS = 1, current alerts can only be cleared through software, When IS = 0, current alerts are cleared automatically
#define CONFIG_VS_BIT           (1 << 12)       // Voltage ALRT Sticky. When VS = 1, voltage alerts can only be cleared through software. When VS = 0, voltage alerts are cleared automatically
#define CONFIG_TS_BIT           (1 << 13)       // Temperature ALRT Sticky. When TS = 1, temperature alerts can only be cleared through software. When TS = 0, temperature alerts are cleared automatically
#define CONFIG_SS_BIT           (1 << 14)       // SOC ALRT Sticky. When SS = 1, SOC alerts can only be cleared through software. When SS = 0, SOC alerts are cleared automatically   
#define CONFIG_TSel_BIT         (1 << 15)       // Set to 0 to use internal die temperature. Set to 1 to measure temperature using external thermistor. Set ETHRM to 1 when TSel is 1

/* Config2 register bits */
// reserved                     (1 << 0)
#define CONFIG2_CPMode_BIT      (1 << 1)        // Constant-Power Mode. Set to 1 to enable constant-power mode. If it is set to 0, all remaining capacity and remaining time
                                                // calculations are estimated assuming a constant-current load. If it is set to 1, the remaining capacity and remaining time calculations are
                                                // estimated assuming a constant-power load
// reserved                     (1 << 2)
#define CONFIG2_MUST_SET_BIT    (1 << 3)
#define CONFIG2_MUST_SET_BIT2   (1 << 4)
#define CONFIG2_LDMDL_BIT       (1 << 5)        // Host sets this bit to 1 to initiate firmware to finish processing a newly loaded model.
                                                // Firmware clears this bit to zero to indicate that model loading is finished
#define CONFIG2_TAlertEn_BIT    (1 << 6)        // Temperature Alert Enable. Set this bit to 1 to enable temperature based alerts. Write this bit to 0 to disable temperature alerts.
                                                // This bit is set to 1 at power-up
#define CONFIG2_dSOCEn_BIT      (1 << 7)        // SOC Change Alert Enable. Set this bit to 1 to enable alert output with the Status.dSOCi bit function. Write this bit to 0 to disable
                                                // the dSOCi alert output. This bit is set to 0 at power-up
#define CONFIG2_POWR_MASK       0x0f00          // Sets the time constant for the AvgPower register. The default POR value of 0100b gives a time constant of 11.25s. The equation
                                                // setting the period is: AvgPower time constant = 45s x 2 в степени(powr-6)
#define CONFIG2_DPEn_BIT       (1 << 12)        // Dynamic Power Enable. When this bit is set to 0, Dynamic Power calculations are disabled and registers MaxPeakPower/SusPeakPower/MPPCurrent/SPPCurrent are not updated by Dynamic Power calculations
#define CONFIG2_AtRateEn_BIT   (1 << 13)        // AtRate Enable. When this bit is set to 0, AtRate calculations are disabled and registers AtQResidual/AtTTE/AtAvSOC/AtAvCap are not updated by AtRate calculations
// reserved                    (1 << 14)
// reserved                    (1 << 15)

#define 	MAX17055_STATUS		 0x00   // The Status register maintains all flags related to alert thresholds and battery insertion or removal
#define 	MAX17055_VALRT_Th	 0x01   // TheVAlrtTh register shown in Table 12 sets upper and lower limits that generate an alert if exceeded by the VCell register value
#define 	MAX17055_TALRT_Th	 0x02   // The TAlrtTh register (Table 13) sets upper and lower limits that generate an alert if exceeded by the Temp register value
#define 	MAX17055_SALRT_Th	 0x03   // The SAlrtTh register shown (Table 14) sets upper and lower limits that generate an alert if exceeded by RepSOC
#define 	MAX17055_AtRate		 0x04   // Host software should write the AtRate register with a negative two’s-complement 16-bit value of a theoretical load current prior to reading
                                                // any of the at-rate output registers (AtTTE, AtAvSOC, AtAvCap).
#define 	MAX17055_RepCap		 0x05   // RepCap or reported remaining capacity in mAh. This register is protected from making sudden jumps during load changes.
#define 	MAX17055_RepSOC		 0x06   // RepSOC is the reported state-of-charge percentage output for use by the application GUI.
#define 	MAX17055_Age		 0x07   // The Age register contains a calculated percentage value of the application’s present cell capacity compared to its original design capacity.
                                                // The result can be used by the host to gauge the battery pack health as compared to a new pack of the same type.
                                                // The equation for the register output is: Age Register(%) = 100%x(FullCapRep register/DesignCap register)
#define 	MAX17055_TEMP		 0x08   // The Temp register provides the temperature measured by the thermistor or die temperature. The Temp register is the input to the fuel gauge algorithm
#define 	MAX17055_VCELL		 0x09   // VCell reports the voltage measured between BATT and CSP
#define 	MAX17055_Current	 0x0A   // The IC measures the voltage between the CSP and CSN pins and the result is stored as a two’s complement value in the Current register
#define 	MAX17055_AvgCurrent	 0x0B   // The AvgCurrent register reports an average of Current register readings
#define 	MAX17055_QResidual	 0x0С   // new  // The QResidual register provides the calculated amount of charge in mAh that is presently inside of, but cannot be removed from the cell
                                                // under present application conditions (load and temperature). This value is subtracted from the MixCap value to determine capacity available
                                                // to the user under present conditions (AvCap)
#define 	MAX17055_MixSOC	         0x0D   // The MixSOC register hold the calculated percentage of the cell before any empty compensation adjustments are performed
#define 	MAX17055_AvSOC		 0x0E   // The AvCap register hold the calculated available percentage of the battery based on all inputs from the ModelGauge m5 algorithm including empty compensation.
                                                // These register provide unfiltered result. Jumps in the reported values can be caused by abrupt changes in load current or temperature
#define 	MAX17055_MixCap		 0x0F   // The MixCap register hold the calculated remaining capacity of the cell before any empty compensation adjustments are performed
#define 	MAX17055_FullCAPRep	 0x10   // This register reports the full capacity that goes with RepCap, generally used for reporting to the GUI. Most applications should only monitor
                                                // FullCapRep, instead of FullCap or FullCapNom. A new full-capacity value is calculated at the end of every charge cycle in the application
#define 	MAX17055_TTE		 0x11   // The TTE register holds the estimated time to empty for the application under present temperature and load conditions.
                                                // The TTE value is determined by relating AvCap with AvgCurrent. The corresponding AvgCurrent filtering gives a delay in TTE, but provides more stable results
#define 	MAX17055_QRTbl00	 0x12   // contain characterization information regarding cell capacity under different application conditions
#define 	MAX17055_FullSOCThr	 0x13   // The FullSOCThr register gates detection of end-of-charge. VFSOC must be larger than the FullSOCThr value before IChgTerm is compared to
                                                // the AvgCurrent register value. The recommended FullSOCThr register setting for most custom characterized applications is 95% (default, 0x5F05)
#define 	MAX17055_RCELL		 0x14   // Initial Value: 0x0290 (160mΩ). The RCell register provides the calculated internal resistance of the cell.
                                                // RCell is determined by comparing open-circuit voltage (VFOCV) against measured voltage (VCell) over a long time period while under load or charge current
// Reserved                              0x15
#define         MAX17055_AvgTA		 0x16   // The AvgTA register reports an average of the readings from the Temp register
#define 	MAX17055_Cycles		 0x17   // The Cycles register maintains a total count of the number of charge/discharge cycles of the cell that have occurred.
                                                // The result is stored as a percentage of a full cycle. For example, a full charge/discharge cycle results in the Cycles register incrementing by 100%.
                                                // The Cycles register accumulates fractional or whole cycles. For example, if a battery is cycles 10% x 10 times, then it tracks 100% of a cycle.
                                                // The Cycles register has a full range of 0 to 655.35 cycles with a 1% LSb
#define 	MAX17055_DesignCap	 0x18   // The DesignCap register holds the expected capacity of the cell. This value is used to determine age and health of the cell by comparing
                                                // against the measured present cell capacity
#define 	MAX17055_AvgVCELL	 0x19   // The AvgVCell register reports an average of the VCell register readings
#define 	MAX17055_MinMaxTemp	 0x1A   // The MaxMinTemp register maintains the maximum and minimum Temp register (08h) values since the last fuelgauge reset or until cleared by host software
#define 	MAX17055_MinMaxVolt	 0x1B   // The MaxMinVolt register maintains the maximum and minimum of VCell register values since device reset
#define 	MAX17055_MinMaxCurr	 0x1C   // The MaxMinCurr register maintains the maximum and minimum Current register values since the last IC reset or until cleared by host software
#define 	MAX17055_CONFIG		 0x1D   // The Config register holds all shutdown enable, alert enable, and temperature enable control bits. Writing a bit location enables the
                                                // corresponding function within 175ms in active mode and 5.625s in hibernate mode
#define 	MAX17055_ICHGTerm	 0x1E   // The IChgTerm register allows the device to detect when a charge cycle of the cell has completed.
                                                // IChgTerm should be programmed to the exact charge termination current used in the application
#define 	MAX17055_AvCap		 0x1F   // The AvCap register hold the calculated available capacity of the battery based on all inputs from the ModelGauge m5 algorithm including empty compensation.
                                                // These register provide unfiltered result. Jumps in the reported values can be caused by abrupt changes in load current or temperature
#define 	MAX17055_TTF	         0x20   // changed      The TTF register holds the estimated time to full for the application under present conditions.
                                                // The TTF value is determined by learning the constant current and constant voltage portions of the charge cycle based on experience of prior charge cycles.
                                                // Time to full is then estimated by comparing present charge current to the charge termination current.
                                                // Operation of the TTF register assumes all charge profiles are consistent in the application
#define 	MAX17055_DevName	 0x21   // The DevName register holds revision information. The initial silicon is DevName = 0x4010
#define 	MAX17055_QRTbl10	 0x22   // contain characterization information regarding cell capacity under different application conditions
#define 	MAX17055_FullCAPNom	 0x23   // This register holds the calculated full capacity of the cell, not including temperature and empty compensation.
                                                // A new full-capacity nominal value is calculated each time a cell relaxation event is detected.
                                                // This register is used to calculate other outputs of the ModelGauge m5 algorithm
// Reserved	                         0x24   // changed
// Reserved                     	 0x25   // changed
// Reserved                     	 0x26   // changed
#define 	MAX17055_AIN		 0x27   // External temperature measurement on the AIN pin is compared to the THRM pin voltage. The MAX17055 stores the result as a ratio-metric
                                                // value from 0% to 100% in the AIN register with an LSB of 0.0122%.
                                                // The TGain, TOff, and Curve register values are then applied to this ratiometric reading to convert the result to temperature
#define 	MAX17055_LearnCFG	 0x28   // Initial Value: 0x4486. The LearnCfg register controls all functions relating to adaptation during operation
#define 	MAX17055_FilterCFG	 0x29   // Initial Value: 0xCEA4. The FilterCfg register sets the averaging time period for all A/D readings, for mixing OCV results and coulomb count results.
                                                // It is recommended that these values are not changed, unless absolutely required by the application. 
#define 	MAX17055_RelaxCFG	 0x2A   // The RelaxCfg register defines how the IC detects if the cell is in a relaxed state with a low dV/dt.
                                                // If AvgCurrent remains below the LOAD threshold while AvgVCell changes less than the dV threshold over two consecutive periods of dt, the
                                                // cell is considered relaxed
#define 	MAX17055_MiscCFG	 0x2B   // Initial Value: 0x3870. The MiscCfg control register enables various other functions of the device.
                                                // The MiscCfg register default values should not be changed unless specifically required by the application
#define 	MAX17055_TGAIN		 0x2C   // The TGain, TOff, and Curve registers are used to calculate temperature from the measurement of the AIN pin with an accuracy of ±3°C over
                                                // a range of -40°C to +85°C. Table 6 lists the recommended TGain, TOff, and Curve register values for common NTC thermistors
#define 	MAX17055_TOFF		 0x2D   // см.0x2C
#define 	MAX17055_CGAIN		 0x2E   // The CGain and COff registers adjust the gain and offset of the current measurement result. The current measurement A/D is factory
                                                // trimmed to data-sheet accuracy without the need for the user to make further adjustments. The default power-up settings for CGain and
                                                // COff apply no adjustments to the Current register reading. For specific application requirements, the CGain and COff registers can be used to
                                                // adjust readings as follows: Current register = Current A/D reading × (CGain register/0400h) + COff register
#define 	MAX17055_COFF		 0x2F   // см. 0x2E
// Reserved                              0x30
// Reserved                              0x31
#define 	MAX17055_QRTbl20	 0x32   // contain characterization information regarding cell capacity under different application conditions
// Reserved                     	 0x33   // changed
#define 	MAX17055_DieTemp	 0x34   // changed      The DieTemp register provides the internal die temperature measurement
#define 	MAX17055_FullCAP	 0x35   // FullCap is the full discharge capacity compensated according to the present conditions.
                                                // A new full-capacity value is calculated continuously as application conditions change (temperature and load)
// Reserved                     	 0x36   // changed
// Reserved                   		 0x37   // changed
#define 	MAX17055_RCOMP0		 0x38   // The RComp0 register holds characterization information critical to computing the open-circuit voltage of a cell under loaded conditions
#define 	MAX17055_TempCo		 0x39   // The TempCo register holds temperature compensation information for the RComp0 register value.
#define 	MAX17055_V_empty	 0x3A   // The VEmpty register sets thresholds related to empty detection during operation
// Reserved                     	 0x3B   // changed
// Reserved                     	 0x3C   // changed
#define 	MAX17055_FSTAT		 0x3D   // The FStat register is a read-only register that monitors the status of the ModelGauge m5 algorithm
#define 	MAX17055_Timer		 0x3E   // new          // TimerH and Timer provide a long-duration time count since last POR.
                                                                // 3.2 hour LSb gives a full scale range for the register of up to 23.94 years.
                                                                // The Timer register LSb is 175.8ms giving a full-scale range of 0 to 3.2 hours. TimerH and Timer can be interpreted together as a 32-bit timer
#define 	MAX17055_SHDNTIMER	 0x3F                   // The ShdnTimer register sets the timeout period from when a shutdown event is detected until the device disables the regulators and enters low-power mode
#define 	MAX17055_UserMem1        0x40   // new
// Reserved                     	 0x41
#define 	MAX17055_QRTbl30	 0x42   // contain characterization information regarding cell capacity under different application conditions
#define 	MAX17055_RGain  	 0x43   // new          // The RGain register sets the value of RGain1 and RGain2 during DBPT register calculation. Initial Value: 0x8080
// Reserved                     	 0x44
#define         MAX17055_dQacc		 0x45   // This register tracks change in battery charge between relaxation points. It is available to the user for debug purposes
#define         MAX17055_dPacc		 0x46   // This register tracks change in battery state of charge between relaxation points. It is available to the user for debug purposes
// Reserved                     	 0x47
// Reserved             		 0x48   // changed
#define 	MAX17055_ConvgCfg  	 0x49   // new          // The ConvgCfg register configures operation of the converge-to-empty feature. The default and recommended value for ConvgCfg is 0x2241
#define 	MAX17055_VFRemCap  	 0x4a   // new          // The VFRemCap register holds the remaining capacity of the cell as determined by the voltage fuel gauge before any empty compensation adjustments are performed.
// Reserved             		 0x4b
// Reserved             		 0x4c
#define 	MAX17055_QH		 0x4D   // The QH register displays the raw coulomb count generated by the device. This register is used internally as an input to the mixing algorithm.
                                                // Monitoring changes in QH over time can be useful for debugging device operation.
// Reserved             		 0x4E   // changed
// Reserved             		 0x4F 

#define         MAX17055_Soft_Wakeup     0x60
#define         MAX17055_WakeUp_command  0x90
#define         MAX17055_ClearWkUp_command  0x00

#define 	MAX17055_OCVTable0       0x80   // Cell characterization information
// ...
#define 	MAX17055_OCVTable15      0x8F   // Cell characterization information

#define 	MAX17055_XTable0         0x90   // Cell characterization information
// ...
#define 	MAX17055_XTable15        0x9F   // Cell characterization information

// The model is located between memory locations 0x80h and 0xAFh

#define 	MAX17055_Status2	 0xB0   // new          // The Status2 register maintains status of various firmware functions
#define 	MAX17055_Power  	 0xB1   // new          // Instant power calculation from immediate current and voltage. The LSB is 0.8mW with a 10mΩ sense resistor
#define 	MAX17055_ID_UserMem2     0xB2   // new
#define 	MAX17055_ID_AvgPower     0xB3   // new          // Filtered average power from the power register. LSB is 0.8mW with a 10mΩ sense resistor.
#define 	MAX17055_ID_IAlertTh     0xB4   // new          // The IAlrtTh register (Table 15) sets upper and lower limits that generate an alert if exceeded by the Current register value
// Reserved             		 0xB5   // new
#define 	MAX17055_CVMixCap        0xB6   // new
#define 	MAX17055_CVHalfTime      0xB7   // new
#define 	MAX17055_CGTempCo        0xB8   // new          // If CGTempCo is nonzero then CGTempCo is used to adjust current measurements for temperature. CGTempCo has a range of 0% to
                                                // 3.1224% per °C with a step size of 3.1224/0x10000 percent per °C. If a copper trace is used to measure battery current, CGTempCo should
                                                // be written to 0x20C8 or 0.4% per °C, which is the approximate temperature coefficient of a copper trace
#define 	MAX17055_Curve           0xB9   // new          // The upper half of the Curve register applies curvature correction current measurements made by the IC when using a copper trace as the sense resistor
                                                // The lower half of the Curve register applies thermistor measurement curvature correction to allow thermistor measurements to be accurate over a wider temperature range.
                                                // A ±3°C accuracy can be achieved over a -40°C to 85°C operating range
#define 	MAX17055_HibCfg          0xBA   // new          // The HibCfg register controls hibernate mode functionality.
                                                                // The MAX17055 enters and exits hibernate when the battery current is less than about C/100.
                                                                //  While in hibernate mode the MAX17055 reduces its operating current to 7µA by reducing ADC sampling to once every 5.625s
#define 	MAX17055_Config2         0xBB   // new          // см. 0x1D
#define 	MAX17055_VRipple         0xBC   // new          // The VRipple register holds the slow average RMS ripple value of VCell register reading variation compared to the AvgVCell register.
                                                                // The default filter time is 22.5 seconds. See RippleCfg register description. VRipple has an LSb weight of 1.25mV/128.
#define 	MAX17055_RippleCfg       0xBD   // new          // The RippleCfg register configures ripple measurement and ripple compensation. The default and recommended value for this register is 0x0204
#define 	MAX17055_TimerH          0xBE   // new          // см. 0x3E
// Reserved                              0xBF   // new

#define 	MAX17055_RSense_UserMem3 0xD0   // new
#define 	MAX17055_SCOcvLim        0xD1   // new          // This register only has usage when ModelCfg.ModelID is selected as 6 (LiFePO4)
//Reserved                               0xD2   // new
#define 	MAX17055_SOCHold         0xD3   // new          // The SOCHold register configures operation of the hold before empty feature and also the enable bit for 99% hold during charge.
                                                                // The default value for SOCHold is 0x1002
#define 	MAX17055_MaxPeakPower    0xD4   // new          // The MAX17055 estimates the maximum instantaneous peak output power of the battery pack in mW, which the battery can support for up
                                                                // to 10ms, given the external resistance and required minimum voltage of the voltage regulator.
                                                                // The MaxPeakPower() value is negative (discharge) and updates every 175ms. LSB is 0.8mW. MaxPeakPower = MPPCurrent() x AvgVCell
#define 	MAX17055_SusPeakPower    0xD5   // new          // The fuel gauge estimates the sustainable peak output power of the battery pack in mW, which the battery supports for up to 10s, given the
                                                                // external resistance and required minimum voltage of the voltage regulator.
                                                                // The SusPeakPower() value is negative and updated each 175ms. LSB is 0.8mW. SusPeakPower = SPPCurrent() x AvgVCell
#define 	MAX17055_PackResistance  0xD6   // new          // When the MAX17055 is installed host-side, simply set PackResistance to zero, since the MAX17055 can observe the total resistance between it and the battery.
                                                                // However, when the MAX17055 is installed pack-side, configure PackResistance according to the total non-cell pack resistance.
                                                                // This should account for all resistances due to cell interconnect, sense resistor, FET, fuse, connector, and other resistance between the cells and output of
                                                                // the battery pack. The cell internal resistance should not be included and is estimated by the MAX17055.
                                                                // 0x1000 is 1000mΩ, which results in an LSB of 0.244140625mΩ per LSB
#define 	MAX17055_SysResistance   0xD7   // new          // Set SysResistance according to the total system resistance. This should include any connector and PCB trace between the MAX17055 and
                                                                // the system at risk for dropout when the voltage falls below MinSysVolt.
                                                                // SysResistance() is initialized to a default value upon removal or insertion of a battery pack. Writes with this function overwrite the default value.
                                                                // 0x1000 is 1000mΩ, which results in an LSB of 0.244140625mΩ per LSB.
#define 	MAX17055_MinSysVoltage   0xD8   // new          // Set MinSysVoltage according to the minimum operating voltage of the system. This is generally associated with a regulator dropout or other
                                                                // system failure/shutdown. The system should still operate normally until this voltage. MinSysVoltage() is initialized to the default value (3.0V).
                                                                // Writes with this function overwrite the default value
#define 	MAX17055_MPPCurrent      0xD9   // new          // The MAX17055 estimates the maximum instantaneous peak current of the battery pack in mA, which the battery can support for up to 10ms,
                                                                // given the external resistance and required minimum voltage of the voltage regulator. The MPPCurrent() value is negative and updates every 175ms
#define 	MAX17055_SPPCurrent      0xDA   // new          // The MAX17055 estimates the sustained peak current of the battery pack in mA, which the battery can support for up to 10s, given the
                                                                // external resistance and required minimum voltage of the voltage regulator. The SPPCurrent() value is negative and updates every 175ms.
#define 	MAX17055_ModelCfg        0xDB   // new          // The ModelCFG register controls basic options of the EZ algorithm
#define 	MAX17055_AtQResidual     0xDC   // new          // The AtQResidual register provides the calculated amount of charge in mAh that is presently inside of, but cannot be removed from the cell
                                                                // under present temperature and hypothetical load (AtRate). This value is subtracted from the MixCap value to determine capacity available
                                                                // to the user (AtAvCap). See the Predicting Run Time for a Hypothetical Load section for more explanation
#define 	MAX17055_AtTTE           0xDD   // new          // The AtTTE register can be used to estimate time to empty for any theoretical load entered into the AtRate register
#define 	MAX17055_AtAvSOC         0xDE   // new          // The AtAvSOC register holds the theoretical state of charge of the cell based on the theoretical load of the AtRate register. The register value
                                                                // is stored as a percentage with a resolution of 1/256 % per LSB. The high byte indicates 1% resolution
#define 	MAX17055_AtAvCap         0xDF   // new          // The AtAvCap register holds the estimated remaining capacity of the cell based on the theoretical load current value of the AtRate register.
                                                                // The value is stored in terms of µVh and must be divided by the application sense-resistor value to determine the remaining capacity in mAh
#define         MAX17055_VFOCV           0xFB   // The VFOCV register contains the calculated open-circuit voltage of the cell as determined by the voltage fuel gauge.
                                                // This value is used in other internal calculations.
#define         MAX17055_VFSOC           0xFF   // The VFSOC register holds the calculated present state of charge of the battery according to the voltage fuel gauge