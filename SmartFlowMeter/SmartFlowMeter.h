struct menuLevel {
  char name[16];
  struct menuLevel *next;
  struct menuLevel *prev;
  struct menuLevel *down;
  struct menuLevel *up;
  void (*DoWork)(void);
};

//struct ADCCONFIG {
//
//  void (adcInterrupt) (void);
//  //void (adc
//};
#define dacChipSelect 28
MCP492X DAC(dacChipSelect);// (MOSI,SCK,CS,LDAC)

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 64, &Wire);
#define OLED_RESET 7
////button definitions/////

// OLED FeatherWing buttons map to different pins depending on board:
#define BUTTON_UP 31
#define BUTTON_MENU 30
#define BUTTON_SELECT 27
#define BUTTON_DOWN 11


SoftwareTimer btnDownTime;
volatile byte buttonState = 0;

//define menu structs
struct menuLevel blankM, displayM, configM, editM, coeffM, addM, subtractM, removeM, saveM, *currentM, *previousM;


//SD CARD CONFIG FILE
#define FILE_NAME_CONFIG   "CONFIG.TXT"
#define FILE_NAME_CONFIG_DATA "CONFIG"
#define FILE_NAME_LOG "LOG"
String  fileNameLogFile = String(FILE_NAME_LOG) + String("_0.txt");

File dataFile;
#define TEST_STRING "SD card CONFIG FILE."

//Internal CONFIG FILE
#define FLASHFILENAME    "/CONFIG"

//will be removed for final version
uint32_t vbat_pin = PIN_VBAT;             // A7 for feather nRF52832, A6 for nRF52840
//#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define ADC_MV_PER_LSB    (0.054931640625F) // 3.6V ADC range and 14-bit ADC resolution = 3600mV/65536
#define ADC_MA_PER_LSB    (0.201416015625F) // 3.6V ADC range and 14-bit ADC resolution = (3600mV/65536)

//sd card chip select CS
const int chipSelect = 29;

//sd card read buffer
String dataString;

// ADC Pins
#define ADC0_PIN   PIN_A0
#define ADC1_PIN   PIN_A1
#define ADC2_PIN   PIN_A2
#define ADC3_PIN   PIN_A3

void (*PIN_A0_Function)(void) = 0;
void (*PIN_A1_Function)(void) = 0;
void (*PIN_A2_Function)(void) = 0;
void (*PIN_A3_Function)(void) = 0;

//coefficients and in memory values
#define COEFF_NUM  9
#define LIMIT_NUM 4
#define UTILITY_NUM 77
/*Utility variables for system calibration
   U0 0-10v upper limit, was freq_limit_upper (not really a limit but a nonlinear changeover to another equation set)
   U1 0-10v lower limit, was power_limit_lower
   U2 4-20mA upper limit was, 4-20mA valve_limit_upper
   U3 4-20mA lower limit,  was valve_limit_lower
   U4 C coefficient number
   U5 M coefficient number
   U6 D coefficient number
   U7 Config File Version Number
   U8 Sample rate in mS, default 500
   U9 Which algorithm to run Initialize calibartion (0,calibrate | 1 //fresh_air_fan();2  //valve_flow_rate();3  //pump_fan_flow_rate(); //4 fan flow) not stored in system, read from sd card only
   U10 calibration coeff, ADC0_PIN
   Ull calibration coeff, ADC1_PIN
   U12 calibration coeff, ADC2_PIN
   U13 calibration coeff, ADC3_PIN
   U14 calibration coeff, 0-10V out, was pump_fan_flow_rate_result
   U15 calibration coeff, 4-20mA out, was valve_flowRate_result
   U16 calibartion coeff offset, ADC0_PIN
   U17 calibartion coeff offset, ADC1_PIN
   U18 calibartion coeff offset, ADC2_PIN
   U19 calibartion coeff offset, ADC3_PIN
   U20 calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result
   U21 calibartion coeff offset, 4-20mA out, was valve_flowRate_result
   U22 ADC0_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was Valve PIN (all input will be normalized according to it's range) /////////////////////
   U23 ADC1_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in , was ADC1_PIN
   U24 ADC2_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was ADC3_PIN
   U25 ADC3_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was ADC2_PINnot used
   U26 ADC_Samples_Per_Second
   U27 ADC_Settling_Percent //float= 0.06;
   U28 DAC0_PIN 0:Not used, 1:C Output, 2:M Output, 3:D Output, 4:R Output
   U29 DAC1_PIN 0:Not used, 1:C Output, 2:M Output, 3:D Output, 4:R Output
   U30 discontinuity  R, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U31 discontinuity  C, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U32 discontinuity  M, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U33 discontinuity  D, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
   U34 discontinuity value R, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U35 discontinuity value C, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U36 discontinuity value M, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U37 discontinuity value D, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
   U38 ADC0 normalized Signal conversion to physical values offset, only used for display
   U39 ADC1 normalized Signal conversion to physical values offset, only used for display
   U40 ADC2 normalized Signal conversion to physical values offset, only used for display
   U41 ADC3 normalized Signal conversion to physical values offset, only used for display
   U42 DAC0 normalized calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result
   U43 DAC1 normalized calibration coeff 4-20mA, valve_flowRate_result, only used for display
   U44 ADC0 normalized Signal conversion to physical values multiplier, only used for display
   U45 ADC1 normalized Signal conversion to physical values multiplier, only used for display
   U46 ADC2 normalized Signal conversion to physical values multiplier, only used for display
   U47 ADC3 normalized Signal conversion to physical values multiplier, only used for display
   U48 DAC0 normalized calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result, only used for display
   U49 DAC1 normalized calibration coeff 4-20mA, valve_flowRate_result, only used for display
   U50 WHpP watt hour per pulse
   U51  Qmax normalization factor
   U52 DPSensorRange
   U53 FrequencyRange
   U54 ADC0 input label/units
   U55 ADC1 input label/units
   U56 ADC2 input label/units
   U57 ADC3 input label/units
   U58 DAC0 input label/units
   U59 DAC1 input label/units
*/


//const float32_t variableName[] PROGMEM = {data0, data1, data3…​}
//roughly 1590 bits, ( COEFF_NUM * 3+LIMIT_NUM+UTILITY_NUM )*32bits, could be compressed perhapse
struct coefficients {
  float32_t r_list[COEFF_NUM];
  float32_t c_list[COEFF_NUM];
  float32_t m_list[COEFF_NUM];
  float32_t d_list[COEFF_NUM];
  float32_t limits[LIMIT_NUM];
  float32_t utility_variables[UTILITY_NUM];
};

//global config storage, gets rebuilt based on SD card CONFIG.TXT, manual data entry, or internal storage
coefficients coeff_buff;
// disables normal output routine, and sets outputs to known configuration so coefficients can be adjusted
bool calibrateEnabled = 0;

float32_t calibration_value = 0;

//adc value temp ADC0-3 + DAC0-1
volatile uint32_t  adc_value_temp[4];
volatile uint32_t  dac_value_temp[2];
volatile uint32_t  adc_value_buffer[4];
volatile uint32_t  adc_value_buffer2[4];
volatile float32_t dac_value_buffer[4];

// R C M D Vout Iout labels
typedef union 
{
  char* list[12];

  struct {
     char * ADC0_unit_label ;
     char * ADC1_unit_label ;
     char * ADC2_unit_label ;
     char * ADC3_unit_label ;
     char * Vout_label ;
     char * Iout_label ;
     char * R_unit_label ;
     char * C_unit_label ;
     char * M_unit_label ;
     char * D_unit_label ;
     char * V_unit_label ;
     char * I_unit_label ;
     int count;
  } var; 
}unit_list_var;

unit_list_var unit_label;
volatile float32_t adc_value_calibrated[4];
volatile float32_t adc_value_normalized[4];
volatile float32_t dac_value_calibrated[2];
//volatile float32_t dac_value_normalized[2];
volatile float32_t * dac_value_normalized_V;
volatile float32_t * dac_value_normalized_I;
volatile float32_t * R_adc_value_normalized;
volatile float32_t * C_adc_value_normalized;
volatile float32_t * M_adc_value_normalized;
volatile float32_t * D_adc_value_normalized;
volatile float32_t * R_dac_value_results = &(dac_value_buffer[0]);
volatile float32_t * C_dac_value_results = &(dac_value_buffer[1]);
volatile float32_t * M_dac_value_results = &(dac_value_buffer[2]);
volatile float32_t * D_dac_value_results = &(dac_value_buffer[3]);
//number of elements in coefficient list up to maximum COEFF_NUM
uint32_t c_num = 4;
uint32_t m_num = 4;
uint32_t d_num = 4;
uint32_t r_num = 4;



const uint32_t ad_buff = 1;
#define BLOCK_SIZE 8
#define NUM_TAPS 32
#define N 8
#define TEST_LENGTH_SAMPLES 1536

#define NUMFRAMES (TEST_LENGTH_SAMPLES / BLOCK_SIZE)

#define BUFSIZE N

float32_t* high_c_list = &coeff_buff.m_list[m_num];
float32_t* low_r_list = &coeff_buff.d_list[d_num];
float32_t* high_m_list = &coeff_buff.m_list[m_num];
float32_t* low_d_list = &coeff_buff.d_list[d_num];
//temp global power series list
float32_t R_power_series_list[COEFF_NUM];
float32_t C_power_series_list[COEFF_NUM];
float32_t M_power_series_list[COEFF_NUM];
float32_t D_power_series_list[COEFF_NUM];


float32_t calibration_buff_X[BUFSIZE];
float32_t calibration_buff_Y[BUFSIZE];

//intermediary result
volatile float32_t powerS = 0;
volatile float32_t powerR = 0;
//float32_t WHpP = 0;
//Valve position input (0-10v signal)
volatile float32_t flowRate_result = 0;
//q15_t  vTST_buff[BUFSIZE];


//dot product results
float32_t dotResult = 0;
//sqrt results
float32_t sqrtResult = 0;
//eff_m
float32_t eff_m = 0;
//eff_dev
float32_t eff_dev = 0;


//valve upper limit
float32_t* VOUT_calib_limit_upper = &(coeff_buff.limits[0]);
//valve lower limit
float32_t* VOUT_calib_limit_lower = &(coeff_buff.limits[1]);
//motor frequency upper limit
float32_t* IOUT_calib_limit_upper = &(coeff_buff.limits[2]);
//motor power lower limit
float32_t* IOUT_calib_limit_lower = &(coeff_buff.limits[3]);
float32_t* algorithmEnable = &(coeff_buff.utility_variables[9]);
//calib offset array pointer
float32_t* adc_calib = &(coeff_buff.utility_variables[10]);
// * U10 calibration coeff, vTST
float32_t* adc0_calib = &(coeff_buff.utility_variables[10]);
// * Ull calibration coeff, deltaP
float32_t* adc1_calib = &coeff_buff.utility_variables[11];
// * U12 calibration coeff, power
float32_t* adc2_calib = &coeff_buff.utility_variables[12];
// * U13 calibration coeff, freq
float32_t* adc3_calib = &coeff_buff.utility_variables[13];
// * U14 calibration coeff, pump_fan_flow_rate_result
float32_t* VOUT_calib = &coeff_buff.utility_variables[14];
// * U15 calibration coeff, valve_flowRate_result
float32_t* IOUT_calib = &coeff_buff.utility_variables[15];
//calib array pointer
float32_t* adc_calib_offset = &(coeff_buff.utility_variables[16]);
// * U16 calibration coeff, vTST
float32_t* adc0_calib_offset = &coeff_buff.utility_variables[16];
// * Ul7 calibration coeff, deltaP
float32_t* adc1_calib_offset = &coeff_buff.utility_variables[17];
// * U18 calibration coeff, power
float32_t* adc2_calib_offset = &coeff_buff.utility_variables[18];
// * U19 calibration coeff, freq
float32_t* adc3_calib_offset = &coeff_buff.utility_variables[19];
// * U20 calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result
float32_t* VOUT_calib_offset = &coeff_buff.utility_variables[20];
// * U21 calibration coeff, valve_flowRate_result
float32_t* IOUT_calib_offset = &coeff_buff.utility_variables[21];
// adc_config array list variable
float32_t* adc_config = &coeff_buff.utility_variables[22];
//  U22 ADC0_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was Valve PIN
float32_t* ADC0_config = &coeff_buff.utility_variables[22];
//  U23 ADC1_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in , was ADC1_PIN
float32_t* ADC1_config = &coeff_buff.utility_variables[23];
// U24 ADC2_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was ADC3_PIN
float32_t* ADC2_config = &coeff_buff.utility_variables[24];
// U25 ADC3_PIN 0: disable, 1: 0-10V in, 2: 4-20mA in, 3: Pulse in, was ADC2_PINnot used
float32_t* ADC3_config = &coeff_buff.utility_variables[25];
//   U26 ADC_Samples_Per_Second,

//   U27 ADC_Settling_Percent
float32_t* adWeight = &coeff_buff.utility_variables[27];
//   U28 DAC0_PIN 0:Not used, 1:C Output, 2:M Output, 3:D Output, 4:R Output
float32_t* DAC0_output_variable = &coeff_buff.utility_variables[28];
//   U29 DAC1_PIN 0:Not used, 1:C Output, 2:M Output, 3:D Output, 4:R Output
float32_t* DAC1_output_variable = &coeff_buff.utility_variables[29];
//   U30 discontinuity  R, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
float32_t* discont_R = &coeff_buff.utility_variables[30];
//   U31 discontinuity  C, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
float32_t* discont_C = &coeff_buff.utility_variables[31];
//   U32 discontinuity  M, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
float32_t* discont_M = &coeff_buff.utility_variables[32];
//   U33 discontinuity  D, 0:Not used, 1: applied to ADC0_PIN, 2: applied to ADC1_PIN, 3: applied to ADC2_PIN, 4: applied to ADC3_PIN
float32_t* discont_D = &coeff_buff.utility_variables[33];
//   U34 discontinuity value R, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
float32_t* discont_value_R = &coeff_buff.utility_variables[34];
//   U35 discontinuity value C, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
float32_t* discont_value_C = &coeff_buff.utility_variables[35];
//   U36 discontinuity value M, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
float32_t* discont_value_M = &coeff_buff.utility_variables[36];
//   U37 discontinuity value D, 0: Not used, anything else, Coeff list lower applied to input < discontinuity value and doeff list upper applied to input >= discontinuity value
float32_t* discont_value_D = &coeff_buff.utility_variables[37];
//   U38 ADC0 Signal conversion to physical values offset, only used for display
//float32_t* adc_disp_offset = &coeff_buff.utility_variables[38];
float32_t* R_disp_offset = &coeff_buff.utility_variables[38];
//   U39 ADC1 Signal conversion to physical values offset, only used for display
float32_t* C_disp_offset = &coeff_buff.utility_variables[39];
//   U40 ADC2 Signal conversion to physical values offset, only used for display
float32_t* M_disp_offset = &coeff_buff.utility_variables[40];
//   U41 ADC3 Signal conversion to physical values offset, only used for display
float32_t* D_disp_offset = &coeff_buff.utility_variables[41];
// * U42 calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result, only used for display
float32_t* VOUT_disp_offset = &coeff_buff.utility_variables[42];
// * U43 calibration coeff 4-20mA, valve_flowRate_result, only used for display
float32_t* IOUT_disp_offset = &coeff_buff.utility_variables[43];
//   U44 ADC0 Signal conversion to physical values multiplier, only used for display
//float32_t* adc_disp_coeff = &coeff_buff.utility_variables[44];
float32_t* R_disp_coeff = &coeff_buff.utility_variables[44];
//   U45 ADC1 Signal conversion to physical values multiplier, only used for display
float32_t* C_disp_coeff = &coeff_buff.utility_variables[45];
//   U46 ADC2 Signal conversion to physical values multiplier, only used for display
float32_t* M_disp_coeff = &coeff_buff.utility_variables[46];
//   U47 ADC3 Signal conversion to physical values multiplier, only used for display
float32_t* D_disp_coeff = &coeff_buff.utility_variables[47];
// * U48 calibartion coeff offset, 0-10V out, was pump_fan_flow_rate_result, only used for display
float32_t* VOUT_disp_coeff = &coeff_buff.utility_variables[48];
// * U49 calibration coeff 4-20mA, valve_flowRate_result, only used for display
float32_t* IOUT_disp_coeff = &coeff_buff.utility_variables[49];
//  U50 WHpP watt hour per pulse 10 imp/kWh is kW=3.6/secondsperflash so, watt is 3600/10 = 360000/period
float32_t* WHpP = &coeff_buff.utility_variables[50];
//  U51 Wmax
float32_t* Qmax = &coeff_buff.utility_variables[51];
//  U52 DPSensorRange
float32_t* DPSensorRange = &coeff_buff.utility_variables[52];
//  U53 FrequencyRange
float32_t* FrequencyRange = &coeff_buff.utility_variables[53];
//  U54-U59 are written directly in readSDConfig
//  U66 impulse / (kW * Hour)
float32_t* imp_kWh  = &coeff_buff.utility_variables[66];
//  U67 display or calculation conversion multiplier
float32_t* ADC0_Conversion_coeff  = &coeff_buff.utility_variables[67];
//  U68 display or calculation conversion multiplier
float32_t* ADC1_Conversion_coeff  = &coeff_buff.utility_variables[68];
//  U69 display or calculation conversion multiplier
float32_t* ADC2_Conversion_coeff  = &coeff_buff.utility_variables[69];
//  U70 display or calculation conversion multiplier
float32_t* ADC3_Conversion_coeff  = &coeff_buff.utility_variables[70];
//  U71 display or calculation conversion offset
float32_t* ADC0_Conversion_offset  = &coeff_buff.utility_variables[71];
//  U72 display or calculation conversion offset
float32_t* ADC1_Conversion_offset  = &coeff_buff.utility_variables[72];
//  U73 display or calculation conversion offset
float32_t* ADC2_Conversion_offset  = &coeff_buff.utility_variables[73];
//  U74 display or calculation conversion offset
float32_t* ADC3_Conversion_offset  = &coeff_buff.utility_variables[74];


//inline conversion functions
static inline volatile float32_t imp_kWh_to_w(volatile float32_t* n_adc_value_normalized){
  return (((3600000 / (*imp_kWh)) / (*n_adc_value_normalized)));
}

static inline volatile float32_t normal_to_DP(volatile float32_t* n_adc_value_normalized){
  return ((*n_adc_value_normalized) * (*DPSensorRange));
}

static inline volatile float32_t normal_to_Frequency(volatile float32_t* n_adc_value_normalized){
  return ((*n_adc_value_normalized) * (*FrequencyRange));
}
static inline volatile float32_t WHpP_to_w(volatile float32_t* n_adc_value_normalized){
  return ((3600*(*WHpP))/(*n_adc_value_normalized));
}


static inline volatile float32_t n_Conversion_Function (volatile float32_t* n_adc_value_normalized, volatile float32_t* n_disp_coeff, volatile float32_t* n_disp_offset){
  return (((*n_adc_value_normalized) * (*n_disp_coeff)) + (*n_disp_offset));
}
static inline volatile float32_t R_Conversion_Function(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, R_disp_coeff, R_disp_offset);
}
static inline volatile float32_t C_Conversion_Function(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, C_disp_coeff, C_disp_offset);
}
static inline volatile float32_t M_Conversion_Function(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, M_disp_coeff, M_disp_offset);
}
static inline volatile float32_t D_Conversion_Function(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, D_disp_coeff, D_disp_offset);
}
static inline volatile float32_t ADC0_Conversion(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, ADC0_Conversion_coeff, ADC0_Conversion_offset);
}
static inline volatile float32_t ADC1_Conversion(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, ADC1_Conversion_coeff, ADC1_Conversion_offset);
}
static inline volatile float32_t ADC2_Conversion(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, ADC2_Conversion_coeff, ADC2_Conversion_offset);
}
static inline volatile float32_t ADC3_Conversion(volatile float32_t *n_adc_value_normalized){
  return n_Conversion_Function(n_adc_value_normalized, ADC3_Conversion_coeff, ADC3_Conversion_offset);
}

//array list of conversion functions

volatile float32_t   (*adc_value_Conversion_Function_list[4])(volatile float32_t *)={ADC0_Conversion,ADC1_Conversion,ADC2_Conversion,ADC3_Conversion};
volatile float32_t    (*ADC0_Conversion_Function)(volatile float32_t *)=*(adc_value_Conversion_Function_list[0]);
volatile float32_t    (*ADC1_Conversion_Function)(volatile float32_t *)=*(adc_value_Conversion_Function_list[1]);
volatile float32_t    (*ADC2_Conversion_Function)(volatile float32_t *)=*(adc_value_Conversion_Function_list[2]);
volatile float32_t    (*ADC3_Conversion_Function)(volatile float32_t *)=*(adc_value_Conversion_Function_list[3]);
volatile float32_t   (*R_Conversion)(volatile float32_t *);
volatile float32_t   (*C_Conversion)(volatile float32_t *);
volatile float32_t   (*M_Conversion)(volatile float32_t *);
volatile float32_t  (*D_Conversion)(volatile float32_t *);




bool (*ADC_COUNTER_IO)();

volatile int32_t    overflow_counter = 0; //timer overflow counter for the pulse timer
volatile int32_t    current_counter_fall_new = 0;
volatile int32_t    current_counter_fall_last = 0;
volatile int32_t    current_counter_fall_old = 0;
volatile int32_t    current_counter_period_fall  = 0;
volatile int32_t    overflow_counter_fall  = 0;
volatile int32_t    current_counter_rise_new = 0;
volatile int32_t    current_counter_rise_last = 0;
volatile int32_t    current_counter_rise_old = 0;
volatile int32_t    current_counter_period_rise  = 0;
volatile int32_t    overflow_counter_rise = 0;
volatile int32_t    current_counter_new = 0;


uint32_t sampleRate = 1000; //in mS, default 500


String inputString = "";
String tempString = "";

volatile bool stringComplete = true;
volatile unsigned long lastMillis = 0;
volatile int flag_counter = 0;

// Software Timer for Analog reads
SoftwareTimer analogReadTimer;

// Software Timer for SD Card Writes
//SoftwareTimer sdCardWriteTimer;
