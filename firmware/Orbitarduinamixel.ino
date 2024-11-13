// Select board "Generic STM32G4 series"
// and board part number "Generic G474VBTx"

#include <Arduino.h>
#include <SimpleFOC.h>
#include <SimpleFOCDrivers.h>
#include "encoders/as5047/MagneticSensorAS5047.h"
#include <BMI088.h>
#include <Dynamixel2Arduino.h>
#include <IWatchdog.h>

// Firmware Revision (odd = dev, even = stable)
#define ORBITARDUINAMIXEL_REV  1 // Dev for first release

uint8_t orbita_firmware_rev = ORBITARDUINAMIXEL_REV;


#define GREEN_LED  PE0
#define RED_LED    PC13
#define TEST_PIN_2 PB12 // Now used as Bot_Motor_Phase_B
#define TEST_PIN_3 PB13 // Now used as Bot_Motor_Phase_C
#define TEST_PIN_4 PA7

#define WATCHDOG_DELAY 4000000 // us

//      Top

const byte topCSA     = PA0;
const byte topCSB     = PA1;
const byte topEncA    = PA8;
const byte topEncB    = PA9;
const byte topEncI    = PE7;
const byte topHallA   = PA8;
const byte topHallB   = PA9;
const byte topHallC   = PA10;
const byte topMotA    = PD12;
const byte topMotB    = PD13;
const byte topMotC    = PD14;
const byte topMotEn   = PD9;
const byte topMotnRST = PD10;
const byte topMotnFlt = PD11;
#define TOP_MOTOR_CMD 'T'

//      Middle
const byte midCSA     = PB1;
const byte midCSB     = PE9;
const byte midEncA    = PA15;
const byte midEncB    = PD4;
const byte midEncI    = PD3;
const byte midHallA   = PA15;
const byte midHallB   = PD4;
const byte midHallC   = PD7;
const byte midMotA    = PB2;
const byte midMotB    = PC12;
const byte midMotC    = PE8;
const byte midMotEn   = PC1;
const byte midMotnRST = PC2;
const byte midMotnFlt = PC3;
#define MID_MOTOR_CMD 'M'

//      Bottom
const byte botCSA     = PE14;
const byte botCSB     = PE15;
const byte botEncA    = PE2;
const byte botEncB    = PE3;
const byte botEncI    = PD2;
const byte botHallA   = PE2;
const byte botHallB   = PE6; // PE3 on Houston-A1 base
const byte botHallC   = PC0; // PE4 on Houston-A1 base
const byte botMotA    = PC6;
const byte botMotB    = PC7;
const byte botMotC    = PC8;
const byte botMotEn   = PE10;
const byte botMotnRST = PE11;
const byte botMotnFlt = PE12;
#define BOT_MOTOR_CMD 'B'


//      Orbita Magnetic Sensor
const byte orbitaSPI_MOSI  = PB5;
const byte orbitaSPI_MISO  = PB4;
const byte orbitaSPI_SCK   = PA5;
const byte orbitaTopSPI_CS = PA4;
const byte orbitaMidSPI_CS = PB7;
const byte orbitaBotSPI_CS = PB6;
#define ORBITA_SENSOR_TYPE AS5047_SPI

//      Orbita IMU
const byte orbitaIMU_Acc_CS = PA3;
const byte orbitaIMU_Gyr_CS = PB9;

/************************ Motors ***********************/
// Maxon ECX Torque 22 M
#define MOTOR_POLE_PAIRS                4
#define MOTOR_PHASE_RESISTANCE          0.763
#define MOTOR_SPEED_COEFF             645.0
// Maxon ENX 22 (encoder)
#define MOTOR_ENCODER_NB_STEPS        512

#define MOTOR_POWER_SUPPLY             12.0 // Volts
#define MOTOR_VOLTAGE_LIMIT             4.0 // Volts - finaly defines Torque resistance
//#define MOTOR_INTENSITY_LIMIT         100.0 // [Amps] - if phase resistance defined
#define MOTOR_VOLTAGE_SENSOR_ALIGN      1.0 // Volts
#define MOTOR_VELOCITY_PID_P            0.03
#define MOTOR_VELOCITY_PID_I            0.50
#define MOTOR_VELOCITY_PID_D            0.00
#define MOTOR_VELOCITY_OUT_RAMP      1000.0
#define MOTOR_VELOCITY_LPF              0.020 // seconds
#define MOTOR_TORQUE_CONTROLLER TorqueControlType::voltage
#define MOTOR_MOTION_CONTROLLER MotionControlType::velocity

#define DRIVER_RESET_DELAY             10 // ms (should be less than 1 ms)

#define TOP_MOTOR_ZERO_ELEC_ANGLE       3.14
#define TOP_MOTOR_NATURAL_DIRECTION     Direction::CCW
#define MID_MOTOR_ZERO_ELEC_ANGLE       3.14
#define MID_MOTOR_NATURAL_DIRECTION     Direction::CCW
#define BOT_MOTOR_ZERO_ELEC_ANGLE       3.14
#define BOT_MOTOR_NATURAL_DIRECTION     Direction::CW

unsigned int motors_init_started = 0;
unsigned int motors_initialized = 0;

#define HELLO_DURATION_MS            1000
#define HELLO_SIN_FREQ                  2.0
#define HELLO_SIN_AMP                   (PI/2.0)

/********************** Motor Top **********************/
HallSensor top_sensor = HallSensor(topHallA, topHallB, topHallC, MOTOR_POLE_PAIRS);
BLDCDriver3PWM top_driver = BLDCDriver3PWM(topMotA, topMotB, topMotC, topMotEn);
BLDCMotor top_motor = BLDCMotor(MOTOR_POLE_PAIRS);//, MOTOR_PHASE_RESISTANCE, MOTOR_SPEED_COEFF);
void top_DoA() {top_sensor.handleA();}
void top_DoB() {top_sensor.handleB();}
void top_DoC() {top_sensor.handleC();}

/********************** Motor Middle *******************/
HallSensor mid_sensor = HallSensor(midHallA, midHallB, midHallC, MOTOR_POLE_PAIRS);
BLDCDriver3PWM mid_driver = BLDCDriver3PWM(midMotA, midMotB, midMotC, midMotEn);
BLDCMotor mid_motor = BLDCMotor(MOTOR_POLE_PAIRS);//, MOTOR_PHASE_RESISTANCE, MOTOR_SPEED_COEFF);
void mid_DoA() {mid_sensor.handleA();}
void mid_DoB() {mid_sensor.handleB();}
void mid_DoC() {mid_sensor.handleC();}

/********************** Motor Bottom *******************/
HallSensor bot_sensor = HallSensor(botHallA, botHallB, botHallC, MOTOR_POLE_PAIRS);
BLDCDriver3PWM bot_driver = BLDCDriver3PWM(botMotA, botMotB, botMotC, botMotEn);
BLDCMotor bot_motor = BLDCMotor(MOTOR_POLE_PAIRS);//, MOTOR_PHASE_RESISTANCE, MOTOR_SPEED_COEFF);
void bot_DoA() {bot_sensor.handleA();}
void bot_DoB() {bot_sensor.handleB();}
void bot_DoC() {bot_sensor.handleC();}

/********************** Orbita Sensor ******************/

SPIClass SPI_1(orbitaSPI_MOSI, orbitaSPI_MISO, orbitaSPI_SCK);
MagneticSensorAS5047 orbita_top_sensor(orbitaTopSPI_CS);
MagneticSensorAS5047 orbita_mid_sensor(orbitaMidSPI_CS);
MagneticSensorAS5047 orbita_bot_sensor(orbitaBotSPI_CS);
float orbita_top_sensor_target    = 0.0;
float orbita_mid_sensor_target    = 0.0;
float orbita_bot_sensor_target    = 0.0;
float orbita_top_present_position = 0.0;
float orbita_mid_present_position = 0.0;
float orbita_bot_present_position = 0.0;

uint8_t orbita_system_check          = 0x00;
uint8_t orbita_motors_drivers_states = 0x00;
float   orbita_motors_voltage_limit  = MOTOR_VOLTAGE_LIMIT;
uint8_t orbita_torque_enable         = 0x00; // Start with torque disabled

/********************* Orbita PID **********************/
float get_Orbita_PID_control(float target, float current);
// PID Tuning, ultimate Ku is reached at ~6000, oscillations Tu are at 1040 ms
#define ORBITA_PID_DEF_P        500.0f
#define ORBITA_PID_DEF_I          0.000f
#define ORBITA_PID_DEF_D          0.000f
#define ORBITA_PID_DEF_LIMIT    380.0f // Velocity limit
#define ORBITA_PID_DEF_RAMP   10000.0f // Not used

PIDController Orbita_Top_PID {ORBITA_PID_DEF_P, ORBITA_PID_DEF_I, ORBITA_PID_DEF_D, ORBITA_PID_DEF_RAMP, ORBITA_PID_DEF_LIMIT};
PIDController Orbita_Mid_PID {ORBITA_PID_DEF_P, ORBITA_PID_DEF_I, ORBITA_PID_DEF_D, ORBITA_PID_DEF_RAMP, ORBITA_PID_DEF_LIMIT};
PIDController Orbita_Bot_PID {ORBITA_PID_DEF_P, ORBITA_PID_DEF_I, ORBITA_PID_DEF_D, ORBITA_PID_DEF_RAMP, ORBITA_PID_DEF_LIMIT};
#define ORBITA_SENSOR_FILTERING 10 // Taking 1 sensor value out of x foc_loop()

/*********************** BMI088 ************************/
Bmi088Accel accel(SPI_1, orbitaIMU_Acc_CS);
Bmi088Gyro  gyro (SPI_1, orbitaIMU_Gyr_CS);

/******************** Dynamixelisation *****************/
#define DXL_SERIAL       Serial3
#define DEBUG_SERIAL     Serial
#define DXL_BAUDRATE      1000000
#define DXL_FIRM_VERSION  1
#define DXL_DEFAULT_ID   70

const int DXL_DIR_PIN = -1;
const float DXL_PROTOCOL_VER_1_0 = 1.0;
const uint16_t DXL_MODEL_NUM = 0x5005;

DYNAMIXEL::SerialPortHandler dxl_port(DXL_SERIAL, DXL_DIR_PIN);
DYNAMIXEL::Slave dxl(dxl_port, DXL_MODEL_NUM);

const uint16_t DXL_REG_ADDR_MODEL_NUMBER            =  0; // DXL_MODEL_NUM
const uint16_t DXL_REG_ADDR_FIRMWARE_REV            =  6; // orbita_firmware_rev
const uint16_t DXL_REG_ADDR_ID                      =  7; // DXL_DEFAULT_ID
const uint16_t DXL_REG_ADDR_SYSTEM_CHECK            =  8; // orbita_system_check
#define DXL_REG_MASK_SYS_RESET       0x80
#define DXL_REG_MASK_BLINK_10S       0x01
const uint16_t DXL_REG_ADDR_MOTORS_DRIVERS_STATES   =  9; // orbita_motors_drivers_states
#define DXL_REG_BIT_TOP_MOTOR_STATE  7
#define DXL_REG_BIT_MID_MOTOR_STATE  6
#define DXL_REG_BIT_BOT_MOTOR_STATE  5
#define DXL_REG_BIT_TOP_DRIVER_STATE 4
#define DXL_REG_BIT_MID_DRIVER_STATE 3
#define DXL_REG_BIT_BOT_DRIVER_STATE 2
const uint16_t DXL_REG_ADDR_MOTOR_VOLTAGE_LIMIT     = 10; // top_motor.voltage_limit

const uint16_t DXL_REG_ADDR_MOTOR_VEL_PID_P         = 18; // top_motor.PID_velocity.P
const uint16_t DXL_REG_ADDR_MOTOR_VEL_PID_I         = 22; // top_motor.PID_velocity.I
const uint16_t DXL_REG_ADDR_MOTOR_VEL_PID_D         = 26; // top_motor.PID_velocity.D
const uint16_t DXL_REG_ADDR_MOTOR_VEL_RAMP_OUT      = 30; // top_motor.PID_velocity.output_ramp
const uint16_t DXL_REG_ADDR_MOTOR_VEL_LP_FILTER     = 34; // top_motor.LPF_velocity.Tf
const uint16_t DXL_REG_ADDR_ORBITA_PID_P            = 38; // Orbita_Top_PID.P
const uint16_t DXL_REG_ADDR_ORBITA_PID_I            = 42; // Orbita_Top_PID.I
const uint16_t DXL_REG_ADDR_ORBITA_PID_D            = 46; // Orbita_Top_PID.D
const uint16_t DXL_REG_ADDR_ORBITA_VEL_LIMIT        = 50; // Orbita_Top_PID.limit

const uint16_t DXL_REG_ADDR_TORQUE_ENABLE           = 58; // orbita_torque_enable
const uint16_t DXL_REG_ADDR_TOP_GOAL_POSITION       = 59; // orbita_top_sensor_target
const uint16_t DXL_REG_ADDR_MID_GOAL_POSITION       = 63; //   ''   mid   ''     ''
const uint16_t DXL_REG_ADDR_BOT_GOAL_POSITION       = 67; //   ''   bot   ''     ''
const uint16_t DXL_REG_ADDR_TOP_PRESENT_POSITION    = 71; // orbita_top_present_position
const uint16_t DXL_REG_ADDR_MID_PRESENT_POSITION    = 75; //   ''   mid   ''     ''
const uint16_t DXL_REG_ADDR_BOT_PRESENT_POSITION    = 79; //   ''   bot   ''     ''

/*******************************************************/

HardwareSerial Serial3(PB11, PB10);  // Houston-J7

void top_init();
void mid_init();
void bot_init();
void top_loop();
void mid_loop();
void bot_loop();

void init_Pacman();

void hello_motors(unsigned int);
void red_alert();
void zen_again();

void init_IO() {
  // Top
  pinMode(topEncA,  INPUT_FLOATING);
  pinMode(topEncB,  INPUT_FLOATING);
  pinMode(topEncI,  INPUT_FLOATING);
  pinMode(topHallA, INPUT_FLOATING);
  pinMode(topHallB, INPUT_FLOATING);
  pinMode(topHallC, INPUT_FLOATING);
  pinMode(topMotA,  OUTPUT_PP);
  pinMode(topMotB,  OUTPUT_PP);
  pinMode(topMotC,  OUTPUT_PP);
  pinMode(topMotEn, OUTPUT_PP);
  pinMode(topMotnRST, INPUT_FLOATING);
  pinMode(topMotnFlt, INPUT_FLOATING);
  pinMode(topCSA,   INPUT_ANALOG);
  pinMode(topCSB,   INPUT_ANALOG);

  digitalWrite(topMotA,  LOW);
  digitalWrite(topMotB,  LOW);
  digitalWrite(topMotC,  LOW);
  digitalWrite(topMotEn, LOW);

  // Middle
  pinMode(midEncA,  INPUT_FLOATING);
  pinMode(midEncB,  INPUT_FLOATING);
  pinMode(midEncI,  INPUT_FLOATING);
  pinMode(midHallA, INPUT_FLOATING);
  pinMode(midHallB, INPUT_FLOATING);
  pinMode(midHallC, INPUT_FLOATING);
  pinMode(midMotA,  OUTPUT_PP);
  pinMode(midMotB,  OUTPUT_PP);
  pinMode(midMotC,  OUTPUT_PP);
  pinMode(midMotEn, OUTPUT_PP);
  pinMode(midMotnRST, INPUT_FLOATING);
  pinMode(midMotnFlt, INPUT_FLOATING);
  pinMode(midCSA,   INPUT_ANALOG);
  pinMode(midCSB,   INPUT_ANALOG);

  digitalWrite(midMotA,  LOW);
  digitalWrite(midMotB,  LOW);
  digitalWrite(midMotC,  LOW);
  digitalWrite(midMotEn, LOW);

  // Bottom
  pinMode(botEncA,  INPUT_FLOATING);
  pinMode(botEncB,  INPUT_FLOATING);
  pinMode(botEncI,  INPUT_FLOATING);
  pinMode(botHallA, INPUT_FLOATING);
  pinMode(botHallB, INPUT_FLOATING);
  pinMode(botHallC, INPUT_FLOATING);
  pinMode(botMotA,  OUTPUT_PP);
  pinMode(botMotB,  OUTPUT_PP);
  pinMode(botMotC,  OUTPUT_PP);
  pinMode(botMotEn, OUTPUT_PP);
  pinMode(botMotnRST, INPUT_FLOATING);
  pinMode(botMotnFlt, INPUT_FLOATING);
  pinMode(botCSA,   INPUT_ANALOG);
  pinMode(botCSB,   INPUT_ANALOG);

  digitalWrite(botMotA,  LOW);
  digitalWrite(botMotB,  LOW);
  digitalWrite(botMotC,  LOW);
  digitalWrite(botMotEn, LOW);
}


void setup_bitbang_hall_testing() {
  pinMode(topHallA, INPUT);
  pinMode(topHallB, INPUT);
  pinMode(topHallC, INPUT);
  pinMode(midHallA, INPUT);
  pinMode(midHallB, INPUT);
  pinMode(midHallC, INPUT);
  pinMode(botHallA, INPUT);
  pinMode(botHallB, INPUT);
  pinMode(botHallC, INPUT);
}
void loop_bitbang_hall_testing() {
    Serial3.print("\tTOP_rd: ");
    Serial3.print(digitalRead(topHallA));
    Serial3.print(digitalRead(topHallB));
    Serial3.print(digitalRead(topHallC));

    Serial3.print("\tMID_bl: ");
    Serial3.print(digitalRead(midHallA));
    Serial3.print(digitalRead(midHallB));
    Serial3.print(digitalRead(midHallC));

    Serial3.print("\tBOT_yl: ");
    Serial3.print(digitalRead(botHallA));
    Serial3.print(digitalRead(botHallB));
    Serial3.print(digitalRead(botHallC));
    Serial3.println();
}


void setup() {
  init_IO();
  //BMI fix
  pinMode(orbitaIMU_Acc_CS, OUTPUT);
  digitalWrite(orbitaIMU_Acc_CS, HIGH);
  pinMode(orbitaIMU_Gyr_CS, OUTPUT);
  digitalWrite(orbitaIMU_Gyr_CS, HIGH);








  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(TEST_PIN_4, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(TEST_PIN_4, LOW);

  //  delay(1000); // Connecting time
  delay(250);  // Just flashing
  digitalWrite(RED_LED, LOW);

  pinMode(topMotnFlt, INPUT_FLOATING);
  pinMode(midMotnFlt, INPUT_FLOATING);
  pinMode(botMotnFlt, INPUT_FLOATING);
  //attachInterrupt(digitalPinToInterrupt(topMotnFlt), red_alert, FALLING);
  //attachInterrupt(digitalPinToInterrupt(midMotnFlt), red_alert, FALLING);
  //attachInterrupt(digitalPinToInterrupt(botMotnFlt), red_alert, FALLING);
  //attachInterrupt(digitalPinToInterrupt(topMotnFlt), zen_again, RISING);
  //attachInterrupt(digitalPinToInterrupt(midMotnFlt), zen_again, RISING);
  //attachInterrupt(digitalPinToInterrupt(botMotnFlt), zen_again, RISING);

  Serial3.begin(115200);
  SimpleFOCDebug::enable(&Serial3);
  Serial3.print("\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n\r\n");
  Serial3.println(F("=== Pollen Robotics: Orbita3D ==="));
  Serial3.print(F("CPU_freq: "));
  Serial3.print(F_CPU/1000000);
  Serial3.println(F("MHz"));
  Serial3.print(F("Power supply: "));
  Serial3.println(bot_driver.voltage_power_supply);

Serial3.println("-------------------------------------------top:"); 
  top_init();
    

  Serial3.println("-------------------------------------------mid:"); 
  mid_init();

  Serial3.println("-------------------------------------------bot:"); 
  bot_init();




  Serial3.println("OK");


/*
  setup_bitbang_hall_testing();
  while(true)
  {
    loop_bitbang_hall_testing();
    delay(100);
  } 
*/

  if (motors_initialized != motors_init_started)
    while (true) {
      Serial3.print  (F("Wrong init ("));
      Serial3.print  (motors_initialized);
      Serial3.print  ('/');
      Serial3.print  (motors_init_started);
      Serial3.println(F(" motors initialized)"));
      digitalWrite(RED_LED, !digitalRead(RED_LED));
      delay(10000);
    }
    
  init_Pacman();
//  hello_motors(HELLO_DURATION_MS);
  Serial3.println("--------------------------------done!");

  
  Serial3.end();

  dxl_port.begin(DXL_BAUDRATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VER_1_0);
  dxl.setFirmwareVersion(DXL_FIRM_VERSION);
  dxl.setID(DXL_DEFAULT_ID);
  init_Dynamixel_registers();

  IWatchdog.begin(WATCHDOG_DELAY);
  digitalWrite(GREEN_LED, HIGH);
}

void loop() {
  top_loop(); // from 12 to 26us @10kHz (with 1 PID calculation out of 10 loops)
              //    -> 28% calculation time
              // Motor_Hall_IT: 2,5us. At 4 Volts_max, 7 IT for 7,84 ms
              //    -> 0,2% calculation time
  mid_loop();
  bot_loop();
  Orbita_loop(); // 142 us @1kHz
  if (dxl.processPacket() == false) { // up to 80 us
    red_alert();
  }
  Orbita_update_slow_registers();
  Orbita_polling_faulty_driver();
}

void Orbita_update_slow_registers() {
  // static uint16_t dec = 0;

  // if(dec++ > 10000) { // Every seconds
// digitalWrite(TEST_PIN_4, HIGH);
    // dec = 0;

static bool is_enabled = !orbita_torque_enable;

    if (orbita_torque_enable) {
      if (!is_enabled) {
        zen_again();

        // Target is set to actual position so that Orbita 
        // stays where it is after re-enabling motors.
        orbita_top_sensor.update();
        orbita_mid_sensor.update();
        orbita_bot_sensor.update();
        orbita_top_present_position = orbita_top_sensor.getAngle();
        orbita_mid_present_position = orbita_mid_sensor.getAngle();
        orbita_bot_present_position = orbita_bot_sensor.getAngle();
        orbita_top_sensor_target    = orbita_top_present_position;
        orbita_mid_sensor_target    = orbita_mid_present_position;
        orbita_bot_sensor_target    = orbita_bot_present_position;

        top_motor.enable();
        mid_motor.enable();
        bot_motor.enable();

        is_enabled = true;
      }
    } else {
      if (is_enabled) {
        red_alert();
        top_motor.disable();
        mid_motor.disable();
        bot_motor.disable();

        is_enabled = false;
      }
    }

    if (true) {
      // Propagating top_motor/Orbita parameters to middle and bottom 
      mid_motor.voltage_limit = top_motor.voltage_limit;
      bot_motor.voltage_limit = top_motor.voltage_limit;
      mid_motor.PID_velocity.P = top_motor.PID_velocity.P;
      bot_motor.PID_velocity.P = top_motor.PID_velocity.P;
      mid_motor.PID_velocity.I = top_motor.PID_velocity.I;
      bot_motor.PID_velocity.I = top_motor.PID_velocity.I;
      mid_motor.PID_velocity.D = top_motor.PID_velocity.D;
      bot_motor.PID_velocity.D = top_motor.PID_velocity.D;
      mid_motor.PID_velocity.output_ramp  = top_motor.PID_velocity.output_ramp;
      bot_motor.PID_velocity.output_ramp  = top_motor.PID_velocity.output_ramp;
      mid_motor.LPF_velocity.Tf = top_motor.LPF_velocity.Tf;
      bot_motor.LPF_velocity.Tf = top_motor.LPF_velocity.Tf;

      Orbita_Mid_PID.P = Orbita_Top_PID.P;
      Orbita_Bot_PID.P = Orbita_Top_PID.P;
      Orbita_Mid_PID.I = Orbita_Top_PID.I;
      Orbita_Bot_PID.I = Orbita_Top_PID.I;
      Orbita_Mid_PID.D = Orbita_Top_PID.D;
      Orbita_Bot_PID.D = Orbita_Top_PID.D;
      Orbita_Mid_PID.limit = Orbita_Top_PID.limit;
      Orbita_Bot_PID.limit = Orbita_Top_PID.limit;
    }

    if(orbita_system_check & DXL_REG_MASK_SYS_RESET)
      orbita_reset();
    if(orbita_system_check & DXL_REG_MASK_BLINK_10S)
      blinking_green_and_red_led_for_10_s();
  // }
// digitalWrite(TEST_PIN_4, LOW);
}

void Orbita_polling_faulty_driver() {
  static uint16_t dec = 0;

  if(dec++ > 10000) { // Every seconds
    dec = 0;
    if(!digitalRead(topMotnFlt)) {
      bitSet(orbita_motors_drivers_states, DXL_REG_BIT_TOP_DRIVER_STATE);
//red_alert_blinking(); // block
      red_alert();
    }
    if(!digitalRead(midMotnFlt)) {
      bitSet(orbita_motors_drivers_states, DXL_REG_BIT_MID_DRIVER_STATE);
//red_alert_blinking(); // block
      red_alert();
    }
    if(!digitalRead(botMotnFlt)) {
      bitSet(orbita_motors_drivers_states, DXL_REG_BIT_BOT_DRIVER_STATE);
//red_alert_blinking(); // block
      red_alert();
    }
  }
}

void top_loop() {
  top_motor.loopFOC();  // 17,5us on Nucleo-F401RE @84MHz
  top_motor.move();     // 12us
/*  top_present_position = top_sensor.getAngle();
  if (dxl.processPacket() == false) {
    red_alert();
  }*/
}

void mid_loop() {
  mid_motor.loopFOC();  // 16 us on Houston @150MHz (motor velocity 1000 rad/s)
  mid_motor.move();     //  9 us on Houston @150MHz (motor velocity 1000 rad/s)
/*  mid_present_position = mid_sensor.getAngle();
  if (dxl.processPacket() == false) {
    red_alert();
  }*/
}

void bot_loop() {
  bot_motor.loopFOC();
  bot_motor.move();
/*  bot_present_position = bot_sensor.getAngle();
  if (dxl.processPacket() == false) {
    red_alert();
  }*/
}

void Orbita_loop() {
  // 140us every 1,09 ms (filtering 1/10) -> 13%
  static uint8_t orbita_sensor_filtering_idx = 0;
  if(orbita_sensor_filtering_idx++ > ORBITA_SENSOR_FILTERING) {
    orbita_sensor_filtering_idx = 0;

    IWatchdog.reload();

    // Getting position from Pacman
    orbita_top_sensor.update(); // 46us
    orbita_mid_sensor.update();
    orbita_bot_sensor.update();
    orbita_top_present_position = orbita_top_sensor.getAngle();
    orbita_mid_present_position = orbita_mid_sensor.getAngle();
    orbita_bot_present_position = orbita_bot_sensor.getAngle();

    // Setting commands to motors -> command = pid ( target - measurement )
    // 8 us every 1,07 ms (filtering 1/10)
    top_motor.target = Orbita_Top_PID(orbita_top_sensor_target - orbita_top_present_position);
    top_motor.target *= -1.0;
    mid_motor.target = Orbita_Mid_PID(orbita_mid_sensor_target - orbita_mid_present_position);
    mid_motor.target *= -1.0;
    bot_motor.target = Orbita_Bot_PID(orbita_bot_sensor_target - orbita_bot_present_position);
    bot_motor.target *= -1.0;
  }
}

void init_Dynamixel_registers() {
  dxl.addControlItem(DXL_REG_ADDR_FIRMWARE_REV,          orbita_firmware_rev);
  dxl.addControlItem(DXL_REG_ADDR_SYSTEM_CHECK,          orbita_system_check);
  dxl.addControlItem(DXL_REG_ADDR_MOTORS_DRIVERS_STATES, orbita_motors_drivers_states);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VOLTAGE_LIMIT,   top_motor.voltage_limit);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VEL_PID_P,       top_motor.PID_velocity.P);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VEL_PID_I,       top_motor.PID_velocity.I);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VEL_PID_D,       top_motor.PID_velocity.D);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VEL_RAMP_OUT,    top_motor.PID_velocity.output_ramp);
  dxl.addControlItem(DXL_REG_ADDR_MOTOR_VEL_LP_FILTER,   top_motor.LPF_velocity.Tf);
  dxl.addControlItem(DXL_REG_ADDR_ORBITA_PID_P,          Orbita_Top_PID.P);
  dxl.addControlItem(DXL_REG_ADDR_ORBITA_PID_I,          Orbita_Top_PID.I);
  dxl.addControlItem(DXL_REG_ADDR_ORBITA_PID_D,          Orbita_Top_PID.D);
  dxl.addControlItem(DXL_REG_ADDR_ORBITA_VEL_LIMIT,      Orbita_Top_PID.limit);
  dxl.addControlItem(DXL_REG_ADDR_TORQUE_ENABLE,         orbita_torque_enable);
  dxl.addControlItem(DXL_REG_ADDR_TOP_GOAL_POSITION,     orbita_top_sensor_target);
  dxl.addControlItem(DXL_REG_ADDR_MID_GOAL_POSITION,     orbita_mid_sensor_target);
  dxl.addControlItem(DXL_REG_ADDR_BOT_GOAL_POSITION,     orbita_bot_sensor_target);
  dxl.addControlItem(DXL_REG_ADDR_TOP_PRESENT_POSITION,  orbita_top_present_position);
  dxl.addControlItem(DXL_REG_ADDR_MID_PRESENT_POSITION,  orbita_mid_present_position);
  dxl.addControlItem(DXL_REG_ADDR_BOT_PRESENT_POSITION,  orbita_bot_present_position);
}

void init_Pacman(void) {
//  BMI088_init(); //seems unstable
  AS5047_init();

  orbita_top_sensor.update();
  orbita_mid_sensor.update();
  orbita_bot_sensor.update();
  // Stay here!
  orbita_top_sensor_target = orbita_top_sensor.getAngle();
  orbita_mid_sensor_target = orbita_mid_sensor.getAngle();
  orbita_bot_sensor_target = orbita_bot_sensor.getAngle();
}

void BMI088_init(void) {
  int status;

  status = accel.begin();
  status = accel.begin(); // ouais bon ça merde au premier essai.
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  status = accel.begin();
  if (status < 0) {
    Serial3.println("Accel Initialization Error");
    Serial3.println(status);
    while (true) {
      Serial3.println("!!error_accel_init!!");
      delay(10000);
      digitalWrite(RED_LED, !digitalRead(RED_LED));
    }
  }
  Serial3.println("Acc. init [OK]");

  status = gyro.begin();
  if (status < 0) {
    Serial3.println("Gyro Initialization Error");
    Serial3.println(status);
    while (true) {
      Serial3.println("!!error_gyro_init!!");
      delay(10000);
      digitalWrite(RED_LED, !digitalRead(RED_LED));
    }
  }
  Serial3.println("Gyro init [OK]");

  for(int i=0 ; i<3 ; i++) {
    accel.readSensor();
    gyro.readSensor();

    Serial3.print("BMI088 [");
    Serial3.print(accel.getAccelX_mss());
    Serial3.print("\t");
    Serial3.print(accel.getAccelY_mss());
    Serial3.print("\t");
    Serial3.print(accel.getAccelZ_mss());
    Serial3.print("]\t[");
    Serial3.print(gyro.getGyroX_rads());
    Serial3.print("\t");
    Serial3.print(gyro.getGyroY_rads());
    Serial3.print("\t");
    Serial3.print(gyro.getGyroZ_rads());
    Serial3.print("]\t[");
    Serial3.print(accel.getTemperature_C());
    Serial3.print("]\r\n");
    delay(100);
  }
}

void AS5047_init(void) {
  orbita_top_sensor.init(&SPI_1);
  orbita_mid_sensor.init(&SPI_1);
  orbita_bot_sensor.init(&SPI_1);

  for(int i=0 ; i<3 ; i++) {
//  while(true) {
    orbita_top_sensor.update();
    orbita_mid_sensor.update();
    orbita_bot_sensor.update();

    Serial3.print("Angle: [");
    Serial3.print(orbita_top_sensor.getAngle());
    Serial3.print("\t");
    Serial3.print(orbita_mid_sensor.getAngle());
    Serial3.print("\t");
    Serial3.print(orbita_bot_sensor.getAngle());
    Serial3.print("] - ");
    Serial3.print("Current Angle: [");
    Serial3.print(orbita_top_sensor.getCurrentAngle()); // No full rotations (!SPI transfert!)
    Serial3.print("\t");
    Serial3.print(orbita_mid_sensor.getCurrentAngle()); // No full rotations (!SPI transfert!)
    Serial3.print("\t");
    Serial3.print(orbita_bot_sensor.getCurrentAngle()); // No full rotations (!SPI transfert!)
    Serial3.println("]");
    delay(100);
  }
}
/*
void hello_motors(unsigned int duration) {
  unsigned int start_time   = millis();
  unsigned long move_Millis = start_time;

  do {
    move_Millis = millis();
    orbita_top_sensor_target = orbita_top_init_pos + HELLO_SIN_AMP * sin(2.0 * PI * HELLO_SIN_FREQ * move_Millis/1000.0);
    orbita_mid_sensor_target = orbita_mid_init_pos + HELLO_SIN_AMP * sin(2.0 * PI * HELLO_SIN_FREQ * move_Millis/1000.0);
    orbita_bot_sensor_target = orbita_bot_init_pos + HELLO_SIN_AMP * sin(2.0 * PI * HELLO_SIN_FREQ * move_Millis/1000.0);
    loop();
  } while ((millis() - start_time) < duration);

  orbita_top_sensor_target = orbita_top_init_pos;
  orbita_mid_sensor_target = orbita_mid_init_pos;
  orbita_bot_sensor_target = orbita_bot_init_pos;
}*/

void top_init() {
  //Serial3.println("-");
  top_sensor.init();
  //  Serial3.println("-"); 
  motors_init_started++;
  top_sensor.enableInterrupts(top_DoA, top_DoB, top_DoC);
//Serial3.println("-");
  top_sensor.pullup = Pullup::USE_EXTERN;
  top_motor.linkSensor(&top_sensor);
  //Serial3.println("-");
  top_driver.voltage_power_supply = MOTOR_POWER_SUPPLY;
  top_driver.init();
  //Serial3.println("-");
  top_motor.linkDriver(&top_driver);
  //Serial3.println("-");
  top_motor.torque_controller = MOTOR_TORQUE_CONTROLLER;
  top_motor.controller = MOTOR_MOTION_CONTROLLER;
  top_motor.voltage_sensor_align = MOTOR_VOLTAGE_SENSOR_ALIGN;
  top_motor.PID_velocity.P = MOTOR_VELOCITY_PID_P;
  top_motor.PID_velocity.I = MOTOR_VELOCITY_PID_I;
  top_motor.PID_velocity.D = MOTOR_VELOCITY_PID_D;
  top_motor.PID_velocity.output_ramp = MOTOR_VELOCITY_OUT_RAMP;
  top_motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
//  top_motor.current_limit = MOTOR_INTENSITY_LIMIT;
  top_motor.LPF_velocity.Tf = MOTOR_VELOCITY_LPF;

  top_motor.init();
    //Serial3.println("-");
  //if (top_motor.initFOC() == 1) {
  
    
  if (top_motor.initFOC(TOP_MOTOR_ZERO_ELEC_ANGLE) == 1) {
    motors_initialized++;
    bitSet(orbita_motors_drivers_states, DXL_REG_BIT_TOP_MOTOR_STATE);
  }
  

  //Serial3.println("-");
  _delay(100);
}

void mid_init() {
  //_delay(1000); 
  //Serial3.println("-");
  mid_sensor.init();
  //_delay(100);
  //Serial3.println("-");
  motors_init_started++;
  mid_sensor.enableInterrupts(mid_DoA, mid_DoB, mid_DoC);
  //_delay(100);
  //Serial3.println("-");
  mid_sensor.pullup = Pullup::USE_EXTERN;
  mid_motor.linkSensor(&mid_sensor);
  //_delay(100); 
  //Serial3.println("-");
  mid_driver.voltage_power_supply = MOTOR_POWER_SUPPLY;
  mid_driver.init();
  //_delay(100);
  //Serial3.println("-");
  mid_motor.linkDriver(&mid_driver);
  //_delay(100);
  //Serial3.println("-");
  mid_motor.torque_controller = MOTOR_TORQUE_CONTROLLER;
  mid_motor.controller = MOTOR_MOTION_CONTROLLER;
  mid_motor.voltage_sensor_align = MOTOR_VOLTAGE_SENSOR_ALIGN;
  mid_motor.PID_velocity.P = MOTOR_VELOCITY_PID_P;
  mid_motor.PID_velocity.I = MOTOR_VELOCITY_PID_I;
  mid_motor.PID_velocity.D = MOTOR_VELOCITY_PID_D;
  mid_motor.PID_velocity.output_ramp = MOTOR_VELOCITY_OUT_RAMP;
  mid_motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
//  mid_motor.current_limit = MOTOR_INTENSITY_LIMIT;
  mid_motor.LPF_velocity.Tf = MOTOR_VELOCITY_LPF;

  
  mid_motor.init();
  //_delay(100);
  //Serial3.println("-");
//  if (mid_motor.initFOC() == 1) {

  
  if (mid_motor.initFOC(MID_MOTOR_ZERO_ELEC_ANGLE) == 1) {
    motors_initialized++;
    bitSet(orbita_motors_drivers_states, DXL_REG_BIT_MID_MOTOR_STATE);
  }
  

  _delay(100);
}

void bot_init() {
  //Serial3.println("-");
  bot_sensor.init();
  //Serial3.println("-"); 
  motors_init_started++;
  bot_sensor.enableInterrupts(bot_DoA, bot_DoB, bot_DoC);
  //Serial3.println("-");
  bot_sensor.pullup = Pullup::USE_EXTERN;
  bot_motor.linkSensor(&bot_sensor);
  //Serial3.println("-");
  bot_driver.voltage_power_supply = MOTOR_POWER_SUPPLY;
  bot_driver.init();
  //Serial3.println("-");
  bot_motor.linkDriver(&bot_driver);
  //Serial3.println("-");
  bot_motor.torque_controller = MOTOR_TORQUE_CONTROLLER;
  bot_motor.controller = MOTOR_MOTION_CONTROLLER;
  bot_motor.voltage_sensor_align = MOTOR_VOLTAGE_SENSOR_ALIGN;
  bot_motor.PID_velocity.P = MOTOR_VELOCITY_PID_P;
  bot_motor.PID_velocity.I = MOTOR_VELOCITY_PID_I;
  bot_motor.PID_velocity.D = MOTOR_VELOCITY_PID_D;
  bot_motor.PID_velocity.output_ramp = MOTOR_VELOCITY_OUT_RAMP;
  bot_motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
//  bot_motor.current_limit = MOTOR_INTENSITY_LIMIT;
  bot_motor.LPF_velocity.Tf = MOTOR_VELOCITY_LPF;

  bot_motor.init();
//  Serial3.println("-");
//  if (bot_motor.initFOC() == 1) {

  
  if (bot_motor.initFOC(BOT_MOTOR_ZERO_ELEC_ANGLE) == 1) {
    motors_initialized++;
    bitSet(orbita_motors_drivers_states, DXL_REG_BIT_BOT_MOTOR_STATE);
  }
  

  _delay(100);
}

void red_alert() {
  digitalWrite(RED_LED, HIGH);
}

void red_alert_blinking() {
  top_motor.disable();
  mid_motor.disable();
  bot_motor.disable();

  while(true) {
    digitalWrite(RED_LED, !digitalRead(RED_LED));
    delay(100);
  }
}

void zen_again() {
  digitalWrite(RED_LED, LOW);
}

void orbita_reset() {
  IWatchdog.begin(IWDG_TIMEOUT_MIN);
  while(true)
    ; // Watchdog reset
}


void blinking_green_and_red_led_for_10_s() {
  for(int i=0 ; i<20 ; i++) {
    IWatchdog.reload();

    digitalWrite(RED_LED,   HIGH);
    digitalWrite(GREEN_LED, LOW);
    dxl.processPacket();
    delay(250);

    digitalWrite(RED_LED,   LOW);
    digitalWrite(GREEN_LED, HIGH);
    dxl.processPacket();
    delay(250);
  }
}
