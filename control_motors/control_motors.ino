#define IS_ESP32

const float MAX_TIME_READ_ZERO = 0.5; // after this time reading zeros and car moving, it should stop
const float MAX_TIME_READ_ZERO_START = 1.2; // special time for starting
const float MACHINE_STATE_UPDATE_INTERVAL_MS = 50;

const float CAR_WHEEL_R = 0.00325;
const float CAR_WHEEL_CIRC = 2 * 3.1415 * CAR_WHEEL_R;
const float CTRL_SENSOR_PULSES_PER_ROT = 10;
const int MIN_PULSE_COUNT = 1;
const float CAR_MAX_VELOCITY = 1; // m/s

const uint8_t IR_HITS_TO_STOP = 2;

const float MAX_CTRL_CURRENT_READ = 1;
// 0-1, alpha for "decay" of PID to stable value (higher is faster)
float ctrl_avg_alpha_up = 0.15;
float ctrl_avg_alpha_down = 0.03;
float ctrl_target_vel = 0.18;
float ctrl_P = 4;
float ctrl_I = 5;
float ctrl_D = 0;

#define CMD_GO_RIGHT 'a'
#define CMD_GO_LEFT 'b'
#define CMD_STOP 'c'
#define CMD_GO_TOGGLE 'd'
#define CMD_CTRL_UPDATE_TARGET_VEL 'i' // [1] ctrl_target_vel = cmd[1...];
#define CMD_CTRL_TOGGLE 'j'
#define CMD_CTRL_UPDATE_PARAM 'k' // [1] parametro pra atualizar: p, i, d; novo valor = = cmd[2...]
#define CMD_REPORT 'r' 
#define CMD_UPDATE_PWM_PERC 'x' // [1] atualizar o valor do PWM (0-255 = 0%-100%)
#define CMD_TIMER_UPDATE_DURATION 'y' // [1] atualizar tempo do move duration em segundos
#define CMD_TIMER_TOGGLE 'z'

bool STATE_TIMER_ON = false;
bool STATE_CONTROL_ACTIVE = true;
unsigned char STATE_PWM_PERC = 255;
float STATE_MOVE_DURATION = 12;


#ifdef IS_ESP32
  #include "BluetoothSerial.h"

  BluetoothSerial SerialBT;
  #define MPRINT SerialBT.printf
  #define now_millis() millis()

  #define WRITE_PIN digitalWrite
  #define READ_PIN digitalRead
  #define WRITE_PWM ledcWrite
#else
  #include <stdio.h>
  #include <stdlib.h>
  #include <time.h>
  #include <stdint.h>
  #include <inttypes.h>
  #include <string>
  #include <vector>

  #define LOW 0
  #define HIGH 1

  #define MPRINT printf
  unsigned long now_millis() {
    return (unsigned long)(clock() * 1000 / CLOCKS_PER_SEC);
  }
  void delay(unsigned long ms) {
    unsigned long start = now_millis();
    while(now_millis() - start < ms);
  }

  std::vector<int> pins_values(256, 0);

  void WRITE_PIN(int pin, unsigned char value) {pins_values[pin] = value; }
  unsigned char READ_PIN(int pin) { return pins_values[pin]; }
  void WRITE_PWM(int channel, unsigned char value) { MPRINT("PWM Channel %d set to %d\n", channel, value); }

  class String {
  private:
      std::string data;

  public:
      String() {}
      String(const char* s) : data(s) {}
      String(const std::string& s) : data(s) {}

      unsigned char operator[](unsigned int idx) const {
          return data[idx];
      }

      String operator+(const String& other) const {
          return String(data + other.data);
      }

      String& operator+=(const String& other) {
          data += other.data;
          return *this;
      }

      int length() const {
          return data.length();
      }

      char* c_str() const {
          return data.c_str();
      }

      // Implicit conversions:
      operator std::string() const { return data; }
  };
#endif

// Pinos dos motores
const int IN1 = 25, IN2 = 26, IN3 = 27, IN4 = 14;
const int ENA = 4, ENB = 5;

// PWM
const int freq = 20000, channelA = 0, channelB = 1, resolution = 8;

// Sensores IR
const int IR_LEFT_DO = 34;
const int IR_RIGHT_DO = 35;
const bool IR_ACTIVE_LOW = true;
uint8_t irHits = 0;

// Encoder
const int ENC_CLK = 32;
const int ENC_DT = 33;
volatile int enc_pulseCount = 0;
volatile int lastCLK = 0;
volatile int lastDT = 0;

#ifdef IS_ESP32
// Interrupção do encoder
void IRAM_ATTR encoderISR() { 
    int clkState = digitalRead(ENC_CLK);
    int dtState  = digitalRead(ENC_DT);

    if(clkState != lastCLK && dtState != lastDT){
      if (clkState != dtState)
        enc_pulseCount++;
    }
    lastCLK = clkState;
    lastDT = dtState;
}
#endif

int get_and_reset_pulse_count() {
    #ifdef IS_ESP32
    noInterrupts();
    int pulses = enc_pulseCount;
    // Only reset when min_pulse count is read
    if(pulses < MIN_PULSE_COUNT)
      pulses = 0;
    else
      enc_pulseCount = 0;
    interrupts();
    return abs(pulses);
    #else
    return 1; // Simulação
    #endif
}

// Estado de direção e movimento
#define MOV_STOPPED (0)
#define MOV_RIGHT (1)
#define MOV_LEFT (2)
int mov_state = MOV_STOPPED;
int last_mov_state = MOV_LEFT;

// Movement timer
unsigned long global_start_time = 0;
unsigned long start_move_time = 0;
float time_since_mov_start() {
  return (now_millis() - start_move_time) * 0.001f;
}
float since_start(unsigned long t) {
  return (t - global_start_time) * 0.001f;
}
float get_time_since_start() {
  return since_start(now_millis());
}

// Control parameters
bool ctrl_is_init = false;
float ctrl_current_vel = 0;
float ctrl_kp_err = 0;
float ctrl_ki_err = 0;
float ctrl_kd_err = 0;

float ctrl_out_pwm = 0; // 0-1
float ctrl_pwm_avg = 0;

unsigned long ctrl_sensor_last_read = 0;
unsigned long ctrl_sensor_first_zero_read = 0;
bool ctrl_car_has_started_moving = false;
bool ctrl_sensor_is_zero = false;


float ctrl_sensor_time_reading_zero(){
  if(!ctrl_sensor_is_zero) return 0;
  return (now_millis() - ctrl_sensor_first_zero_read) * 0.001f;
}

void ctrl_reset(){
    ctrl_is_init = false;

    ctrl_kp_err = 0;
    ctrl_ki_err = 0;
    ctrl_kd_err = 0;
    ctrl_sensor_last_read = 0;
    ctrl_sensor_first_zero_read = 0;
    ctrl_sensor_is_zero = false;
    ctrl_car_has_started_moving = false;
    ctrl_pwm_avg = 0;
}

void ctrl_init(){
    ctrl_reset();
    ctrl_sensor_last_read = now_millis();
    ctrl_sensor_first_zero_read = now_millis();
    ctrl_is_init = true;
}

float ctrl_pulses2dist(int n_pulses){
    return CAR_WHEEL_CIRC * n_pulses / CTRL_SENSOR_PULSES_PER_ROT;
}

void ctrl_update_read_values(float dist){
    const unsigned long now = now_millis();
    const float dt = (now - ctrl_sensor_last_read) * 0.001f;
    if(dt <= 0) return;

    if(dist == 0){
      if(!ctrl_sensor_is_zero){
        ctrl_sensor_is_zero = true;
        ctrl_sensor_first_zero_read = now;
      }
    } else {
      // update last read only when vel is read
      ctrl_car_has_started_moving = true;
      ctrl_sensor_is_zero = false;
      ctrl_sensor_last_read = now;
      ctrl_sensor_first_zero_read = now;
    }

    ctrl_current_vel = dist / dt;
    if(ctrl_current_vel >= MAX_CTRL_CURRENT_READ) {
      ctrl_current_vel = MAX_CTRL_CURRENT_READ;
    }

    const float last_kp_err = ctrl_kp_err;
    const float err = ctrl_target_vel - ctrl_current_vel;

    ctrl_kp_err = err;
    ctrl_ki_err += err * dt;
    ctrl_kd_err = (ctrl_kp_err - last_kp_err) / dt;
}

void ctrl_update_out_pwm() {
    // PID output
    float P = ctrl_P * ctrl_kp_err;
    float I = ctrl_I * ctrl_ki_err;
    float D = ctrl_D * ctrl_kd_err;

    float curr_pwm = P + I + D;
    if(curr_pwm > 1){
      curr_pwm = 1;
    } else if (curr_pwm < 0){
      curr_pwm = 0;
    }

    const float alpha_use = curr_pwm >= ctrl_pwm_avg ? ctrl_avg_alpha_up : ctrl_avg_alpha_down;
    ctrl_pwm_avg = ctrl_pwm_avg * (1.0 - alpha_use) + curr_pwm * alpha_use;
    ctrl_out_pwm = ctrl_pwm_avg;
    if(ctrl_out_pwm > 1){
      ctrl_out_pwm = 1;
    } else if (ctrl_out_pwm < 0){
      ctrl_out_pwm = 0;
    }
}

void ctrl_cycle() {
  const int pulses = get_and_reset_pulse_count();
  if(pulses > 0){
    // MPRINT("read %d pulses %d clk state %d dt State %d...\n", pulses, lastCLK, lastDT);
  }
  const float dist = ctrl_pulses2dist(pulses);

  if(!ctrl_is_init) return;
  ctrl_update_read_values(dist);
  ctrl_update_out_pwm();
}

void update_mov_state(int new_mov){
  if(new_mov == MOV_RIGHT) {
    if(mov_state != MOV_RIGHT){
      MPRINT("cmd MOV_RIGHT\n");
      last_mov_state = MOV_RIGHT;
      start_move_time = now_millis();
      ctrl_init();
    }
  } else if(new_mov == MOV_LEFT) {
    if(mov_state != MOV_LEFT){
      MPRINT("cmd MOV_LEFT\n");
      last_mov_state = MOV_LEFT;
      start_move_time = now_millis();
      ctrl_init();
    }
  } else if(new_mov == MOV_STOPPED) {
    MPRINT("parou comando (%d reads) (%.3f s mov)\n", irHits, time_since_mov_start());
    ctrl_reset();
    irHits = 0;
  }
  mov_state = new_mov;
}

float safeParseFloat(const char *s, bool *ok) {
    char *end;
    double val = strtod(s, &end);
    if (end == s) {
        *ok = false;  // No digits found
        return 0;
    }
    *ok = true;
    return (float)val;
}

void treat_cmd(String cmd) {
  // 'i' + n = atualizar velocidade target do controle
  // 'j' = toggle do sistema de controle
  // 'x' + n = atualizar o valor do PWM ('x' + porcentagem de 0-255, com 255=1, 0=1)
  // 'y' + n = atualizar tempo do move duration ('y' + tempo em segundos)
  // 'z' = toggle dot state timer
  if(cmd.length() < 1) return;

  if(cmd[0] == CMD_GO_RIGHT) {
    MPRINT("CMD_GO_RIGHT received\n");
    update_mov_state(MOV_RIGHT);
  } else if(cmd[0] == CMD_GO_LEFT) {
    MPRINT("CMD_GO_LEFT received\n");
    update_mov_state(MOV_LEFT);
  } else if(cmd[0] == CMD_STOP) {
    MPRINT("CMD_STOP received\n");
    update_mov_state(MOV_STOPPED);
  } else if(cmd[0] == CMD_GO_TOGGLE) {
    int new_mov = MOV_STOPPED;
    if(mov_state == MOV_STOPPED){
      if(last_mov_state == MOV_LEFT){
        new_mov = MOV_RIGHT;
      } else {
        new_mov = MOV_LEFT;
      }
    }
    MPRINT("Toggled mov state (new_mov=%d)\n", new_mov);
    update_mov_state(new_mov);
  } else if(cmd[0] == CMD_CTRL_UPDATE_TARGET_VEL) {
    if(cmd.length() < 2) return;
    bool is_ok = false;
    const float parsed_float = safeParseFloat(&cmd[1], &is_ok);
    if(!is_ok){
      MPRINT("Unable to parse float from command\n");
      return;
    }
    ctrl_target_vel = parsed_float;
    MPRINT("Updated ctrl_target_vel=%.3f\n", ctrl_target_vel);
  } else if(cmd[0] == CMD_CTRL_TOGGLE) {
    STATE_CONTROL_ACTIVE = !STATE_CONTROL_ACTIVE;
    MPRINT("Toggle timer STATE_CONTROL_ACTIVE=%d\n", STATE_CONTROL_ACTIVE);
  } else if(cmd[0] == CMD_CTRL_UPDATE_PARAM) {
    if(cmd.length() < 3) return;
    bool is_ok = false;
    const float parsed_float = safeParseFloat(&cmd[2], &is_ok);
    if(!is_ok){
      MPRINT("Unable to parse float from command\n");
      return;
    }
    if(cmd[1] == 'p'){
      ctrl_P = parsed_float;
    } else if(cmd[1] == 'i'){
      ctrl_I = parsed_float;
    } else if(cmd[1] == 'd'){
      ctrl_D = parsed_float;
    }
    MPRINT("Updated PID (P=%.2f, I=%.2f, D=%.2f)\n", ctrl_P, ctrl_I, ctrl_D);
  } else if(cmd[0] == CMD_UPDATE_PWM_PERC) {
    if(cmd.length() < 2) return;
    STATE_PWM_PERC = cmd[1];
    MPRINT("Updated STATE_PWM_PERC=%d\n", STATE_PWM_PERC);
  } else if(cmd[0] == CMD_TIMER_UPDATE_DURATION) {
    if(cmd.length() < 2) return;
    STATE_MOVE_DURATION = cmd[1];
    MPRINT("Updated STATE_MOVE_DURATION=%d\n", STATE_MOVE_DURATION);
  } else if(cmd[0] == CMD_TIMER_TOGGLE) {
    STATE_TIMER_ON = !STATE_TIMER_ON;
    MPRINT("Toggle timer STATE_TIMER_ON=%d\n", STATE_TIMER_ON);
  } else if(cmd[0] == CMD_REPORT) {
    print_state();
  } else {
    MPRINT("unrecognized cmd: ");
    MPRINT(cmd.c_str());
    MPRINT("\n");
  }
}

bool mov_must_stop(){
  if(mov_state == MOV_STOPPED) return false;

  const float limit_time_use = ctrl_car_has_started_moving? MAX_TIME_READ_ZERO : MAX_TIME_READ_ZERO_START;
  if(ctrl_sensor_time_reading_zero() > limit_time_use){
    MPRINT("parou tempo zerado (%.3f s lendo) (%.3f s mov) (limit %.3f s)\n", 
        ctrl_sensor_time_reading_zero(), time_since_mov_start(), limit_time_use);
    return true;
  }
  if(mov_state == MOV_LEFT){
    irHits += READ_PIN(IR_LEFT_DO) == LOW;
  }
  else if(mov_state == MOV_RIGHT) {
    irHits += READ_PIN(IR_RIGHT_DO) == LOW;
  }
  if(irHits > IR_HITS_TO_STOP){
      MPRINT("parou IR (%d reads) (%.3f s mov)\n", irHits, time_since_mov_start());
      return true;
  }
  if(STATE_TIMER_ON && time_since_mov_start() >= STATE_MOVE_DURATION){
    MPRINT("parou timer (%.3f s)\n", time_since_mov_start());
    return true;
  }

  return false;
}

void mov_machine_state() {
  ctrl_cycle();

  // TODO: CHECK IF THESE DIRECTIONS ARE CORRECT
  if(mov_state == MOV_RIGHT){
    WRITE_PIN(IN1, LOW);  WRITE_PIN(IN2, HIGH);
    WRITE_PIN(IN3, HIGH); WRITE_PIN(IN4, LOW);
  } else if (mov_state == MOV_LEFT) {
    WRITE_PIN(IN1, HIGH); WRITE_PIN(IN2, LOW);
    WRITE_PIN(IN3, LOW);  WRITE_PIN(IN4, HIGH);
  } else if (mov_state == MOV_STOPPED) {
    WRITE_PIN(IN1, LOW); WRITE_PIN(IN2, LOW);
    WRITE_PIN(IN3, LOW);  WRITE_PIN(IN4, LOW);
    WRITE_PWM(channelA, 0); WRITE_PWM(channelB, 0);
    return;
  }

  unsigned char pwm_write = STATE_PWM_PERC;
  if(STATE_CONTROL_ACTIVE){
    pwm_write = (unsigned char)(ctrl_out_pwm * 255);
  }

  WRITE_PWM(channelA, pwm_write); WRITE_PWM(channelB, pwm_write);
  print_pwm_state();
}


void setup() {
  global_start_time = now_millis();
  #ifdef IS_ESP32
  Serial.begin(115200);
  SerialBT.begin("PosturePal");
  #endif

  MPRINT("Bluetooth iniciado!\n");

  #ifdef IS_ESP32
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  ledcSetup(channelA, freq, resolution);
  ledcSetup(channelB, freq, resolution);
  ledcAttachPin(ENA, channelA);
  ledcAttachPin(ENB, channelB);

  pinMode(IR_LEFT_DO, INPUT);
  pinMode(IR_RIGHT_DO, INPUT);

  pinMode(ENC_DT, INPUT_PULLUP);
  pinMode(ENC_CLK, INPUT_PULLUP);
  lastCLK = digitalRead(ENC_CLK);
  attachInterrupt(digitalPinToInterrupt(ENC_CLK), encoderISR, CHANGE);
  #else
  WRITE_PIN(IR_LEFT_DO, HIGH);
  WRITE_PIN(IR_RIGHT_DO, HIGH);
  #endif

  ctrl_reset();
  update_mov_state(MOV_STOPPED);
}

void print_pwm_state() {
  MPRINT("-------- PWM --------\n");
  MPRINT("target_vel: %.3f, current_vel: %.3f\n", ctrl_target_vel, ctrl_current_vel);
  MPRINT("ctrl_is_zero: %d, ctrl_first_zero_read: %.3f\n", int(ctrl_sensor_is_zero), since_start(ctrl_sensor_first_zero_read));

  MPRINT("P: %.2f, I: %.2f, D: %.2f\n", ctrl_P, ctrl_I, ctrl_D);
  MPRINT("kp_err: %.3f, ki_err: %.3f, kd_err: %.3f\n", ctrl_kp_err, ctrl_ki_err, ctrl_kd_err);
  MPRINT("ctrl_out: %.3f ctrl_pwm_avg: %.3f\n", ctrl_out_pwm, ctrl_pwm_avg);
  MPRINT("pwm_out: %d\n", (unsigned char)(ctrl_out_pwm * 255));
  MPRINT("---------------------\n");
}

void print_mov_state() {
  MPRINT("-------- MOV --------\n");
  MPRINT("Mov state: ");
  if(mov_state == MOV_RIGHT) MPRINT("RIGHT\n");
  else if(mov_state == MOV_LEFT) MPRINT("LEFT\n");
  else if(mov_state == MOV_STOPPED) MPRINT("STOPPED\n");
  MPRINT("Last Mov state: ");
  if(last_mov_state == MOV_LEFT) MPRINT("LEFT\n");
  else if(last_mov_state == MOV_RIGHT) MPRINT("MOV_RIGHT\n");
  MPRINT("---------------------\n");
}

void print_options_state() {
  MPRINT("-------- OPT --------\n");
  MPRINT("STATE_CONTROL_ACTIVE: %d\n", STATE_CONTROL_ACTIVE);
  MPRINT("STATE_PWM_PERC: %d\n", int(STATE_PWM_PERC));
  MPRINT("STATE_TIMER_ON: %d\n", int(STATE_TIMER_ON));
  MPRINT("STATE_MOVE_DURATION: %d\n", int(STATE_MOVE_DURATION));
  MPRINT("---------------------\n");
}

void print_state(){
  MPRINT("NOW: %.3f\n", get_time_since_start());

  print_pwm_state();
  print_mov_state();
  print_options_state();

  MPRINT("\n");
}

void loop() {
  // Comandos Bluetooth
  #ifdef IS_ESP32
  if (SerialBT.available()) {
    String comando = SerialBT.readStringUntil('\n');

    // comando.trim();
    // comando.toLowerCase();

    MPRINT("Comando recebido: '");
    MPRINT(comando.c_str());
    MPRINT("'\n");

    treat_cmd(comando);
  }
  #endif

  if(mov_must_stop()){
    update_mov_state(MOV_STOPPED);
  }

  mov_machine_state();
  delay(MACHINE_STATE_UPDATE_INTERVAL_MS);
}

#ifndef IS_ESP32
int main(){
  setup();
  update_mov_state(MOV_RIGHT);
  while(true){
    loop();
    print_state();
  }
  return 0;
}
#endif