#include <Arduino.h>
#include <math.h>
#include <WiFi.h>

// --- MICRO-ROS INCLUDES ---
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/pose2_d.h> 

// ---------------- CONFIGURACIÓN SOFTWARE ----------------
//char ssid[] = "DIGIFIBRA-TySA";
//char password[] = "T7zKKUR4b5kS";
char ssid[] = "HUAWEI P smart 2019";
char password[] = "miaumiau";

char agent_ip[] = "10.158.222.179";
size_t agent_port = 8888; 

// --- OBJETOS ROS ---
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
rcl_publisher_t pose_publisher;
geometry_msgs__msg__Pose2D pose_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;


// ---------------- CONFIGURACIÓN HARDWARE ----------------

// --- SENSORES DE PROXIMIDAD (SONAR) ---
// Sensor 1 (Frontal)
const int pinTrig1 = 14;
const int pinEcho1 = 12;

// Sensor 2 (Izquierdo)
const int pinTrig2 = 33;
const int pinEcho2 = 35; // Pin Input Only (OK para Echo)

// Sensor 3 (Derecho)
const int pinTrig3 = 32; 
const int pinEcho3 = 34; // Pin Input Only (OK para Echo)

// Variables para almacenar las distancias leídas
float dist_frontal = 0.0;
float dist_izq = 0.0;
float dist_der = 0.0;


// --- Motor 1 (Derecha) --- 
const int pwmPin_m1_A = 19;  
const int pwmPin_m1_B = 18;
const int encPin_m1_A = 23; 
const int encPin_m1_B = 22;

// --- Motor 2 (Izquierda) --- 
const int pwmPin_m2_A = 25;  
const int pwmPin_m2_B = 26;
const int encPin_m2_A = 16; 
const int encPin_m2_B = 17;

// LED para estado de conexión
const int LED_PIN = 2; 

// ---------------- PARÁMETROS DEL ROBOT ----------------
const float distancia_ruedas = 0.21; 
const float radio_rueda = 0.0335;      
const int vueltas = 1500;            

// Configuración PWM
const int freq = 1000;
const int resolution = 10;

// Configuración Timer
hw_timer_t *timer = NULL;
const double tiempo_muestreo = 0.01; 

// ---------------- VARIABLES DE ESTADO ----------------
// --- Variables Motor 1 ---
volatile long contador_m1 = 0;
long cont_ant_m1 = 0;
double vel_filtrada_m1 = 0.0;
double error_ant_m1 = 0.0;
double error_acum_m1 = 0.0;
double deriv_filt_m1 = 0.0;
double referencia_m1 = 0.0; 

// --- Variables Motor 2 ---
volatile long contador_m2 = 0;
long cont_ant_m2 = 0;
double vel_filtrada_m2 = 0.0;
double error_ant_m2 = 0.0;
double error_acum_m2 = 0.0;
double deriv_filt_m2 = 0.0;
double referencia_m2 = 0.0; 

// --- PID Constantes ---
double Kp = 0.17661; 
double Ki = 5.0;    
double Kd = 0.005; 

// --- Filtros ---
double alpha_vel = 0.7;      
double alpha_derivada = 0.7; 

// --- CONTROL DE NAVEGACIÓN ---
float vel_ang = 0; 
float vel_lin = 0;

// --- ODOMETRÍA ---
double theta = 0.0;
double pos_x = 0.0;
double pos_y = 0.0;

// ---------------- INTERRUPCIONES ENCODERS ----------------
void IRAM_ATTR isr_m1_A(){ if (digitalRead(encPin_m1_A) == digitalRead(encPin_m1_B)){ contador_m1++; } else{ contador_m1--; }}
void IRAM_ATTR isr_m1_B(){ if (digitalRead(encPin_m1_A) == digitalRead(encPin_m1_B)){ contador_m1--; } else{ contador_m1++; }}
void IRAM_ATTR isr_m2_A(){ if (digitalRead(encPin_m2_A) == digitalRead(encPin_m2_B)){ contador_m2++; } else{ contador_m2--; }}
void IRAM_ATTR isr_m2_B(){ if (digitalRead(encPin_m2_A) == digitalRead(encPin_m2_B)){ contador_m2--; } else{ contador_m2++; }}

// ---------------- FUNCIONES AUXILIARES ----------------

// --- FUNCIÓN DE LECTURA DE SONAR ---
// Devuelve la distancia en cm. Si falla o está muy lejos, devuelve 300.0
float leerSonar(int trigPin, int echoPin) {
  // 1. Limpiar Trigger
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // 2. Disparo (10 microsegundos)
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // 3. Lectura del Eco con Timeout
  // 25000 micros = 25ms (aprox 4 metros). 
  // Importante: El timeout evita que el robot se quede "congelado" si un cable falla.
  long duracion = pulseIn(echoPin, HIGH, 25000);

  if (duracion == 0) return 300.0; // Si no hay eco, asumimos camino libre

  // 4. Calcular distancia en cm
  return (duracion * 0.0343) / 2.0;
}

double calcula_PID (double velocidad_input, double valor_buscado, double &error_ant, double &error_acum, double &deriv_filt){
  if (valor_buscado == 0 && abs(velocidad_input) < 10) { error_acum = 0; return 0; }
  double error = valor_buscado - velocidad_input;
  double derivada_raw = (error - error_ant) / tiempo_muestreo;
  deriv_filt = alpha_derivada * deriv_filt + (1.0 - alpha_derivada) * derivada_raw;
  error_acum += error * tiempo_muestreo;
  if (error_acum > 4000) error_acum = 4000; 
  if (error_acum < -4000) error_acum = -4000;
  double pwm = Kp * error + Ki * error_acum + Kd * deriv_filt;
  error_ant = error; 
  if (pwm > 1023) pwm = 1023; else if (pwm < -1023) pwm = -1023;
  return pwm;
}

void P3_actua(int pwmPin, int pwmPin2, double valor_pwm_){
 if (valor_pwm_>=0) { ledcWrite(pwmPin, (int)valor_pwm_); ledcWrite(pwmPin2, 0); } 
 else { ledcWrite(pwmPin2, (int)(-1*valor_pwm_)); ledcWrite(pwmPin, 0); }
}

// ---------------- TIMER ISR (ODOMETRÍA + PID) ----------------
void IRAM_ATTR ontimerISR(){
  // Encoders y Velocidad
  double dif_m1 = contador_m1 - cont_ant_m1;
  double vel_cruda_m1 = dif_m1 / tiempo_muestreo; 
  vel_filtrada_m1 = alpha_vel * vel_filtrada_m1 + (1.0 - alpha_vel) * vel_cruda_m1;
  cont_ant_m1 = contador_m1;
  
  double dif_m2 = contador_m2 - cont_ant_m2;
  double vel_cruda_m2 = dif_m2 / tiempo_muestreo;
  vel_filtrada_m2 = alpha_vel * vel_filtrada_m2 + (1.0 - alpha_vel) * vel_cruda_m2;
  cont_ant_m2 = contador_m2;

  // Odometría
  double v_der = (vel_filtrada_m1 / (double)vueltas) * 2.0 * M_PI * radio_rueda;
  double v_izq = (vel_filtrada_m2 / (double)vueltas) * 2.0 * M_PI * radio_rueda;
  double v_robot = (v_der + v_izq) / 2.0;
  double w_robot = (v_der - v_izq) / distancia_ruedas;
  double delta_theta = w_robot * tiempo_muestreo;
  
  theta -= delta_theta; // Invertido según tu lógica
  if (theta > M_PI) theta -= 2.0 * M_PI;
  if (theta < -M_PI) theta += 2.0 * M_PI;

  pos_x -= v_robot * cos(theta - (delta_theta / 2.0)) * tiempo_muestreo;
  pos_y -= v_robot * sin(theta - (delta_theta / 2.0)) * tiempo_muestreo;


  // PID
  float pwm_der = calcula_PID(vel_filtrada_m1, referencia_m1, error_ant_m1, error_acum_m1, deriv_filt_m1);
  P3_actua(pwmPin_m1_A, pwmPin_m1_B, pwm_der);

  float pwm_izq = calcula_PID(vel_filtrada_m2, referencia_m2, error_ant_m2, error_acum_m2, deriv_filt_m2);
  P3_actua(pwmPin_m2_A, pwmPin_m2_B, pwm_izq);
}

// ---------------- CALLBACK DE ROS ----------------
void subscription_callback(const void * msgin) {  
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  vel_lin = msg->linear.x;
  vel_ang = msg->angular.z;
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);

  // 1. Configurar Pines de Sonares
  pinMode(pinTrig1, OUTPUT); pinMode(pinEcho1, INPUT);
  pinMode(pinTrig2, OUTPUT); pinMode(pinEcho2, INPUT);
  pinMode(pinTrig3, OUTPUT); pinMode(pinEcho3, INPUT);

  // 2. Conexión WiFi y Micro-ROS
  set_microros_wifi_transports(ssid, password, agent_ip, agent_port);

  Serial.println("--- Iniciando ESP32 Robot ---");
  Serial.print("Conectando WiFi ");
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); Serial.print(".");
  }
  Serial.println("\nWiFi OK.");
  digitalWrite(LED_PIN, HIGH); delay(1000);

  Serial.print("Conectando Agente...");
  allocator = rcl_get_default_allocator();
  while (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Reintentando Agente...");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(1000); 
    if (WiFi.status() != WL_CONNECTED) WiFi.reconnect();
  }
  Serial.println("Agente OK!");
  digitalWrite(LED_PIN, LOW); 

  // 3. Inicializar Nodos
  rclc_node_init_default(&node, "esp32_robot", "", &support);
  rclc_publisher_init_default(&pose_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D), "/robot_pose");
  rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  
  // 4. Hardware Motores
  ledcAttach(pwmPin_m1_A, freq, resolution); ledcAttach(pwmPin_m1_B, freq, resolution);
  pinMode(encPin_m1_A, INPUT); pinMode(encPin_m1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPin_m1_A), isr_m1_A, CHANGE); attachInterrupt(digitalPinToInterrupt(encPin_m1_B), isr_m1_B, CHANGE);

  ledcAttach(pwmPin_m2_A, freq, resolution); ledcAttach(pwmPin_m2_B, freq, resolution);
  pinMode(encPin_m2_A, INPUT); pinMode(encPin_m2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPin_m2_A), isr_m2_A, CHANGE); attachInterrupt(digitalPinToInterrupt(encPin_m2_B), isr_m2_B, CHANGE);

  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &ontimerISR);
  timerAlarm(timer, tiempo_muestreo * 1000000, true, 0);
}

// ---------------- LOOP PRINCIPAL ----------------
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // --- A. LECTURA DE SENSORES DE PROXIMIDAD ---
  // Se ejecuta cada 150ms para no saturar y no bloquear el bucle con pulseIn
  static unsigned long last_sonar_read = 0;
  if (millis() - last_sonar_read > 100) {
    last_sonar_read = millis();

    // 1. Leer Frontal
    dist_frontal = leerSonar(pinTrig1, pinEcho1);
    delay(10); // Pequeña pausa para evitar ecos cruzados

    // 2. Leer Izquierdo
    dist_izq = leerSonar(pinTrig2, pinEcho2);
    delay(10);

    // 3. Leer Derecho
    dist_der = leerSonar(pinTrig3, pinEcho3);

    // DEBUG: Ver distancias en Serial
    Serial.printf("F:%.0fcm L:%.0fcm R:%.0fcm\n", dist_frontal, dist_izq, dist_der);
  }


  // --- B. CÁLCULO DE REFERENCIAS PID (Cinemática Inversa) ---
  float v_rad_der = -(vel_lin + vel_ang * distancia_ruedas / 2.0) / radio_rueda;
  float v_rad_izq = -(vel_lin - vel_ang * distancia_ruedas / 2.0) / radio_rueda;

  noInterrupts(); 
  if(dist_der>15 and dist_izq>15 and dist_frontal>15 ){
    referencia_m1 = v_rad_der * (vueltas / (2.0 * M_PI)); 
    referencia_m2 = v_rad_izq * (vueltas / (2.0 * M_PI)); 
  }
  else{
    referencia_m1=0;
    referencia_m2=0;
  }
  
  // Copia de variables para publicar
  double x_pub = pos_x;
  double y_pub = pos_y;
  double th_pub = theta;
  interrupts();

  // --- C. PUBLICAR POSE ---
  static unsigned long last_pub = 0;
  if (millis() - last_pub > 50) { // 20Hz
    last_pub = millis();

    pose_msg.x = x_pub;
    pose_msg.y = y_pub;
    pose_msg.theta = th_pub;

    rcl_publish(&pose_publisher, &pose_msg, NULL);
  }
}