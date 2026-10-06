#include <Arduino.h>
#include <math.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// ---------------- CONFIGURACIÓN WIFI ----------------
const char* ssid = "iPhone de Arturo";
const char* password = "miaumiauu";

// ---------------- SERVIDOR WEB ----------------
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------- PÁGINA WEB ----------------
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
  <title>ESP32 Robot SLAM</title>
  <style>
    body { font-family: 'Segoe UI', sans-serif; text-align: center; margin: 0; background-color: #1a1a1a; color: white; touch-action: none; }
    
    #data-panel { display: flex; justify-content: space-around; background: #2a2a2a; padding: 10px; border-bottom: 2px solid #444; }
    .sensor-box { background: #333; padding: 5px; width: 30%; border-radius: 5px; }
    .val-label { font-weight: bold; color: #00ff88; font-size: 18px; display: block; }

    .main-container { display: flex; flex-wrap: wrap; justify-content: center; gap: 20px; padding: 20px; }
    .col { display: flex; flex-direction: column; align-items: center; gap: 10px; }
    .col-left { width: 260px; } .col-center { width: 320px; } .col-right { width: 240px; }

    #map-wrapper { position: relative; width: 320px; height: 320px; background: #000; border: 2px solid #555; border-radius: 10px; overflow: hidden; touch-action: none; }
    canvas { position: absolute; top: 0; left: 0; }
    #gridCanvas { z-index: 1; opacity: 0.9; }
    #robotCanvas { z-index: 2; }

    .nav-box { background: #2a2a2a; padding: 10px; border-radius: 8px; width: 100%; box-sizing: border-box; border: 1px solid #444; }
    .nav-title { font-size: 11px; color: #aaa; text-transform: uppercase; border-bottom: 1px solid #555; margin-bottom: 5px; }
    input { width: 50px; background: #444; border: 1px solid #666; color: white; text-align: center; border-radius: 4px; padding: 4px; }
    button { width: 100%; padding: 8px; border: none; border-radius: 4px; cursor: pointer; font-weight: bold; margin-top: 5px; background: #007bff; color: white; }
    button:active { background: #0056b3; }
    .stop-btn { background: #cc3333 !important; }
    .follow-btn { background: #8e44ad !important; margin-top: 15px; padding: 12px; }
    .finger-btn { background: #e67e22 !important; margin-top: 10px; }
    .active-mode { border: 2px solid #fff; box-shadow: 0 0 10px #fff; }

    .add-btn { width: auto; background: #28a745; padding: 2px 10px; }
    .del-btn { width: 20px; height: 20px; padding: 0; background: #dc3545; font-size: 12px; }

    #p-list { max-height: 150px; overflow-y: auto; display: flex; flex-direction: column; gap: 4px; }
    .p-row { display: flex; gap: 5px; align-items: center; justify-content: center; background: #333; padding: 2px; }

    #joy-cont { width: 200px; height: 200px; background: #333; border-radius: 50%; border: 4px solid #444; position: relative; }
    #joy { width: 60px; height: 60px; background: linear-gradient(145deg, #007bff, #0056b3); border-radius: 50%; position: absolute; top: 50%; left: 50%; transform: translate(-50%, -50%); }
    
    /* Indicador de mando */
    #gp-status { margin-top: 5px; font-size: 12px; color: #555; }
    .gp-active { color: #00ff88 !important; font-weight: bold; }
  </style>
</head>
<body>

  <div id="data-panel">
    <div class="sensor-box">Izq<br><span id="sl" class="val-label">--</span></div>
    <div class="sensor-box">Front<br><span id="sf" class="val-label">--</span></div>
    <div class="sensor-box">Der<br><span id="sr" class="val-label">--</span></div>
  </div>

  <div class="main-container">
    <div class="col col-left">
      <div class="nav-box">
        <div class="nav-title">Ir a Punto</div>
        X:<input id="ix" value="1.0" step="0.1"> Y:<input id="iy" value="0.0" step="0.1">
        <button onclick="sendG()">IR</button>
      </div>
      <div class="nav-box">
        <div class="nav-title">Circulo</div>
        R:<input id="ir" value="0.5" step="0.1"> m
        <button onclick="sendC()">INICIAR</button>
      </div>
      <div class="nav-box">
        <div class="nav-title">Trayectoria</div>
        <div id="p-list"></div>
        <div style="text-align:center; margin-top:5px;"><button class="add-btn" onclick="addP()">+</button></div>
        <button onclick="sendP()" style="background:#e67e22">EJECUTAR RUTA</button>
      </div>
      <button onclick="stop()" class="stop-btn">PARADA EMERGENCIA</button>
    </div>

    <div class="col col-center">
      <div id="map-wrapper">
        <canvas id="gridCanvas" width="320" height="320"></canvas>
        <canvas id="robotCanvas" width="320" height="320"></canvas>
      </div>
      <div style="font-family:monospace; color:#aaa; font-size:12px;">X:<span id="px">0</span> Y:<span id="py">0</span> Th:<span id="pt">0</span></div>
      
      <button id="btnFinger" onclick="toggleFinger()" class="finger-btn">MODO: SEGUIR DEDO (OFF)</button>
      <button onclick="clearM()" style="width:100px; font-size:10px; background:#555; margin-top:5px;">Borrar Mapa</button>
    </div>

    <div class="col col-right">
      <div id="joy-cont"><div id="joy"></div></div>
      <div id="gp-status">🎮 Mando: Desconectado</div>
      
      <div class="nav-box" style="margin-top:10px; padding:5px;">
        <span style="font-size:14px; font-weight:bold;">Vel Max:</span>
        <input id="max_v_in" type="number" value="0.4" step="0.05" style="width:60px;" onchange="updV()"> m/s
      </div>

      <button onclick="sendF()" class="follow-btn">SEGUIRME (SONAR)</button>
    </div>
  </div>

<script>
  const SENSORS = {
    F: {x: 0.16, y: 0.00, a: 0.0},                
    L: {x: 0.14, y: 0.055, a: 40 * Math.PI/180},   
    R: {x: 0.14, y:-0.055, a: -35 * Math.PI/180}   
  };

  var ws = new WebSocket(`ws://${window.location.hostname}/ws`);
  
  var gC = document.getElementById("gridCanvas").getContext("2d");
  var rC = document.getElementById("robotCanvas").getContext("2d");
  var mapWrap = document.getElementById("map-wrapper");
  var w=320, h=320, scale=50, cX=w/2, cY=h/2;
  
  var grid = {}; 
  var gSize = 0.05; 
  var path = [], pCount=0;
  var maxVel = 0.4;

  var fingerMode = false;
  var lastFingerSend = 0;

  function init() {
    addP(0.5, 0.0); addP(1.0, 0.5);
    clearM();
    ws.onmessage = (e) => {
      var d = JSON.parse(e.data);
      updateUI(d);
      updateGrid(d.x, d.y, d.th, d.s_f, SENSORS.F);
      updateGrid(d.x, d.y, d.th, d.s_l, SENSORS.L);
      updateGrid(d.x, d.y, d.th, d.s_r, SENSORS.R);
      drawScene(d.x, d.y, d.th);
    };
    // INICIAR BUCLE DEL GAMEPAD
    setInterval(updateGamepad, 50);
  }

  // --- LOGICA GAMEPAD (PS4) ---
  function updateGamepad() {
      var gamepads = navigator.getGamepads();
      var gp = gamepads[0]; // Cogemos el primer mando

      var status = document.getElementById('gp-status');
      if (gp) {
          if(!status.classList.contains('gp-active')) {
              status.innerText = "🎮 Mando: CONECTADO";
              status.classList.add('gp-active');
          }

          // LECTURA DE EJES (PS4 ESTANDAR)
          // Axis 1: Stick Izq Vertical (-1 Arriba, +1 Abajo)
          // Axis 0: Stick Izq Horizontal (-1 Izq, +1 Der)
          
          var yRaw = gp.axes[1];
          var xRaw = gp.axes[0];

          // Deadzone (zona muerta) para evitar drift
          if (Math.abs(yRaw) < 0.1) yRaw = 0;
          if (Math.abs(xRaw) < 0.1) xRaw = 0;

          // BOTONES
          // Boton 0: X (Cross) -> PARADA
          if (gp.buttons[0].pressed) {
              stop();
              return;
          }
          // Boton 5: R1 -> TURBO (Sobreescribe maxVel)
          var currentMax = maxVel;
          if (gp.buttons[5].pressed) currentMax = 0.6;

          // CALCULO DE VELOCIDADES
          // Invertimos Y porque el mando da -1 arriba, y queremos +velocidad
          var vLin = -yRaw * currentMax; 
          // Invertimos X porque derecha es +1, y el robot gira negativo a derecha
          var vAng = -xRaw * 1.5; 

          // ENVIAR SOLO SI HAY MOVIMIENTO (O SI ESTABAMOS MOVIENDONOS)
          // Usamos un pequeño umbral para no saturar el wifi con ceros
          if (Math.abs(vLin) > 0 || Math.abs(vAng) > 0) {
              if (ws.readyState == 1) ws.send(`V,${vLin.toFixed(2)},${vAng.toFixed(2)}`);
          } else {
             // Si soltamos el mando, mandamos un par de paradas para asegurar
             // (No implementado para no saturar, el PID del robot se encarga si no recibe comandos)
          }

      } else {
          status.innerText = "🎮 Mando: Desconectado (Pulsa un botón)";
          status.classList.remove('gp-active');
      }
  }

  function toggleFinger() {
      fingerMode = !fingerMode;
      var btn = document.getElementById('btnFinger');
      if(fingerMode) {
          btn.innerText = "SEGUIR DEDO (ON)";
          btn.classList.add("active-mode");
      } else {
          btn.innerText = "MODO: SEGUIR DEDO (OFF)";
          btn.classList.remove("active-mode");
          ws.send("V,0,0"); 
      }
  }

  mapWrap.addEventListener('mousedown', handleMapInput);
  mapWrap.addEventListener('mousemove', handleMapInput);
  mapWrap.addEventListener('touchstart', handleMapInput);
  mapWrap.addEventListener('touchmove', handleMapInput);

  function handleMapInput(e) {
      if(!fingerMode) return;
      if(e.type === 'mousemove' && e.buttons === 0) return;
      e.preventDefault(); 
      var rect = mapWrap.getBoundingClientRect();
      var clientX = e.touches ? e.touches[0].clientX : e.clientX;
      var clientY = e.touches ? e.touches[0].clientY : e.clientY;
      var canvasX = clientX - rect.left;
      var canvasY = clientY - rect.top;
      var clickY_m = (cX - canvasX) / scale;
      var clickX_m = (cY - canvasY) / scale;
      var now = Date.now();
      if(now - lastFingerSend > 150) { 
          ws.send(`G,${clickX_m.toFixed(2)},${clickY_m.toFixed(2)}`);
          lastFingerSend = now;
      }
  }

  function updV() {
    var v = parseFloat(document.getElementById('max_v_in').value);
    if(isNaN(v) || v < 0.1) v = 0.1;
    maxVel = v;
    ws.send("S," + maxVel); 
  }

  function updateGrid(rx, ry, rth, dist_cm, sOff) {
    if(dist_cm > 250 || dist_cm <= 0) return; 
    var dist_m = dist_cm / 100.0;
    var angulo_mapa = rth; 
    var cosT = Math.cos(angulo_mapa);
    var sinT = Math.sin(angulo_mapa);
    var sx = rx + (sOff.x * cosT - sOff.y * sinT);
    var sy = ry + (sOff.x * sinT + sOff.y * cosT);
    var base_sth = angulo_mapa + sOff.a;

    var offsets = [0];
    if (dist_cm > 150) { offsets = [-0.05, 0, 0.05]; }

    for (var k = 0; k < offsets.length; k++) {
        var sth = base_sth + offsets[k];
        var ox = sx + dist_m * Math.cos(sth);
        var oy = sy + dist_m * Math.sin(sth);
        var steps = Math.ceil(dist_m / gSize);
        for(var i=0; i<=steps; i++) {
            var t = i/steps;
            var cx = sx + (ox-sx)*t;
            var cy = sy + (oy-sy)*t;
            var gx = Math.round(cx/gSize), gy = Math.round(cy/gSize);
            var key = gx+","+gy;
            if(grid[key]===undefined) grid[key]=0;
            if(i==steps) { grid[key] += 3; if(grid[key] > 10) grid[key] = 10; } 
            else { grid[key] -= 1; if(grid[key] < 0) grid[key] = 0; }
            drawCell(gx, gy, grid[key]);
        }
    }
  }

  function drawCell(gx, gy, val) {
      var scX = cX - (gy * gSize * scale);
      var scY = cY - (gx * gSize * scale);
      var sz = gSize * scale;
      if(val > 5) { gC.fillStyle = `rgba(0, 255, 0, ${val/10})`; gC.fillRect(scX, scY, sz, sz); } 
      else { gC.clearRect(scX, scY, sz, sz); }
  }

  function drawScene(rx, ry, rth) {
      rC.clearRect(0,0,w,h);
      rC.save(); rC.translate(cX, cY);
      if(path.length==0 || Math.abs(rx-path[path.length-1].x)>0.05) path.push({x:rx, y:ry});
      rC.beginPath(); rC.strokeStyle="rgba(255,255,0,0.5)"; rC.lineWidth=2;
      for(let p of path) rC.lineTo(-p.y*scale, -p.x*scale); rC.stroke();
      rC.translate(-ry*scale, -rx*scale);
      rC.rotate(-rth); 
      rC.fillStyle="#00d2ff"; rC.fillRect(-10,-10,20,20); 
      rC.strokeStyle="red"; rC.lineWidth=3; 
      rC.beginPath(); rC.moveTo(0,0); rC.lineTo(0, -20); rC.stroke(); 
      rC.restore();
  }

  function updateUI(d) {
      document.getElementById('sf').innerText=d.s_f.toFixed(0);
      document.getElementById('sl').innerText=d.s_l.toFixed(0);
      document.getElementById('sr').innerText=d.s_r.toFixed(0);
      document.getElementById('px').innerText=d.x.toFixed(2);
      document.getElementById('py').innerText=d.y.toFixed(2);
      document.getElementById('pt').innerText=d.th.toFixed(2);
  }

  function sendG() { ws.send(`G,${id('ix').value},${id('iy').value}`); }
  function sendC() { ws.send(`C,${id('ir').value}`); }
  function sendF() { ws.send("F"); }
  function stop() { ws.send("V,0,0"); toggleFinger(); if(fingerMode) toggleFinger(); }
  function clearM() { grid={}; gC.clearRect(0,0,w,h); } 
  
  function sendP() {
      var rows = document.getElementsByClassName('p-row');
      let cmd="P";
      for(let r of rows) {
          let ins = r.getElementsByTagName('input');
          cmd += `,${ins[0].value},${ins[1].value}`;
      }
      ws.send(cmd);
  }

  function addP(x=0,y=0) {
      pCount++;
      var d = document.createElement('div'); d.className='p-row'; d.id='pr-'+pCount;
      d.innerHTML=`<span style='color:#00d2ff;width:15px'>${pCount}</span> X:<input value='${x}' step='0.1'> Y:<input value='${y}' step='0.1'> <button class='del-btn' onclick='remP(${pCount})'>x</button>`;
      id('p-list').appendChild(d);
  }
  function remP(i) { id('pr-'+i).remove(); }
  function id(i) { return document.getElementById(i); }

  var jC = id("joy-cont"), joy = id("joy"), drag=false;
  jC.addEventListener('touchstart', sD); jC.addEventListener('mousedown', sD);
  document.addEventListener('touchmove', dD); document.addEventListener('mousemove', dD);
  document.addEventListener('touchend', eD); document.addEventListener('mouseup', eD);

  function sD(e) { drag=true; }
  function eD(e) { drag=false; joy.style.top="50%"; joy.style.left="50%"; if(ws.readyState==1) ws.send("V,0,0"); }
  
  function dD(e) {
      if(!drag) return; e.preventDefault();
      var cx = e.touches?e.touches[0].clientX:e.clientX;
      var cy = e.touches?e.touches[0].clientY:e.clientY;
      var rect = jC.getBoundingClientRect();
      var dx = cx - (rect.left+rect.width/2), dy = cy - (rect.top+rect.height/2);
      var dist = Math.sqrt(dx*dx+dy*dy);
      if(dist>80) { dx=(dx/dist)*80; dy=(dy/dist)*80; }
      joy.style.left=(50+dx/rect.width*100)+"%"; joy.style.top=(50+dy/rect.height*100)+"%";
      if(ws.readyState==1) ws.send(`V,${(-(dy/80)*maxVel).toFixed(2)},${(-(dx/80)*1.5).toFixed(2)}`);
  }

  window.onload = init;
</script>
</body>
</html>
)rawliteral";



// ---------------- CONFIGURACIÓN HARDWARE ----------------
const int pwmPin_m1_A = 19;  const int pwmPin_m1_B = 18;
const int encPin_m1_A = 23;  const int encPin_m1_B = 22;
const int pwmPin_m2_A = 25;  const int pwmPin_m2_B = 26;
const int encPin_m2_A = 16;  const int encPin_m2_B = 17;
const int pinTrig1 = 14; const int pinEcho1 = 12; 
const int pinTrig2 = 33; const int pinEcho2 = 35; 
const int pinTrig3 = 32; const int pinEcho3 = 34; 
const int LED_PIN = 2; 

const float distancia_ruedas = 0.21; 
const float radio_rueda = 0.0335;       
const int vueltas = 1500;             
const int freq = 1000; const int resolution = 10;
hw_timer_t *timer = NULL;
const double tiempo_muestreo = 0.01; 

volatile long contador_m1 = 0; long cont_ant_m1 = 0; double vel_filtrada_m1 = 0.0, error_ant_m1 = 0.0, error_acum_m1 = 0.0, deriv_filt_m1 = 0.0, referencia_m1 = 0.0; 
volatile long contador_m2 = 0; long cont_ant_m2 = 0; double vel_filtrada_m2 = 0.0, error_ant_m2 = 0.0, error_acum_m2 = 0.0, deriv_filt_m2 = 0.0, referencia_m2 = 0.0; 
double Kp = 0.17661, Ki = 5.0, Kd = 0.005; 
double alpha_vel = 0.7, alpha_derivada = 0.7; 

float vel_ang = 0, vel_lin = 0;
double theta = 0.0, pos_x = 0.0, pos_y = 0.0;
float dist_frontal = 0.0, dist_izq = 0.0, dist_der = 0.0;

// Variables Navegación
int nav_mode = 0; 
double target_x = 0.0, target_y = 0.0; 
double circle_radius = 0.0;
double circle_total_turned = 0.0;
double prev_theta_circle = 0.0;

double max_lin_vel = 0.4; 

#define MAX_PATH_POINTS 50
double path_x[MAX_PATH_POINTS];
double path_y[MAX_PATH_POINTS];
int path_index = 0;
int path_len = 0;

unsigned long last_command_time = 0;

// ---------------- INTERRUPCIONES ----------------
void IRAM_ATTR isr_m1_A(){ if (digitalRead(encPin_m1_A) == digitalRead(encPin_m1_B)){ contador_m1++; } else{ contador_m1--; }}
void IRAM_ATTR isr_m1_B(){ if (digitalRead(encPin_m1_A) == digitalRead(encPin_m1_B)){ contador_m1--; } else{ contador_m1++; }}
void IRAM_ATTR isr_m2_A(){ if (digitalRead(encPin_m2_A) == digitalRead(encPin_m2_B)){ contador_m2++; } else{ contador_m2--; }}
void IRAM_ATTR isr_m2_B(){ if (digitalRead(encPin_m2_A) == digitalRead(encPin_m2_B)){ contador_m2--; } else{ contador_m2++; }}

// ------------ Para leer los sensores-------------
float leerSonar(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10); digitalWrite(trigPin, LOW);
  long duracion = pulseIn(echoPin, HIGH, 25000); 
  if (duracion == 0) return 400.0; 
  return (duracion * 0.0343) / 2.0;
}

// ---------------- CALCULA EL PID ----------------
double calcula_PID (double velocidad_input, double valor_buscado, double &error_ant, double &error_acum, double &deriv_filt){
  if (valor_buscado == 0 && abs(velocidad_input) < 10) { error_acum = 0; return 0; }
  double error = valor_buscado - velocidad_input;
  double derivada_raw = (error - error_ant) / tiempo_muestreo;
  deriv_filt = alpha_derivada * deriv_filt + (1.0 - alpha_derivada) * derivada_raw;
  error_acum += error * tiempo_muestreo;
  if (error_acum > 4000) error_acum = 4000; if (error_acum < -4000) error_acum = -4000;
  double pwm = Kp * error + Ki * error_acum + Kd * deriv_filt;
  error_ant = error; 
  if (pwm > 1023) pwm = 1023; else if (pwm < -1023) pwm = -1023;
  return pwm;
}

// ------------- ACTUA EN LOS MOTORES -------------
void P3_actua(int pwmPin, int pwmPin2, double valor_pwm_){
 if (valor_pwm_>=0) { ledcWrite(pwmPin, (int)valor_pwm_); ledcWrite(pwmPin2, 0); } 
 else { ledcWrite(pwmPin2, (int)(-1*valor_pwm_)); ledcWrite(pwmPin, 0); }
}

// --- TEMPORIZADOR PARA ACTUALIZAR LAS CUENTAS ---
void IRAM_ATTR ontimerISR(){
  double dif_m1 = contador_m1 - cont_ant_m1; vel_filtrada_m1 = alpha_vel * vel_filtrada_m1 + (1.0 - alpha_vel) * (dif_m1 / tiempo_muestreo); cont_ant_m1 = contador_m1;
  double dif_m2 = contador_m2 - cont_ant_m2; vel_filtrada_m2 = alpha_vel * vel_filtrada_m2 + (1.0 - alpha_vel) * (dif_m2 / tiempo_muestreo); cont_ant_m2 = contador_m2;

  double v_der = (vel_filtrada_m1 / (double)vueltas) * 2.0 * M_PI * radio_rueda;
  double v_izq = (vel_filtrada_m2 / (double)vueltas) * 2.0 * M_PI * radio_rueda;
  double v_robot = (v_der + v_izq) / 2.0;
  double w_robot = (v_der - v_izq) / distancia_ruedas;
  double delta_theta = w_robot * tiempo_muestreo;
  
  theta -= delta_theta;
  if (theta > M_PI) theta -= 2.0 * M_PI; if (theta < -M_PI) theta += 2.0 * M_PI;
  pos_x -= v_robot * cos(theta - (delta_theta / 2.0)) * tiempo_muestreo;
  pos_y -= v_robot * sin(theta - (delta_theta / 2.0)) * tiempo_muestreo;

  float pwm_der = calcula_PID(vel_filtrada_m1, referencia_m1, error_ant_m1, error_acum_m1, deriv_filt_m1);
  float pwm_izq = calcula_PID(vel_filtrada_m2, referencia_m2, error_ant_m2, error_acum_m2, deriv_filt_m2);
  P3_actua(pwmPin_m1_A, pwmPin_m1_B, pwm_der);
  P3_actua(pwmPin_m2_A, pwmPin_m2_B, pwm_izq);
}

//Esto establece la comunicación web-robot. Básicamente su objetivo es determinar qué comandos se han pulsado en la pagina web
//y traducirlos a ordenes directas al robot, configurando variables como nav_mode, que es el modo de navegacion
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) {
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
      data[len] = 0; char* msg = (char*)data;
      if (msg[0] == 'V') {
        nav_mode = 0; 
        char* token = strtok(msg, ","); token = strtok(NULL, ","); if (token) vel_lin = atof(token);
        token = strtok(NULL, ","); if (token) vel_ang = atof(token);
        last_command_time = millis(); 
      }
      else if (msg[0] == 'S') {
        char* token = strtok(msg, ","); token = strtok(NULL, ","); 
        if (token) max_lin_vel = atof(token);
      }
      else if (msg[0] == 'G') {
        char* token = strtok(msg, ","); token = strtok(NULL, ","); if (token) target_x = atof(token);
        token = strtok(NULL, ","); if (token) target_y = atof(token);
        nav_mode = 1; 
      }
      else if (msg[0] == 'C') {
        char* token = strtok(msg, ","); token = strtok(NULL, ","); if (token) circle_radius = atof(token);
        if (circle_radius == 0) circle_radius = 0.1;
        nav_mode = 2; circle_total_turned = 0.0; noInterrupts(); prev_theta_circle = theta; interrupts();
      }
      else if (msg[0] == 'P') {
        char* token = strtok(msg, ",");
        int idx = 0;
        while(idx < MAX_PATH_POINTS) {
           token = strtok(NULL, ","); if(token == NULL) break; path_x[idx] = atof(token);
           token = strtok(NULL, ","); if(token == NULL) break; path_y[idx] = atof(token);
           idx++;
        }
        path_len = idx; path_index = 0; nav_mode = 3; 
      }
      else if (msg[0] == 'F') { nav_mode = 4; }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  //Para los sensores ultrasonido
  pinMode(pinTrig1, OUTPUT); pinMode(pinEcho1, INPUT);
  pinMode(pinTrig2, OUTPUT); pinMode(pinEcho2, INPUT);
  pinMode(pinTrig3, OUTPUT); pinMode(pinEcho3, INPUT);

  //Para las PWM
  ledcAttach(pwmPin_m1_A, freq, resolution); ledcAttach(pwmPin_m1_B, freq, resolution);
  pinMode(encPin_m1_A, INPUT); pinMode(encPin_m1_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPin_m1_A), isr_m1_A, CHANGE); attachInterrupt(digitalPinToInterrupt(encPin_m1_B), isr_m1_B, CHANGE);

  ledcAttach(pwmPin_m2_A, freq, resolution); ledcAttach(pwmPin_m2_B, freq, resolution);
  pinMode(encPin_m2_A, INPUT); pinMode(encPin_m2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(encPin_m2_A), isr_m2_A, CHANGE); attachInterrupt(digitalPinToInterrupt(encPin_m2_B), isr_m2_B, CHANGE);

  //Configuración wifi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { digitalWrite(LED_PIN, !digitalRead(LED_PIN)); delay(200); }
  digitalWrite(LED_PIN, HIGH); Serial.println(WiFi.localIP());

  //Para el servidor web
  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ request->send_P(200, "text/html", index_html); });
  server.begin();

  //Para el temporizador
  timer = timerBegin(1000000); 
  timerAttachInterrupt(timer, &ontimerISR);
  timerAlarm(timer, tiempo_muestreo * 1000000, true, 0);
}

void loop() {
  ws.cleanupClients(); 

  // --- CONTROL ---
  
  if (nav_mode == 0) { // MANUAL, JOYSTICK WEB O MANDO
    if (millis() - last_command_time > 500) { vel_lin = 0; vel_ang = 0; }
  }
  
  else if (nav_mode == 1) { // VE AL PUNTO X E Y
    noInterrupts(); double cx = pos_x, cy = pos_y, ct = theta; interrupts();
    double dx = target_x - cx; double dy = target_y - cy;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (distance < 0.015) { nav_mode = 0; vel_lin = 0; vel_ang = 0; } 
    else {
      double target_theta = atan2(dy, dx); double err_theta = target_theta - ct;
      while(err_theta > M_PI) err_theta -= 2.0 * M_PI; while(err_theta < -M_PI) err_theta += 2.0 * M_PI;
      vel_ang = 2.5 * err_theta; 
      if(vel_ang > 1.0) vel_ang = 1.0; if(vel_ang < -1.0) vel_ang = -1.0;
      if (abs(err_theta) < 0.35) { 
        if(distance > 0.4) vel_lin = max_lin_vel; 
        else vel_lin = max_lin_vel / 2.0; 
      } else { vel_lin = 0.0; }
    }
  }

  else if (nav_mode == 2) { // HAZ UN CIRCULO
      noInterrupts(); double current_th = theta; interrupts();
      double d_theta = current_th - prev_theta_circle;
      while(d_theta > M_PI) d_theta -= 2.0 * M_PI; while(d_theta < -M_PI) d_theta += 2.0 * M_PI;
      circle_total_turned += abs(d_theta); prev_theta_circle = current_th;
      if (circle_total_turned >= 2.0 * M_PI * 0.98) { nav_mode = 0; vel_lin = 0; vel_ang = 0; }
      else { 
        vel_ang = 0.2*M_PI; 
        vel_lin = min(vel_ang*circle_radius,max_lin_vel); 
        if (abs(vel_ang) > 1.5) vel_ang = (vel_ang > 0) ? 1.5 : -1.5; 
      }
  }

  else if (nav_mode == 3) { // FOLLOW THE CARROT
    if (path_index >= path_len) { nav_mode = 0; vel_lin = 0; vel_ang = 0; } 
    else {
        double cur_tx = path_x[path_index]; double cur_ty = path_y[path_index];
        noInterrupts(); double cx = pos_x, cy = pos_y, ct = theta; interrupts();
        double dx = cur_tx - cx; double dy = cur_ty - cy;
        double distance = sqrt(dx*dx + dy*dy);
        double tolerance = 0.05; if (path_index == path_len - 1) tolerance = 0.03;
        if (distance < tolerance) { path_index++; } 
        else {
           double target_theta = atan2(dy, dx); double err_theta = target_theta - ct;
           while(err_theta > M_PI) err_theta -= 2.0 * M_PI; while(err_theta < -M_PI) err_theta += 2.0 * M_PI;
           vel_ang = 2.5 * err_theta; 
           if(vel_ang > 1.0) vel_ang = 1.0; if(vel_ang < -1.0) vel_ang = -1.0;
           if (abs(err_theta) < 0.35) { 
             if(distance > 0.4) vel_lin = max_lin_vel; 
             else vel_lin = max_lin_vel / 2.0; 
           } else { vel_lin = 0.0; }
        }
    }
  }

  else if (nav_mode == 4) { // SIGUEME 
    float target_dist = 30.0; float deadband = 5.0; 
    if (dist_izq < 50 && dist_der > 50) vel_ang = 0.8; 
    else if (dist_der < 50 && dist_izq > 50) vel_ang = -0.8; 
    else vel_ang = 0.0;

    if (dist_frontal > 0 && dist_frontal < 100) { 
       float error = dist_frontal - target_dist;
       if (abs(error) > deadband) {
          vel_lin = error * 0.005; 
          if (vel_lin > max_lin_vel) vel_lin = max_lin_vel; 
          if (vel_lin < -max_lin_vel) vel_lin = -max_lin_vel;
       } else vel_lin = 0;
    } else { if(vel_ang == 0) vel_lin = 0; }
  }

  // --- SENSORES & WS ---
  //Leen los sensores y representan su valor en la web, asi como pintar sobre el mapa
  static unsigned long last_sonar = 0;
  if (millis() - last_sonar > 150) {
    last_sonar = millis();
    dist_frontal = leerSonar(pinTrig1, pinEcho1); delay(5);
    dist_izq = leerSonar(pinTrig2, pinEcho2); delay(5);
    dist_der = leerSonar(pinTrig3, pinEcho3);
    char json_msg[150];
    noInterrupts(); double cx = pos_x, cy = pos_y, ct = theta; interrupts();
    snprintf(json_msg, sizeof(json_msg), 
      "{\"s_f\":%.0f,\"s_l\":%.0f,\"s_r\":%.0f,\"x\":%.2f,\"y\":%.2f,\"th\":%.2f}",
      dist_frontal, dist_izq, dist_der, cx, cy, ct);
    ws.textAll(json_msg);
  }

  // --- SEGURIDAD Y ACTUACION ---
  float v_linear_safe = vel_lin;
  //El robot se para INDEPENDIENTEMENTE DE SU MODO si hay un obstaculo delante (evita choques)
  if (nav_mode != 4 && dist_frontal < 15.0 && dist_frontal > 0 && vel_lin > 0) { v_linear_safe = 0; }
  
  float v_rad_der = -(v_linear_safe + vel_ang * distancia_ruedas / 2.0) / radio_rueda;
  float v_rad_izq = -(v_linear_safe - vel_ang * distancia_ruedas / 2.0) / radio_rueda;

  //Desactivo las interrupciones mientras ajusto la referencia a los motores para que no cambie el numero de cuentas mientras calcula su PWM
  noInterrupts();
  referencia_m1 = v_rad_der * (vueltas / (2.0 * M_PI)); 
  referencia_m2 = v_rad_izq * (vueltas / (2.0 * M_PI)); 
  interrupts();
}