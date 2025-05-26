#include <math.h>
#include <Adafruit_GPS.h>
#include <Wire.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

#define DEBUGSerial Serial

const float origin_lat = 47.987003;
const float origin_lon = -1.554807;
const float origin_alt = 82.80;

const float RADIUS_METERS = 500.0; // rayon du cercle (1 km de diametre)
const float EARTH_RADIUS = 6378137.0; // rayon moyen de la Terre en m

const unsigned long interval = 100; // ms
unsigned long last_time = 0;
float angle = 0.0;

float current_speed = 0.0; // m/s
const float max_speed_kmh = 400.0;
const float max_speed = max_speed_kmh / 3.6;
const float acceleration = 1.0; // m/s^2

void setup() {
  DEBUGSerial.begin(115200);
  while (!DEBUGSerial);
  delay(500);
  DEBUGSerial.println("lat,long,height,day,month,year,hour,minute,second,millis,pdop,speed_kmh,ori_x,ori_y,ori_z,gyro_x,gyro_y,gyro_z,acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,lin_x,lin_y,lin_z,grav_x,grav_y,grav_z,temp,cal_sys,cal_gyro,cal_accel,cal_mag");
}

void loop() {
  unsigned long now = millis();
  if (now - last_time < interval) return;
  unsigned long delta_t = now - last_time;
  last_time = now;

  // Mise à jour de la vitesse
  current_speed += acceleration * (delta_t / 1000.0);
  if (current_speed > max_speed) current_speed = 0;

  // Mise à jour de l'angle
  float delta_angle = current_speed / RADIUS_METERS * (delta_t / 1000.0); // radian = v / r * dt
  angle += delta_angle;
  if (angle >= 2 * PI) angle -= 2 * PI;

  float dLat = (RADIUS_METERS / EARTH_RADIUS) * cos(angle);
  float dLon = (RADIUS_METERS / (EARTH_RADIUS * cos(radians(origin_lat)))) * sin(angle);

  float lat = origin_lat + degrees(dLat);
  float lon = origin_lon + degrees(dLon);

  int day = 23, month = 5, year = 2025;
  int hour = 10, minute = 30, second = (now / 1000) % 60;
  int millisec = now % 1000;
  float speed_kmh = current_speed * 3.6;

  DEBUGSerial.print(lat, 6); DEBUGSerial.print(',');
  DEBUGSerial.print(lon, 6); DEBUGSerial.print(',');
  DEBUGSerial.print(origin_alt); DEBUGSerial.print(',');
  DEBUGSerial.print(day); DEBUGSerial.print(',');
  DEBUGSerial.print(month); DEBUGSerial.print(',');
  DEBUGSerial.print(year); DEBUGSerial.print(',');
  DEBUGSerial.print(hour); DEBUGSerial.print(',');
  DEBUGSerial.print(minute); DEBUGSerial.print(',');
  DEBUGSerial.print(second); DEBUGSerial.print(',');
  DEBUGSerial.print(millisec); DEBUGSerial.print(',');
  DEBUGSerial.print(1.0); DEBUGSerial.print(','); // PDOP fictif
  DEBUGSerial.print(speed_kmh, 2);

  for (int i = 0; i < 24; ++i) {
    DEBUGSerial.print(",0.00");
  }

  DEBUGSerial.print(",25,3,3,3,3");
  DEBUGSerial.println();
}
