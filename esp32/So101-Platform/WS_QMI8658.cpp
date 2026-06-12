#include "WS_QMI8658.h"

#define I2C_SDA 47
#define I2C_SCL 48

SensorQMI8658 QMI;

IMUdata Accel;
IMUdata Gyro;

float QMI8658_A_y;
float QMI8658_A_x;

void QMI8658_Init() {
  Wire.begin(I2C_SDA, I2C_SCL);
  if (!QMI.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("QMI8658 not found - check wiring");
    while (1) {
      delay(1000);
    }
  }

  QMI.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  QMI.configGyroscope(
      SensorQMI8658::GYR_RANGE_64DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_3);

  QMI.enableGyroscope();
  QMI.enableAccelerometer();
}

void QMI8658_Loop() {
  if (QMI.getDataReady()) {
    QMI.getAccelerometer(Accel.x, Accel.y, Accel.z);
    QMI.getGyroscope(Gyro.x, Gyro.y, Gyro.z);
  }
}

bool QMI8658_read(float &ax, float &ay, float &az, float &gx, float &gy, float &gz, float &tempC) {
  if (!QMI.getDataReady()) {
    return false;
  }

  bool ok = QMI.getAccelerometer(ax, ay, az);
  ok &= QMI.getGyroscope(gx, gy, gz);
  tempC = QMI.getTemperature_C();
  return ok;
}

String QMI8658_get_A_x() {
  if (QMI.getDataReady()) {
    if (QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
      QMI8658_A_x = Accel.x * 10.0;
    }
  }
  return String(QMI8658_A_x);
}

String QMI8658_get_A_y() {
  if (QMI.getDataReady()) {
    if (QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
      QMI8658_A_y = Accel.y * 10.0;
    }
  }
  return String(QMI8658_A_y);
}

float QMI8658_get_A_fx() {
  if (QMI.getDataReady()) {
    if (QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
      QMI8658_A_x = Accel.x * 10.0;
    }
  }
  return QMI8658_A_x;
}

float QMI8658_get_A_fy() {
  if (QMI.getDataReady()) {
    if (QMI.getAccelerometer(Accel.x, Accel.y, Accel.z)) {
      QMI8658_A_y = Accel.y * 10.0;
    }
  }
  return QMI8658_A_y;
}
