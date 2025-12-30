#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# BerryIMU: Inclinación + Heading con Kalman Filter
# Autor: Adaptado por Carlos

import sys
import time
import math
import datetime
import IMU

# =================== CONSTANTES =====================
RAD_TO_DEG = 57.29578
M_PI = 3.14159265358979323846
G_GAIN = 0.070          # [deg/s/LSB] ajuste según configuración del giroscopio
AA = 0.40               # Complementary filter constant (todavía usado para calcular tilt)
MAG_LPF_FACTOR = 0.4
ACC_LPF_FACTOR = 0.4
ACC_MEDIANTABLESIZE = 9
MAG_MEDIANTABLESIZE = 9

# ================== CALIBRACIÓN BRÚJULA ==================
magXmin = -510
magYmin = -245
magZmin = -746
magXmax = 1922
magYmax = 1614
magZmax = 650

# ================== VARIABLES KALMAN ==================
Q_angle = 0.02
Q_gyro = 0.0015
R_angle = 0.005
y_bias = x_bias = 0.0
XP_00 = XP_01 = XP_10 = XP_11 = 0.0
YP_00 = YP_01 = YP_10 = YP_11 = 0.0
KFangleX = KFangleY = 0.0

# ================== FUNCIONES KALMAN ====================
def kalmanFilterY(accAngle, gyroRate, DT):
    global KFangleY, y_bias, YP_00, YP_01, YP_10, YP_11
    KFangleY += DT * (gyroRate - y_bias)
    YP_00 += -DT * (YP_10 + YP_01) + Q_angle*DT
    YP_01 += -DT * YP_11
    YP_10 += -DT * YP_11
    YP_11 += Q_gyro*DT
    y = accAngle - KFangleY
    S = YP_00 + R_angle
    K_0 = YP_00 / S
    K_1 = YP_10 / S
    KFangleY += K_0 * y
    y_bias += K_1 * y
    YP_00 -= K_0 * YP_00
    YP_01 -= K_0 * YP_01
    YP_10 -= K_1 * YP_00
    YP_11 -= K_1 * YP_01
    return KFangleY

def kalmanFilterX(accAngle, gyroRate, DT):
    global KFangleX, x_bias, XP_00, XP_01, XP_10, XP_11
    KFangleX += DT * (gyroRate - x_bias)
    XP_00 += -DT * (XP_10 + XP_01) + Q_angle*DT
    XP_01 += -DT * XP_11
    XP_10 += -DT * XP_11
    XP_11 += Q_gyro*DT
    x = accAngle - KFangleX
    S = XP_00 + R_angle
    K_0 = XP_00 / S
    K_1 = XP_10 / S
    KFangleX += K_0 * x
    x_bias += K_1 * x
    XP_00 -= K_0 * XP_00
    XP_01 -= K_0 * XP_01
    XP_10 -= K_1 * XP_00
    XP_11 -= K_1 * XP_01
    return KFangleX

# =================== INICIALIZACIÓN ====================
IMU.detectIMU()
if IMU.BerryIMUversion == 99:
    print("No BerryIMU found... exiting")
    sys.exit()
IMU.initIMU()

# Variables para filtros low-pass y mediana
oldAcc = [0.0, 0.0, 0.0]
oldMag = [0.0, 0.0, 0.0]
acc_medianX = [1]*ACC_MEDIANTABLESIZE
acc_medianY = [1]*ACC_MEDIANTABLESIZE
acc_medianZ = [1]*ACC_MEDIANTABLESIZE
mag_medianX = [1]*MAG_MEDIANTABLESIZE
mag_medianY = [1]*MAG_MEDIANTABLESIZE
mag_medianZ = [1]*MAG_MEDIANTABLESIZE

# Ángulos giroscopio iniciales
gyroAngle = [0.0, 0.0, 0.0]

prevTime = datetime.datetime.now()

# =================== BUCLE PRINCIPAL ===================
while True:
    # ---------- Lectura sensores ----------
    ACCx = IMU.readACCx()
    ACCy = IMU.readACCy()
    ACCz = IMU.readACCz()
    GYRx = IMU.readGYRx()
    GYRy = IMU.readGYRy()
    GYRz = IMU.readGYRz()
    MAGx = IMU.readMAGx()
    MAGy = IMU.readMAGy()
    MAGz = IMU.readMAGz()

    # ---------- Calibración magnetómetro ----------
    MAGx -= (magXmin + magXmax)/2
    MAGy -= (magYmin + magYmax)/2
    MAGz -= (magZmin + magZmax)/2

    # ---------- Periodo de loop ----------
    currTime = datetime.datetime.now()
    LP = (currTime - prevTime).total_seconds()
    prevTime = currTime

    # ---------- Filtro low-pass ----------
    for i, val in enumerate([ACCx, ACCy, ACCz]):
        oldAcc[i] = val * ACC_LPF_FACTOR + oldAcc[i]*(1-ACC_LPF_FACTOR)
    ACCx, ACCy, ACCz = oldAcc

    for i, val in enumerate([MAGx, MAGy, MAGz]):
        oldMag[i] = val * MAG_LPF_FACTOR + oldMag[i]*(1-MAG_LPF_FACTOR)
    MAGx, MAGy, MAGz = oldMag

    # ---------- Filtro mediana ----------
    def median_filter(table, value):
        table.pop()
        table.insert(0, value)
        sorted_table = sorted(table)
        return sorted_table[len(sorted_table)//2]

    ACCx = median_filter(acc_medianX, ACCx)
    ACCy = median_filter(acc_medianY, ACCy)
    ACCz = median_filter(acc_medianZ, ACCz)
    MAGx = median_filter(mag_medianX, MAGx)
    MAGy = median_filter(mag_medianY, MAGy)
    MAGz = median_filter(mag_medianZ, MAGz)

    # ---------- Conversión giroscopio ----------
    rate_gyr_x = GYRx * G_GAIN
    rate_gyr_y = GYRy * G_GAIN
    rate_gyr_z = GYRz * G_GAIN

    gyroAngle[0] += rate_gyr_x*LP
    gyroAngle[1] += rate_gyr_y*LP
    gyroAngle[2] += rate_gyr_z*LP

    # ---------- Ángulos acelerómetro ----------
    AccXangle = math.atan2(ACCy, ACCz)*RAD_TO_DEG
    AccYangle = (math.atan2(ACCz, ACCx)+M_PI)*RAD_TO_DEG
    AccYangle = AccYangle-270 if AccYangle>90 else AccYangle+90

    # ---------- Kalman Filter ----------
    kalmanX = kalmanFilterX(AccXangle, rate_gyr_x, LP)
    kalmanY = kalmanFilterY(AccYangle, rate_gyr_y, LP)

    # ---------- Tilt-compensated Heading ----------
    accXnorm = ACCx / math.sqrt(ACCx**2 + ACCy**2 + ACCz**2)
    accYnorm = ACCy / math.sqrt(ACCx**2 + ACCy**2 + ACCz**2)
    pitch = math.asin(accXnorm)
    roll = -math.asin(accYnorm/math.cos(pitch))

    if IMU.BerryIMUversion in [1,3]:
        magXcomp = MAGx*math.cos(pitch)+MAGz*math.sin(pitch)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)-MAGz*math.sin(roll)*math.cos(pitch)
    else:
        magXcomp = MAGx*math.cos(pitch)-MAGz*math.sin(pitch)
        magYcomp = MAGx*math.sin(roll)*math.sin(pitch)+MAGy*math.cos(roll)+MAGz*math.sin(roll)*math.cos(pitch)

    tiltHeadingMag = 180*math.atan2(magYcomp, magXcomp)/M_PI
    if tiltHeadingMag < 0:
        tiltHeadingMag += 360

    # ---------- Ajuste a Norte verdadero ----------
    DECLINATION = -15.5  # declinación magnética en Paraguay (grados)
    tiltHeadingTrue = tiltHeadingMag + DECLINATION
    if tiltHeadingTrue < 0:
        tiltHeadingTrue += 360
    elif tiltHeadingTrue >= 360:
        tiltHeadingTrue -= 360

    # ---------- Salida ----------
    print(f"Kalman Inclination | X: {kalmanX:.2f}°, Y: {kalmanY:.2f}° | Heading True: {tiltHeadingTrue:.2f}°")

