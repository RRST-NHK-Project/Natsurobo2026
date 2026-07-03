// MPU9250 / MPU6500 + ESP32
// 段差検知・動的再キャリブレーション
// imu.ino (CUBE型3軸姿勢制御) のアルゴリズムをMPU9250向けに移植
// ROS2 serial_bridge向けASCIIシリアル出力

#include <Wire.h>

// ---- I2Cアドレス (AD0=GND → 0x68, AD0=3.3V → 0x69) ----
#define MPU_ADDR      0x68

// ---- レジスタ ----
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_CFG   0x1C
#define REG_GYRO_CFG    0x1B
#define REG_DLPF_CFG    0x1A
#define REG_ACCEL_OUT   0x3B  // Accel X/Y/Z, Temp, Gyro X/Y/Z (各2byte = 14byte)

// ---- スケール設定 ----
// ACCEL: ±8G → LSB = 4096 counts/G  (REG_ACCEL_CFG = 0x10)
// GYRO:  ±500dps → LSB = 65.536 counts/dps  (REG_GYRO_CFG = 0x08)
// ジャイロのscale定数: π/180 × 1000 = 17.4533
//   = (1[deg/s] / 65.536[LSB/dps]) × (π/180) × 65536
#define GYRO_SCALE_FACTOR  17.45329252f

// ---- パラメータ (要調整) ----
#define CAL_SAMPLES       2048   // 起動キャリブレーションサンプル数 (~2秒)
#define STABLE_N          30     // 安定判定ウィンドウ (サンプル数)
#define STABLE_THRESH     300    // 安定判定分散しきい値 (LSB^2, 小さいほど厳しい)
#define STEP_THRESH       1500   // 段差検知しきい値 (水平加速度の絶対値, LSB)
#define STEP_COOLDOWN_MS  500    // 段差後の待機時間 (ms)

// ---- union型 (imu.inoと同じ固定小数点方式) ----
// B32.X : 32bit精度で積分・演算
// B16.X : B32.Xの上位16bit → 実際のセンサー値スケール
typedef union {
  struct { int32_t X, Y, Z; } B32;
  struct { int16_t XL, X, YL, Y, ZL, Z; } B16;
} VEC3;

// ---- グローバル変数 ----
VEC3 accRaw, accRawLPF, gyroRaw;
VEC3 Vg;   // 重力ベクトル (姿勢の核心)

int32_t accCal[3]  = {};
int32_t gyroCal[3] = {};

float angle[3] = {};       // [0]=ROLL, [1]=PITCH, [2]=YAW (deg)
float ROLL_ADJUST  = 0.0f;
float PITCH_ADJUST = 0.0f;

// ---- 段差検知用 ----
int16_t stabBuf[STABLE_N][3];
int     stabIdx    = 0;
bool    stepActive = false;
uint32_t stepTime  = 0;

// ---- タイマー ----
uint32_t loopTimer = 0;
uint32_t angTimer  = 0;


// ================================================================
// ハードウェア低レイヤー
// ================================================================

void writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// キャリブレーション前の生読み取り (オフセット未適用)
void readRaw(int16_t a[3], int16_t g[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(REG_ACCEL_OUT);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14);
  while (Wire.available() < 14) {}
  a[0] = Wire.read() << 8 | Wire.read();  // Accel X
  a[1] = Wire.read() << 8 | Wire.read();  // Accel Y
  a[2] = Wire.read() << 8 | Wire.read();  // Accel Z
  Wire.read(); Wire.read();               // Temp (捨てる)
  g[0] = Wire.read() << 8 | Wire.read();  // Gyro X
  g[1] = Wire.read() << 8 | Wire.read();  // Gyro Y
  g[2] = Wire.read() << 8 | Wire.read();  // Gyro Z
}


// ================================================================
// MPU9250/6500 初期設定
// ================================================================

void setupMPU9250() {
  writeReg(REG_PWR_MGMT_1, 0x80);  // デバイスリセット
  delay(100);
  writeReg(REG_PWR_MGMT_1, 0x01);  // クロック: ジャイロX軸PLL (精度向上)
  delay(100);
  writeReg(REG_DLPF_CFG,   0x03);  // DigitalLPF ~41Hz (遅延 5.9ms)
  writeReg(REG_ACCEL_CFG,  0x10);  // ±8G, LSB=4096
  writeReg(REG_GYRO_CFG,   0x08);  // ±500dps, LSB=65.536
  delay(50);

  // 接続確認 (WHO_AM_I: MPU9250=0x71, MPU6500=0x70)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
  uint8_t whoAmI = Wire.read();
  Serial.print("STATUS:WHO_AM_I=0x");
  Serial.println(whoAmI, HEX);
  if (whoAmI != 0x71 && whoAmI != 0x70) {
    Serial.println("STATUS:WARNING_UNEXPECTED_DEVICE_ID");
  }
}


// ================================================================
// 起動キャリブレーション
// 【重要】呼び出し時は機体を水平・静止させること
// ================================================================

void calibrateIMU() {
  Serial.println("STATUS:CALIBRATING_KEEP_STILL");
  int32_t aSum[3] = {}, gSum[3] = {};
  int16_t a[3], g[3];

  for (int i = 0; i < CAL_SAMPLES; i++) {
    readRaw(a, g);
    aSum[0] += a[0]; aSum[1] += a[1]; aSum[2] += a[2];
    gSum[0] += g[0]; gSum[1] += g[1]; gSum[2] += g[2];
    delay(1);
  }

  accCal[0]  = aSum[0] / CAL_SAMPLES;
  accCal[1]  = aSum[1] / CAL_SAMPLES;
  accCal[2]  = aSum[2] / CAL_SAMPLES - 4096; // Z軸: 1G (=4096LSB) を除く
  gyroCal[0] = gSum[0] / CAL_SAMPLES;
  gyroCal[1] = gSum[1] / CAL_SAMPLES;
  gyroCal[2] = gSum[2] / CAL_SAMPLES;

  Serial.println("STATUS:CALIBRATED");
  Serial.printf("CAL_OFFSET: acc=[%d,%d,%d] gyro=[%d,%d,%d]\n",
    (int)accCal[0], (int)accCal[1], (int)accCal[2],
    (int)gyroCal[0], (int)gyroCal[1], (int)gyroCal[2]);
}


// ================================================================
// メイン読み取り (キャリブレーション適用 + LPF)
// ================================================================

void readMPU9250() {
  int16_t a[3], g[3];
  readRaw(a, g);

  // オフセット引き算 → B16.X にセット (= B32.X の上位16bit)
  accRaw.B16.X = a[0] - (int16_t)accCal[0];
  accRaw.B16.Y = a[1] - (int16_t)accCal[1];
  accRaw.B16.Z = a[2] - (int16_t)accCal[2];
  gyroRaw.B16.X = g[0] - (int16_t)gyroCal[0];
  gyroRaw.B16.Y = g[1] - (int16_t)gyroCal[1];
  gyroRaw.B16.Z = g[2] - (int16_t)gyroCal[2];

  // 指数移動平均LPF (α = 1/4 = >>2)
  accRawLPF.B32.X += (accRaw.B32.X - accRawLPF.B32.X) >> 2;
  accRawLPF.B32.Y += (accRaw.B32.Y - accRawLPF.B32.Y) >> 2;
  accRawLPF.B32.Z += (accRaw.B32.Z - accRawLPF.B32.Z) >> 2;
}


// ================================================================
// atan2 近似 (imu.inoから流用, 4次多項式近似)
// 戻り値: 角度 × 100 (単位: 0.01度, 例: 4500 = 45.00度)
// ================================================================

int16_t _atan2(int16_t _y, int16_t _x) {
  int16_t x = abs(_x), y = abs(_y);
  float z;
  bool c = (y < x);
  if (c) z = (float)y / x;
  else   z = (float)x / y;

  int16_t a = z * (z * (z * (829*z - 2011) - 58) + 5741);

  if (c) {
    if (_x > 0) { if (_y < 0) a *= -1; }
    if (_x < 0) { if (_y > 0) a = 18000 - a; else a = a - 18000; }
  } else {
    if (_x > 0) { if (_y > 0) a = 9000 - a; else a = a - 9000; }
    if (_x < 0) { if (_y > 0) a = a + 9000; else a = -a - 9000; }
  }
  return a;
}


// ================================================================
// 姿勢角計算 (imu.inoの手法を移植)
//
// アルゴリズム: 重力ベクトル Vg を姿勢状態として保持
//   ① ジャイロ積分: Vg を小角度近似で回転 (短期の信頼源)
//   ② 加速度補正:  実際の加速度ベクトルで Vg を引き戻す (長期の信頼源)
//   ③ atan2 で ROLL / PITCH を算出
// ================================================================

void computeAngle() {
  uint32_t now = micros();
  float dt2 = (now - angTimer) * 1e-6f;
  angTimer = now;

  readMPU9250();

  // ジャイロ → 回転量 (32bit固定小数点スケール)
  // sinθ ≈ θ の近似: 小角度での行列回転に使う
  int32_t sX = (int32_t)(gyroRaw.B16.X * GYRO_SCALE_FACTOR * dt2);
  int32_t sY = (int32_t)(gyroRaw.B16.Y * GYRO_SCALE_FACTOR * dt2);
  int32_t sZ = (int32_t)(gyroRaw.B16.Z * GYRO_SCALE_FACTOR * dt2);

  // ① Vg をジャイロで回転
  //    Vg' = R(ω) × Vg  (小角度近似の3×3回転行列)
  VEC3 Vg0;
  Vg0.B32.X =  Vg.B32.X + Vg.B16.Y * sZ - Vg.B16.Z * sY;
  Vg0.B32.Y = -Vg.B16.X * sZ + Vg.B32.Y + Vg.B16.Z * sX;
  Vg0.B32.Z =  Vg.B16.X * sY - Vg.B16.Y * sX + Vg.B32.Z;
  Vg = Vg0;

  // ② 加速度で Vg を補正 (α = 1/16 = >>4)
  //    加速度は長期的に必ず重力方向を指すのでドリフト補正になる
  Vg.B32.X += (accRawLPF.B32.X - Vg.B32.X) >> 4;
  Vg.B32.Y += (accRawLPF.B32.Y - Vg.B32.Y) >> 4;
  Vg.B32.Z += (accRawLPF.B32.Z - Vg.B32.Z) >> 4;

  // ③ atan2 で角度算出 (0.01度単位 → 度に変換)
  angle[0] =  _atan2(Vg.B16.Y, Vg.B16.Z) * 0.01f;                    // ROLL
  angle[1] = -_atan2(Vg.B16.X, sqrtf((float)Vg.B16.Y * Vg.B16.Y
                                    + (float)Vg.B16.Z * Vg.B16.Z)) * 0.01f; // PITCH
  angle[2] += -gyroRaw.B16.Z * (1.0f / 65.536f) * dt2;               // YAW (積分のみ)
}


// ================================================================
// 段差検知 + 動的再キャリブレーション
//
// 仕組み:
//   段差を越えると水平加速度が急増 → stepActive = true
//   その後静止すると Vg を現在の加速度でリセット
//   → 新しい「水平」基準として再定義される
// ================================================================

void updateStabBuf() {
  stabBuf[stabIdx][0] = accRaw.B16.X;
  stabBuf[stabIdx][1] = accRaw.B16.Y;
  stabBuf[stabIdx][2] = accRaw.B16.Z;
  stabIdx = (stabIdx + 1) % STABLE_N;
}

bool isStable() {
  int32_t mX = 0, mY = 0;
  for (int i = 0; i < STABLE_N; i++) {
    mX += stabBuf[i][0];
    mY += stabBuf[i][1];
  }
  mX /= STABLE_N;
  mY /= STABLE_N;

  int32_t var = 0;
  for (int i = 0; i < STABLE_N; i++) {
    int32_t dx = stabBuf[i][0] - mX;
    int32_t dy = stabBuf[i][1] - mY;
    var += dx*dx + dy*dy;
  }
  return (var / STABLE_N) < STABLE_THRESH;
}

void detectAndRecalibrate() {
  updateStabBuf();

  // 水平方向 (X+Y) の加速度合計で段差を検知
  bool shake = (abs(accRaw.B16.X) + abs(accRaw.B16.Y)) > STEP_THRESH;

  if (shake && !stepActive) {
    stepActive = true;
    stepTime   = millis();
    Serial.println("STATUS:STEP_DETECTED");
  }

  if (stepActive) {
    uint32_t elapsed = millis() - stepTime;

    // 十分時間が経ち、かつ安定している → 再キャリブレーション
    if (!shake && elapsed > STEP_COOLDOWN_MS && isStable()) {
      // Vg を現在の加速度方向でリセット (新しい水平基準)
      Vg.B16.X = accRawLPF.B16.X;
      Vg.B16.Y = accRawLPF.B16.Y;
      Vg.B16.Z = accRawLPF.B16.Z;

      // 角度オフセットを更新 (段差前後のズレを吸収)
      ROLL_ADJUST  = -angle[0];
      PITCH_ADJUST = -angle[1];

      stepActive = false;
      Serial.println("STATUS:RECALIBRATED");
      Serial.printf("OFFSET_UPDATED: roll=%.2f pitch=%.2f\n",
        ROLL_ADJUST, PITCH_ADJUST);
    }
  }
}


// ================================================================
// setup / loop
// ================================================================

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);       // SDA=GPIO21, SCL=GPIO22
  Wire.setClock(400000);    // 400kHz (Fast Mode)
  delay(500);

  setupMPU9250();
  calibrateIMU();

  // Vg を最初の加速度値で初期化
  readMPU9250();
  Vg.B16.X = accRawLPF.B16.X;
  Vg.B16.Y = accRawLPF.B16.Y;
  Vg.B16.Z = accRawLPF.B16.Z;

  angTimer  = micros();
  loopTimer = micros();

  Serial.println("STATUS:READY");
}

void loop() {
  uint32_t now = micros();
  if (now - loopTimer < 10000) return;  // 10ms = 100Hz
  loopTimer = now;

  computeAngle();
  detectAndRecalibrate();

  float roll  = angle[0] + ROLL_ADJUST;
  float pitch = angle[1] + PITCH_ADJUST;
  float yaw   = angle[2];

  // ---- ROS2 serial_bridge 向け出力 ----
  // フォーマット: IMU:<roll>,<pitch>,<yaw>,<step_flag>
  //   step_flag: 0=通常, 1=段差検知中
  Serial.print("IMU:");
  Serial.print(roll,  2);
  Serial.print(",");
  Serial.print(pitch, 2);
  Serial.print(",");
  Serial.print(yaw,   2);
  Serial.print(",");
  Serial.println(stepActive ? 1 : 0);
}
