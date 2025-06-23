#include <Arduino.h>

// 基本型定義
typedef unsigned char u8;
typedef short s16;
typedef unsigned short u16;

// ピン定義
#define PIN_EN 33

// 命令定義
#define INST_SYNC_WRITE 0x83 // 同期書き込み命令（STS_SyncWritePos用）
#define INST_REG_WRITE 0x04  // 非同期書き込み命令
#define INST_REG_ACTION 0x05 // 非同期実行命令

// SMS/STSレジスタ定義
#define SMS_STS_ACC 41
#define SMS_STS_TORQUE_ENABLE 40

// 最大パケットサイズ定義
#define MAX_PACKET_SIZE 256

// 位置値の定数定義
#define POSITION_MIN 0
#define POSITION_MAX 4095

// サーボレジスタアドレス定義
#define REG_ID 5
#define REG_CURRENT_POSITION 56
#define REG_VOLTAGE 62
#define REG_LOCK 55

// コマンド定義
#define CMD_PING 1
#define CMD_READ 2
#define CMD_WRITE 3

// 電圧範囲定義
#define VOLTAGE_MIN 50  // 5.0V
#define VOLTAGE_MAX 140 // 14.0V

// グローバル変数
HardwareSerial *servoSerial = &Serial1; // 使用するシリアルポート
// u8 responseLevel = 0;                   // 応答不要（送信専用モード）
// unsigned long ioTimeout = 100;
u8 targetServoID = 1;  // ターゲットサーボID
u8 syncServoCount = 6; // [x]コマンドで同時制御するサーボ数（デフォルト6個）

// エンディアン変換関数（SMS/STS用：ビッグエンディアン）
void Host2SMS(u8 *DataL, u8 *DataH, u16 Data)
{
  *DataH = (Data >> 8);   // 上位バイト
  *DataL = (Data & 0xff); // 下位バイト
}

// シリアル送信関数
void writeServo(u8 data)
{
  servoSerial->write(data);
}

void writeServo(u8 *data, int length)
{
  servoSerial->write(data, length);
}

void flushServo()
{
  servoSerial->flush();
  delay(5); // 送信完了確保
}

// 受信バッファフラッシュ
void flushReceiveBuffer()
{
  while (servoSerial->read() != -1)
    ;
}

// チェックサム計算
u8 calcChecksum(u8 *data, int length)
{
  u8 checksum = 0;
  for (int i = 2; i < length - 1; i++)
  {
    checksum += data[i];
  }
  return ~checksum;
}

// 半二重通信用コマンド送信（読み取り用）
void sendCommand(u8 *command, int length)
{
  // 受信バッファクリア
  flushReceiveBuffer();

  // 送信モードに切り替え
  digitalWrite(PIN_EN, HIGH);
  delayMicroseconds(30);

  // データ送信
  servoSerial->write(command, length);
  servoSerial->flush();

  // 受信モードに切り替え
  digitalWrite(PIN_EN, LOW);
  delayMicroseconds(50);
}

// 応答受信
bool receiveResponse(u8 *response, int expectedLength, unsigned long timeout)
{
  unsigned long startTime = millis();
  int bytesReceived = 0;

  while (bytesReceived < expectedLength && (millis() - startTime) < timeout)
  {
    if (servoSerial->available())
    {
      response[bytesReceived] = servoSerial->read();
      bytesReceived++;
    }
  }

  // 受信完了後、送信専用モードに戻す（元の設定に復帰）
  digitalWrite(PIN_EN, HIGH);
  delayMicroseconds(30);

  return bytesReceived == expectedLength;
}

// STS_SyncWritePos実装（一括送信最適化版）
bool STS_SyncWritePos(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  // 入力チェック
  if (IDN == 0 || IDN > 255)
  {
    Serial.println("Error: Invalid servo count");
    return false;
  }

  // 各サーボのデータサイズ = 7バイト (ACC + Position + Time + Speed)
  const u8 dataPerServo = 7;
  const u8 messageLength = ((dataPerServo + 1) * IDN + 4); // +1はID, +4は基本ヘッダ

  // パケット全体のサイズ計算（ヘッダ7バイト + データ部 + チェックサム1バイト）
  const u16 totalPacketSize = 7 + (dataPerServo + 1) * IDN + 1;

  if (totalPacketSize > MAX_PACKET_SIZE)
  {
    Serial.println("Error: Packet size too large");
    return false;
  }

  Serial.print("SyncWrite: Preparing ");
  Serial.print(IDN);
  Serial.print(" servos, packet size: ");
  Serial.print(totalPacketSize);
  Serial.println(" bytes");

  // パケットバッファ準備
  u8 packet[MAX_PACKET_SIZE];
  u16 packetIndex = 0;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xfe; // ブロードキャストID
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = INST_SYNC_WRITE;
  packet[packetIndex++] = SMS_STS_ACC;  // 開始レジスタアドレス
  packet[packetIndex++] = dataPerServo; // 各サーボのデータ長

  // チェックサム計算開始
  u8 checksum = 0xfe + messageLength + INST_SYNC_WRITE + SMS_STS_ACC + dataPerServo;

  // 各サーボのIDとデータをパケットに追加
  for (u8 i = 0; i < IDN; i++)
  {
    s16 pos = Position[i];

    // 負の位置の処理（SMS/STS仕様）
    if (pos < 0)
    {
      pos = -pos;
      pos |= (1 << 15); // 方向ビットを設定
    }

    // 速度とACCの取得（NULLチェック付き）
    u16 speed = Speed ? Speed[i] : 0;
    u8 acc = ACC ? ACC[i] : 0;

    // サーボIDを追加
    packet[packetIndex++] = ID[i];
    checksum += ID[i];

    // 7バイトのデータを追加
    packet[packetIndex++] = acc; // 加速度
    checksum += acc;

    // 位置データ（2バイト）
    u8 posL, posH;
    Host2SMS(&posL, &posH, pos);
    packet[packetIndex++] = posL;
    packet[packetIndex++] = posH;
    checksum += posL + posH;

    // 時間データ（2バイト、0固定）
    packet[packetIndex++] = 0;
    packet[packetIndex++] = 0;
    // チェックサムには0を加算（変化なし）

    // 速度データ（2バイト）
    u8 speedL, speedH;
    Host2SMS(&speedL, &speedH, speed);
    packet[packetIndex++] = speedL;
    packet[packetIndex++] = speedH;
    checksum += speedL + speedH;

    // 角度計算（位置値を角度に変換）
    float angle = ((float)Position[i] / 4096.0) * 360.0 - 180.0;

    Serial.print("  ID ");
    Serial.print(ID[i]);
    Serial.print(": Pos=");
    Serial.print(angle, 1);
    Serial.print("deg (");
    Serial.print(Position[i]);
    Serial.print("), Speed=");
    Serial.print(speed);
    Serial.print(", ACC=");
    Serial.println(acc);
  }

  // チェックサムを追加
  packet[packetIndex++] = ~checksum;

  // パケット全体を一括送信
  Serial.print("Sending packet: ");
  Serial.print(packetIndex);
  Serial.println(" bytes");

  // 半二重制御での送信
  flushReceiveBuffer();
  digitalWrite(PIN_EN, HIGH);
  delayMicroseconds(30);

  servoSerial->write(packet, packetIndex);
  servoSerial->flush();

  // 送信専用モードを維持（SyncWriteは応答なし）
  delayMicroseconds(50);

  Serial.println("SyncWrite packet sent successfully");
  return true;
}

// トルクイネーブル（半二重通信対応版）
bool EnableTorque(u8 ID, u8 Enable)
{
  // パケットバッファ準備
  u8 packet[8]; // 最大8バイト
  u8 packetIndex = 0;
  u8 messageLength = 4;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = ID;
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = CMD_WRITE;             // 通常のWRITE命令を使用
  packet[packetIndex++] = SMS_STS_TORQUE_ENABLE; // レジスタアドレス40

  // データ追加
  packet[packetIndex++] = Enable; // 1=トルクオン, 0=トルクオフ

  // チェックサム計算と追加
  u8 checksum = ID + messageLength + CMD_WRITE + SMS_STS_TORQUE_ENABLE + Enable;
  packet[packetIndex++] = ~checksum;

  // 半二重制御での送信
  flushReceiveBuffer();
  digitalWrite(PIN_EN, HIGH);
  delayMicroseconds(30);

  // パケット送信
  servoSerial->write(packet, packetIndex);
  servoSerial->flush();

  // 送信専用モードを維持
  delayMicroseconds(50);

  return true;
}

// 汎用パケット構築・送信関数
bool sendPacket(u8 targetID, u8 instruction, u8 startAddress, u8 *data, u8 dataLength)
{
  // パケットサイズ計算
  u8 messageLength = dataLength ? (dataLength + 3) : 2;
  u8 totalSize = dataLength ? (6 + dataLength + 1) : (5 + 1);

  if (totalSize > MAX_PACKET_SIZE)
  {
    Serial.println("Error: Packet too large");
    return false;
  }

  // パケットバッファ準備
  u8 packet[MAX_PACKET_SIZE];
  u8 packetIndex = 0;

  // ヘッダ構築
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = targetID;
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = instruction;

  // アドレスとデータ（存在する場合）
  u8 checksum = targetID + messageLength + instruction;
  if (data && dataLength > 0)
  {
    packet[packetIndex++] = startAddress;
    checksum += startAddress;

    for (u8 i = 0; i < dataLength; i++)
    {
      packet[packetIndex++] = data[i];
      checksum += data[i];
    }
  }

  // チェックサム追加
  packet[packetIndex++] = ~checksum;

  // 一括送信
  writeServo(packet, packetIndex);
  flushServo();

  return true;
}

// ★ ヘルパー関数: 角度から位置値への変換
s16 angleToPosition(float angle)
{
  // 角度範囲チェック（-180° 〜 180°）
  if (angle < -180.0 || angle > 180.0)
  {
    return -1; // エラー値
  }

  // 角度を位置値に変換（-180°〜180° → 0-4095）
  // 0° = 2048 (中央), -180° = 0, 180° = 4095
  float normalizedAngle = angle + 180.0; // -180°〜180° を 0°〜360° に変換
  s16 position = (s16)((normalizedAngle / 360.0) * 4096.0);
  if (position < POSITION_MIN)
    position = POSITION_MIN;
  if (position > POSITION_MAX)
    position = POSITION_MAX;

  return position;
}

// // 設定関数
// void setServoSerial(HardwareSerial *serial)
// {
//   servoSerial = serial;
// }

// void setIOTimeout(unsigned long timeout)
// {
//   ioTimeout = timeout;
// }

// void setResponseLevel(u8 level)
// {
//   responseLevel = level;
// }

// サーボ設定
// u8 armIDs[] = {1, 2, 3, 4, 5, 6};
// s16 ServoPositions[] = {0, 0, 0, 0, 0, 0};
// u16 ServoSpeeds[] = {0, 0, 0, 0, 0, 0};
// u8 ServoACCs[] = {0, 0, 0, 0, 0, 0};

// 往復移動用変数
bool currentDirection = true; // true: 4000へ, false: 500へ
unsigned long lastMoveTime = 0;
const unsigned long MOVE_INTERVAL = 2000; // 2秒間隔

// 往復移動の実行（最適化版）
void executeReciprocationSync()
{
  unsigned long currentTime = millis();

  if (currentTime - lastMoveTime >= MOVE_INTERVAL)
  {
    // 目標位置を決定
    s16 targetPosition = currentDirection ? 4000 : 500;

    Serial.print("Moving to ");
    Serial.print(currentDirection ? "MAX" : "MIN");
    Serial.print(" position: ");
    Serial.println(targetPosition);

    // アクティブなサーボ（ID1,2）の設定
    u8 activeIDs[] = {1, 2};
    s16 activePositions[] = {targetPosition, targetPosition};
    u16 activeSpeeds[] = {1000, 1000};
    u8 activeACCs[] = {50, 50};

    // STS_SyncWritePosで一括送信（最適化版）
    STS_SyncWritePos(activeIDs, 2, activePositions, activeSpeeds, activeACCs);

    // 方向切り替え
    currentDirection = !currentDirection;
    lastMoveTime = currentTime;

    Serial.print("Next move in ");
    Serial.print(MOVE_INTERVAL / 1000);
    Serial.println(" seconds");
    Serial.println();
  }
}

// [r]コマンド: サーボ位置読み取り
void executeReadPositionCommand()
{
  Serial.print("Reading position from servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_CURRENT_POSITION, 2, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（8バイト期待）
  u8 response[8];
  if (receiveResponse(response, 8, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      u16 position = (response[6] << 8) | response[5];
      float angle = ((float)position / 4096.0) * 360.0 - 180.0;

      Serial.print("Position: ");
      Serial.print(position);
      Serial.print(" (");
      Serial.print(angle, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("Invalid response received");
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [v]コマンド: サーボ電圧読み取り
void executeReadVoltageCommand()
{
  Serial.print("Reading voltage from servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, REG_VOLTAGE, 1, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（7バイト期待）
  u8 response[7];
  if (receiveResponse(response, 7, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      u8 voltage = response[5];
      float voltageV = voltage / 10.0;

      Serial.print("Voltage: ");
      Serial.print(voltage);
      Serial.print(" (");
      Serial.print(voltageV, 1);
      Serial.print("V)");

      if (voltage < VOLTAGE_MIN)
      {
        Serial.println(" [Warning] Low voltage");
      }
      else if (voltage > VOLTAGE_MAX)
      {
        Serial.println(" [Warning] High voltage");
      }
      else
      {
        Serial.println(" [OK]");
      }
    }
    else
    {
      Serial.println("Invalid response received");
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [z]コマンド: ターゲットID変更
void executeChangeTargetIdCommand()
{
  Serial.print("Current target ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter new target ID (1-253): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  int newId = input.toInt();

  // ID範囲チェック
  if (newId < 1 || newId > 253)
  {
    Serial.println("Error: ID out of range (1-253)");
    return;
  }

  targetServoID = newId;
  Serial.print("Target ID changed to: ");
  Serial.println(targetServoID);
}

// [n]コマンド: 同時制御サーボ数設定
void executeSetSyncServoCountCommand()
{
  Serial.print("Current sync servo count: ");
  Serial.println(syncServoCount);
  Serial.println("Enter sync servo count (1-255): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  int newCount = input.toInt();

  // サーボ数範囲チェック
  if (newCount < 1 || newCount > 255)
  {
    Serial.println("Error: Sync servo count out of range (1-255)");
    return;
  }

  syncServoCount = newCount;
  Serial.print("Sync servo count changed to: ");
  Serial.print(syncServoCount);
  Serial.println(" (ID1 to ID" + String(syncServoCount) + ")");
}

// [a]コマンド: 全サーボ位置読み出し
void executeReadAllPositionsCommand()
{
  Serial.print("Reading positions from ");
  Serial.print(syncServoCount);
  Serial.println(" servos...");
  Serial.println("ID\tPosition\tAngle(°)");
  Serial.println("-------------------------");

  u8 connectedCount = 0;

  for (u8 id = 1; id <= syncServoCount; id++)
  {
    // 位置読み取りコマンド構築
    u8 command[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_CURRENT_POSITION, 2, 0};
    command[7] = calcChecksum(command, 8);

    // コマンド送信
    sendCommand(command, 8);

    // 応答受信（8バイト期待）
    u8 response[8];
    if (receiveResponse(response, 8, 100))
    {
      // 応答検証
      if (response[0] == 0xFF && response[1] == 0xFF && response[2] == id)
      {
        u16 position = (response[6] << 8) | response[5];
        float angle = ((float)position / 4096.0) * 360.0 - 180.0;

        Serial.print(id);
        Serial.print("\t");
        Serial.print(position);
        Serial.print("\t\t");
        Serial.print(angle, 1);
        Serial.println("°");

        connectedCount++;
      }
      else
      {
        Serial.print(id);
        Serial.println("\t[Error] Invalid response");
      }
    }
    else
    {
      Serial.print(id);
      Serial.println("\t[Error] No response");
    }

    delay(10); // 次の読み取りまで少し待機
  }

  Serial.println("-------------------------");
  Serial.print("Connected servos: ");
  Serial.print(connectedCount);
  Serial.print("/");
  Serial.println(syncServoCount);

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [c]コマンド: 全サーボ位置を0にリセット
void executeResetAllPositionsCommand()
{
  Serial.print("Resetting ");
  Serial.print(syncServoCount);
  Serial.println(" servos to position 0 (center)...");

  // 動的配列で同時制御サーボ数に対応
  u8 *allIDs = new u8[syncServoCount];
  s16 *allPositions = new s16[syncServoCount];
  u16 *allSpeeds = new u16[syncServoCount];
  u8 *allACCs = new u8[syncServoCount];

  // 配列を初期化（全て位置0 = 中央位置2048）
  for (u8 i = 0; i < syncServoCount; i++)
  {
    allIDs[i] = i + 1;      // ID1から順番
    allPositions[i] = 2048; // 中央位置（0度）
    allSpeeds[i] = 800;     // 少し遅めの速度
    allACCs[i] = 30;        // 低い加速度
  }

  STS_SyncWritePos(allIDs, syncServoCount, allPositions, allSpeeds, allACCs);

  Serial.println("All servos reset to center position");

  // メモリ解放
  delete[] allIDs;
  delete[] allPositions;
  delete[] allSpeeds;
  delete[] allACCs;
}

// [f]コマンド: サーボ認識
void executeServoDetectionCommand()
{
  Serial.println("Enter maximum ID to scan (1-253): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  int maxId = input.toInt();

  // ID範囲チェック
  if (maxId < 1 || maxId > 253)
  {
    Serial.println("Error: ID out of range (1-253)");
    return;
  }

  Serial.print("Scanning servos from ID1 to ID");
  Serial.print(maxId);
  Serial.println("...");
  Serial.println();

  u8 foundServos[254]; // 最大253個のサーボ
  u8 foundCount = 0;

  // プログレス表示用
  int progressStep = maxId / 10;
  if (progressStep == 0)
    progressStep = 1;

  for (u8 id = 1; id <= maxId; id++)
  {
    // プログレス表示
    if (id % progressStep == 0 || id == maxId)
    {
      Serial.print("Progress: ");
      Serial.print((id * 100) / maxId);
      Serial.println("%");
    }

    // PINGコマンド構築
    u8 command[6] = {0xFF, 0xFF, id, 2, CMD_PING, 0};
    command[5] = calcChecksum(command, 6);

    // コマンド送信
    sendCommand(command, 6);

    // 応答受信（6バイト期待）
    u8 response[6];
    if (receiveResponse(response, 6, 30)) // 短いタイムアウト
    {
      // 応答検証
      if (response[0] == 0xFF && response[1] == 0xFF && response[2] == id)
      {
        foundServos[foundCount] = id;
        foundCount++;
        Serial.print("Found servo ID: ");
        Serial.println(id);
      }
    }

    delay(5); // 次のスキャンまで少し待機
  }

  // 結果表示
  Serial.println();
  Serial.println("=== Servo Detection Results ===");
  Serial.print("Scanned range: ID1 to ID");
  Serial.println(maxId);
  Serial.print("Found ");
  Serial.print(foundCount);
  Serial.println(" servo(s)");

  if (foundCount > 0)
  {
    Serial.print("Connected IDs: ");
    for (u8 i = 0; i < foundCount; i++)
    {
      Serial.print(foundServos[i]);
      if (i < foundCount - 1)
        Serial.print(", ");
    }
    Serial.println();

    // 同時制御サーボ数の提案
    if (foundCount <= 255)
    {
      Serial.println();
      Serial.print("Suggestion: Use 'n' command to set sync servo count to ");
      Serial.print(foundCount);
      Serial.println(" for optimal control");
    }
  }
  else
  {
    Serial.println("No servos detected");
  }

  Serial.println();

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [e]コマンド: 全サーボトルクオン
void executeEnableTorqueAllCommand()
{
  Serial.print("Enabling torque for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (ID1-ID");
  Serial.print(syncServoCount);
  Serial.println(")...");

  u8 successCount = 0;

  for (u8 id = 1; id <= syncServoCount; id++)
  {
    if (EnableTorque(id, 1))
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Torque enabled");
      successCount++;
    }
    else
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Failed to enable torque");
    }
    delay(50); // 各サーボ間で少し待機
  }

  Serial.print("Torque enabled for ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(syncServoCount);
  Serial.println(" servos");
}

// [d]コマンド: 全サーボトルクオフ
void executeDisableTorqueAllCommand()
{
  Serial.print("Disabling torque for ");
  Serial.print(syncServoCount);
  Serial.print(" servos (ID1-ID");
  Serial.print(syncServoCount);
  Serial.println(")...");

  u8 successCount = 0;

  for (u8 id = 1; id <= syncServoCount; id++)
  {
    if (EnableTorque(id, 0))
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Torque disabled");
      successCount++;
    }
    else
    {
      Serial.print("ID ");
      Serial.print(id);
      Serial.println(": Failed to disable torque");
    }
    delay(50); // 各サーボ間で少し待機
  }

  Serial.print("Torque disabled for ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(syncServoCount);
  Serial.println(" servos");
}

// [p]コマンド: 指定アドレスのレジスタ値読み取り
void executeReadRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (0-255): ");

  // アドレス入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String addressInput = Serial.readString();
  addressInput.trim();

  int address = addressInput.toInt();

  // アドレス範囲チェック
  if (address < 0 || address > 255)
  {
    Serial.println("Error: Address out of range (0-255)");
    return;
  }

  Serial.println("Enter data length to read (1-4 bytes): ");

  // データ長入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String lengthInput = Serial.readString();
  lengthInput.trim();

  int dataLength = lengthInput.toInt();

  // データ長範囲チェック
  if (dataLength < 1 || dataLength > 4)
  {
    Serial.println("Error: Data length out of range (1-4)");
    return;
  }

  Serial.print("Reading ");
  Serial.print(dataLength);
  Serial.print(" byte(s) from address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  // 読み取りコマンド構築
  u8 command[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, (u8)address, (u8)dataLength, 0};
  command[7] = calcChecksum(command, 8);

  // コマンド送信
  sendCommand(command, 8);

  // 応答受信（ヘッダ5バイト + データ + チェックサム1バイト）
  u8 expectedLength = 5 + dataLength + 1;
  u8 response[10]; // 最大10バイト

  if (receiveResponse(response, expectedLength, 100))
  {
    // 応答検証
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      Serial.print("Register ");
      Serial.print(address);
      Serial.print(" (0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.print("): ");

      // データ部分を表示
      if (dataLength == 1)
      {
        u8 value = response[5];
        Serial.print("Dec=");
        Serial.print(value);
        Serial.print(", Hex=0x");
        if (value < 16)
          Serial.print("0");
        Serial.print(value, HEX);
        Serial.print(", Bin=");
        Serial.println(value, BIN);
      }
      else if (dataLength == 2)
      {
        u16 value = (response[6] << 8) | response[5]; // ビッグエンディアン
        Serial.print("Dec=");
        Serial.print(value);
        Serial.print(", Hex=0x");
        if (value < 0x1000)
          Serial.print("0");
        if (value < 0x100)
          Serial.print("0");
        if (value < 0x10)
          Serial.print("0");
        Serial.print(value, HEX);
        Serial.print(", Bin=");
        Serial.println(value, BIN);
      }
      else
      {
        // 3-4バイトの場合は個別に表示
        Serial.print("Raw bytes: ");
        for (int i = 0; i < dataLength; i++)
        {
          u8 value = response[5 + i];
          Serial.print("0x");
          if (value < 16)
            Serial.print("0");
          Serial.print(value, HEX);
          if (i < dataLength - 1)
            Serial.print(" ");
        }
        Serial.println();
      }
    }
    else
    {
      Serial.println("Invalid response received");
      Serial.print("Response header: ");
      for (int i = 0; i < 3; i++)
      {
        Serial.print("0x");
        if (response[i] < 16)
          Serial.print("0");
        Serial.print(response[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
    }
  }
  else
  {
    Serial.println("No response from servo");
  }

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [s]コマンド: 指定アドレスにレジスタ値書き込み
void executeWriteRegisterCommand()
{
  Serial.print("Current target servo ID: ");
  Serial.println(targetServoID);
  Serial.println("Enter register address (0-255): ");

  // アドレス入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String addressInput = Serial.readString();
  addressInput.trim();

  int address = addressInput.toInt();

  // アドレス範囲チェック
  if (address < 0 || address > 255)
  {
    Serial.println("Error: Address out of range (0-255)");
    return;
  }

  Serial.println("Enter data length to write (1-2 bytes): ");

  // データ長入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String lengthInput = Serial.readString();
  lengthInput.trim();

  int dataLength = lengthInput.toInt();

  // データ長範囲チェック
  if (dataLength < 1 || dataLength > 2)
  {
    Serial.println("Error: Data length out of range (1-2)");
    return;
  }

  Serial.print("Enter decimal value to write (");
  if (dataLength == 1)
  {
    Serial.print("0-255");
  }
  else
  {
    Serial.print("0-65535");
  }
  Serial.println("): ");

  // 値入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String valueInput = Serial.readString();
  valueInput.trim();

  long value = valueInput.toInt();

  // 値範囲チェック
  if (dataLength == 1 && (value < 0 || value > 255))
  {
    Serial.println("Error: Value out of range (0-255) for 1 byte");
    return;
  }
  else if (dataLength == 2 && (value < 0 || value > 65535))
  {
    Serial.println("Error: Value out of range (0-65535) for 2 bytes");
    return;
  }

  Serial.print("Writing ");
  Serial.print(value);
  Serial.print(" (0x");
  if (value < 16)
    Serial.print("0");
  Serial.print(value, HEX);
  Serial.print(") to address ");
  Serial.print(address);
  Serial.print(" of servo ID ");
  Serial.println(targetServoID);

  // 書き込みコマンド構築
  u8 command[10]; // 最大10バイト
  u8 commandIndex = 0;
  u8 messageLength = 3 + dataLength;

  // ヘッダ構築
  command[commandIndex++] = 0xFF;
  command[commandIndex++] = 0xFF;
  command[commandIndex++] = targetServoID;
  command[commandIndex++] = messageLength;
  command[commandIndex++] = CMD_WRITE;
  command[commandIndex++] = (u8)address;

  // データ追加
  if (dataLength == 1)
  {
    command[commandIndex++] = (u8)value;
  }
  else // dataLength == 2
  {
    // ビッグエンディアンで送信
    command[commandIndex++] = (u8)(value & 0xFF);        // 下位バイト
    command[commandIndex++] = (u8)((value >> 8) & 0xFF); // 上位バイト
  }

  // チェックサム計算
  u8 checksum = targetServoID + messageLength + CMD_WRITE + (u8)address;
  if (dataLength == 1)
  {
    checksum += (u8)value;
  }
  else
  {
    checksum += (u8)(value & 0xFF);
    checksum += (u8)((value >> 8) & 0xFF);
  }
  command[commandIndex++] = ~checksum;

  // 半二重制御での送信
  flushReceiveBuffer();
  digitalWrite(PIN_EN, HIGH);
  delayMicroseconds(30);

  // コマンド送信
  servoSerial->write(command, commandIndex);
  servoSerial->flush();

  // 送信専用モードを維持
  delayMicroseconds(50);

  Serial.println("Write command sent successfully");

  // 確認のため値を読み返す
  Serial.println("Verifying written value...");
  delay(50);

  // 読み取りコマンド構築
  u8 readCommand[8] = {0xFF, 0xFF, targetServoID, 4, CMD_READ, (u8)address, (u8)dataLength, 0};
  readCommand[7] = calcChecksum(readCommand, 8);

  // 読み取りコマンド送信
  sendCommand(readCommand, 8);

  // 応答受信8
  u8 expectedLength = 5 + dataLength + 1;
  u8 response[10];

  if (receiveResponse(response, expectedLength, 100))
  {
    if (response[0] == 0xFF && response[1] == 0xFF && response[2] == targetServoID)
    {
      if (dataLength == 1)
      {
        u8 readValue = response[5];
        Serial.print("Verification: Read value = ");
        Serial.print(readValue);
        Serial.print(" (0x");
        if (readValue < 16)
          Serial.print("0");
        Serial.print(readValue, HEX);
        Serial.print(")");
        if (readValue == (u8)value)
        {
          Serial.println(" [OK]");
        }
        else
        {
          Serial.println(" [MISMATCH]");
        }
      }
      else
      {
        u16 readValue = (response[6] << 8) | response[5];
        Serial.print("Verification: Read value = ");
        Serial.print(readValue);
        Serial.print(" (0x");
        if (readValue < 0x1000)
          Serial.print("0");
        if (readValue < 0x100)
          Serial.print("0");
        if (readValue < 0x10)
          Serial.print("0");
        Serial.print(readValue, HEX);
        Serial.print(")");
        if (readValue == (u16)value)
        {
          Serial.println(" [OK]");
        }
        else
        {
          Serial.println(" [MISMATCH]");
        }
      }
    }
    else
    {
      Serial.println("Verification failed: Invalid response");
    }
  }
  else
  {
    Serial.println("Verification failed: No response");
  }

  // 送信専用モードに確実に復帰
  digitalWrite(PIN_EN, HIGH);
}

// [x]コマンド: 全サーボに角度を一斉送信v
void executeAngleCommand()
{
  Serial.println("Enter angle (-180 to 180 degrees): ");

  // 入力待ち
  while (!Serial.available())
  {
    delay(10);
  }

  String input = Serial.readString();
  input.trim();

  float angle = input.toFloat();

  // 角度範囲チェック
  if (angle < -180.0 || angle > 180.0)
  {
    Serial.println("Error: Angle out of range (-180 to 180)");
    return;
  }

  // 角度を位置値に変換
  s16 position = angleToPosition(angle);
  if (position == -1)
  {
    Serial.println("Error: Invalid angle");
    return;
  }

  Serial.print("Converting ");
  Serial.print(angle);
  Serial.print(" degrees to position ");
  Serial.println(position);

  // 動的配列で同時制御サーボ数に対応
  u8 *allIDs = new u8[syncServoCount];
  s16 *allPositions = new s16[syncServoCount];
  u16 *allSpeeds = new u16[syncServoCount];
  u8 *allACCs = new u8[syncServoCount];

  // 配列を初期化（ID1から順番）
  for (u8 i = 0; i < syncServoCount; i++)
  {
    allIDs[i] = i + 1; // ID1から順番
    allPositions[i] = position;
    allSpeeds[i] = 1000;
    allACCs[i] = 50;
  }

  Serial.print("Sending angle command to ");
  Serial.print(syncServoCount);
  Serial.print(" servos (ID1-ID");
  Serial.print(syncServoCount);
  Serial.println(")...");

  STS_SyncWritePos(allIDs, syncServoCount, allPositions, allSpeeds, allACCs);

  Serial.println("Angle command sent successfully");

  // メモリ解放
  delete[] allIDs;
  delete[] allPositions;
  delete[] allSpeeds;
  delete[] allACCs;
}

// パケット内容をHEXで表示するデバッグ関数
void debugPacketContent(u8 ID[], u8 IDN, s16 Position[], u16 Speed[], u8 ACC[])
{
  Serial.println("\n=== Packet Debug ===");

  // パケット構築（デバッグ用）
  const u8 dataPerServo = 7;
  const u8 messageLength = ((dataPerServo + 1) * IDN + 4);
  const u16 totalPacketSize = 7 + (dataPerServo + 1) * IDN + 1;

  u8 packet[MAX_PACKET_SIZE];
  u16 packetIndex = 0;

  // ヘッダ
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xff;
  packet[packetIndex++] = 0xfe;
  packet[packetIndex++] = messageLength;
  packet[packetIndex++] = INST_SYNC_WRITE;
  packet[packetIndex++] = SMS_STS_ACC;
  packet[packetIndex++] = dataPerServo;

  u8 checksum = 0xfe + messageLength + INST_SYNC_WRITE + SMS_STS_ACC + dataPerServo;

  // データ部
  for (u8 i = 0; i < IDN; i++)
  {
    s16 pos = Position[i];
    if (pos < 0)
    {
      pos = -pos;
      pos |= (1 << 15);
    }

    u16 speed = Speed ? Speed[i] : 0;
    u8 acc = ACC ? ACC[i] : 0;

    packet[packetIndex++] = ID[i];
    checksum += ID[i];

    packet[packetIndex++] = acc;
    checksum += acc;

    u8 posL, posH;
    Host2SMS(&posL, &posH, pos);
    packet[packetIndex++] = posL;
    packet[packetIndex++] = posH;
    checksum += posL + posH;

    packet[packetIndex++] = 0;
    packet[packetIndex++] = 0;

    u8 speedL, speedH;
    Host2SMS(&speedL, &speedH, speed);
    packet[packetIndex++] = speedL;
    packet[packetIndex++] = speedH;
    checksum += speedL + speedH;
  }

  packet[packetIndex++] = ~checksum;

  // HEX表示
  Serial.print("Packet (");
  Serial.print(packetIndex);
  Serial.print(" bytes): ");
  for (u16 i = 0; i < packetIndex; i++)
  {
    if (packet[i] < 0x10)
      Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void disp_menu()
{
  Serial.println("============================================");
  Serial.print("Current target ID: ");
  Serial.print(targetServoID);
  Serial.print(", Sync servo count: ");
  Serial.println(syncServoCount);
  Serial.println("Menu: [f] Find servos   [n] Set sync servo count [z] Set target ID ");
  Serial.println("      [v] Read voltage  [r] Read position        [p] Read register");
  Serial.println("      [c] All to center [a] Read all positions   [s] Write register");
  Serial.println("      [e] All torque ON [d] All torque OFF       [x] Send angle to all ");
  Serial.println("Input Command:");
}

// 初期化
void setup()
{
  Serial.begin(115200);
  Serial1.begin(1000000);

  // ENピン設定（送信専用モード）
  pinMode(PIN_EN, OUTPUT);
  digitalWrite(PIN_EN, HIGH);

  delay(1000);

  Serial.println("=====================================");
  Serial.println("SMS/STS Servo SyncWrite Optimized");
  Serial.println("Batch Transmission Version");
  Serial.println("=====================================");
  Serial.println("Communication method: SyncWrite (0x83)");
  Serial.println("Optimization: Single writeServo() call");
  Serial.println("Packet size: Calculated dynamically");
  Serial.println("=====================================");

  // サーボのトルクイネーブル
  Serial.println("Enabling torque for active servos...");
  EnableTorque(1, 1);
  delay(100);
  EnableTorque(2, 1);
  delay(100);

  // 初期位置設定
  Serial.println("Setting initial positions...");
  u8 initIDs[] = {1, 2, 3, 4, 5, 6};
  s16 initPositions[] = {2048, 2048, 2048, 2048, 2048, 2048};
  u16 initSpeeds[] = {300, 300, 300, 300, 300, 300};
  u8 initACCs[] = {30, 30, 30, 30, 30, 30};

  // デバッグ表示
  debugPacketContent(initIDs, 6, initPositions, initSpeeds, initACCs);

  STS_SyncWritePos(initIDs, 6, initPositions, initSpeeds, initACCs);
  delay(2000); // 初期位置到達待ち
  disp_menu();
  lastMoveTime = millis();
}

void loop()
{
  // シリアル入力チェック
  if (Serial.available())
  {
    String command = Serial.readString();
    command.trim();

    if (command == "x")
    {
      executeAngleCommand();
    }
    else if (command == "r")
    {
      executeReadPositionCommand();
    }
    else if (command == "v")
    {
      executeReadVoltageCommand();
    }
    else if (command == "z")
    {
      executeChangeTargetIdCommand();
    }
    else if (command == "n")
    {
      executeSetSyncServoCountCommand();
    }
    else if (command == "a")
    {
      executeReadAllPositionsCommand();
    }
    else if (command == "c")
    {
      executeResetAllPositionsCommand();
    }
    else if (command == "f")
    {
      executeServoDetectionCommand();
    }
    else if (command == "e")
    {
      executeEnableTorqueAllCommand();
    }
    else if (command == "d")
    {
      executeDisableTorqueAllCommand();
    }
    else if (command == "p")
    {
      executeReadRegisterCommand();
    }
    else if (command == "s")
    {
      executeWriteRegisterCommand();
    }
    else if (command.length() == 0 || command == "\r" || command == "\n")
    {
      disp_menu();
    }
    else if (command.length() > 0)
    {
      Serial.println("Unknown command. Available commands:");
      disp_menu();
    }
  }

  delay(50); // CPU負荷軽減
}
