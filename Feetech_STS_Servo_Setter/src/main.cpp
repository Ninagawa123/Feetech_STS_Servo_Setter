#include <Arduino.h>

// ESP32 STS サーボ制御 - SyncWrite機能付き完全版
// Feetech STS サーボ公式プロトコル準拠 + 複数サーボ同期制御
// 2025 Izumi Ninagawa

// ピン定義
#define ENPIN 33 // 半二重回路のENピン
#define RXD1 32  // ESP32 UART1 RXピン
#define TXD1 27  // ESP32 UART1 TXピン

// タイミング設定（マイクロ秒）
#define TX_STABILIZE_DELAY 50   // 送信モード安定化時間
#define RX_STABILIZE_DELAY 100  // 受信モード安定化時間
#define STS_TIMEOUT 8000        // 受信タイムアウト
#define INTER_BYTE_TIMEOUT 2000 // バイト間タイムアウト

// データ定義
#define MAX_DATA_LENGTH 64
#define POSITION_MIN 0
#define POSITION_MAX 4095
#define VOLTAGE_MIN 50
#define VOLTAGE_MAX 140
#define MAX_SERVOS 8 // 同期制御可能最大サーボ数

// コマンド定義
#define CMD_PING 1
#define CMD_READ 2
#define CMD_WRITE 3
#define CMD_REG_WRITE 4
#define CMD_ACTION 5
#define CMD_RESET 6
#define CMD_SYNC_WRITE 83

// レジスタアドレス定義
#define REG_ID 5
#define REG_TORQUE_ENABLE 40 // 修正: STS公式仕様では40
#define REG_TARGET_POSITION 30
#define REG_CURRENT_POSITION 56 // 修正: STS公式仕様では56
#define REG_VOLTAGE 62          // 修正: STS公式仕様では62
#define REG_MOVING 66           // 修正: STS公式仕様では66
#define REG_LOCK 55
#define UNLOCK_VALUE 0 // EEPROMロック解除値
#define LOCK_VALUE 1   // EEPROMロック値

byte buffer[MAX_DATA_LENGTH];

// グローバル変数
byte targetServoId = 1; // コントロール対象のサーボID

// 複数サーボ制御用データ構造
struct ServoData
{
  byte id;
  int position;
  int speed;
  int time;
};

// デバッグ用表示関数
void disp_dechex(int num)
{
  Serial.print(num, DEC);
  Serial.print("(0x");
  if (num < 0x1000)
    Serial.print("0");
  if (num < 0x100)
    Serial.print("0");
  if (num < 0x10)
    Serial.print("0");
  Serial.print(num, HEX);
  Serial.print(")");
}

// チェックサム計算
byte sts_calcCkSum(byte arr[], int len)
{
  int checksum = 0;
  for (int i = 2; i < len - 1; i++)
  {
    checksum += arr[i];
  }
  return ~((byte)(checksum & 0xFF));
}

// 関数宣言（parseResponseより前に配置）
bool extractDataFromPacket(byte *data, int dataStart, int expectedDataBytes, int *extractedValue);

// 送信制御（無遅延版）
void sts_sendCommand(byte arr[], int len)
{
  // 受信バッファクリア（無出力）
  while (Serial1.available())
  {
    Serial1.read();
  }

  // 送信モード（最短切り替え）
  digitalWrite(ENPIN, HIGH);
  delayMicroseconds(TX_STABILIZE_DELAY);

  // データ送信（ノンストップ）
  for (int i = 0; i < len; i++)
  {
    Serial1.write(arr[i]);
  }

  // 送信完了待ち（ハードウェアレベル）
  Serial1.flush();

  // 受信モードに切り替え
  digitalWrite(ENPIN, LOW);
  delayMicroseconds(RX_STABILIZE_DELAY);
}

// 受信制御（高速応答対応版）
int sts_receiveResponse(byte *responseBuffer, int maxBytes, unsigned long totalWaitMicros)
{
  int bytesReceived = 0;
  unsigned long startTime = micros();

  // 高速サーボに対応した短時間高密度受信
  while ((micros() - startTime) < totalWaitMicros && bytesReceived < maxBytes)
  {
    if (Serial1.available())
    {
      responseBuffer[bytesReceived] = Serial1.read();
      bytesReceived++;
    }
    // 遅延なし（マイクロ秒単位のポーリング）
  }

  return bytesReceived;
}

// パケット解析（ヘッダーなし対応版）
bool parseResponse(byte *data, int length, int expectedDataBytes, int *extractedValue)
{
  if (length < 4)
  { // 最小: ID + Length + Error + Checksum
    return false;
  }

  // パターン1: 標準ヘッダー付きパケット検索
  for (int i = 0; i <= length - 6; i++)
  {
    if (data[i] == 0xFF && data[i + 1] == 0xFF)
    {
      byte id = data[i + 2];
      byte packetLength = data[i + 3];
      byte error = data[i + 4];

      Serial.print("OK [標準] ID=");
      Serial.print(id);
      Serial.print(" Len=");
      Serial.print(packetLength);
      Serial.print(" Err=0x");
      if (error < 0x10)
        Serial.print("0");
      Serial.print(error, HEX);

      return extractDataFromPacket(data, i + 5, expectedDataBytes, extractedValue);
    }
  }

  // パターン2: ヘッダーなしパケット検索（FF FFが欠落）
  if (length >= 4 && data[0] <= 253)
  { // 有効なサーボID範囲
    byte id = data[0];
    byte packetLength = data[1];
    byte error = data[2];
    byte checksum = data[3];

    // 簡易チェックサム検証
    byte expectedChecksum = ~((id + packetLength + error) & 0xFF);

    if (packetLength == 2 && checksum == expectedChecksum)
    {
      Serial.print("OK [ヘッダーなし] ID=");
      Serial.print(id);
      Serial.print(" Len=");
      Serial.print(packetLength);
      Serial.print(" Err=0x");
      if (error < 0x10)
        Serial.print("0");
      Serial.print(error, HEX);

      if (expectedDataBytes == 0)
      {
        return true; // Ping応答
      }
    }
  }

  // パターン3: より長いヘッダーなしパケット
  if (length >= 6 && data[0] <= 253)
  {
    byte id = data[0];
    byte packetLength = data[1];
    byte error = data[2];

    if (packetLength == 4)
    { // 2バイトデータ + エラー + チェックサム
      Serial.print("OK [ヘッダーなし長] ID=");
      Serial.print(id);
      Serial.print(" Len=");
      Serial.print(packetLength);
      Serial.print(" Err=0x");
      if (error < 0x10)
        Serial.print("0");
      Serial.print(error, HEX);

      return extractDataFromPacket(data, 3, expectedDataBytes, extractedValue);
    }
  }

  return false;
}

// データ抽出ヘルパー関数（実装）
bool extractDataFromPacket(byte *data, int dataStart, int expectedDataBytes, int *extractedValue)
{
  if (expectedDataBytes == 1)
  {
    *extractedValue = data[dataStart];
    Serial.print(" Val=");
    Serial.print(*extractedValue);
    return true;
  }
  else if (expectedDataBytes == 2)
  {
    // リトルエンディアン（修正）- STSサーボは実際にはリトルエンディアン
    *extractedValue = data[dataStart] + (data[dataStart + 1] << 8);
    Serial.print(" Pos=");
    disp_dechex(*extractedValue);
    if (*extractedValue >= POSITION_MIN && *extractedValue <= POSITION_MAX)
    {
      Serial.print("(");
      Serial.print((*extractedValue * 360.0) / 4096.0, 1);
      Serial.print("°)");
    }
    else
    {
      Serial.print("(範囲外)");
    }
    return true;
  }
  else if (expectedDataBytes == 0)
  {
    return true; // Ping
  }

  return false;
}

// エラー詳細表示
void displayErrorDetails(byte error)
{
  if (error != 0)
  {
    Serial.print(" [");
    if (error & 0x01)
      Serial.print("電圧 ");
    if (error & 0x02)
      Serial.print("角度 ");
    if (error & 0x04)
      Serial.print("過熱 ");
    if (error & 0x08)
      Serial.print("電流 ");
    if (error & 0x20)
      Serial.print("負荷 ");
    Serial.print("]");
  }
}

// 送受信統合関数（シリアル出力遅延除去版）
bool sts_command(byte *command, int commandLength, int expectedDataBytes, int *result, const char *description)
{
  // 送信（シリアル出力なし）
  sts_sendCommand(command, commandLength);

  // 受信（即座に開始）
  int bytesReceived = sts_receiveResponse(buffer, MAX_DATA_LENGTH, STS_TIMEOUT);

  // 送受信完了後にまとめて出力
  Serial.print(description);
  Serial.print("... Rcvd(");
  Serial.print(bytesReceived);
  Serial.print("): ");

  if (bytesReceived == 0)
  {
    Serial.println("ERROR:  受信なし");
    return false;
  }

  for (int i = 0; i < bytesReceived; i++)
  {
    if (buffer[i] < 0x10)
      Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }

  // パケット解析
  if (parseResponse(buffer, bytesReceived, expectedDataBytes, result))
  {
    Serial.println(" OK");
    return true;
  }
  else
  {
    Serial.println(" ERROR:  解析失敗");
    return false;
  }
}

// 個別コマンド関数群
bool sts_ping(byte id)
{
  byte message[6] = {0xFF, 0xFF, id, 2, CMD_PING, 0};
  message[5] = sts_calcCkSum(message, 6);

  int dummy;
  return sts_command(message, 6, 0, &dummy, "[Control]Ping");
}

bool sts_readPosition(byte id, int *position)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_CURRENT_POSITION, 2, 0};
  message[7] = sts_calcCkSum(message, 8);

  return sts_command(message, 8, 2, position, "位置読取");
}

bool sts_readVoltage(byte id, int *voltage)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_VOLTAGE, 1, 0};
  message[7] = sts_calcCkSum(message, 8);

  bool result = sts_command(message, 8, 1, voltage, "[VOLT] 電圧読取");
  if (result && voltage)
  {
    Serial.print(" (");
    Serial.print(*voltage / 10.0, 1);
    Serial.print("V)");
    if (*voltage < VOLTAGE_MIN)
    {
      Serial.print(" [Caution]低電圧");
    }
    else if (*voltage > VOLTAGE_MAX)
    {
      Serial.print(" [Caution]過電圧");
    }
    else
    {
      Serial.print(" OK:正常");
    }
  }
  return result;
}

bool sts_readMoving(byte id, int *moving)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_MOVING, 1, 0};
  message[7] = sts_calcCkSum(message, 8);

  return sts_command(message, 8, 1, moving, "[START] 移動状態");
}

bool sts_readId(byte id, int *readId)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_READ, REG_ID, 1, 0};
  message[7] = sts_calcCkSum(message, 8);

  bool result = sts_command(message, 8, 1, readId, "ID読取");
  if (result && readId)
  {
    Serial.print(" (ID=");
    Serial.print(*readId);
    Serial.print(")");
  }
  return result;
}

bool sts_enableTorque(byte id, byte enable)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_WRITE, REG_TORQUE_ENABLE, enable, 0};
  message[7] = sts_calcCkSum(message, 8);

  sts_sendCommand(message, 8);
  Serial.print("トルク");
  Serial.print(enable ? "有効化" : "無効化");
  Serial.println("... 送信完了");
  return true;
}

bool sts_writePosition(byte id, int position, int time = 1500, int speed = 100)
{
  if (position < POSITION_MIN || position > POSITION_MAX)
  {
    Serial.print("ERROR:  位置範囲外: ");
    Serial.println(position);
    return false;
  }

  byte message[13] = {
      0xFF, 0xFF, id, 9, CMD_WRITE, REG_TARGET_POSITION,
      (byte)(position & 0xFF),        // 位置下位
      (byte)((position >> 8) & 0xFF), // 位置上位
      (byte)(time & 0xFF),            // 時間下位
      (byte)((time >> 8) & 0xFF),     // 時間上位
      (byte)(speed & 0xFF),           // 速度下位
      (byte)((speed >> 8) & 0xFF),    // 速度上位
      0};
  message[12] = sts_calcCkSum(message, 13);

  Serial.print("[TARGET] 目標=");
  disp_dechex(position);
  Serial.print("(");
  Serial.print((position * 360.0) / 4096.0, 1);
  Serial.print("°) 速度=");
  Serial.print(speed);
  Serial.print(" 時間=");
  Serial.print(time);
  Serial.print("ms... ");

  sts_sendCommand(message, 13);
  Serial.println("送信完了");
  return true;
}

// ★ 修正版: SyncWrite - 複数サーボ同期制御（返信なし）
bool sts_syncWrite(ServoData servos[], int numServos)
{
  if (numServos == 0 || numServos > MAX_SERVOS)
  {
    Serial.print("ERROR:  サーボ数エラー: ");
    Serial.println(numServos);
    return false;
  }

  // SyncWriteパケット構築
  // ヘッダー: FF FF FE (ブロードキャストID) LENGTH CMD_SYNC_WRITE REG_ADDR DATA_LENGTH
  int dataLength = 6;                            // 位置(2) + 時間(2) + 速度(2)
  int packetLength = 4 + dataLength * numServos; // CMD + REG + LEN + (ID + DATA) * numServos

  byte message[128]; // 十分なサイズを確保
  int index = 0;

  // ヘッダー
  message[index++] = 0xFF;
  message[index++] = 0xFF;
  message[index++] = 0xFE;                // ブロードキャストID
  message[index++] = packetLength;        // パケット長
  message[index++] = CMD_SYNC_WRITE;      // コマンド
  message[index++] = REG_TARGET_POSITION; // レジスタアドレス
  message[index++] = dataLength;          // 各サーボのデータ長

  // 各サーボのデータ
  for (int i = 0; i < numServos; i++)
  {
    if (servos[i].position < POSITION_MIN || servos[i].position > POSITION_MAX)
    {
      Serial.print("ERROR:  サーボ");
      Serial.print(servos[i].id);
      Serial.print(" 位置範囲外: ");
      Serial.println(servos[i].position);
      return false;
    }

    message[index++] = servos[i].id;                     // サーボID
    message[index++] = servos[i].position & 0xFF;        // 位置下位
    message[index++] = (servos[i].position >> 8) & 0xFF; // 位置上位
    message[index++] = servos[i].time & 0xFF;            // 時間下位
    message[index++] = (servos[i].time >> 8) & 0xFF;     // 時間上位
    message[index++] = servos[i].speed & 0xFF;           // 速度下位
    message[index++] = (servos[i].speed >> 8) & 0xFF;    // 速度上位
  }

  // チェックサム
  message[index] = sts_calcCkSum(message, index + 1);
  index++;

  // デバッグ出力
  Serial.print("[SYNC] SyncWrite ");
  Serial.print(numServos);
  Serial.print("台同期制御: ");
  for (int i = 0; i < numServos; i++)
  {
    Serial.print("ID");
    Serial.print(servos[i].id);
    Serial.print("→");
    disp_dechex(servos[i].position);
    if (i < numServos - 1)
      Serial.print(", ");
  }
  Serial.print("... ");

  // 送信のみ（返信待機なし）
  sts_sendCommand(message, index);
  Serial.println("OK: 同期送信完了（返信なし）");

  return true;
}

// ★ 新機能: SyncWrite結果確認（個別読み取り）
void sts_verifySyncWrite(ServoData expectedServos[], int numServos)
{
  Serial.println("[CHECK]SyncWrite結果確認:");

  delay(500); // 移動開始待ち

  bool allSuccess = true;
  for (int i = 0; i < numServos; i++)
  {
    int currentPos;
    Serial.print("  ID");
    Serial.print(expectedServos[i].id);
    Serial.print(": ");

    if (sts_readPosition(expectedServos[i].id, &currentPos))
    {
      int error = abs(currentPos - expectedServos[i].position);
      Serial.print("目標=");
      disp_dechex(expectedServos[i].position);
      Serial.print(" 実際=");
      disp_dechex(currentPos);
      Serial.print(" 誤差=");
      Serial.print(error);

      if (error < 30)
      {
        Serial.println(" [Check]");
      }
      else if (error < 100)
      {
        Serial.println(" [Caution]");
        allSuccess = false;
      }
      else
      {
        Serial.println(" ERROR: ");
        allSuccess = false;
      }
    }
    else
    {
      Serial.println(" ERROR:  読み取り失敗");
      allSuccess = false;
    }
    delay(150); // 各読み取り間隔
  }
}

// ★ 新機能: EEPROM ロック制御
bool sts_setLock(byte id, byte lockValue)
{
  byte message[8] = {0xFF, 0xFF, id, 4, CMD_WRITE, REG_LOCK, lockValue, 0};
  message[7] = sts_calcCkSum(message, 8);

  Serial.print("[ROCK]EEPROM ");
  Serial.print(lockValue == UNLOCK_VALUE ? "ロック解除" : "ロック");
  Serial.print(" (ID ");
  Serial.print(id);
  Serial.print(")... ");

  sts_sendCommand(message, 8);
  Serial.println("送信完了");
  delay(100); // EEPROM書き込み待ち
  return true;
}

// ★ 新機能: サーボID変更
bool sts_changeId(byte currentId, byte newId)
{
  // 入力検証
  if (currentId == 0 || currentId > 253 || newId == 0 || newId > 253)
  {
    Serial.println("ERROR:  無効なID (有効範囲: 1-253)");
    return false;
  }

  if (currentId == newId)
  {
    Serial.println("ERROR:  現在のIDと同じです");
    return false;
  }

  Serial.println("サーボID変更開始...");
  Serial.print("現在ID: ");
  Serial.print(currentId);
  Serial.print(" → 新ID: ");
  Serial.println(newId);

  // ステップ1: 現在のIDでPing確認
  Serial.print("[1]  現在ID確認... ");
  if (!sts_ping(currentId))
  {
    Serial.println("ERROR:  現在IDのサーボが応答しません");
    return false;
  }
  Serial.println("OK: 確認完了");

  // ステップ2: 新IDで既存サーボがないか確認
  Serial.print("[2]  新ID重複確認... ");
  if (sts_ping(newId))
  {
    Serial.println("ERROR:  新IDのサーボが既に存在します");
    return false;
  }
  Serial.println("OK: 重複なし");

  // ステップ3: EEPROMロック解除
  Serial.print("[3]  EEPROMロック解除... ");
  if (!sts_setLock(currentId, UNLOCK_VALUE))
  {
    Serial.println("ERROR:  ロック解除失敗");
    return false;
  }
  Serial.println("OK: 解除完了");

  // ステップ4: ID書き込み
  Serial.print("[4]  新ID書き込み... ");
  byte message[8] = {0xFF, 0xFF, currentId, 4, CMD_WRITE, REG_ID, newId, 0};
  message[7] = sts_calcCkSum(message, 8);

  sts_sendCommand(message, 8);
  delay(200); // EEPROM書き込み完了待ち
  Serial.println("OK: 書き込み完了");

  // ステップ5: EEPROMロック（新IDで）
  Serial.print("[5]  EEPROMロック (新ID)... ");
  delay(500); // ID変更反映待ち
  if (!sts_setLock(newId, LOCK_VALUE))
  {
    Serial.println("[Caution] ロック失敗（機能的には問題なし）");
  }
  else
  {
    Serial.println("OK: ロック完了");
  }

  // ステップ6: 変更確認
  Serial.print("[6]  変更確認... ");
  delay(300);
  if (sts_ping(newId))
  {
    Serial.println("OK: ID変更成功!");

    // 古いIDが応答しないことを確認
    delay(200);
    if (!sts_ping(currentId))
    {
      Serial.println("ID変更完全成功");
      return true;
    }
    else
    {
      Serial.println("[Caution] 古いIDがまだ応答しています");
      return false;
    }
  }
  else
  {
    Serial.println("ERROR:  新IDで応答がありません");
    return false;
  }
}

// ★ 新機能: 工場出荷時設定リセット（ID=1に戻す）
bool sts_resetToFactory(byte currentId)
{
  Serial.println("工場出荷時設定リセット開始...");

  if (currentId == 1)
  {
    Serial.println("OK: 既にID=1です");
    return true;
  }

  return sts_changeId(currentId, 1);
}

// ★ 新機能: ID連番設定（複数サーボの一括ID設定）
void sts_setBatchIds(byte startId = 1)
{
  Serial.println("連番ID設定モード");
  Serial.println("各サーボを1台ずつ接続してEnterを押してください");
  Serial.println("'q'で終了");

  byte currentNewId = startId;

  while (true)
  {
    Serial.print("\n現在設定するID: ");
    Serial.print(currentNewId);
    Serial.println(" (Enterで実行, 'q'で終了)");

    // シリアル入力待機
    while (!Serial.available())
    {
      delay(100);
    }

    String input = Serial.readString();
    input.trim();

    if (input == "q" || input == "Q")
    {
      Serial.println("[FINISH] 連番設定終了");
      break;
    }

    // 現在接続されているサーボを検索
    Serial.println("[CHECK]接続サーボ検索中...");
    byte foundId = 0;
    for (byte id = 1; id <= 253; id++)
    {
      if (sts_ping(id))
      {
        if (foundId == 0)
        {
          foundId = id;
        }
        else
        {
          Serial.println("ERROR:  複数のサーボが検出されました。1台のみ接続してください");
          foundId = 0;
          break;
        }
      }
      delay(10);
    }

    if (foundId == 0)
    {
      Serial.println("ERROR:  サーボが見つかりません");
      continue;
    }

    if (foundId == currentNewId)
    {
      Serial.print("OK: ID ");
      Serial.print(currentNewId);
      Serial.println(" は既に設定済みです");
    }
    else
    {
      Serial.print("ID ");
      Serial.print(foundId);
      Serial.print(" のサーボを ID ");
      Serial.print(currentNewId);
      Serial.println(" に変更中...");

      if (sts_changeId(foundId, currentNewId))
      {
        Serial.print("OK: ID ");
        Serial.print(currentNewId);
        Serial.println(" 設定完了");
      }
      else
      {
        Serial.println("ERROR:  ID変更失敗");
        continue;
      }
    }

    currentNewId++;

    if (currentNewId > 253)
    {
      Serial.println("最大ID到達");
      break;
    }
  }
}

// ★ 新機能: 複数サーボ一括位置読み取り
void sts_readMultiplePositions(byte ids[], int numServos)
{
  Serial.print("");
  Serial.print(numServos);
  Serial.println("台位置一括読み取り:");

  for (int i = 0; i < numServos; i++)
  {
    int position;
    Serial.print("  ID");
    Serial.print(ids[i]);
    Serial.print(": ");
    if (sts_readPosition(ids[i], &position))
    {
      Serial.print("位置=");
      disp_dechex(position);
      Serial.print(" (");
      Serial.print((position * 360.0) / 4096.0, 1);
      Serial.println("°)");
    }
    else
    {
      Serial.println("読み取り失敗");
    }
    delay(100); // サーボ間の通信間隔
  }
}

// ★ 新機能: 全サーボ一括トルク制御
void sts_enableMultipleTorque(byte ids[], int numServos, byte enable)
{
  Serial.print("[TORQUE_MULTI] ");
  Serial.print(numServos);
  Serial.print("台トルク一括");
  Serial.print(enable ? "有効化" : "無効化");
  Serial.println(":");

  for (int i = 0; i < numServos; i++)
  {
    Serial.print("  ID");
    Serial.print(ids[i]);
    Serial.print(": ");
    sts_enableTorque(ids[i], enable);
    delay(50);
  }
}

// ★ 新機能: サーボスキャン（接続確認）
void sts_scanServos(byte startId = 1, byte endId = 10)
{
  Serial.print("[CHECK]サーボスキャン (ID ");
  Serial.print(startId);
  Serial.print("-");
  Serial.print(endId);
  Serial.println("):");

  int foundCount = 0;
  for (byte id = startId; id <= endId; id++)
  {
    Serial.print("  ID");
    Serial.print(id);
    Serial.print(": ");
    if (sts_ping(id))
    {
      Serial.println("OK: 発見");
      foundCount++;
    }
    else
    {
      Serial.println("ERROR:  未接続");
    }
    delay(100);
  }

  Serial.print("結果: ");
  Serial.print(foundCount);
  Serial.print("/");
  Serial.print(endId - startId + 1);
  Serial.println("台のサーボが見つかりました");
  Serial.println();
}

// コマンド一覧表示
void showCommandList()
{
  Serial.println("=== コマンド一覧 ===");
  Serial.print(" [p] Ping テスト (現在のターゲット: ID");
  Serial.print(targetServoId);
  Serial.println(" )            [z] ターゲットサーボID変更");
  Serial.println(" [r] 位置READ      [v] 電圧READ    [m] 移動状態確認  [c] センター移動  [s] 全STATUS確認");
  Serial.println(" [a] 複数位置READ  [n] サーボSCAN  [e] 全トルク有効  [d] 全トルク無効  [y] SyncWriteテスト");
  Serial.println(" [j] ID読み取り    [i] ID変更      [f] 工場リセット  [b] 連番ID設定");
  Serial.println(" [t] 通信テスト    [h] ヘルプ表示");
}

// シリアルコマンド処理（SyncWrite機能追加）
void handleSerialCommand()
{
  if (Serial.available())
  {
    char cmd = Serial.read();
    while (Serial.available())
      Serial.read();

    Serial.println();
    int value;

    switch (cmd)
    {
    case 'p':
    case 'P':
      Serial.print("=== Ping テスト (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      if (sts_ping(targetServoId))
      {
        Serial.println("OK: 接続成功");
      }
      else
      {
        Serial.println("ERROR:  接続失敗");
      }
      break;

    case 'i':
    case 'I':
    {
      Serial.println("=== サーボID変更 ===");
      Serial.println("現在のIDを入力してください (1-253):");

      while (!Serial.available())
        delay(100);
      int currentId = Serial.parseInt();
      while (Serial.available())
        Serial.read(); // バッファクリア

      Serial.println("新しいIDを入力してください (1-253):");
      while (!Serial.available())
        delay(100);
      int newId = Serial.parseInt();
      while (Serial.available())
        Serial.read(); // バッファクリア

      Serial.print("確認: ID ");
      Serial.print(currentId);
      Serial.print(" → ID ");
      Serial.print(newId);
      Serial.println(" に変更しますか? (y/n)");

      while (!Serial.available())
        delay(100);
      char confirm = Serial.read();
      while (Serial.available())
        Serial.read(); // バッファクリア

      if (confirm == 'y' || confirm == 'Y')
      {
        sts_changeId(currentId, newId);
      }
      else
      {
        Serial.println("ERROR:  キャンセルされました");
      }
      break;
    }

    case 'f':
    case 'F':
    {
      Serial.println("=== 工場出荷時リセット ===");
      Serial.println("現在のIDを入力してください:");

      while (!Serial.available())
        delay(100);
      int currentId = Serial.parseInt();
      while (Serial.available())
        Serial.read(); // バッファクリア

      Serial.print("確認: ID ");
      Serial.print(currentId);
      Serial.println(" → ID 1 にリセットしますか? (y/n)");

      while (!Serial.available())
        delay(100);
      char confirm = Serial.read();
      while (Serial.available())
        Serial.read(); // バッファクリア

      if (confirm == 'y' || confirm == 'Y')
      {
        sts_resetToFactory(currentId);
      }
      else
      {
        Serial.println("ERROR:  キャンセルされました");
      }
      break;
    }

    case 'b':
    case 'B':
      Serial.println("=== 連番ID設定 ===");
      sts_setBatchIds(1);
      break;

    case 'r':
    case 'R':
      Serial.print("=== 位置読み取り (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      if (sts_readPosition(targetServoId, &value))
      {
        Serial.print("現在位置: ");
        disp_dechex(value);
        Serial.print(" (");
        Serial.print((value * 360.0) / 4096.0, 1);
        Serial.println("°)");
      }
      break;

    case 'v':
    case 'V':
      Serial.print("=== 電圧読み取り (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      if (sts_readVoltage(targetServoId, &value))
      {
        Serial.print("電圧: ");
        Serial.print(value / 10.0, 1);
        Serial.println("V");
      }
      break;

    case 'm':
    case 'M':
      Serial.print("=== 移動状態磺認 (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      if (sts_readMoving(targetServoId, &value))
      {
        Serial.print("移動状態: ");
        Serial.println(value == 0 ? "停止中" : "移動中");
      }
      break;

    case 'c':
    case 'C':
      Serial.print("=== センター移動 (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      sts_writePosition(targetServoId, 2048, 2000, 80);
      break;

    case 's':
    case 'S':
      Serial.print("=== 全ステータス確認 (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      sts_readPosition(targetServoId, &value);
      delay(300);
      sts_readVoltage(targetServoId, &value);
      delay(300);
      sts_readMoving(targetServoId, &value);
      break;

    case 'y':
    case 'Y':
    {
      Serial.println("=== SyncWrite テスト ===");
      ServoData testServos[3] = {
          {1, 1000, 100, 1500},
          {2, 2000, 100, 1500},
          {3, 3000, 100, 1500}};
      if (sts_syncWrite(testServos, 3))
      {
        sts_verifySyncWrite(testServos, 3);
      }
      break;
    }

    case 'a':
    case 'A':
    {
      Serial.println("=== 複数位置読み取り ===");
      byte ids[] = {1, 2, 3};
      sts_readMultiplePositions(ids, 3);
      break;
    }

    case 'n':
    case 'N':
      Serial.println("=== サーボスキャン ===");
      sts_scanServos(1, 5);
      break;

    case 'e':
    case 'E':
    {
      Serial.println("=== 全トルク有効化 ===");
      byte ids[] = {1, 2, 3};
      sts_enableMultipleTorque(ids, 3, 1);
      break;
    }

    case 'd':
    case 'D':
    {
      Serial.println("=== 全トルク無効化 ===");
      byte ids[] = {1, 2, 3};
      sts_enableMultipleTorque(ids, 3, 0);
      break;
    }

    case 't':
    case 'T':
    {
      Serial.print("=== 通信テスト (ID");
      Serial.print(targetServoId);
      Serial.println(") ===");
      int successCount = 0;
      for (int i = 0; i < 10; i++)
      {
        Serial.print("Test ");
        Serial.print(i + 1);
        Serial.print("/10: ");
        if (sts_ping(targetServoId))
        {
          successCount++;
        }
        delay(200);
      }
      Serial.print("成功率: ");
      Serial.print(successCount * 10);
      Serial.println("%");
      break;
    }

    case 'j':
    case 'J':
    {
      Serial.println("=== ID読み取り ===");
      Serial.println("読み取りたいID (1-253):");
      while (!Serial.available())
        delay(100);
      int targetId = Serial.parseInt();
      while (Serial.available())
        Serial.read();

      if (targetId >= 1 && targetId <= 253)
      {
        int readId;
        if (sts_readId(targetId, &readId))
        {
          Serial.print("OK: ID ");
          Serial.print(targetId);
          Serial.print(" の応答: 実際のID=");
          Serial.println(readId);
        }
        else
        {
          Serial.println("ERROR:  応答がありません");
        }
      }
      else
      {
        Serial.println("ERROR:  無効なIDです");
      }
      break;
    }

    case 'h':
    case 'H':
      showCommandList();
      break;

    case 'z':
    case 'Z':
    {
      Serial.println("=== ターゲットサーボID変更 ===");
      Serial.print("現在のターゲット: ID");
      Serial.println(targetServoId);
      Serial.println("新しいターゲットID (1-253):");

      while (!Serial.available())
        delay(100);
      int newTargetId = Serial.parseInt();
      while (Serial.available())
        Serial.read();

      if (newTargetId >= 1 && newTargetId <= 253)
      {
        Serial.print("確認: ターゲットを ID");
        Serial.print(targetServoId);
        Serial.print(" → ID");
        Serial.print(newTargetId);
        Serial.println(" に変更しますか? (y/n)");

        while (!Serial.available())
          delay(100);
        char confirm = Serial.read();
        while (Serial.available())
          Serial.read();

        if (confirm == 'y' || confirm == 'Y')
        {
          targetServoId = newTargetId;
          Serial.print("OK: ターゲットサーボを ID");
          Serial.print(targetServoId);
          Serial.println(" に変更しました");

          // 新しいターゲットIDでPingテスト
          Serial.print("接続確認... ");
          if (sts_ping(targetServoId))
          {
            Serial.println("OK: 接続成功");
          }
          else
          {
            Serial.println("WARN: 応答なし");
          }
        }
        else
        {
          Serial.println("CANCEL: キャンセルされました");
        }
      }
      else
      {
        Serial.println("ERROR: 無効なIDです (1-253)");
      }
      break;
    }

    default:
      showCommandList();
      break;
    }
    Serial.println();
  }
}

void setup()
{
  // ピン初期化
  pinMode(ENPIN, OUTPUT);
  digitalWrite(ENPIN, LOW);

  // シリアル初期化
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }

  Serial1.begin(1000000, SERIAL_8N1, RXD1, TXD1);
  while (!Serial1)
  {
    ;
  }

  delay(500);

  // スタートアップ
  Serial.println();
  Serial.println("╔══════════════════════════════════════════════════════════╗");
  Serial.println("║          Feetech STS サーボ設定システム v0.1             ║");
  Serial.println("╚══════════════════════════════════════════════════════════╝");
  Serial.print("ENピン状態: 現在");
  Serial.println(digitalRead(ENPIN) ? "HIGH(送信)" : "LOW(受信)");
  Serial.print("UART設定: 1Mbps, 8N1, TX=");
  Serial.print(TXD1);
  Serial.print(", RX=");
  Serial.println(RXD1);
  Serial.println();

  // 手動制御モードで起動

  // 基本的な接続確認を実行
  Serial.println("サーボ接続テスト...");

  bool connected = false;
  for (int attempt = 1; attempt <= 3; attempt++)
  {
    Serial.print("試行 ");
    Serial.print(attempt);
    Serial.print("/3: ");
    if (sts_ping(targetServoId))
    {
      connected = true;
      break;
    }
    delay(500);
  }

  if (connected)
  {
    Serial.println("サーボ接続成功!");
  }
  else
  {
    Serial.println("ERROR:  サーボ接続失敗");
    Serial.println();
    Serial.println("トラブルシューティング:");
    Serial.println("1. 配線確認 - EN(33), TX(27), RX(32), GND");
    Serial.println("2. 電源確認 - 6-14V");
    Serial.println("3. サーボID確認 - デフォルト=1");
    Serial.println("4. ボーレート確認 - 1Mbps");
    Serial.println();
  }

  // コマンド一覧を表示
  showCommandList();
  Serial.println();
}

void loop()
{
  // シリアルコマンド処理
  handleSerialCommand();

  delay(100);
}