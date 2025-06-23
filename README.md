# Feetech STS Servo Setter  
  
ESP32を使用してFeetech STSサーボの設定・制御を行うためのテストコードです。半二重通信回路経由でサーボの詳細制御、ID管理、診断機能を提供します。
(このREADMEはAIで生成しているため、まだご参考程度に。)  
  
## 概要  
  
このプロジェクトは、Feetech STSサーボの包括的な制御と設定を行うESP32ベースのテストコードです。個別コマンド、同期書き込み操作、ID管理、診断ツールを提供します。  
  
## ハードウェア要件  
  
- **ターゲットボード**: ESP32 (esp32dev)  
- **フレームワーク**: Arduino  
- **通信設定**: 1Mbps, 8N1  
- **ピン配線**:  
  - EN Pin (半二重制御): GPIO 33  
  - TX Pin: GPIO 27  
  - RX Pin: GPIO 32  
  - シリアルモニター速度: 115200 baud  
  
## ビルドとアップロード  
  
```bash
# プロジェクトをビルド
pio run  
  
# ESP32にアップロード
pio run -t upload  
  
# シリアル出力をモニター
pio device monitor  
  
# ビルドクリーン
pio run -t clean  
  
# 特定の環境でビルド  
pio run -e esp32dev  
```
  
## 主要機能  
  
### 通信プロトコル  
- **個別コマンド**: Ping、位置・電圧・トルク制御の読み取り/書き込み
- **同期書き込み**: SyncWriteを使用した複数サーボの同時制御
- **ID管理**: EEPROMロック/アンロックによる完全なサーボID変更ワークフロー
- **診断ツール**: サーボスキャン、通信テスト、ステータス検証
  
### コマンドリファレンス  
  
| コマンド | 機能 | 説明 |
|---------|------|------|
| `f` | Find servos | サーボを検索・発見 |
| `n` | Set sync servo count | 同期制御するサーボ数を設定 |
| `z` | Set target ID | ターゲットサーボIDを設定 |
| `v` | Read voltage | サーボの電圧を読み取り |
| `r` | Read position | サーボの現在位置を読み取り |
| `p` | Read register | 指定レジスタアドレスの値を読み取り |
| `s` | Write register | 指定レジスタアドレスに値を書き込み |
| `c` | All to center | 全サーボを中央位置(0度)にリセット |
| `a` | Read all positions | 全サーボの位置を一括読み取り |
| `e` | All torque ON | 全サーボのトルクを一括オン |
| `d` | All torque OFF | 全サーボのトルクを一括オフ |
| `x` | Send angle to all | 全サーボに指定角度を一斉送信 |
  
### 詳細なコマンド使用例  
  
#### [p] Read register コマンド  
```
p
→ Enter register address (0-255): 40
→ Enter data length to read (1-4 bytes): 1
→ Register 40 (0x28): Dec=1, Hex=0x01, Bin=1
```

#### [s] Write register コマンド
```
s
→ Enter register address (0-255): 40
→ Enter data length to write (1-2 bytes): 1
→ Enter decimal value to write (0-255): 1
→ Writing 1 (0x01) to address 40 of servo ID 1
→ Verification: Read value = 1 (0x01) [OK]
```

#### [x] Send angle to all コマンド
```
x
→ Enter angle (-180 to 180 degrees): 90
→ Converting 90.0 degrees to position 3072
→ Sending angle command to 6 servos (ID1-ID6)...
```

#### [f] Find servos コマンド
```
f
→ Enter maximum ID to scan (1-253): 10
→ Scanning servos from ID1 to ID10...
→ Found servo ID: 1
→ Found servo ID: 2
→ Connected servos: 2/10
```

#### [e]/[d] Torque control コマンド
```
e  (全サーボトルクオン)
→ Enabling torque for 6 servos (ID1-ID6)...
→ ID 1: Torque enabled
→ ID 2: Torque enabled
→ Torque enabled for 6/6 servos

d  (全サーボトルクオフ)
→ Disabling torque for 6 servos (ID1-ID6)...
→ ID 1: Torque disabled
→ ID 2: Torque disabled  
→ Torque disabled for 6/6 servos
```

## 技術仕様

### サーボ制御パラメータ
- **位置範囲**: 0-4095 (0-360度)
- **角度範囲**: -180度 〜 +180度
- **通信タイムアウト**: 100ms
- **最大同期制御サーボ数**: 255個
- **デフォルトサーボID**: 1、有効範囲: 1-253

### 重要なレジスタアドレス
- **ID**: 5
- **トルクイネーブル**: 40
- **現在位置**: 56
- **電圧**: 62
- **ロック**: 55

### 電圧監視
- **最小電圧**: 5.0V (50)
- **最大電圧**: 14.0V (140)
- 範囲外の場合は警告を表示

### SyncWrite機能
- **最大パケットサイズ**: 256バイト
- **データ形式**: ACC + Position + Time + Speed (7バイト/サーボ)
- **角度表示**: 位置値と併せて角度も表示
- **エラーハンドリング**: パケットサイズ検証とチェックサム検証

## アーキテクチャ

### コア通信レイヤー
- `sendCommand()`: 半二重送信制御を処理
- `receiveResponse()`: タイムアウト付き高速応答受信
- `calcChecksum()`: パケット解析（標準ヘッダー、ヘッダーレス両対応）
- `EnableTorque()`: 統一された送受信ラッパーとエラーハンドリング

### プロトコル実装
- **個別コマンド**: Ping、位置/電圧/トルク制御の読み書き操作
- **同期書き込み**: ブロードキャストパケットによる複数サーボ同時制御
- **ID管理**: EEPROMロック/アンロック付き完全サーボID変更ワークフロー
- **診断ツール**: サーボスキャン、通信テスト、ステータス検証

## 開発ノート

- コード内のコメントとシリアル出力は日本語
- 包括的な自己テスト機能を内蔵:
  - 自動サーボスキャンと接続確認
  - 成功率計算による通信信頼性テスト
  - 位置エラーチェックによるSyncWrite結果検証
  - マルチサーボステータス監視と診断

### グローバル設定変数
- `targetServoID`: 現在のターゲットサーボID (デフォルト: 1)
- `syncServoCount`: 同期制御するサーボ数 (デフォルト: 6)

## トラブルシューティング

### 接続エラーの場合
1. **配線確認** - EN(33), TX(27), RX(32), GND
2. **電源確認** - 6-14V供給
3. **サーボID確認** - デフォルト=1
4. **ボーレート確認** - 1Mbps

### よくある問題
| 症状 | 原因 | 対処法 |
|------|------|--------|
| 応答なし | 配線不良・ID不一致 | 配線とIDを再確認 |
| 位置読み取り失敗 | 電源不足・通信エラー | 電源電圧と配線確認 |
| トルク制御失敗 | レジスタアクセスエラー | [s]コマンドでレジスタ40を直接操作 |

## ライセンス

このプロジェクトはテスト目的で作成されています。Feetech STSサーボの制御方法を学習するためのリファレンス実装として使用してください。

## 貢献

バグ報告や機能改善の提案は歓迎します。プルリクエストを送信する前に、既存のコード規約に従ってください。

## 免責事項

このコードはテスト目的で提供されています。実際の本番環境で使用する前に、十分にテストを行ってください。サーボの損傷や予期しない動作について、作者は責任を負いません。

## ライセンス

MIT License

## 関連リンク

- [Feetech公式サイト](https://www.feetechrc.com/)
- [STS3215仕様書](https://www.waveshare.com/wiki/ST3215_Servo)
- [PlatformIO](https://platformio.org/)
