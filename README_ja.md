# NTP Sample

ベンダー拡張による即時時刻同期とゲートウェイ機能を備えた、Windows向けの高精度NTPサーバー・クライアント実装です。

## 特徴

- **高精度時刻表現**: プラットフォーム非依存の`TimeSpec`（64ビット秒 + 32ビットナノ秒）
- **NTPサーバー** (`ntpserver`): ベンダー拡張をサポートする最小限のNTPv4サーバー
- **NTPクライアント** (`ntpclock`): 適応的補正を備えた時刻同期サービス
- **Pushプッシュ通知**: ベンダー拡張フィールドによる即時時刻変更伝播
- **ゲートウェイモード**: 上流同期と下流配信による多段NTP展開
- **適応的時計補正**: オフセット量に基づくslew/stepの自動選択

## プロジェクト構成

```
ntpsample/
├── ntpserver/          # NTPサーバーライブラリ
│   ├── include/        # 公開ヘッダ（TimeSource、NtpServer、TimeSpec）
│   ├── src/            # サーバー実装
│   └── example/        # サーバーサンプルアプリケーション
├── ntpclock/           # NTPクライアントライブラリ
│   ├── include/        # 公開ヘッダ（ClockService、Options）
│   ├── src/            # クライアント実装と同期ロジック
│   └── example/        # クライアントとゲートウェイのサンプルアプリケーション
└── cmake/              # ビルド設定
```

## ビルド要件

- CMake 3.20以上
- Visual Studio 2022 Build Tools（C++）または Visual Studio 2022
- Winsock2をサポートするWindows SDK

## ビルド手順

### Visual Studioでのビルド（x64）

```bash
# 構成
cmake -S . -B build -G "Visual Studio 17 2022" -A x64

# ビルド
cmake --build build --config Release
```

### ビルド成果物

ライブラリ:
- `build/ntpserver/Release/ntpserver.lib` - NTPサーバーライブラリ
- `build/ntpclock/Release/ntpclock.lib` - NTPクライアントライブラリ

実行ファイル:
- `build/ntpserver/Release/ntpserver_example.exe` - シンプルなNTPサーバー
- `build/ntpclock/Release/ntpclock_example.exe` - NTPクライアントサンプル
- `build/ntpclock/Release/ntpclock_gateway.exe` - NTPゲートウェイ

テスト:
- `build/ntpserver/Release/ntpserver_tests.exe`
- `build/ntpclock/Release/ntpclock_tests.exe`

### テストの実行

```bash
ctest --test-dir build -C Release
```

またはテスト実行ファイルを直接実行:
```bash
./build/ntpserver/Release/ntpserver_tests.exe
./build/ntpclock/Release/ntpclock_tests.exe
```

### オフライン環境での GoogleTest 依存関係

ntpserver / ntpclock のテストは CMake の `FetchContent` 経由で GoogleTest を取得します。インターネットに接続できない環境では、事前にダウンロードしたアーカイブを次のように渡してください。

```bash
cmake -S . -B build -G "Visual Studio 17 2022" -A x64 ^
  -DNTP_GTEST_ARCHIVE=C:/deps/googletest-1.17.0.tar.gz
```

`NTP_GTEST_ARCHIVE` を指定した場合、GitHub からの clone ではなくそのアーカイブが使用されます。

## 使用例

### 1. シンプルなNTPサーバー

UDPポート9123でNTPサーバーを起動:

```bash
./build/ntpserver/Release/ntpserver_example.exe --port 9123
```

デフォルトでは、システムクロック（`QueryPerformanceCounter`）を時刻ソースとして使用します。

### 2. NTPクライアント

NTPサーバーと時刻同期:

```bash
./build/ntpclock/Release/ntpclock_example.exe --ip 127.0.0.1 --port 9123 --poll 10000
```

オプション:
- `--ip IP`: NTPサーバーのIPアドレス（デフォルト: 127.0.0.1）
- `--port N`: NTPサーバーのポート（デフォルト: 123）
- `--poll ms`: ポーリング間隔（ミリ秒）（デフォルト: 10000）
- `--step ms`: Step閾値（ミリ秒）（デフォルト: 200）
- `--slew ms_per_s`: Slew速度（ms/秒）（デフォルト: 5.0）

### 3. 多段NTPゲートウェイ

上流サーバーと同期し、下流クライアントに配信するNTPゲートウェイを作成:

```bash
# ポート9123で上流サーバーを起動
./build/ntpserver/Release/ntpserver_example.exe --port 9123

# ゲートウェイを起動: 9123から同期、9124で配信
./build/ntpclock/Release/ntpclock_gateway.exe --upstream-ip 127.0.0.1 --upstream-port 9123 --serve-port 9124 --poll 10000

# 下流クライアントをゲートウェイに接続
./build/ntpclock/Release/ntpclock_example.exe --ip 127.0.0.1 --port 9124
```

ゲートウェイは以下を実行します:
1. `ClockService`を使用して上流サーバーと同期
2. `NtpServer`を使用して同期済み時刻を下流クライアントに配信
3. 上流からのPush通知を下流クライアントに伝播

### 4. Push通知のテスト

Push通知により、ポーリング間隔を待つことなく時刻変更が即座に伝播されます。

サーバーコンソールで時刻変更をトリガー:
```
add 31536000    # 1年分（31536000秒）を追加
```

接続されているクライアントはPush通知を受信し、次のポーリングを待たずに即座に同期します。

## アーキテクチャ

### TimeSpec

整数フィールドを使用した高精度時刻表現:
- `int64_t sec`: エポックからの秒数
- `uint32_t nsec`: ナノ秒（0-999999999）

浮動小数点演算による精度損失を回避しつつ、NTPタイムスタンプの全範囲をサポートします。

### ClockService（NTPクライアント）

コア同期ロジック:
1. **Exchangeプロトコル**: クライアントがリクエストを送信、サーバーがタイムスタンプ（T1-T4）で応答
2. **オフセット計算**: `offset = ((T2-T1) + (T3-T4)) / 2` をTimeSpec演算で実行
3. **時計補正**: オフセット量に基づく適応的なslew（漸進的）またはstep（即時）
4. **単調性の保証**: 時刻の逆行を防止（step補正後を除く）
5. **Pushサポート**: ベンダー拡張フィールドを受信し、即座にExchangeを実行

### NtpServer

軽量なNTPv4サーバー:
1. **クライアント追跡**: 最近見たクライアントエンドポイントのリストを保持
2. **ベンダー拡張**: 応答に絶対時刻とレートを含める
3. **Push通知**: 時刻ソース変更時に全追跡クライアントへ制御スナップショットをブロードキャスト
4. **TimeSource抽象化**: 任意の時刻ソース（システムクロック、同期クロックなど）から配信可能

### ゲートウェイアーキテクチャ

ゲートウェイはClockServiceとNtpServerを組み合わせます:
1. **ClockService**が上流サーバーと同期し、補正済み時刻を維持
2. **TimeSourceアダプタ**がClockServiceの時刻をNtpServerに公開
3. **ステータス監視**が補正を検出し、下流へのPush通知をトリガー
4. **Push伝播**により、多段展開でも即座に同期

## 設定

### ClockServiceオプション

```cpp
auto opts = ntpclock::Options::Builder()
    .PollIntervalMs(10000)      // 10秒ごとにポーリング
    .StepThresholdMs(200)       // オフセット > 200msでstep
    .SlewRateMsPerSec(5.0)      // 5ms/秒でslew
    .Build();
```

### NtpServer設定

```cpp
auto server_opts = ntpserver::Options::Builder()
    .Stratum(2)                                // Stratumレベル
    .Precision(-20)                            // 精度（2^-20秒）
    .RefId(ntpserver::MakeRefId("LOCL"))       // リファレンスID（"LOCL"）
    .Build();

ntpserver::NtpServer server;
server.Start(port, &time_source, server_opts);  // 時刻ソースを指定して起動
```

## API概要

### ClockService

```cpp
#include "ntpclock/clock_service.hpp"

ntpclock::ClockService clock;
clock.Start("127.0.0.1", 9123, options);
ntpserver::TimeSpec now = clock.NowUnix();
ntpclock::Status status = clock.GetStatus();
clock.Stop();
```

### NtpServer

```cpp
#include "ntpserver/ntp_server.hpp"

auto server_opts =
    ntpserver::Options::Builder().Stratum(1).RefId(ntpserver::MakeRefId("GPS"))
        .Build();
ntpserver::NtpServer server;
server.Start(9123, &time_source, server_opts);
server.NotifyControlSnapshot();  // Push通知をブロードキャスト
server.Stop();
```

### TimeSourceインターフェース

```cpp
class MyTimeSource : public ntpserver::TimeSource {
 public:
  ntpserver::TimeSpec NowUnix() override {
    // 現在のUNIX時刻を返す
  }
  double GetRate() const override {
    // クロックレートを返す（1.0 = 公称）
  }
};
```

## ライセンス

Copyright (c) 2025

## 作成者

ベンダー拡張による即時同期機能を備えた高精度NTP実装のデモンストレーションとして作成されました。
