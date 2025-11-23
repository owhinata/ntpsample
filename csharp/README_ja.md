# NTP Implementation for .NET

.NET 8.0向けの高性能かつ本番環境対応のNTP（Network Time Protocol）サーバー・クライアント実装です。C++リファレンス実装から完全な機能互換性を保ちながら移植されています。

## 特徴

- ✅ **NTPv4プロトコル** - RFC 5905完全準拠
- ✅ **ベンダー拡張** - カスタムABS/RATE/PUSH通知サポート
- ✅ **高精度** - マイクロ秒レベルの時刻同期
- ✅ **クロスプラットフォーム** - Windows、Linux、macOS対応
- ✅ **本番環境対応** - 228個のユニットテストで徹底検証
- ✅ **C++互換** - C++リファレンス実装と100%相互運用可能
- ✅ **高パフォーマンス** - ネイティブC++版に対してわずか3.5%の性能差

## クイックスタート

### 前提条件

- .NET 8.0 SDK以降
- Linux、macOS、またはWindows

### ビルド

```bash
# 全プロジェクトをビルド
dotnet build

# テスト実行
dotnet test
```

### NTPサーバーの実行

```bash
# 基本的な使い方（デフォルトポート9123）
dotnet run --project NtpServerExample

# カスタムポート指定
dotnet run --project NtpServerExample -- --port 9123

# デバッグログ有効
dotnet run --project NtpServerExample -- --port 9123 --debug
```

**サーバーコマンド（標準入力）：**
```
help          - 利用可能なコマンドを表示
now           - 現在のサーバー時刻を表示
abs <SEC>     - 絶対時刻を設定（Unix秒）
add <SEC>     - 現在時刻にオフセットを追加（秒）
rate <RATE>   - 時刻進行レートを設定（1.0 = 通常）
reset         - 実システム時刻へリセット
quit          - サーバー停止
```

### NTPクライアントの実行

```bash
# サーバーと同期（デフォルト: localhost:9123）
dotnet run --project NtpClockExample

# カスタムサーバー指定
dotnet run --project NtpClockExample -- --server 192.168.1.100 --port 9123

# デバッグログ有効
dotnet run --project NtpClockExample -- --debug
```

### ゲートウェイの実行（クライアント + サーバー）

```bash
# ゲートウェイモード: 上流と同期し、下流クライアントに配信
dotnet run --project NtpClockGateway -- \
  --upstream-server 192.168.1.100 \
  --upstream-port 9123 \
  --server-port 9124
```

## プロジェクト構成

```
csharp/
├── NtpServer/              # サーバーライブラリ
│   ├── NtpServer.cs        # パブリックAPI
│   └── Internal/           # 内部実装
├── NtpClock/               # クライアントライブラリ
│   ├── ClockService.cs     # パブリックAPI
│   └── Internal/           # 内部実装
├── NtpServer.Tests/        # サーバーユニットテスト（68テスト）
├── NtpClock.Tests/         # クライアントユニットテスト（160テスト）
├── NtpServerExample/       # サーバーサンプルアプリ
├── NtpClockExample/        # クライアントサンプルアプリ
└── NtpClockGateway/        # ゲートウェイサンプルアプリ
```

## ライブラリの使用方法

### NTPサーバー

```csharp
using NtpServer;
using NtpServer.Internal;

// オプション付きでサーバーを作成
var options = new Options.Builder()
    .Stratum(1)
    .RefId(RefIdHelper.MakeRefId("LOCL"))
    .LogSink(msg => Console.WriteLine(msg))
    .Build();

var server = new NtpServer.NtpServer();
server.Start(port: 9123, options: options);

// 時刻制御
var clock = new StopwatchClock();
clock.SetAbsolute(TimeSpec.FromDouble(1234567890.0));
server.Start(port: 9123, timeSource: clock, options: options);

// サーバー停止
server.Stop();
```

### NTPクライアント

```csharp
using NtpClock;
using NtpClock.Internal;

// オプション付きでクライアントを作成
var options = new Options.Builder()
    .ServerEndpoint(new IPEndPoint(IPAddress.Loopback, 9123))
    .PollIntervalMs(10000)
    .LogSink(msg => Console.WriteLine(msg))
    .Build();

var client = new ClockService();
client.Start(options);

// 現在の同期時刻を取得
TimeSpec now = client.NowUnix();
Console.WriteLine($"現在時刻: {now.Sec}.{now.Nsec:D9}");

// 同期ステータスを取得
var status = client.GetStatus();
Console.WriteLine($"同期済み: {status.Synchronized}");
Console.WriteLine($"オフセット: {status.OffsetS:F6} 秒");
Console.WriteLine($"RTT: {status.RttMs:F3} ms");

// クライアント停止
client.Stop();
```

## パフォーマンス

C++リファレンス実装とのベンチマーク結果：

| 指標 | C++ | C# | 差分 |
|------|-----|----|------|
| **スループット** | 581 req/s | 560 req/s | -3.5% ✅ |
| **メモリ (RSS)** | 2.84 MB | 169.7 MB | +167 MB* |
| **レイテンシ** | ~0.5 ms | ~0.5 ms | ≈0% ✅ |

*.NETランタイムオーバーヘッド（~160 MB）を含む。サーバーアプリケーションでは許容範囲

詳細なベンチマークは [../BENCHMARK.md](../BENCHMARK.md) を参照してください。

## テスト

```bash
# 全テスト実行
dotnet test

# 特定のテストプロジェクトを実行
dotnet test NtpServer.Tests/
dotnet test NtpClock.Tests/

# カバレッジ付きで実行
dotnet test --collect:"XPlat Code Coverage"
```

**テスト結果：**
- ✅ 68 NtpServerテスト（100%合格）
- ✅ 160 NtpClockテスト（100%合格）
- ✅ 77.7%行カバレッジ（コアロジック: 100%）

## 設定

### サーバーオプション

```csharp
var options = new Options.Builder()
    .Stratum(1)                          // Stratumレベル（デフォルト: 1）
    .Precision(-20)                      // クロック精度（デフォルト: -20）
    .RefId(0x4C4F434C)                   // リファレンスID "LOCL"
    .ClientRetention(TimeSpan.FromMinutes(60))  // クライアント追跡時間
    .LogSink(msg => Console.WriteLine(msg))     // ログコールバック
    .Build();
```

### クライアントオプション

```csharp
var options = new Options.Builder()
    .ServerEndpoint(new IPEndPoint(IPAddress.Parse("192.168.1.100"), 9123))
    .PollIntervalMs(10000)               // ポーリング間隔（デフォルト: 10秒）
    .StepThresholdMs(200)                // Step vs Slewしきい値（デフォルト: 200ms）
    .SlewRateMsPerSec(5.0)               // Slewレート（デフォルト: 5 ms/s）
    .MaxRttMs(100)                       // 最大許容RTT（デフォルト: 100ms）
    .MinSamplesToLock(3)                 // ロックまでの最小サンプル数（デフォルト: 3）
    .OffsetWindow(5)                     // オフセットウィンドウサイズ（デフォルト: 5）
    .SkewWindow(10)                      // スキューウィンドウサイズ（デフォルト: 10）
    .LogSink(msg => Console.WriteLine(msg))
    .Build();
```

## アーキテクチャ

### 主要コンポーネント

**NtpServer:**
- `NtpServer` - メインサーバークラス（UDPリスナー、パケット処理）
- `StopwatchClock` - レート制御可能な調整可能時刻ソース
- `ClientTracker` - Push通知のためのクライアントエンドポイント追跡
- `NtpVendorExt` - ベンダー拡張フィールド処理

**NtpClock:**
- `ClockService` - メイン同期サービス
- `SyncEstimatorState` - OLS回帰によるオフセット/スキュー推定
- `ClockCorrector` - 時刻補正（Slew vs Step判定）
- `VendorHintProcessor` - ベンダーヒント処理とエポック検出
- `UdpSocket` - UDP通信とメッセージ分類

### ベンダー拡張

カスタムNTP拡張フィールド（Type: 0xFF01）のサポート：

- **ABS** - 絶対時刻設定
- **RATE** - 時刻進行レート調整
- **PUSH** - サーバー起動のクライアント通知
- **エポック追跡** - サーバー再起動検出

## 相互運用性

C++リファレンス実装と完全互換：

```bash
# C++サーバー ↔ C#クライアント
./build/ntpserver/ntpserver_example --port 9123 &
dotnet run --project NtpClockExample

# C#サーバー ↔ C++クライアント
dotnet run --project NtpServerExample -- --port 9123 &
./build/ntpclock/ntpclock_example --server localhost --port 9123

# 3層構成: C#サーバー → C#ゲートウェイ → C#クライアント
dotnet run --project NtpServerExample -- --port 9123 &
dotnet run --project NtpClockGateway -- --upstream-port 9123 --server-port 9124 &
dotnet run --project NtpClockExample -- --port 9124
```

## C++からの移植

C++実装の忠実な移植版として以下を実現：

- ✅ 同じアルゴリズムとロジック
- ✅ 同じ設定デフォルト値
- ✅ 同じプロトコル動作
- ✅ 同じベンダー拡張フォーマット
- ✅ ほぼ同一のパフォーマンス

詳細な移植ドキュメントは [MIGRATION_PLAN.md](MIGRATION_PLAN.md) を参照してください。

## トラブルシューティング

### ポートが既に使用中

```bash
# ポートを使用しているプロセスを検索
lsof -i :9123

# プロセスを終了
pkill -f NtpServerExample
```

### 権限エラー（1024未満のポート）

sudoで実行するか、1024以上のポートを使用：

```bash
dotnet run --project NtpServerExample -- --port 9123  # OK
sudo dotnet run --project NtpServerExample -- --port 123  # sudoが必要
```

### クロックが同期しない

1. サーバーが起動しているか確認: `netstat -an | grep 9123`
2. デバッグログを有効化: `--debug`フラグ
3. ファイアウォール設定を確認
4. ネットワーク接続を確認: `ping <server>`

## 開発

### コードスタイル

- C# 10機能使用
- Nullable参照型有効
- コレクション操作でLINQ使用
- 設定にBuilderパターン
- リソース管理にIDisposable

### テストの追加

```csharp
[TestClass]
public class MyTests
{
    [TestMethod]
    public void TestSomething()
    {
        // 準備
        var server = new NtpServer.NtpServer();

        // 実行
        bool started = server.Start(9123);

        // 検証
        Assert.IsTrue(started);

        // クリーンアップ
        server.Stop();
    }
}
```

## ライセンス

詳細は [../LICENSE](../LICENSE) を参照してください。

## 参考資料

- RFC 5905 - Network Time Protocol Version 4
- [C++リファレンス実装](../README.md)
- [移植計画](MIGRATION_PLAN.md)
- [ベンチマーク結果](../BENCHMARK.md)

## サポート

問題や質問について：
- GitHubでissueを開く
- 実装詳細は [MIGRATION_PLAN.md](MIGRATION_PLAN.md) を参照
- パフォーマンスデータは [../BENCHMARK.md](../BENCHMARK.md) を参照
