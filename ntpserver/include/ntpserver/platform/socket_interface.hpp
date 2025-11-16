#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace ntpserver {
namespace platform {

// エンドポイント情報(IPアドレスとポート)
struct Endpoint {
  std::string address;  // IPv4アドレス文字列 (例: "192.168.1.1")
  uint16_t port;

  Endpoint() : port(0) {}
  Endpoint(const std::string& addr, uint16_t p) : address(addr), port(p) {}
};

// プラットフォーム非依存のソケットインターフェース
class ISocket {
 public:
  virtual ~ISocket() = default;

  // ソケットの初期化(プラットフォーム固有の初期化処理を含む)
  // 戻り値: 成功時true、失敗時false
  virtual bool Initialize() = 0;

  // 指定ポートにバインド
  // port: バインドするポート番号
  // 戻り値: 成功時true、失敗時false
  virtual bool Bind(uint16_t port) = 0;

  // 受信可能データの待機(タイムアウト付き)
  // timeout_us: タイムアウト時間(マイクロ秒)
  // 戻り値: データ受信可能な場合true、タイムアウトまたはエラー時false
  virtual bool WaitReadable(int64_t timeout_us) = 0;

  // データグラムの受信
  // from: 送信元エンドポイント情報を格納(出力パラメータ)
  // data: 受信データを格納するバッファ(出力パラメータ)
  // max_size: 受信する最大サイズ(バイト)
  // 戻り値: 成功時true、失敗時false
  virtual bool Receive(Endpoint* from, std::vector<uint8_t>* data,
                       size_t max_size) = 0;

  // データグラムの送信
  // to: 送信先エンドポイント情報
  // data: 送信するデータ
  // 戻り値: 成功時true、失敗時false
  virtual bool Send(const Endpoint& to, const std::vector<uint8_t>& data) = 0;

  // ソケットのクローズ(プラットフォーム固有のクリーンアップを含む)
  virtual void Close() = 0;

  // 最後に発生したエラーの説明を取得
  // 戻り値: エラーメッセージ文字列
  virtual std::string GetLastError() const = 0;

  // ソケットが有効かどうかを確認
  // 戻り値: 有効な場合true、無効な場合false
  virtual bool IsValid() const = 0;
};

// プラットフォーム固有のソケット実装を生成するファクトリ関数
// 戻り値: プラットフォームに応じたISocket実装のインスタンス
std::unique_ptr<ISocket> CreatePlatformSocket();

}  // namespace platform
}  // namespace ntpserver
