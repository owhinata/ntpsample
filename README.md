# ntpsample

## Build (Windows, MSVC)

Prerequisites
- CMake 3.20+
- Visual Studio 2022 Build Tools（C++）または Visual Studio 2022

ソース配置
- ライブラリ/サンプルは `ntpserver/` ディレクトリ配下にあります。

### Visual Studio 生成（x64）

```bash
cmake -S ntpserver -B build -G "Visual Studio 17 2022" -A x64 -D NTP_SERVER_BUILD_EXAMPLE=ON
cmake --build build --config Release
```

生成物
- ライブラリ: `build/Release/ntpserver.lib`
- サンプル: `build/Release/ntpserver_example.exe`

### 実行（サンプル）

```bash
./build/Release/ntpserver_example.exe
```

既定では UDP ポート `9123` で待受します。

### テストの実行

```bash
ctest --test-dir build -C Release
```
