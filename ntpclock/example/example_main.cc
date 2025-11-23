// Copyright (c) 2025 <Your Name>
/**
 * @file
 * @brief Example program for ClockService with simple CLI options.
 *
 * Usage:
 *   ntpclock_example --ip 127.0.0.1 --port 9123 \
 *     --poll 10000 --step 200 --slew 5.0 --max-rtt 100 \
 *     --min-samples 3 --offset-window 5 --skew-window 10
 */

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "ntpclock/clock_service.hpp"

namespace {
/**
 * @brief Thread-safe logger for debug messages.
 */
class Logger {
 public:
  explicit Logger(bool enabled) : enabled_(enabled) {}

  void Log(const std::string& msg) {
    if (!enabled_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    std::fprintf(stderr, "%s\n", msg.c_str());
  }

 private:
  bool enabled_;
  std::mutex mutex_;
};

// On Windows consoles, enable ANSI escape processing for in-place refresh.
#if defined(_WIN32)
#include <windows.h>

void ShowCur() {
  std::printf("\x1b[?25h");
  std::fflush(stdout);
}

BOOL WINAPI ConsoleCtrlHandler(DWORD ctrlType) {
  // Handle Ctrl+C, Ctrl+Break, and console close events
  if (ctrlType == CTRL_C_EVENT || ctrlType == CTRL_BREAK_EVENT ||
      ctrlType == CTRL_CLOSE_EVENT) {
    ShowCur();
    return FALSE;  // Allow default handler to terminate
  }
  return FALSE;
}

void EnableVirtualTerminal() {
  HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
  if (hOut == INVALID_HANDLE_VALUE) return;
  DWORD mode = 0;
  if (!GetConsoleMode(hOut, &mode)) return;
  mode |= ENABLE_VIRTUAL_TERMINAL_PROCESSING;
  SetConsoleMode(hOut, mode);
}
int TermWidth() {
  CONSOLE_SCREEN_BUFFER_INFO info{};
  HANDLE hOut = GetStdHandle(STD_OUTPUT_HANDLE);
  if (GetConsoleScreenBufferInfo(hOut, &info)) {
    return static_cast<int>(info.srWindow.Right - info.srWindow.Left + 1);
  }
  return 120;
}
#else
void ShowCur() {
  std::printf("\x1b[?25h");
  std::fflush(stdout);
}

void SignalHandler(int sig) {
  (void)sig;
  ShowCur();
  std::exit(0);
}

void EnableVirtualTerminal() {}
int TermWidth() { return 120; }
#endif

std::string OneLine(const std::string& s) {
  std::string out;
  out.reserve(s.size());
  for (char c : s) {
    if (c == '\n' || c == '\r') {
      out.push_back(' ');
    } else {
      out.push_back(c);
    }
  }
  return out;
}
void HideCur() { std::printf("\x1b[?25l"); }
void MoveRow(int row) { std::printf("\x1b[%d;1H", row); }
static void AtExitShowCur() { ShowCur(); }
void PrintUsage() {
  std::fprintf(stderr,
               "Usage: ntpclock_example --ip A.B.C.D --port N [options]\n"
               "Options:\n"
               "  --ip A.B.C.D         (default 127.0.0.1)\n"
               "  --port N             (default 9123)\n"
               "  --poll ms            (default 10000)\n"
               "  --step ms            (default 200)\n"
               "  --slew ms_per_s      (default 5.0)\n"
               "  --max-rtt ms         (default 100)\n"
               "  --min-samples n      (default 3)\n"
               "  --offset-window n    (default 5)\n"
               "  --skew-window n      (default 10)\n"
               "  --utc                (default: JST)\n"
               "  --debug              Enable debug logging\n");
}
}  // namespace

int main(int argc, char** argv) {
  std::string ip = "127.0.0.1";
  uint16_t port = 9123;
  bool opt_utc = false;  // default: JST
  bool debug = false;

  auto builder = ntpclock::Options::Builder();

  for (int i = 1; i < argc; ++i) {
    std::string a(argv[i]);
    auto need = [&](int more) { return i + more < argc; };
    if (a == "--ip" && need(1)) {
      ip = argv[++i];
    } else if (a == "--port" && need(1)) {
      port = static_cast<uint16_t>(std::atoi(argv[++i]));
    } else if (a == "--poll" && need(1)) {
      builder.PollIntervalMs(std::atoi(argv[++i]));
    } else if (a == "--step" && need(1)) {
      builder.StepThresholdMs(std::atoi(argv[++i]));
    } else if (a == "--slew" && need(1)) {
      builder.SlewRateMsPerSec(std::atof(argv[++i]));
    } else if (a == "--max-rtt" && need(1)) {
      builder.MaxRttMs(std::atoi(argv[++i]));
    } else if (a == "--min-samples" && need(1)) {
      builder.MinSamplesToLock(std::atoi(argv[++i]));
    } else if (a == "--offset-window" && need(1)) {
      builder.OffsetWindow(std::atoi(argv[++i]));
    } else if (a == "--skew-window" && need(1)) {
      builder.SkewWindow(std::atoi(argv[++i]));
    } else if (a == "--utc") {
      opt_utc = true;
    } else if (a == "--debug") {
      debug = true;
    } else if (a == "-h" || a == "--help") {
      PrintUsage();
      return 0;
    } else {
      std::fprintf(stderr, "Unknown or incomplete option: %s\n", a.c_str());
      PrintUsage();
      return 2;
    }
  }

  // Create logger
  Logger logger(debug);
  auto log_callback = [&logger](const std::string& msg) { logger.Log(msg); };

  builder.LogSink(log_callback);

  ntpclock::ClockService svc;
  auto opt = builder.Build();
  std::printf("Starting sync to %s:%u\n", ip.c_str(), port);
  EnableVirtualTerminal();
  HideCur();
  std::atexit(AtExitShowCur);
#if defined(_WIN32)
  SetConsoleCtrlHandler(ConsoleCtrlHandler, TRUE);
#else
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);
#endif

  if (!svc.Start(ip, port, opt)) {
    std::fprintf(stderr, "Failed to start ClockService\n");
    return 1;
  }

  // Double-buffer: update only changed lines to reduce flicker.
  std::vector<std::string> prev(11);
  std::printf("\x1b[H");
  while (true) {
    const int cols = TermWidth();
    ntpclock::Status st = svc.GetStatus();
    double now_s = svc.NowUnix().ToDouble();
    std::string now_line;
    {
      const int offset = opt_utc ? 0 : 9 * 3600;
      const char* tzlabel = opt_utc ? "UTC" : "JST";
      int64_t isec = static_cast<int64_t>(now_s);
      int64_t adj = isec + offset;
      std::tm tm{};
#if defined(_WIN32)
      time_t t = static_cast<time_t>(adj);
      gmtime_s(&tm, &t);
#else
      time_t t = static_cast<time_t>(adj);
      tm = *std::gmtime(&t);
#endif
      double frac = now_s - static_cast<double>(isec);
      if (frac < 0) frac = 0;
      int ms = static_cast<int>(frac * 1000.0 + 0.5);
      if (ms >= 1000) {
        ms -= 1000;
      }
      char tbuf[64];
      std::snprintf(tbuf, sizeof(tbuf),
                    "now=%04d-%02d-%02d %02d:%02d:%02d.%03d %s",
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                    tm.tm_min, tm.tm_sec, ms, tzlabel);
      now_line = tbuf;
    }
    std::string err =
        st.last_error.empty() ? std::string("") : OneLine(st.last_error);
    int cap = (cols - 7 > 0) ? (cols - 7) : 0;
    if (static_cast<int>(err.size()) > cap) {
      err.resize(static_cast<size_t>(cap));
    }

    char buf[256];
    std::vector<std::string> cur;
    cur.reserve(11);
    cur.emplace_back(now_line);
    std::snprintf(buf, sizeof(buf), "sync=%s",
                  st.synchronized ? "true" : "false");
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "rtt_ms=%d  delay_s=%.3f", st.rtt_ms,
                  st.last_delay_s);
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "offset_s=%.6f  skew_ppm=%.1f", st.offset_s,
                  st.skew_ppm);
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "samples=%d  last_update=%.3f", st.samples,
                  st.last_update.ToDouble());
    cur.emplace_back(buf);
    const char* corr =
        st.last_correction == ntpclock::Status::Correction::Step
            ? "Step"
            : (st.last_correction == ntpclock::Status::Correction::Slew
                   ? "Slew"
                   : "None");
    std::snprintf(buf, sizeof(buf), "last_corr=%s  amount=%.6f", corr,
                  st.last_correction_amount_s);
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf),
                  "offset_window=%d  skew_window=%d  window_count=%d",
                  st.offset_window, st.skew_window, st.window_count);
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "median=%.6f  min=%.6f  max=%.6f",
                  st.offset_median_s, st.offset_min_s, st.offset_max_s);
    cur.emplace_back(buf);
    std::snprintf(buf, sizeof(buf), "applied=%.6f  target=%.6f",
                  st.offset_applied_s, st.offset_target_s);
    cur.emplace_back(buf);
    cur.emplace_back(std::string("error=") + err);
    while (cur.size() < prev.size()) cur.emplace_back("");
    for (auto& s : cur) {
      if (static_cast<int>(s.size()) > cols) {
        s.resize(static_cast<size_t>(cols));
      }
    }
    for (size_t i = 0; i < cur.size(); ++i) {
      if (prev[i] == cur[i]) continue;
      MoveRow(static_cast<int>(i) + 1);
      std::printf("\x1b[2K");
      std::fwrite(cur[i].data(), 1, cur[i].size(), stdout);
      std::printf("\n");
    }
    std::fflush(stdout);
    prev.swap(cur);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // not reached
  return 0;
}
