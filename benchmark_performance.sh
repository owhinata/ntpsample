#!/bin/bash
set -e

echo "=== NTP Performance Benchmark ==="
echo ""

PORT=9123
NUM_REQUESTS=1000

# C++サーバーベンチマーク
if [ -f "build/ntpserver/ntpserver_example" ]; then
    echo "Testing C++ Server..."
    ./build/ntpserver/ntpserver_example --port $PORT > /dev/null 2>&1 &
    CPP_PID=$!
    sleep 2
    
    START=$(date +%s.%N)
    for i in $(seq 1 $NUM_REQUESTS); do
        printf '\x23\x00\x00\x00' | nc -u -w 0 localhost $PORT > /dev/null 2>&1 || true
    done
    END=$(date +%s.%N)
    
    CPP_DURATION=$(echo "$END - $START" | bc -l)
    CPP_THROUGHPUT=$(echo "scale=2; $NUM_REQUESTS / $CPP_DURATION" | bc -l)
    
    kill $CPP_PID 2>/dev/null || true
    wait $CPP_PID 2>/dev/null || true
    sleep 1
    
    echo "  Duration: $CPP_DURATION sec"
    echo "  Throughput: $CPP_THROUGHPUT req/s"
    echo ""
else
    echo "C++ server not found"
    CPP_THROUGHPUT=0
fi

# C#サーバーベンチマーク
echo "Testing C# Server..."
dotnet run --project csharp/NtpServerExample/NtpServerExample.csproj -- --port $PORT > /dev/null 2>&1 &
CS_PID=$!
sleep 3

START=$(date +%s.%N)
for i in $(seq 1 $NUM_REQUESTS); do
    printf '\x23\x00\x00\x00' | nc -u -w 0 localhost $PORT > /dev/null 2>&1 || true
done
END=$(date +%s.%N)

CS_DURATION=$(echo "$END - $START" | bc -l)
CS_THROUGHPUT=$(echo "scale=2; $NUM_REQUESTS / $CS_DURATION" | bc -l)

kill $CS_PID 2>/dev/null || true
wait $CS_PID 2>/dev/null || true

echo "  Duration: $CS_DURATION sec"
echo "  Throughput: $CS_THROUGHPUT req/s"
echo ""

# 比較
echo "==================================="
echo "Performance Comparison:"
echo "==================================="
echo "C++:  $CPP_THROUGHPUT req/s"
echo "C#:   $CS_THROUGHPUT req/s"

if [ "$(echo "$CPP_THROUGHPUT > 0" | bc -l)" -eq 1 ]; then
    DIFF=$(echo "scale=2; ($CS_THROUGHPUT - $CPP_THROUGHPUT) / $CPP_THROUGHPUT * 100" | bc -l)
    echo "Diff: $DIFF%"
    
    ABS_DIFF=$(echo "$DIFF" | sed 's/-//')
    if [ "$(echo "$ABS_DIFF <= 20" | bc -l)" -eq 1 ]; then
        echo "Status: ✅ Within ±20%"
    else
        echo "Status: ❌ Outside ±20%"
    fi
fi
