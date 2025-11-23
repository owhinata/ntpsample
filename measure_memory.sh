#!/bin/bash

PORT=9123

echo "=== NTP Server Memory Usage ==="
echo ""

# C++サーバー測定
echo "Measuring C++ Server..."
(sleep 10; echo "quit") | ./build/ntpserver/ntpserver_example --port ${PORT} > /dev/null 2>&1 &
CPP_PID=$!
sleep 3

if ps -p ${CPP_PID} > /dev/null 2>&1; then
    CPP_RSS=$(ps -o rss= -p ${CPP_PID} | tr -d ' ')
    if [ -n "${CPP_RSS}" ] && [ "${CPP_RSS}" -gt 0 ]; then
        CPP_MB=$(echo "scale=2; ${CPP_RSS} / 1024" | bc)
        echo "  PID: ${CPP_PID}"
        echo "  RSS: ${CPP_MB} MB"
    else
        CPP_MB=0
    fi
else
    CPP_MB=0
fi

kill ${CPP_PID} 2>/dev/null || true
wait ${CPP_PID} 2>/dev/null || true
sleep 1

echo ""

# C#サーバー測定
echo "Measuring C# Server..."
(sleep 15) | dotnet run --project csharp/NtpServerExample/NtpServerExample.csproj -- --port ${PORT} > /dev/null 2>&1 &
CS_PID=$!
sleep 4

if ps -p ${CS_PID} > /dev/null 2>&1; then
    CS_RSS=$(ps -o rss= -p ${CS_PID} | tr -d ' ')
    if [ -n "${CS_RSS}" ] && [ "${CS_RSS}" -gt 0 ]; then
        CS_MB=$(echo "scale=2; ${CS_RSS} / 1024" | bc)
        echo "  PID: ${CS_PID}"
        echo "  RSS: ${CS_MB} MB"
    else
        CS_MB=0
    fi
else
    CS_MB=0
fi

kill ${CS_PID} 2>/dev/null || true
wait ${CS_PID} 2>/dev/null || true

echo ""
echo "==================================="
echo "Memory Usage:"
echo "==================================="
echo "C++: ${CPP_MB} MB"
echo "C#:  ${CS_MB} MB"

if [ "$(echo "${CPP_MB} > 0" | bc)" -eq 1 ]; then
    DIFF=$(echo "scale=2; ${CS_MB} - ${CPP_MB}" | bc)
    PERCENT=$(echo "scale=1; (${CS_MB} - ${CPP_MB}) / ${CPP_MB} * 100" | bc)
    echo ""
    echo "Difference: +${DIFF} MB (+${PERCENT}%)"
    echo "Note: C# includes .NET runtime (~30-40 MB overhead)"
fi
