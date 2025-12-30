#!/bin/bash
echo "Starting CPU stress..."
yes > /dev/null &
PID=$!
echo "CPU stress process started with PID: $PID"
echo $PID > /tmp/cpu_stress.pid
