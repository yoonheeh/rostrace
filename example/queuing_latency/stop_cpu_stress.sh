#!/bin/bash
if [ -f /tmp/cpu_stress.pid ]; then
  PID=$(cat /tmp/cpu_stress.pid)
  echo "Stopping CPU stress process with PID: $PID"
  kill $PID
  rm /tmp/cpu_stress.pid
else
  echo "CPU stress process not found or already stopped."
fi
