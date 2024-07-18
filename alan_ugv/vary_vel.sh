#!/bin/bash

# Publish "data: 1.0" for the first 10 seconds
rostopic pub /acc_scale std_msgs/Float32 "data: 1.0" &
PUBLISHER_PID=$!
sleep 10
kill $PUBLISHER_PID

# Publish "data: 2.0" for the next 10 seconds
rostopic pub /acc_scale std_msgs/Float32 "data: 2.0" &
PUBLISHER_PID=$!
sleep 10
kill $PUBLISHER_PID

# Publish "data: 3.0" for the last 10 seconds
rostopic pub /acc_scale std_msgs/Float32 "data: 3.0" &
PUBLISHER_PID=$!
sleep 10
kill $PUBLISHER_PID


# Publish "data: 4.0" for the last 10 seconds
rostopic pub /acc_scale std_msgs/Float32 "data: 4.0" &
PUBLISHER_PID=$!
sleep 10
kill $PUBLISHER_PID

# Publish "data: 1.0" for the last 10 seconds
rostopic pub /acc_scale std_msgs/Float32 "data: 1.0" &
PUBLISHER_PID=$!
sleep 10
kill $PUBLISHER_PID
