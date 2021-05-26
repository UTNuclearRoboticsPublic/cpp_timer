#!/bin/bash
# Sleep for a few seconds...
echo "Sleeping 3 seconds..."
sleep 3

# Publish a message
# Note any terminal command is valid here
echo "Publishing a message from a shell script"
rostopic pub /chatter std_msgs/String "data: 'Hello from a shell script'" --once
