
#!/bin/bash
# Proper header for a Bash script.

protoc --python_out=./ --java_out=./protobuf/ ./protobuf/heatmap.proto