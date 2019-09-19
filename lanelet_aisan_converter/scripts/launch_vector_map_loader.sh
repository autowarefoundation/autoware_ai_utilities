#!/bin/bash

# VectorMap output directory
VECTOR_MAP_DIR=/tmp/test_lanelet_aisan_converter/aisan_vector_map

# Wait until CSV files are generated
while :
do
  if [ -f "$VECTOR_MAP_DIR/.completion_notice" ]; then
    rm $VECTOR_MAP_DIR/.completion_notice
    break
  fi
done

# Collect CSV file names
ARGS=""
for file in `find $VECTOR_MAP_DIR -maxdepth 1 -type f -name "*.csv"`; do
  ARGS="$ARGS $file"
done

# Load CSV files and publish VectorMap messages
rosrun map_file vector_map_loader $ARGS
