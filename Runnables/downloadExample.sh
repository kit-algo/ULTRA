#!/bin/bash

# Define the target directory and URL
TARGET_DIR="../Datasets/Karlsruhe/GTFS"
ZIP_URL="https://projekte.kvv-efa.de/GTFS/google_transit.zip"
ZIP_FILE="$TARGET_DIR/google_transit.zip"

# Create the target directory if it doesn't exist
mkdir -p $TARGET_DIR

# Download the zip file
wget -O $ZIP_FILE $ZIP_URL

# Extract the zip file into the target directory
unzip -o $ZIP_FILE -d $TARGET_DIR

# Clean up the zip file
rm $ZIP_FILE

echo "Download and extraction complete."