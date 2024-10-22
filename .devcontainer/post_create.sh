#!/bin/sh

echo "Post-create script starting..."

# Extract year and version from the URL
URL="https://packages.wpilib.workers.dev/installer/v2024.3.2/Linux/WPILib_Linux-2024.3.2.tar.gz"
VERSION=$(echo $URL | grep -oP 'v\K[0-9]{4}\.[0-9]+\.[0-9]+')
YEAR=${VERSION%%.*}

# Check if WPILib is already set up
if [ ! -d "$HOME/wpilib/${VERSION%%.*}" ]; then
    echo "Downloading WPILib..."
    sudo wget $URL
    echo "Extracting WPILib..."
    sudo tar -xzvf WPILib_Linux-$VERSION.tar.gz
    echo "Running WPILibInstaller..."
    cd WPILib_Linux-$VERSION
    sudo chmod +x WPILibInstaller
    sudo ./WPILibInstaller
else
    echo "WPILib is already set up"
fi

echo "Post-create script finished."
