#!/bin/sh

apt-get update && apt-get install -y sudo && echo 'ubuntu ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

# Check if wget is installed
if ! command -v wget &> /dev/null
then
    echo "wget could not be found, installing..."
    apt-get update && apt-get install -y wget
else
    echo "wget is already installed"
fi

# Check if openjdk-21-jre is installed
if ! dpkg -l | grep -q openjdk-21-jre
then
    echo "openjdk-21-jre could not be found, installing..."
    apt install -y openjdk-21-jre
else
    echo "openjdk-21-jre is already installed"
fi

# Extract year and version from the URL
URL="https://packages.wpilib.workers.dev/installer/v2024.3.2/Linux/WPILib_Linux-2024.3.2.tar.gz"
VERSION=$(echo $URL | grep -oP 'v\K[0-9]{4}\.[0-9]+\.[0-9]+')

# Check if WPILib is already set up
if [ ! -d "$HOME/wpilib/${VERSION%%.*}" ]; then
    wget $URL
    tar -xzvf WPILib_Linux-$VERSION.tar.gz
    mkdir ${VERSION%%.*}
    cd ${VERSION%%.*}
    mv ../WPILib_Linux-$VERSION/WPILib_Linux-$VERSION-artifacts.tar.gz .
    tar xf WPILib_Linux-$VERSION-artifacts.tar.gz
    rm WPILib_Linux-$VERSION-artifacts.tar.gz
    cd ..
    mkdir -p ~/wpilib
    mv ${VERSION%%.*} ~/wpilib
    rm -rf WPILib_Linux-*
else
    echo "WPILib is already set up"
fi
