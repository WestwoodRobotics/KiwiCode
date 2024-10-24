#!/bin/sh

echo "Post-create script starting..."

# Extract year and version from the URL
URL="https://packages.wpilib.workers.dev/installer/v2024.3.2/Linux/WPILib_Linux-2024.3.2.tar.gz"
VERSION=$(echo $URL | grep -oP 'v\K[0-9]{4}\.[0-9]+\.[0-9]+')
YEAR=${VERSION%%.*}

# # Check if WPILib is already set up
# if [ ! -d "$HOME/wpilib/${VERSION%%.*}" ]; then
#     echo "Downloading WPILib..."
#     sudo wget $URL
#     echo "Extracting WPILib..."
#     sudo tar -xzvf WPILib_Linux-$VERSION.tar.gz
#     echo "Running WPILibInstaller..."
#     cd WPILib_Linux-$VERSION
#     sudo chmod +x WPILibInstaller
    
# else
#     echo "WPILib is already set up"
# fi

# Check if WPILib is already set up
if [ ! -d "$HOME/wpilib/${VERSION%%.*}" ]; then
    echo "Downloading WPILib..."
    sudo wget $URL
    echo "Extracting WPILib..."
    sudo tar -xzvf WPILib_Linux-$VERSION.tar.gz
    echo "Setting up WPILib directory..."
    sudo mkdir ${VERSION%%.*}
    cd ${VERSION%%.*}
    sudo mv ../WPILib_Linux-$VERSION/WPILib_Linux-$VERSION-artifacts.tar.gz .
    echo "Extracting WPILib artifacts..."
    sudo tar xf WPILib_Linux-$VERSION-artifacts.tar.gz
    sudo rm WPILib_Linux-$VERSION-artifacts.tar.gz
    cd ..
    sudo mkdir -p ~/wpilib
    sudo mv ${VERSION%%.*} ~/wpilib
    sudo rm -rf WPILib_Linux-*
    # Run ToolsUpdater.py
    cd ~/wpilib/$YEAR/tools/ && sudo python3 ToolsUpdater.py

    # Install VS Code extensions
    cd ~/wpilib/$YEAR/vsCodeExtensions && sudo find . -name "*.vsix" | sudo xargs -I {} code --install-extension {}
else
    echo "WPILib is already set up"
fi