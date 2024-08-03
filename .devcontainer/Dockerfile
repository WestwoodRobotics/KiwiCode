FROM ubuntu:22.04

# Install wget
RUN apt-get update && apt-get install -y wget
RUN apt install -y openjdk-21-jre

# Extract year and version from the URL
ARG URL="https://packages.wpilib.workers.dev/installer/v2024.3.2/Linux/WPILib_Linux-2024.3.2.tar.gz"
RUN VERSION=$(echo $URL | grep -oP 'v\K[0-9]{4}\.[0-9]+\.[0-9]+') && \
    wget $URL && \
    tar -xzvf WPILib_Linux-$VERSION.tar.gz && \
    mkdir ${VERSION%%.*} && \
    cd ${VERSION%%.*} && \
    mv ../WPILib_Linux-$VERSION/WPILib_Linux-$VERSION-artifacts.tar.gz . && \
    tar xf WPILib_Linux-$VERSION-artifacts.tar.gz && \
    rm WPILib_Linux-$VERSION-artifacts.tar.gz && \
    cd .. && \
    mkdir -p ~/wpilib && \
    mv ${VERSION%%.*} ~/wpilib && \
    rm -rf WPILib_Linux-*

