FROM ubuntu:22.04

# Extract year and version from the URL
ARG URL="https://packages.wpilib.workers.dev/installer/v2024.3.2/Linux/WPILib_Linux-2024.3.2.tar.gz"
RUN YEAR=$(echo $URL | grep -oP 'v\K[0-9]{4}') && \
    VERSION=$(echo $URL | grep -oP 'v[0-9]{4}\.\K[0-9]+\.[0-9]+') && \
    wget $URL && \
    tar -xzvf WPILib_Linux-$VERSION.tar.gz && \
    mkdir $YEAR && \
    cd $YEAR && \
    mv ../WPILib_Linux-$VERSION/WPILib_Linux-$VERSION-artifacts.tar.gz . && \
    tar xf WPILib_Linux-$VERSION-artifacts.tar.gz && \
    rm WPILib_Linux-$VERSION-artifacts.tar.gz && \
    cd .. && \
    mkdir -p ~/wpilib && \
    mv $YEAR ~/wpilib && \
    rm -rf WPILib_Linux-*
