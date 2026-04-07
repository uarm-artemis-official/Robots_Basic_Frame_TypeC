# syntax=docker/dockerfile:1

# FROM alpine:latest as base
# RUN apk add gcc g++ cmake git ninja-build gdb

# # Creates an "alias" for cube-cmake to use the generic cmake installed.
# # This is because tooling for buidling and testing uses "cube-cmake"
# # command which is installed by STM32 VSCode extension. The extension
# # is not easy to install so this is a workaround.
# # (Potential) TODO: migrate entire environment and only develop out of container.

FROM ubuntu:24.04 as base

# General dependencies
RUN apt update && apt install -y software-properties-common build-essential git ninja-build gdb gcc-13 g++-13 cmake git-lfs gcc-arm-none-eabi

# Install CubeMX 
# Install CubeMX dependencies
RUN apt install -y openjdk-11-jdk libxtst6 libxrender1 libgtk-3-0t64 unzip

# Run CubeMX installer
# Make sure to run `git lfs pull` if you are using fresh clone otherwise the
# installer will be a pointer and not a full file.
COPY ./extern/ /cubemx-installer/
RUN unzip /cubemx-installer/stm32cubemx-lin-v6-16-1.zip -d cubemx-installer
RUN /cubemx-installer/SetupSTM32CubeMX-6.16.1 /cubemx-installer/auto-install.xml

# Extract CLT
COPY ./extern/stm32cubeclt_1.19.0.zip /clt-installer/stm32cubeclt_1.19.0.zip
RUN unzip /clt-installer/stm32cubeclt_1.19.0.zip
RUN echo "deb http://security.ubuntu.com/ubuntu jammy-security main universe" >> /etc/apt/sources.list && apt-get update && apt-get install -y libncurses5

# Create helper commands. Simply run "cubemx" in the terminal to launch CubeMX.
RUN echo "cd /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/ && ./STM32CubeMX" > /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/run.sh
RUN chmod +x /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/run.sh
RUN ln -s /usr/local/STMicroelectronics/STM32Cube/STM32CubeMX/run.sh /usr/bin/cubemx