# syntax=docker/dockerfile:1

FROM alpine:latest as base
RUN apk add gcc g++ cmake git ninja-build gdb

# Creates an "alias" for cube-cmake to use the generic cmake installed.
# This is because tooling for buidling and testing uses "cube-cmake"
# command which is installed by STM32 VSCode extension. The extension
# is not easy to install so this is a workaround.
# (Potential) TODO: migrate entire environment and only develop out of container.
RUN ln -s /usr/bin/cmake /usr/bin/cube-cmake
