FROM ubuntu:18.04

ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -y locales

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN apt-get install -y \
    build-essential \
    cmake \
    libopencv-dev

ENV SRC_PATH=/source/qr_recognition
ENV BUILD_PATH=${SRC_PATH}/dockerbuild

RUN mkdir -p ${BUILD_PATH}

COPY . ${SRC_PATH}

WORKDIR ${BUILD_PATH}

RUN cmake .. && make -j