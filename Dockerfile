# Portable CPU baseline. Replace this base with the organizer's required Jetson
# image when its exact platform contract is available. No GPU stack is assumed.
FROM ubuntu:22.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake libopencv-dev \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /opt/uavloc
COPY CMakeLists.txt ./
COPY include ./include
COPY src ./src
COPY tools ./tools
RUN cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && cmake --build build --target uav_localizer run_demo -j2
ENTRYPOINT ["/opt/uavloc/build/uav_localizer"]
