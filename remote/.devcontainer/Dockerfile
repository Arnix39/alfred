FROM rust:latest
ARG USERNAME=arnix
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user and add it to sudoers
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y libusb-1.0-0-dev libftdi1-dev libudev-dev libssl-dev udev openocd gdb-multiarch qemu-system-arm minicom
RUN mkdir -p /etc/udev/rules.d
RUN echo "SUBSYSTEM==\"usb\", ATTRS{idVendor}==\"0483\", ATTRS{idProduct}==\"374b\", MODE=\"0666\"" | tee -a /etc/udev/rules.d/99-stm32f3disc.rules > /dev/null
ENV SHELL /bin/bash

# Set default user
USER $USERNAME

RUN rustup target add thumbv7em-none-eabihf
RUN rustup component add llvm-tools-preview
RUN cargo install cargo-binutils

CMD ["/bin/bash"]