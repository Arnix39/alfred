FROM ros:humble
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
RUN apt-get install -y python3-pip
RUN apt-get install -y g++-aarch64-linux-gnu g++-arm-linux-gnueabihf lcov
ENV SHELL /bin/bash

# Set default user
USER $USERNAME

CMD ["/bin/bash"]