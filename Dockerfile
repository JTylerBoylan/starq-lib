# Start from the official Ubuntu image
FROM ubuntu:latest

# Install necessary packages
RUN apt-get update && \
    apt-get install -y \
        g++ \
        cmake \
        git \
        libsocketcan-dev \
        can-utils \
        libeigen3-dev

# Create a new user with a specific UID and GID, and set up the workspace
RUN useradd -m -u 1000 -s /bin/bash user && \
    mkdir -p /app && \
    chown -R user:user /app
WORKDIR /app

# Switch to the new non-root user
USER user

# Set the default command to execute when creating a new container
CMD ["bash"]
