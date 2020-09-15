FROM ros:melodic

LABEL mantainer="carlo.laviola@gmail.com"

# Install needed software
RUN apt-get update &&\
    apt-get install -y

# Copy our code from the current folder to /app inside the container
RUN mkdir /app

# Set up forwarding to use the GUI on the host machine
RUN export uid=1000 gid=1000 && \
    mkdir -p /home/developer && \
    echo "developer:x:${uid}:${gid}:Developer,,,:/home/developer:/bin/bash" >> /etc/passwd && \
    echo "developer:x:${uid}:" >> /etc/group && \
    echo "developer ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/developer && \
    chmod 0440 /etc/sudoers.d/developer && \
    chown ${uid}:${gid} -R /home/developer

WORKDIR /home/developer

# Change user and set up the environment variables
USER developer

ENV HOME=/home/developer
