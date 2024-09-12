FROM mcr.microsoft.com/devcontainers/base:ubuntu-24.04

# Mount for docker-in-docker 
VOLUME [ "/var/lib/docker" ]

# Copy the post_create.sh script to the container
COPY post_create.sh /usr/local/bin/post_create.sh

# Make the post_create.sh script executable
RUN chmod +x /usr/local/bin/post_create.sh

# Fire Docker/Moby script if needed along with Oryx's benv
ENTRYPOINT [ "/usr/local/share/docker-init.sh", "/usr/local/share/ssh-init.sh", "benv" ]
CMD [ "sleep", "infinity" ]

LABEL dev.containers.features="common"
