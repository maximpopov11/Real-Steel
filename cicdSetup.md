# GitLab Runner with Podman Setup Guide

## Installation Steps

### 1. Install GitLab Runner

```bash
# Add GitLab repository
curl -L "https://packages.gitlab.com/install/repositories/runner/gitlab-runner/script.rpm.sh" | sudo bash

# Install GitLab Runner
sudo dnf install gitlab-runner
```

### 2. Register the Runner

```bash
# Replace with your actual GitLab URL and registration token
gitlab-runner register --url https://git.las.iastate.edu --token YOUR_REGISTRATION_TOKEN
```

When prompted:
* Confirm your GitLab URL
* Enter a runner name (e.g., "ros_runner")
* Select "docker" as the executor
* Enter "ubuntu:20.04" as the default Docker image

### 3. Configure Podman

```bash
# Install Podman if not already installed
sudo dnf install podman

# Enable the Podman socket service for your user
systemctl --user enable --now podman.socket

# Verify the socket is active
systemctl --user status podman.socket

# Enable linger to keep services running after logout
sudo loginctl enable-linger $(id -u)
```

### 4. Configure User Namespace Mapping

```bash
# Add entries to subuid and subgid files for your username
sudo sh -c "echo '$(whoami):100000:65536' >> /etc/subuid"
sudo sh -c "echo '$(whoami):100000:65536' >> /etc/subgid"

# Apply the changes
podman system reset
podman system migrate
```

### 5. Edit GitLab Runner Configuration
Edit the GitLab Runner configuration file:

```bash
vim ~/.gitlab-runner/config.toml
```

Replace the content with this (update USER_ID and TOKEN with your values):

```toml
[[runners]]
  url = "https://git.las.iastate.edu"
  id = 2647
  environment = ["FF_NETWORK_PER_BUILD=1"]
  token = "YOUR_REGISTRATION_TOKEN"
  executor = "docker"
  [runners.cache]
    MaxUploadedArchiveSize = 0
  [runners.cache.s3]
  [runners.cache.gcs]
  [runners.cache.azure]
  [runners.docker]
    host = "unix:///run/user/YOUR_USER_ID/podman/podman.sock"
    security_opt = ["label=disable"]
    tls_verify = false
    image = "ubuntu:20.04"
    privileged = false
    disable_entrypoint_overwrite = false
    oom_kill_disable = false
    disable_cache = false
    volumes = ["/cache"]
    shm_size = 0
    network_mtu = 0
```

* Replace `YOUR_USER_ID` with your user ID (get it with `id -u`)
* Replace `YOUR_REGISTRATION_TOKEN` with your actual runner token

### 6. Restart GitLab Runner

```bash
sudo gitlab-runner restart
```

### 7. Verify Setup

```bash
# Check GitLab Runner status
gitlab-runner status

# Verify Podman socket is running
systemctl --user status podman.socket
```