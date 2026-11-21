# QNX SDP 8.0 build environment (Linux host tools) for macOS hosts.
# The SDP itself is licensed and is NOT baked in: it's bind-mounted from the
# Mac at runtime (see run.sh). Build with:  docker/run.sh build
FROM --platform=linux/amd64 ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential make cmake bear git ca-certificates \
        openssh-client rsync file less python3 python3-venv \
    && rm -rf /var/lib/apt/lists/*

# Auto-source the QNX environment for every interactive/login shell.
# run.sh passes QNX_SDP = the SDP's Mac path (mounted at the same path).
ENV QNX_SDP=/opt/qnx800
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh \
    && echo '[ -f "$QNX_SDP/qnxsdp-env.sh" ] && . "$QNX_SDP/qnxsdp-env.sh" >/dev/null' >> /etc/bash.bashrc

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
