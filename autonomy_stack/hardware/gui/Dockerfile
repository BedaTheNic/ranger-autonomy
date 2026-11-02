ARG BASE_IMAGE=ghcr.io/bedathenic/ranger-autonomy@sha256:6ef3524d6e2cb94aff4fdeb99252928d3b8633bd6d7a7c412530c93fba996085
FROM ${BASE_IMAGE}
USER root
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rviz2 ros-jazzy-nav2-rviz-plugins \
    xvfb x11vnc novnc websockify openbox supervisor x11-utils \
    fonts-dejavu-core libgl1-mesa-dri libglx-mesa0 \
    && rm -rf /var/lib/apt/lists/*
COPY autonomy_stack/hardware/gui /opt/autonomy/gui
RUN chmod +x /opt/autonomy/gui/*.sh && mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root
ENV DISPLAY=:99 QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 \
    XDG_RUNTIME_DIR=/tmp/runtime-root RANGER_GUI_PORT=6080 \
    RANGER_GUI_NAVIGATION_TOOLS=false
HEALTHCHECK --interval=15s --timeout=5s --start-period=40s --retries=3 \
    CMD python3 /opt/autonomy/gui/health.py
ENTRYPOINT ["/opt/autonomy/gui/start.sh"]
