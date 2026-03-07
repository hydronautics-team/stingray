FROM hydronautics/zbar:humble

WORKDIR /stingray
COPY . /stingray
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select stingray_launch stingray_cam"


COPY docker/qr_trigger/entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

RUN echo 'source /stingray/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
