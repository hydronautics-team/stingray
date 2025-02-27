FROM hydronautics/zbar:humble

WORKDIR /stingray
COPY . /stingray
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --packages-select stingray_launch stingray_cam"


RUN echo 'source /stingray/install/setup.bash' >> /root/.bashrc

ENTRYPOINT ["/stingray/docker/qr_trigger/entrypoint.sh"]

CMD ["bash"]
