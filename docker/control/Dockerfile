FROM hydronautics/stingray_core:latest

RUN apt update && apt install -y --no-install-recommends graphviz graphviz-dev python3-pip
RUN pip install transitions[diagrams]

WORKDIR /stingray
COPY . /stingray
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && source /additional_packages/install/setup.bash && source /stingray_core/install/setup.bash && colcon build --packages-select stingray_devices stingray_interfaces stingray_launch stingray_missions stingray_movement stingray_utils"

RUN chmod +x  /stingray/docker/control/entrypoint.sh
ENTRYPOINT ["/stingray/docker/control/entrypoint.sh"]

RUN echo 'source /stingray/install/setup.bash' >> /root/.bashrc

CMD ["bash"]
