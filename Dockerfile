FROM stingray_core:latest

RUN apt update && apt install -y --no-install-recommends ros-iron-usb-cam ros-iron-zbar-ros graphviz graphviz-dev
RUN pip install transitions[diagrams]

RUN echo 'source /additional_packages_ws/install/setup.bash' >> /root/.bashrc

WORKDIR /stingray
CMD ["bash"]
