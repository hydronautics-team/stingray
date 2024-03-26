FROM hydronautics/stingray_core:latest

RUN apt update && apt install -y --no-install-recommends graphviz graphviz-dev python3-pip
RUN pip install transitions[diagrams]

RUN echo 'source /additional_packages/install/setup.bash' >> /root/.bashrc

WORKDIR /stingray
CMD ["bash"]
