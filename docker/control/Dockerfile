FROM hydronautics/stingray_core:latest

RUN apt update && apt install -y --no-install-recommends graphviz graphviz-dev python3-pip
RUN /usr/bin/python3 -m pip install --break-system-packages transitions[diagrams]

WORKDIR /stingray

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x  /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]
