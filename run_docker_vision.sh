docker run --rm -it --gpus all \
    -v $HOME/welt_auv:/welt_auv \
    --ipc=host \
    --device=/dev/video0 \
    --net=host \
    hydronautics/stingray:vision \
    bash
