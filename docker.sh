#!/usr/bin/env bash

docker run -it --privileged \
    -e DISPLAY=:0 \
    -v /Users/marcelojacinto/Developer/pegasus_ws:/home/pegasus/pegasus_ws/ \
    --name=pegasus_container pegasus:latest 

docker container rm pegasus_container 

# -v /tmp/.X11-unix:/tmp/.X11-unix:ro \