docker stop planbeamer-videoprocessing || true
docker rm planbeamer-videoprocessing || true
docker run -it --name planbeamer-videoprocessing --net=host -v ${PWD}:/home --privileged -p 2222:2222 -e DISPLAY=${DISPLAY} --device /dev/video0:/dev/video0 --volume="${HOME}/.Xauthority:/root/.Xauthority:rw" videoprocessing:latest
