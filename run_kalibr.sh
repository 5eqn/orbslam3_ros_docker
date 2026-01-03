xhost +local:root
docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" -v "~/foo:/foo" stereolabs/kalibr:kinetic
