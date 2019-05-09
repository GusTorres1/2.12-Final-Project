#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/robot/Final/2.12-Final-Project/src/odrive_ros"

# snsure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robot/Final/2.12-Final-Project/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robot/Final/2.12-Final-Project/install/lib/python2.7/dist-packages:/home/robot/Final/2.12-Final-Project/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robot/Final/2.12-Final-Project/build" \
    "/usr/bin/python" \
    "/home/robot/Final/2.12-Final-Project/src/odrive_ros/setup.py" \
    build --build-base "/home/robot/Final/2.12-Final-Project/build/odrive_ros" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/robot/Final/2.12-Final-Project/install" --install-scripts="/home/robot/Final/2.12-Final-Project/install/bin"
