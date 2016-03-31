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

echo_and_run cd "/home/cam/Documents/catkin_ws/src/mbzirc_task2"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/cam/Documents/catkin_ws/install/lib/python2.7/dist-packages:/home/cam/Documents/catkin_ws/src/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/cam/Documents/catkin_ws/src" \
    "/usr/bin/python" \
    "/home/cam/Documents/catkin_ws/src/mbzirc_task2/setup.py" \
    build --build-base "/home/cam/Documents/catkin_ws/src/mbzirc_task2" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/cam/Documents/catkin_ws/install" --install-scripts="/home/cam/Documents/catkin_ws/install/bin"
