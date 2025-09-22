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
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/zkhappyfol/catkin_ws/src/my_first_robot"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/zkhappyfol/catkin_ws/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/zkhappyfol/catkin_ws/install/lib/python3/dist-packages:/home/zkhappyfol/catkin_ws/build/my_first_robot/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/zkhappyfol/catkin_ws/build/my_first_robot" \
    "/usr/bin/python3" \
    "/home/zkhappyfol/catkin_ws/src/my_first_robot/setup.py" \
     \
    build --build-base "/home/zkhappyfol/catkin_ws/build/my_first_robot" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/zkhappyfol/catkin_ws/install" --install-scripts="/home/zkhappyfol/catkin_ws/install/bin"
