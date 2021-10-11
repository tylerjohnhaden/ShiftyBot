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

echo_and_run cd "/home/robot5/tyler_scratch/src/shifty_common"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/robot5/tyler_scratch/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/robot5/tyler_scratch/install/lib/python2.7/dist-packages:/home/robot5/tyler_scratch/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/robot5/tyler_scratch/build" \
    "/usr/bin/python2" \
    "/home/robot5/tyler_scratch/src/shifty_common/setup.py" \
    build --build-base "/home/robot5/tyler_scratch/build/shifty_common" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/robot5/tyler_scratch/install" --install-scripts="/home/robot5/tyler_scratch/install/bin"
