# Need to forward X11.
# reference: https://answers.ros.org/question/300113/docker-how-to-use-rviz-and-gazebo-from-a-container/
XAUTH=/tmp/.docker.xauth
echo "Preparing Xauthority data..."
xauth_list=$(xauth nlist :0 | tail -n 1 | sed -e 's/^..../ffff/')
if [ ! -f $XAUTH ]; then
    if [ ! -z "$xauth_list" ]; then
        echo $xauth_list | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi

echo "Done."
echo ""
echo "Verifying file contents:"
file $XAUTH
echo "--> It should say \"X11 Xauthority data\"."
echo ""
echo "Permissions:"
ls -FAlh $XAUTH
echo ""
echo "Running docker..."

docker run -it\
        --volume $(pwd):/repo/\
        --env "TERM=xterm-256color"\
        --env "DISPLAY=$DISPLAY"\
        --volume /tmp/.X11-unix/:/tmp/.X11-unix:rw\
        --env "XAUTHORITY=$XAUTH"\
        --volume $XAUTH:$XAUTH\
        --privileged\
        --network=host\
        --name="SPOT"\
        robotdev:noetic