if [ ! -e /usr/include/opencv ] && [ -e /usr/include/opencv4 ] ; then 
    ln -s /usr/include/opencv4/ /usr/include/opencv
fi
