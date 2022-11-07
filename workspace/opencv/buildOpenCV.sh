#!/usr/bin/env bash

rm -fR build
mkdir build && cd build
cmake  .. -Wno-dev \
-D BUILD_opencv_calib3d=OFF \
-D BUILD_opencv_core=ON \
-D BUILD_opencv_dnn=OFF \
-D BUILD_opencv_features2d=OFF \
-D BUILD_opencv_flann=OFF \
-D BUILD_opencv_gapi=OFF \
-D BUILD_opencv_highgui=OFF \
-D BUILD_opencv_imgcodecs=ON \
-D BUILD_opencv_imgproc=ON \
-D BUILD_opencv_java=ON \
-D BUILD_opencv_js=OFF \
-D BUILD_opencv_ml=OFF \
-D BUILD_opencv_objc=ON \
-D BUILD_opencv_objdetect=OFF \
-D BUILD_opencv_photo=OFF \
-D BUILD_opencv_python=ON \
-D BUILD_opencv_stitching=OFF \
-D BUILD_opencv_ts=OFF \
-D BUILD_opencv_video=OFF \
-D BUILD_opencv_videoio=ON \
-D BUILD_opencv_world=OFF
cmake --build . -j8
make install

