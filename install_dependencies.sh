SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]-$0}" )" >/dev/null 2>&1 && pwd )"
OPENVINODIR=~/intel/
SETUPVARS=~/intel/openvino_2022/setupvars.sh
OPENCV_INSTALL_DIR=~/.local/opencv4-openvino

ask_for_opencv_install(){
	while true; do
	echo "Attempting to compile OpenCV with iGPU support and install it to $OPENCV_INSTALL_DIR"
	read -p "Do you want to proceed? (y/n) " yn

	case $yn in 
		[yY] )
			break;;
		[nN] ) echo exiting...;
			exit;;
		* ) echo invalid response;;
	esac

	done
}

ask_for_openvino_install(){
	while true; do
	echo "Attempting to install OpenVINO and install it to $OPENVINODIR"
	read -p "Do you want to proceed? (y/n) " yn

	case $yn in 
		[yY] )
			break;;
		[nN] ) echo exiting...;
			exit;;
		* ) echo invalid response;;
	esac

	done
}

install_openvino(){
	ask_for_openvino_install
	cd /tmp/
	mkdir -p openvino && cd openvino
	rm -rf *
	wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.2/linux/l_openvino_toolkit_ubuntu20_2022.2.0.7713.af16ea1d79a_x86_64.tgz --no-check-certificate
	file="$(ls)"
	wget https://storage.openvinotoolkit.org/repositories/openvino/packages/2022.2/linux/l_openvino_toolkit_ubuntu20_2022.2.0.7713.af16ea1d79a_x86_64.tgz.sha256 --no-check-certificate
	sha256sum -c $file.sha256
	mkdir -p ~/intel/
	rm -rf ~/intel/*
	tar xf $file -C ~/intel/
	cd ~/intel/
	extracted_folder="$(ls)"
	ln -s $extracted_folder openvino_2022
	cd openvino_2022/install_dependencies/
	echo "Attempting to install the opencl drivers with the following command:"
	echo "sudo -E $(pwd)/install_NEO_OCL_driver.sh"
	sudo -E ./install_NEO_OCL_driver.sh
	cd $SCRIPT_DIR
	echo "source $SETUPVARS" >> setupvars.bash
}

install_opencv(){
	ask_for_opencv_install
	echo "INSTALL THE REQUIRED PACKAGES FOR OPENCV"
	sudo apt-get install \
	build-essential \
	cmake \
	ninja-build \
	libgtk-3-dev \
	libpng-dev \
	libjpeg-dev \
	libwebp-dev \
	libtiff5-dev \
	libopenexr-dev \
	libopenblas-dev \
	libx11-dev \
	libavutil-dev \
	libavcodec-dev \
	libavformat-dev \
	libswscale-dev \
	libavresample-dev \
	libtbb2 \
	libssl-dev \
	libva-dev \
	libmfx-dev \
	libgstreamer1.0-dev \
	libgstreamer-plugins-base1.0-dev \
	nasm

	source $SETUPVARS
	
	cd /tmp/
	mkdir -p opencv4 && cd opencv4
	rm -rf *
	
	git clone --recurse-submodules https://github.com/opencv/opencv.git
	mkdir build-opencv && cd build-opencv
	cmake \
	-D BUILD_INFO_SKIP_EXTRA_MODULES=ON \
	-D BUILD_EXAMPLES=OFF \
	-D BUILD_JASPER=OFF \
	-D BUILD_JAVA=OFF \
	-D BUILD_JPEG=ON \
	-D BUILD_APPS_LIST=version \
	-D BUILD_opencv_apps=ON \
	-D BUILD_opencv_java=OFF \
	-D BUILD_OPENEXR=OFF \
	-D BUILD_PNG=ON \
	-D BUILD_TBB=OFF \
	-D BUILD_WEBP=OFF \
	-D BUILD_ZLIB=ON \
	-D WITH_1394=OFF \
	-D WITH_CUDA=OFF \
	-D WITH_EIGEN=OFF \
	-D WITH_GPHOTO2=OFF \
	-D WITH_GSTREAMER=ON \
	-D OPENCV_GAPI_GSTREAMER=OFF \
	-D WITH_GTK_2_X=OFF \
	-D WITH_IPP=ON \
	-D WITH_JASPER=OFF \
	-D WITH_LAPACK=OFF \
	-D WITH_MATLAB=OFF \
	-D WITH_MFX=ON \
	-D WITH_OPENCLAMDBLAS=OFF \
	-D WITH_OPENCLAMDFFT=OFF \
	-D WITH_OPENEXR=OFF \
	-D WITH_OPENJPEG=OFF \
	-D WITH_QUIRC=OFF \
	-D WITH_TBB=OFF \
	-D WITH_TIFF=OFF \
	-D WITH_VTK=OFF \
	-D WITH_WEBP=OFF \
	-D CMAKE_USE_RELATIVE_PATHS=ON \
	-D CMAKE_SKIP_INSTALL_RPATH=ON \
	-D ENABLE_BUILD_HARDENING=ON \
	-D ENABLE_CONFIG_VERIFICATION=ON \
	-D ENABLE_PRECOMPILED_HEADERS=OFF \
	-D ENABLE_CXX11=ON \
	-D INSTALL_PDB=ON \
	-D INSTALL_TESTS=ON \
	-D INSTALL_C_EXAMPLES=ON \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D CMAKE_INSTALL_PREFIX=install \
	-D OPENCV_SKIP_PKGCONFIG_GENERATION=ON \
	-D OPENCV_SKIP_PYTHON_LOADER=OFF \
	-D OPENCV_SKIP_CMAKE_ROOT_CONFIG=ON \
	-D OPENCV_GENERATE_SETUPVARS=OFF \
	-D OPENCV_BIN_INSTALL_PATH=bin \
	-D OPENCV_INCLUDE_INSTALL_PATH=include \
	-D OPENCV_LIB_INSTALL_PATH=lib \
	-D OPENCV_CONFIG_INSTALL_PATH=cmake \
	-D OPENCV_3P_LIB_INSTALL_PATH=3rdparty \
	-D OPENCV_SAMPLES_SRC_INSTALL_PATH=samples \
	-D OPENCV_DOC_INSTALL_PATH=doc \
	-D OPENCV_OTHER_INSTALL_PATH=etc \
	-D OPENCV_LICENSES_INSTALL_PATH=etc/licenses \
	-D OPENCV_INSTALL_FFMPEG_DOWNLOAD_SCRIPT=ON \
	-D BUILD_opencv_world=OFF \
	-D BUILD_opencv_python2=OFF \
	-D BUILD_opencv_python3=OFF \
	-D HIGHGUI_PLUGIN_LIST=all \
	-D CPU_BASELINE=SSE4_2 \
	-D OPENCV_IPP_GAUSSIAN_BLUR=ON \
	-D WITH_OPENVINO=ON \
	-D VIDEOIO_PLUGIN_LIST=ffmpeg,gstreamer,mfx \
	-D CMAKE_EXE_LINKER_FLAGS=-Wl,--allow-shlib-undefined \
	-D CMAKE_BUILD_TYPE=Release ../opencv/
	cmake --build . -j8
	cmake --install . --prefix $OPENCV_INSTALL_DIR
	echo "export OpenCV_DIR=$OPENCV_INSTALL_DIR/cmake" >> $SETUPVARS
	echo "export LD_LIBRARY_PATH=\"$OPENCV_INSTALL_DIR/lib\${LD_LIBRARY_PATH:+:\$LD_LIBRARY_PATH}\"" >> $SETUPVARS
}

if [ -f "$SETUPVARS" ]; then
	echo "Found OpenVino!"
else
	install_openvino 
fi

source "$SETUPVARS" >> /dev/null
if [ -z "$OpenCV_DIR" ]; then 
	install_opencv
else
	echo "Found OpenCV!"
fi

echo "#!/bin/bash" > setupvars.bash
echo "source $SETUPVARS" >> setupvars.bash
rosdep install --from-paths .