wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18617/l_openvino_toolkit_p_2022.1.0.643_offline.sh
chmod +x l_openvino_toolkit_p_2022.1.0.643_offline.sh 
./l_openvino_toolkit_p_2022.1.0.643_offline.sh -a -s --eula accept --components default:intel.openvino.lin.iecpu:intel.openvino.lin.iegpu --install-dir .
rm openvino_2022
mv openvino_2022.1.0.643 openvino_2022
rm -r logs
install_dir=$(pwd)/openvino_2022
cd $install_dir/install_dependencies
echo "Attempting to install openvino dependencies. May need password..."
echo "Script:" "$(pwd)/install_openvino_dependencies.sh"
sudo -E ./install_openvino_dependencies.sh
cd $install_dir/extras/scripts
./download_opencv.sh
cd $install_dir/install_dependencies/
echo "Attempting to install gpu driver components. May need password..."
echo "Script:" "$(pwd)/install_NEO_OCL_driver.sh"
sudo -E ./install_NEO_OCL_driver.sh
cd $install_dir/..
rm l_openvino_toolkit_p_2022.1.0.643_offline.sh
cd src/robocup-object-detection/object_detection
rosdep install --from-paths .