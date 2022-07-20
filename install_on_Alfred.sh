echo "Removing old files..."
ssh arnix@alfred_ros2 'rm -r /home/arnix/Alfred/'
echo "Copying new files..."
scp -r ./workspace/install/ arnix@alfred_ros2:/home/arnix/Alfred/
echo "Sourcing environment..."
ssh arnix@alfred_ros2 '. /home/arnix/Alfred/setup.bash'
echo "Installation done!"
