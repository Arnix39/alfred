echo "Removing old files..."
ssh arnix@alfred_ros2 'rm -r /home/arnix/Alfred/'
echo "Copying new files..."
scp -r $1/install/ arnix@alfred_ros2:/home/arnix/Alfred/
echo "Installation done!"
