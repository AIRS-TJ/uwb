# uwb

	$ cd ~/catkin_ws/src/
  
	$ git clone  https://github.com/AIRS-TJ/uwb.git

运行：

 如果端口是默认端口/dev/ttyACM0
 
	$ rosrun marvelmind_nav hedge_rcv_bin
  
  
  如果端口是/dev/ttyACM1
  
	$ rosrun marvelmind_nav hedge_rcv_bin /dev/ttyACM1
  
	$ rosrun marvelmind_nav subscriber_test
  
  运行uwb里程计：
  
	$ rosrun odom hedge_odom
    
  
  
  
