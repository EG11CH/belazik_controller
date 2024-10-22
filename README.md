ros2_ws/                   
├── src/
  ├── rack_pinion_controller/   
         ├── CMakeLists.txt 
         ├── package.xml
         ├── plugin.xml
         ├── config/        
         │   └── test.yaml  
         ├── urdf/          
         │   └── test.urdf  
         ├── launch/        
         │   └── launch.py 
         ├── include/       
         │   └── rack_pinion_controller/
         │       └── ackermann_steering_controller.hpp
         ├── src/          
             └── test.cpp   

(robot_urdf folder and math.cpp dont necessary for successfull build)
