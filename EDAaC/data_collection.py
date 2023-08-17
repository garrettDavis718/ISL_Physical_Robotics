import os
import cv2
import pyzed.sl as sl
import pandas as pd
import numpy as np
import time
import math
import threading
import signal

if __name__ == "__main__":
    print("Running object detection ... Press 'Esc' to quit")
    zed = sl.Camera()
    print("correctly runnning")
    dir_path = "/home/mrrobot/Documents/ISL-Projects-main/TurtlebotZED/data_collection"
    EXPERIMENT = 'test_1'
    
    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 15                        

    init_params.coordinate_units = sl.UNIT.FOOT
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP  
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    #init_params.depth_maximum_distance = 100
    #init_params.depth_minimum_distance = 6

    # Open Camera
    status = zed.open(init_params)
    if status != sl.ERROR_CODE.SUCCESS:
        print("here1")
        print(repr(status))
        exit()
    print("here2")
    # Runtime parameters
    runtime_params = sl.RuntimeParameters()
    runtime_params.confidence_threshold = 50
    runtime_params.measure3D_reference_frame = sl.REFERENCE_FRAME.WORLD

    # Enable positional tracking module
    positional_tracking_parameters = sl.PositionalTrackingParameters()
    # If the camera is static in space, enabling this setting below provides better depth quality and faster computation
    positional_tracking_parameters.set_as_static = False
    positional_tracking_parameters.set_floor_as_origin = False    

    # Enable object detection module
    initial_position = sl.Transform()
    initial_translation = sl.Translation()
    initial_rotation = sl.Rotation()

    trans_x = 3
    trans_y = 34/12 #34/12
    trans_z = -5
    initial_translation.init_vector(trans_x, trans_y, trans_z)
    initial_position.set_translation(initial_translation)
    # Rotation Matrix
    roll = 0
    pitch = (math.pi/2)
    yaw = 0
    
    information_filename = "testFiles-1" + "pos_"+str(trans_x)+"-"+str(round(trans_y,2))+"-"+str(trans_z)+"+rot_"+str(roll)+"-"+str(round(pitch, 2))+"-"+str(yaw)
    
    initial_position.set_euler_angles(roll, pitch, yaw, radian=True)
    positional_tracking_parameters.set_initial_world_transform(initial_position)
    zed.enable_positional_tracking(positional_tracking_parameters)

    # Enable object detection module
    obj_param = sl.ObjectDetectionParameters()
    obj_param.detection_model = sl.DETECTION_MODEL.MULTI_CLASS_BOX
    # Defines if the object detection will track objects across images flow.
    obj_param.enable_tracking = True
    zed.enable_object_detection(obj_param)

   # Configure object detection runtime parameters
    obj_runtime_param = sl.ObjectDetectionRuntimeParameters()
    detection_confidence = 60
    obj_runtime_param.detection_confidence_threshold = detection_confidence
    # To select a set of specific object classes
    obj_runtime_param.object_class_filter = [sl.OBJECT_CLASS.PERSON]
    # To set a specific threshold
    obj_runtime_param.object_class_detection_confidence_threshold = {sl.OBJECT_CLASS.PERSON: detection_confidence} 


    # Create objects that will store SDK outputs
    camera_infos = zed.get_camera_information()
    point_cloud = sl.Mat(camera_infos.camera_resolution.width, camera_infos.camera_resolution.height, sl.MAT_TYPE.F32_C4, sl.MEM.CPU) # Potentially changing to GPU
    objects = sl.Objects()
    image_left = sl.Mat()

    # Utilities for 2D display
    display_resolution = sl.Resolution(camera_infos.camera_resolution.width, camera_infos.camera_resolution.height)

    # Camera pose
    cam_w_pose = sl.Pose()
    cam_c_pose = sl.Pose()
    num = 0

    while num <= 3:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Retrieve objects
            returned_state = zed.retrieve_objects(objects, obj_runtime_param)
            
            tracking_state = zed.get_position(cam_w_pose, sl.REFERENCE_FRAME.WORLD)
            #print(tracking_state)
            #print(cam_w_pose.pose_data())             

            if (returned_state == sl.ERROR_CODE.SUCCESS and objects.is_new):
                # Retrieve point cloudrrrrrr
                zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA, sl.MEM.CPU, display_resolution)
                point_cloud.write(os.path.join(dir_path, f'pointcloud_exp_{information_filename}'), sl.MEM.CPU) #NAME
               
                # Retrieve image
                zed.retrieve_image(image_left, sl.VIEW.LEFT, sl.MEM.CPU, display_resolution)
                image_np = image_left.get_data()
                cv2.imwrite(os.path.join(dir_path, f'image_exp_{information_filename}.png'), image_np) #NAME



                df = pd.DataFrame(columns=['Class','Class Confidence', 'Label', 'Id', 'Object_Position', 'Object_Dimensions', '2D_Bounding_Box', '3D_Bounding_Box', 'Distance_From_Camera', 'Camera_Position'])

                print(len(objects.object_list))
                
                if len(objects.object_list):
                    for obj in objects.object_list:
                        #print('Bounding Box: ', obj.bounding_box, '\nPosition:', obj.position)
                        #print('\n')
                        position = obj.position
                        straight = math.sqrt(position[0]**2 + position[2]**2)
                        data = {'Class': obj.label, 'Class Confidence': obj.confidence,
                                'Label': obj.sublabel,
                                'Id': obj.id,
                                'Object_Position': position,
                                'Object_Dimensions': obj.dimensions,
                                '2D_Bounding_Box': obj.bounding_box_2d,
                                '3D_Bounding_Box': obj.bounding_box,
                                'Distance_From_Camera': straight,
                                'Camera_Position': cam_w_pose.pose_data()} # Fix camera position data
                        #print(data)
                        df.loc[len(df)] = data
          
                   
                df.to_csv(os.path.join(dir_path, f'data_exp_{information_filename}.csv'))
                num += 1

    image_left.free(sl.MEM.CPU)
    point_cloud.free(sl.MEM.CPU)

    # Disable modules and close camera
    zed.disable_object_detection()
    zed.disable_positional_tracking()

    zed.close()
