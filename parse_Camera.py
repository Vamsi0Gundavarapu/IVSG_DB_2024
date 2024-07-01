def parseCamera(self, trip_id, sensor_id, bag_file_id, bag_file, image_topic, camera_info_topic, output_file_name_images, output_file_name_camera_info, rotate=False, angle=0):
    
    # Import necessary libraries
    import psycopg2
    import cv2
    import numpy as np
    from cv_bridge import CvBridge, CvBridgeError
    
    # Initialize CvBridge to convert ROS image messages to OpenCV images
    bridge = CvBridge()
    
    # Open a file to write camera information metadata
    camera_info_file = open(output_file_name_camera_info, "w")

    # Extract camera information
    for _, msg, _ in bag_file.read_messages(topics=[camera_info_topic]):
        camera_info_file.write(f"{msg.width},{msg.height},{','.join(map(str, msg.K))},{','.join(map(str, msg.D))}\n")
        break  # Only process the first message

    camera_info_file.close()

    # Process images from the specified image topic
    image_file = open(output_file_name_images, "w")
    for topic, msg, t in bag_file.read_messages(topics=[image_topic]):
        try:
            # Convert ROS image to OpenCV format
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Optionally rotate the image based on user input
            if rotate:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE if angle == 90 else cv2.ROTATE_180)

            # Generate a unique MD5 filename for each image
            md5_filename = self.saveMD5Image(img)
            
            #Extract the timestamp for each image
            timestamp = msg.header.stamp.to_sec()
            formatted_time = self.unixTimeToTimeStamp(timestamp)

            # Write image metadata to file
            image_file.write(f"{sensor_id},{bag_file_id},{formatted_time},{timestamp},{md5_filename}\n")

        except CvBridgeError as e:
            print("Failed to convert image:", e)

    # Close the image file
    image_file.close()

    # self.db to set up handle database transactions
    class DatabaseManager:
        def __init__(self, dbname, user, password, host):
            
            # Establish a connection to the PostgreSQL database
            self.conn = psycopg2.connect(dbname='postgres', user='postgres', password='1234', host='localhost')

    def bulk_insert(self, table, fields, data_file):

        # Perform a bulk insert of the processed data into the specified table in the database
        with open(data_file, 'r') as file:
            cursor = self.conn.cursor()
            for line in file:
                data = line.strip().split(',')
                cursor.execute(f"INSERT INTO {table} ({','.join(fields)}) VALUES ({','.join(['%s'] * len(data))})", data)
            self.conn.commit()
            cursor.close()

