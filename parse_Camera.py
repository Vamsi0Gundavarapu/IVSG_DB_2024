def parseCamera(self, trip_id, sensor_id, bag_file_id, bag_file, image_topic, camera_info_topic, output_file_name_images, output_file_name_camera_info, rotate=False, angle=0):
    import psycopg2
    import cv2
    import numpy as np
    from cv_bridge import CvBridge, CvBridgeError

    bridge = CvBridge()
    camera_info_file = open(output_file_name_camera_info, "w")

    # Extract camera information
    for _, msg, _ in bag_file.read_messages(topics=[camera_info_topic]):
        camera_info_file.write(f"{msg.width},{msg.height},{','.join(map(str, msg.K))},{','.join(map(str, msg.D))}\n")
        break  # Only process the first message

    camera_info_file.close()

    # Process images from the image topic
    image_file = open(output_file_name_images, "w")
    for topic, msg, t in bag_file.read_messages(topics=[image_topic]):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if rotate:
                img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE if angle == 90 else cv2.ROTATE_180)
            md5_filename = self.saveMD5Image(img)
            timestamp = msg.header.stamp.to_sec()
            formatted_time = self.unixTimeToTimeStamp(timestamp)

            # Write to file
            image_file.write(f"{sensor_id},{bag_file_id},{formatted_time},{timestamp},{md5_filename}\n")

        except CvBridgeError as e:
            print("Failed to convert image:", e)

    image_file.close()

    # self.db to set up handle database transactions
    class DatabaseManager:
        def __init__(self, dbname, user, password, host):
            self.conn = psycopg2.connect(dbname='postgres', user='postgres', password='1234', host='localhost')

    def bulk_insert(self, table, fields, data_file):
        with open(data_file, 'r') as file:
            cursor = self.conn.cursor()
            for line in file:
                data = line.strip().split(',')
                cursor.execute(f"INSERT INTO {table} ({','.join(fields)}) VALUES ({','.join(['%s'] * len(data))})", data)
            self.conn.commit()
            cursor.close()

