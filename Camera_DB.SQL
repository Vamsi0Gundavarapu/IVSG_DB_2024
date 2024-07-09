-- tables
-- Table: Camera
CREATE TABLE Camera (
    camera_id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    file_name varchar(32)  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitutde real  NOT NULL,
    geography GEOGRAPHY(point,4326)  NOT NULL,
    roll real  NOT NULL,
    pitch real  NOT NULL,
    yaw real  NOT NULL,
    seconds bigint  NOT NULL,
    nanoseconds bigint  NOT NULL,
    seconds_triggered bigint  NOT NULL,
    nanoseconds_triggered bigint  NOT NULL,
    time real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT Camera_pk PRIMARY KEY (camera_id)
);

-- Table: CameraImages
CREATE TABLE CameraImages (
    camera_id int  NOT NULL,
    ImageID int  NOT NULL,
    image timestamp  NOT NULL,
    VehicleID int  NOT NULL,
    WorkZoneID int  NOT NULL,
    CameraID int  NOT NULL,
    ImagePath VARCHAR(255)  NOT NULL,
    latitude real  NOT NULL CHECK ((latitude >= -90 AND latitude <= 90)),
    longitude real  NOT NULL CHECK ((longitude >= -180 AND longitude <= 180)),
    altitude real  NOT NULL,
    speed real  NOT NULL,
    compass varchar(5)  NOT NULL,
    weather_conditions varchar(50)  NOT NULL,
    lighting_conditions varchar(50)  NOT NULL,
    road_type varchar(50)  NOT NULL,
    obstacle_detected boolean  NOT NULL,
    obstacle_type varchar(50)  NOT NULL,
    lane_num int  NOT NULL,
    traffic_density varchar(50)  NOT NULL,
    WorkZone_Status varchar(50)  NOT NULL,
    Mapping_Vehicle_Info_VehicleID int  NOT NULL,
    CONSTRAINT CameraImages_pk PRIMARY KEY (ImageID)
);

-- Table: Mapping_Vehicle_Info
CREATE TABLE Mapping_Vehicle_Info (
    VehicleID serial  NOT NULL,
    VehicleModel varchar(50)  NOT NULL,
    LicensePlate varchar(20)  NOT NULL,
    mileage bigint  NOT NULL,
    CONSTRAINT Mapping_Vehicle_Info_pk PRIMARY KEY (VehicleID)
);

-- foreign keys
-- Reference: CameraImages_Camera (table: CameraImages)
ALTER TABLE CameraImages ADD CONSTRAINT CameraImages_Camera
    FOREIGN KEY (camera_id)
    REFERENCES Camera (camera_id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: CameraImages_Mapping_Vehicle_Info (table: CameraImages)
ALTER TABLE CameraImages ADD CONSTRAINT CameraImages_Mapping_Vehicle_Info
    FOREIGN KEY (Mapping_Vehicle_Info_VehicleID)
    REFERENCES Mapping_Vehicle_Info (VehicleID)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;
-- End of file.
