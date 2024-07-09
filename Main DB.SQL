-- tables
-- Table: GIS
CREATE TABLE GIS (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    type varchar(50)  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
    geography GEOGRAPHY(point,4326)  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT GIS_pk PRIMARY KEY (id)
);

-- Table: HD_Mapping_Data
CREATE TABLE HD_Mapping_Data (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    region_name varchar(100)  NOT NULL,
    map_version varchar(50)  NOT NULL,
    map_data bytea  NOT NULL,
    date_created timestamp  NOT NULL,
    date_updated timestamp  NOT NULL,
    CONSTRAINT HD_Mapping_Data_pk PRIMARY KEY (id)
);

-- Table: Vehicle
CREATE TABLE Vehicle (
    id serial  NOT NULL,
    name text  NOT NULL,
    mileage int  NOT NULL,
    fuel_level int  NOT NULL,
    CONSTRAINT id PRIMARY KEY (id)
);

-- Table: bag_files
CREATE TABLE bag_files (
    id serial  NOT NULL,
    trip_id int  NOT NULL,
    file_path text  NOT NULL,
    date_time date  NOT NULL,
    date_time_end date  NOT NULL,
    parsed boolean  NOT NULL,
    date_added timestamp  NOT NULL,
    Vehicle_id int  NOT NULL,
    CONSTRAINT bag_files_pk PRIMARY KEY (id)
);

-- Table: base_stations
CREATE TABLE base_stations (
    id serial  NOT NULL,
    name text  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitutde real  NOT NULL,
    geography serial  NOT NULL,
    latitude_std real  NOT NULL,
    longitude_std real  NOT NULL,
    altitude_std real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT base_stations_pk PRIMARY KEY (id)
);

-- Table: camera
CREATE TABLE camera (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    bag_files int  NOT NULL,
    file_name varchar(32)  NOT NULL,
    camera_model varchar(50)  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
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
    field_of_view FLOAT  NOT NULL,
    resolution varchar(20)  NOT NULL,
    installation_height FLOAT  NOT NULL,
    CONSTRAINT camera_pk PRIMARY KEY (id)
);

-- Table: camera_parameters
CREATE TABLE camera_parameters (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    focal_x real  NOT NULL,
    focal_y real  NOT NULL,
    center_x real  NOT NULL,
    center_y real  NOT NULL,
    skew real  NOT NULL,
    image_width int  NOT NULL,
    image_height int  NOT NULL,
    direction_k1 real  NOT NULL,
    direction_k2 real  NOT NULL,
    direction_k3 real  NOT NULL,
    direction_p1 real  NOT NULL,
    direction_p2 real  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT camera_parameters_pk PRIMARY KEY (id)
);

-- Table: encoder
CREATE TABLE encoder (
    id serial  NOT NULL,
    sensors_2_id int  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    revolutions int  NOT NULL,
    left_counts int  NOT NULL,
    right_counts int  NOT NULL,
    left_delta_counts int  NOT NULL,
    right_delta_counts int  NOT NULL,
    left_angular_velocity float  NOT NULL,
    right_angular_velocity float  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitutde real  NOT NULL,
    geography GEOGRAPHY(POINT,4326)  NOT NULL,
    seconds bigint  NOT NULL,
    nanoseconds bigint  NOT NULL,
    time FLOAT  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    encoder_model varchar(50)  NOT NULL,
    installation_position varchar(50)  NOT NULL,
    CONSTRAINT encoder_pk PRIMARY KEY (id)
);

-- Table: encoder_parameters
CREATE TABLE encoder_parameters (
    id serial  NOT NULL,
    sensors_id int  NOT NULL,
    counts_per_revolution int  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT encoder_parameters_pk PRIMARY KEY (id)
);

-- Table: laser
CREATE TABLE laser (
    id serial  NOT NULL,
    bag_files_2_id int  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    scan_time real  NOT NULL,
    range real  NOT NULL,
    angle real  NOT NULL,
    intensity real  NOT NULL,
    latitude real  NOT NULL CHECK ((latitude >= -90 AND latitude <= 90)),
    longitude real  NOT NULL CHECK ((longitude >= -180 AND longitude <= 180)),
    altitude real  NOT NULL,
    geography GEOGRAPHY(point,4326)  NOT NULL,
    roll real  NOT NULL,
    pitch real  NOT NULL,
    yaw real  NOT NULL,
    seconds bigint  NOT NULL,
    nanoseconds bigint  NOT NULL,
    time real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT laser_pk PRIMARY KEY (id)
);

-- Table: laser_parameters
CREATE TABLE laser_parameters (
    id serial  NOT NULL,
    angle_min real  NOT NULL,
    angle_max real  NOT NULL,
    angle_increment int  NOT NULL,
    time_increment real  NOT NULL,
    range_min real  NOT NULL,
    range_max real  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT laser_parameters_pk PRIMARY KEY (id)
);

-- Table: road_friction
CREATE TABLE road_friction (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    friction_coefficient real  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
    geography GEOGEPHY(point,4326))  NOT NULL,
    seconds bigint  NOT NULL,
    nanoseconds bigint  NOT NULL,
    time real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    road_surface_type varchar(50)  NOT NULL,
    road_condition varchar(50)  NOT NULL,
    speed_limit FLOAT  NOT NULL,
    lane_markings boolean  NOT NULL,
    CONSTRAINT road_friction_pk PRIMARY KEY (id)
);

-- Table: sensors
CREATE TABLE sensors (
    id serial  NOT NULL,
    type int  NOT NULL,
    serial_num varchar(255)  NOT NULL,
    company_name varchar(255)  NOT NULL,
    product_name varchar(255)  NOT NULL,
    date_added timestamp  NOT NULL,
    encoder_parameters_id int  NOT NULL,
    CONSTRAINT sensors_pk PRIMARY KEY (id)
);

-- Table: steering_angle
CREATE TABLE steering_angle (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    left_counts FLOAT  NOT NULL,
    right_counts FLOAT  NOT NULL,
    left_counts_filtered FLOAT  NOT NULL,
    rights_counts_filtered FLOAT  NOT NULL,
    left_angle real  NOT NULL,
    right_angle real  NOT NULL,
    angle real  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
    geography GEOGRAPHY(point,4326)  NOT NULL,
    seconds bigint  NOT NULL,
    nanoseconds bigint  NOT NULL,
    time real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT steering_angle_pk PRIMARY KEY (id)
);

-- Table: track_laps
CREATE TABLE track_laps (
    id serial  NOT NULL,
    name text  NOT NULL,
    date date  NOT NULL,
    DGPS_base_station_id int  NOT NULL,
    description text  NOT NULL,
    driver text  NOT NULL,
    passengers text  NOT NULL,
    notes text  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT track_laps_pk PRIMARY KEY (id)
);

-- foreign keys
-- Reference: GIS_bag_files (table: GIS)
ALTER TABLE GIS ADD CONSTRAINT GIS_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: HD_Mapping_Data_bag_files (table: HD_Mapping_Data)
ALTER TABLE HD_Mapping_Data ADD CONSTRAINT HD_Mapping_Data_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: bag_files_Vehicle (table: bag_files)
ALTER TABLE bag_files ADD CONSTRAINT bag_files_Vehicle
    FOREIGN KEY (Vehicle_id)
    REFERENCES Vehicle (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: camera_bag_files (table: camera)
ALTER TABLE camera ADD CONSTRAINT camera_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: camera_parameters_bag_files (table: camera_parameters)
ALTER TABLE camera_parameters ADD CONSTRAINT camera_parameters_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: camera_parameters_sensors (table: camera_parameters)
ALTER TABLE camera_parameters ADD CONSTRAINT camera_parameters_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: camera_sensors (table: camera)
ALTER TABLE camera ADD CONSTRAINT camera_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: encoder_parameters_sensors (table: encoder_parameters)
ALTER TABLE encoder_parameters ADD CONSTRAINT encoder_parameters_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: encoder_sensors (table: encoder)
ALTER TABLE encoder ADD CONSTRAINT encoder_sensors
    FOREIGN KEY (sensors_2_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: laser_bag_files (table: laser)
ALTER TABLE laser ADD CONSTRAINT laser_bag_files
    FOREIGN KEY (bag_files_2_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: road_friction_bag_files (table: road_friction)
ALTER TABLE road_friction ADD CONSTRAINT road_friction_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: road_friction_sensors (table: road_friction)
ALTER TABLE road_friction ADD CONSTRAINT road_friction_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: steering_angle_bag_files (table: steering_angle)
ALTER TABLE steering_angle ADD CONSTRAINT steering_angle_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: steering_angle_sensors (table: steering_angle)
ALTER TABLE steering_angle ADD CONSTRAINT steering_angle_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;
-- End of file.
