-- tables
-- Table: Lidar_Velodyne
CREATE TABLE Lidar_Velodyne (
    id serial  NOT NULL,
    seq int  NOT NULL,
    timestamp timestamp  NOT NULL,
    seconds bigint  NOT NULL,
    nanonseconds bigint  NOT NULL,
    time real  NOT NULL,
    file_name varchar(255)  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT Lidar_Velodyne_pk PRIMARY KEY (id)
);

-- Table: Vehicle
CREATE TABLE Vehicle (
    id serial  NOT NULL,
    name text  NOT NULL,
    CONSTRAINT Vehicle_pk PRIMARY KEY (id)
);

-- Table: bag_files
CREATE TABLE bag_files (
    id serial  NOT NULL,
    Vehicle_id int  NOT NULL,
    trips_id int  NOT NULL,
    file_path text  NOT NULL,
    name text  NOT NULL,
    datetime timestamp  NOT NULL,
    datetime_end timestamp  NOT NULL,
    parsed bool  NOT NULL,
    dateadded timestamp  NOT NULL,
    CONSTRAINT bag_files_pk PRIMARY KEY (id)
);

-- Table: base_stations
CREATE TABLE base_stations (
    id serial  NOT NULL,
    name text  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
    geography point,4326  NOT NULL,
    latitude_std real  NOT NULL,
    longitude_std real  NOT NULL,
    altitude_std real  NOT NULL,
    timestamp timestamp  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT base_stations_pk PRIMARY KEY (id)
);

-- Table: laser
CREATE TABLE laser (
    id serial  NOT NULL,
    bag_files_id int  NOT NULL,
    sensors_id int  NOT NULL,
    scan_time real  NOT NULL,
    ranges text  NOT NULL,
    intensities text  NOT NULL,
    latitude real  NOT NULL,
    longitude real  NOT NULL,
    altitude real  NOT NULL,
    geogaphy GEOGRAPHY(POINT,4326)  NOT NULL,
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
    sensors_id int  NOT NULL,
    angle_min real  NOT NULL,
    angle_max real  NOT NULL,
    angle_increment real  NOT NULL,
    time_increment real  NOT NULL,
    range_min real  NOT NULL,
    range_max real  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT laser_parameters_pk PRIMARY KEY (id)
);

-- Table: sensors
CREATE TABLE sensors (
    id serial  NOT NULL,
    type int  NOT NULL,
    serial_number varchar(255)  NOT NULL,
    company_name varchar(255)  NOT NULL,
    product_name varchar(255)  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT sensors_pk PRIMARY KEY (id)
);

-- Table: trips
CREATE TABLE trips (
    id serial  NOT NULL,
    name text  NOT NULL,
    date date  NOT NULL,
    base_stations_id int  NOT NULL,
    description text  NOT NULL,
    driver text  NOT NULL,
    passengers text  NOT NULL,
    notes text  NOT NULL,
    date_added timestamp  NOT NULL,
    CONSTRAINT trips_pk PRIMARY KEY (id)
);

-- foreign keys
-- Reference: bag_files_Vehicle (table: bag_files)
ALTER TABLE bag_files ADD CONSTRAINT bag_files_Vehicle
    FOREIGN KEY (Vehicle_id)
    REFERENCES Vehicle (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: bag_files_trips (table: bag_files)
ALTER TABLE bag_files ADD CONSTRAINT bag_files_trips
    FOREIGN KEY (trips_id)
    REFERENCES trips (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: laser_bag_files (table: laser)
ALTER TABLE laser ADD CONSTRAINT laser_bag_files
    FOREIGN KEY (bag_files_id)
    REFERENCES bag_files (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: laser_parameters_sensors (table: laser_parameters)
ALTER TABLE laser_parameters ADD CONSTRAINT laser_parameters_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: laser_sensors (table: laser)
ALTER TABLE laser ADD CONSTRAINT laser_sensors
    FOREIGN KEY (sensors_id)
    REFERENCES sensors (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;

-- Reference: trips_base_stations (table: trips)
ALTER TABLE trips ADD CONSTRAINT trips_base_stations
    FOREIGN KEY (base_stations_id)
    REFERENCES base_stations (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;
-- End of file.
