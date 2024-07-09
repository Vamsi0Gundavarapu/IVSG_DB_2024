-- tables
-- Table: Mapping_van_Data
CREATE TABLE Mapping_van_Data (
    id serial  NOT NULL,
    Vehicles_id int  NOT NULL,
    sent_time timestamp  NOT NULL,
    latitude real  NOT NULL CHECK ((latitude >= -90 AND latitude <= 90)),
    longitude real  NOT NULL CHECK ((longitude >= -180 AND longitude <= 180)),
    geography GEOGRAPHY(point,4326)  NOT NULL,
    CONSTRAINT Mapping_van_Data_pk PRIMARY KEY (id)
);

-- Table: Vehicles
CREATE TABLE Vehicles (
    id serial  NOT NULL,
    vehicle_type varchar(50)  NOT NULL,
    CONSTRAINT Vehicles_pk PRIMARY KEY (id)
);

-- foreign keys
-- Reference: Mapping_van_Data_Vehicles (table: Mapping_van_Data)
ALTER TABLE Mapping_van_Data ADD CONSTRAINT Mapping_van_Data_Vehicles
    FOREIGN KEY (Vehicles_id)
    REFERENCES Vehicles (id)  
    NOT DEFERRABLE 
    INITIALLY IMMEDIATE
;
-- End of file.
