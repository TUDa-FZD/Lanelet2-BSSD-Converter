import os
import logging
import tempfile as tf

import osmium
import lanelet2
from lanelet2.projection import UtmProjector

from .util import make_positive

logger = logging.getLogger('framework.io_handler')


class IoHandler:
    """
    This class is combining all methods required for input and output operations in a class and stores necessary
    attributes, such as the projector including its origin coordinates.

    Attributes
    ----------
        input_path : path
            Path of the input file
        origin_coordinates : list
            origin coordinates that are used by the Lanelet2 projector for lat/long - metric conversions as origin
        projector : UtmProjector
            Storing Reservation elements as values using their ID as the key.
        _tmp_directory : path
            Temporarily created directory to store separate files for Lanelet2 and BSSD objects to eventually merge them.
        _tmp_lanelet_file : path
            Temporary file path for a file that stores all the Lanelet2 objects after the framework was running.
        _tmp_bssd_file : path
            Temporary file path for a file that stores all the BSSD objects after the framework was running.

    Methods
    -------
        __init__():
            Initiates the dictionaries for each layer.
        add(BssdElement):
            adds an element to its respective layer and returning the element for further usage
        create_placeholder(lanelet=None, bdr_alg=None, bdr_agst=None):
            Creates a placeholder BehaviorSpace and gives the opportunity to add longitudinal boundaries as well as lanelets.
        create_behavior(leftBdr, rightBdr, longBdr):
            Creates a placeholder Behavior and aggregates the lateral boundaries and the longitudinal boundary.
    """
    def __init__(self, path, origin_coordinates=None):
        self.input_path = path
        if origin_coordinates:
            self.origin_coordinates = origin_coordinates
            logger.debug(f'Using given coordinates {self.origin_coordinates} for origin of the projection.')
        else:
            self.autodetect_coordinates()
            logger.debug(f'Automatically detected coordinates {self.origin_coordinates} for origin of the projection.')
        self.projector = UtmProjector(lanelet2.io.Origin(self.origin_coordinates[0], self.origin_coordinates[1]))

        self._tmp_directory = tf.TemporaryDirectory()
        self._tmp_lanelet_file = os.path.join(self._tmp_directory.name, "ll2.osm")
        self._tmp_bssd_file = os.path.join(self._tmp_directory.name, "bssd.osm")

    def load_map(self):
        """Load a Lanelet2-map from a given file and create a map for storing its data in a map class.
        First, check every item of each layer for being negative and assign a positive ID if necessary."""
        map_lanelet = lanelet2.io.load(self.input_path, self.projector)

        make_positive(map_lanelet.pointLayer)
        make_positive(map_lanelet.lineStringLayer)
        make_positive(map_lanelet.polygonLayer)
        make_positive(map_lanelet.laneletLayer)
        make_positive(map_lanelet.areaLayer)
        make_positive(map_lanelet.regulatoryElementLayer)

        return map_lanelet

    def autodetect_coordinates(self):
        """Automatically detect coordinates of a given lanelet2 map that can be used for coordinate projection."""
        coordinates = []
        # open input file
        with open(self.input_path) as map_file:

            # run for-loop for every line in the OSM-file
            for line in map_file.readlines():

                # search for both lat and lon in one line. Because both ' and " can be used in OSM take account for that
                # If found extract the respective lat and lon coordinates and store them in a list
                if 'lat=\'' in line and 'lon=\'' in line:
                    coordinates.append(float(line.split("lat=\'", 1)[1].split('\'', 1)[0]))
                    coordinates.append(float(line.split("lon=\'", 1)[1].split('\'', 1)[0]))
                    break
                elif 'lat=\"' in line and 'lon=\"' in line:
                    coordinates.append(float(line.split("lat=\"", 1)[1].split('\"', 1)[0]))
                    coordinates.append(float(line.split("lon=\"", 1)[1].split('\"', 1)[0]))
                    break
        # store the coordinates in the IoHandler class attributes
        self.origin_coordinates = coordinates

    def save_map(self, map_ll, file_path=None):
        """
        Save the Lanelet2 objects of a map using the Lanelet2 writing function. Map is saved in a temporary file.
        Optionally, it is possible to give a file path to not store the map temporarly but make it permanently available.
        Furthermore, this function includes a call to the reverse changes function to remove non-original attributes
        from lanelet elements.

        Parameters:
            map_ll (laneletMap):Lanelet map object that contains all the Lanelet2 objects of the map.
            file_path (path):Optional file path to save the map to.

        Returns:
            logger (logging):logger object that contains the different handlers required in the framework.
            log_file (path):Automatically created path the log file ('{map_name}+_BSSD.log').
        """
        if not file_path:
            file_path = self._tmp_lanelet_file

        # ---- Reverse Changes ----
        # Comment this block to keep the lanelet attributes used in this framework in the map
        map_ll = self.reverse_changes(map_ll)
        # -------------------------

        lanelet2.io.write(file_path, map_ll, self.projector)

    def write_bssd_elements(self, bssd_map, file_path=None):
        """
        Save the BSSD objects of a map using a SimpleWrite of PyOsmium. Map is saved in a temporary file.
        Optionally, it is possible to give a file path to not store the map temporarly but make it permanently available.

        Parameters:
            bssd_map (laneletMap):Lanelet map object that contains all the Lanelet2 objects of the map.
            file_path (path):Optional file path to save the map to.

        Returns:
            logger (logging):logger object that contains the different handlers required in the framework.
            log_file (path):Automatically created path the log file ('{map_name}+_BSSD.log').
        """
        if not file_path:
            file_path = self._tmp_bssd_file

        writer_bssd = osmium.SimpleWriter(file_path)

        for layer, layerdict in iter(bssd_map):
            for id_obj, bssd_object in layerdict.items():
                writer_bssd.add_relation(bssd_object.attributes.get_osmium())

        writer_bssd.close()

    def merge_files(self, file='map.osm'):
        """
        Uses the temporary existing Lanelet2 and BSSD map files to read their contents and merge them in an OSM conform
        way. The output file is store in the Output folder.

        Parameters:
            file (path):Filename of the original map file. Output filename is based on it and extended by _BSSD.
        """

        # path_output = 'Output/' + file[:-4] + '_BSSD.osm'
        path_output = file[:-4] + '_BSSD.osm'

        # Reading data from Lanelet2 file except the last line
        with open(self._tmp_lanelet_file) as fp:
            map_data_lanelet2 = fp.readlines()[:-1]

        # Reading data from BSSD file expect the first two lines
        with open(self._tmp_bssd_file) as fp:
            map_data_bssd = fp.readlines()[2:]

        # Merging both files
        map_combined = map_data_lanelet2 + map_data_bssd

        # Overwrite an existing file with the same name
        if os.path.isfile(path_output):
            os.remove(path_output)
        with open(path_output, 'w') as fp:
            fp.writelines(map_combined)

        logger.info(f'Saved file as {path_output}')

    @staticmethod
    def reverse_changes(map_lanelet):
        """
        Removes lanelet tags that have been set during the framework for storing data. To keep the original Lanelet2
        elements as they were, this function deletes the attributes from the lanelets. At the moment data for bicycle
        lanes and for speed limit are stored in the lanelet attributes.

        Parameters:
            map_lanelet (laneletMap):Lanelet map object that contains all the Lanelet2 objects of the map.
        """
        # Run a for-loop for every lanelet in the map
        for lanelet in map_lanelet.laneletLayer:

            # Remove the affected attributes
            del lanelet.attributes['relevant_bicycle_lane']
            del lanelet.attributes['own_speed_limit']
            del lanelet.attributes['other_speed_limit']
            del lanelet.attributes['own_speed_limit_link']
            del lanelet.attributes['other_speed_limit_link']
            del lanelet.attributes['along_speed_limit']
            del lanelet.attributes['against_speed_limit']

        logger.debug(f'All lanelet tags that were added within this framework succesfully removed.')
        return map_lanelet

