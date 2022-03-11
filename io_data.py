import os
import osmium
import lanelet2
from lanelet2.projection import UtmProjector
import tempfile as tf


class io_handler():

    def __init__(self, path, origin_coordinates=None):
        self.input_path = path
        if origin_coordinates:
            self.origin_coordinates = origin_coordinates
        else:
            self.autodetect_coordinates()
        self.projector = UtmProjector(lanelet2.io.Origin(self.origin_coordinates[0], self.origin_coordinates[1]))

        self._tmp_dir = tf.TemporaryDirectory()
        self._tmp_ll_file = os.path.join(self._tmp_dir.name, "ll2.osm")
        self._tmp_bssd_file = os.path.join(self._tmp_dir.name, "bssd.osm")

    def load_map(self):
        # Load a Lanelet2-map from a given file and creating a map and graph for storing its data in variables.
        return lanelet2.io.load(self.input_path, self.projector)

    def autodetect_coordinates(self):

        coordinates = []
        with open(self.input_path) as fp:
            for line in fp.readlines():
                if 'lat=\'' in line and 'lon=\'' in line:
                    coordinates.append(float(line.split("lat=\'", 1)[1].split('\'', 1)[0]))
                    coordinates.append(float(line.split("lon=\'", 1)[1].split('\'', 1)[0]))
                    break
                elif 'lat=\"' in line and 'lon=\"' in line:
                    coordinates.append(float(line.split("lat=\"", 1)[1].split('\"', 1)[0]))
                    coordinates.append(float(line.split("lon=\"", 1)[1].split('\"', 1)[0]))
                    break
        # self.origin_coordinates = [round(num, 2) for num in coordinates]
        self.origin_coordinates = coordinates

    def get_RoutingGraph(self, map_lanelet):

        traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        graph = lanelet2.routing.RoutingGraph(map_lanelet, traffic_rules)

        return graph

    def save_map(self, map_ll, file_path=None):
        if not file_path:
            file_path = self._tmp_ll_file
        lanelet2.io.write(file_path, map_ll, self.projector)

    def write_bssd_elements(self, bssd, file=None):

        if not file:
            file = self._tmp_bssd_file

        writer_bssd = osmium.SimpleWriter(file)

        for layer, layerdict in iter(bssd):
            for id_obj, bssd_object in layerdict.items():
                writer_bssd.add_relation(bssd_object.get_osmium())

        writer_bssd.close()

    def merge_files(self, file_final='output.osm'):
        data = data2 = ""
        path_output = 'Output/' + file_final[:-4] + '_BSSD.osm'

        # Reading data from Lanelet2 file and deleting the file afterwards
        with open(self._tmp_ll_file) as fp:
            data = fp.readlines()[:-1]

        # Reading data from BSSD file and deleting the file afterwards
        with open(self._tmp_bssd_file) as fp:
            data2 = fp.readlines()[2:]

        # Merging both files
        data += data2

        if os.path.isfile(path_output):
            os.remove(path_output)
        with open(path_output, 'w') as fp:
            fp.writelines(data)
