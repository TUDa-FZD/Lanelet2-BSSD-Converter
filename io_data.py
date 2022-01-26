import os
import osmium
import lanelet2
from lanelet2.projection import UtmProjector


def load_map(path, origin_coordinates=None):
    # Load a Lanelet2-map from a given file and creating a map and graph for storing its data in variables.

    if origin_coordinates is None:
        origin_coordinates = [49, 8.4]
    projector = UtmProjector(lanelet2.io.Origin(origin_coordinates[0], origin_coordinates[1]))
    map_ll = lanelet2.io.load(path, projector)

    return map_ll


def get_RoutingGraph(map_lanelet):

    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(map_lanelet, traffic_rules)

    return graph


def save_map(map_ll, file_path, origin_coordinates=None):
    if origin_coordinates is None:
        origin_coordinates = [49, 8.4]

    projector = UtmProjector(lanelet2.io.Origin(origin_coordinates[0], origin_coordinates[1]))
    lanelet2.io.write(file_path, map_ll, projector)


def write_bssd_elements(bssd, file):
    if os.path.isfile(file):
        os.remove(file)

    writer_bssd = osmium.SimpleWriter(file)

    for layer, layerdict in iter(bssd):
        for id_obj, bssd_object in layerdict.items():
            writer_bssd.add_relation(bssd_object)

    writer_bssd.close()


def merge_files(file1, file2, file3='output.osm'):
    data = data2 = ""
    path_output = 'Output/' + file3[:-4] + '_BSSD.osm'

    # Reading data from Lanelet2 file and deleting the file afterwards
    with open(file1) as fp:
        data = fp.readlines()[:-1]
    os.remove(file1)

    # Reading data from BSSD file and deleting the file afterwards
    with open(file2) as fp:
        data2 = fp.readlines()[2:]
    os.remove(file2)

    # Merging both files
    data += data2

    if os.path.isfile(path_output):
        os.remove(path_output)
    with open(path_output, 'w') as fp:
        fp.writelines(data)
