import os
import osmium
import lanelet2
from lanelet2.projection import UtmProjector


def load_map(path):
    # Load a Lanelet2-map from a given file and creating a map and graph for storing its data in variables.

    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
    map_ll = lanelet2.io.load(path, projector)
    traffic_rules = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                  lanelet2.traffic_rules.Participants.Vehicle)
    graph = lanelet2.routing.RoutingGraph(map_ll, traffic_rules)

    return map_ll, graph


def save_map(map_ll, file_path):
    projector = UtmProjector(lanelet2.io.Origin(49, 8.4))
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

    # Reading data from file1
    with open(file1) as fp:
        data = fp.readlines()[:-1]
    os.remove(file1)

    # Reading data from file2
    with open(file2) as fp:
        data2 = fp.readlines()[2:]
    os.remove(file2)

    # Merging 2 files
    data += data2

    if os.path.isfile(file3):
        os.remove(file3)
    with open(file3, 'w') as fp:
        fp.writelines(data)
