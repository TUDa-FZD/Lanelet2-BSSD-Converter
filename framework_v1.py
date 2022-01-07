import lanelet2
import os
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
from lanelet2.projection import UtmProjector
import constants


def framework():
    # Process Lanelet2 map and derive behavior spaces

    # Load example file from lanelet2
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "res/mapping_example.osm")
    map_ll, graph = load_map(example_file)

    # Make list with all IDs of lanelets that are relevant
    ll_rel = [item.id for item in map_ll.laneletLayer if ll_relevant(item.attributes)]

    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    while ll_rel:
        # print('start')
        ll_rel = ll_recursion(map_ll.laneletLayer[ll_rel[0]], ll_rel, graph, map_ll)

    # Save edited .osm-map to desired file
    file = "lanelet2_map_edit.osm"
    save_map(map_ll, file)


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


def ll_recursion(ll, ll_rel, graph, map_ll, direction=None, d_id=None):

    if ll.id in ll_rel:
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        ll_rel.remove(ll.id)

        # Perform derivation of behavior space from current lanelet
        # atm only long. boundary
        fid, bid, map_ll = derive_abs_geom(ll, map_ll, direction, d_id)

        # Call function in itself for the succeeding and preceding lanelet and hand over information
        # about already created boundaries
        for successor in graph.following(ll):
            ll_recursion(successor, ll_rel, graph, map_ll, 'fwd', fid)
        for predecessor in graph.previous(ll):
            ll_recursion(predecessor,  ll_rel, graph, map_ll, 'bwd', bid)
            
    return ll_rel


def ll_relevant(att):
    # Determine the relevance of a lanelet by first checking its subtype (for instance: shouldn't be "stairs")
    # and second if any overriding participant-tags are being used

    if att['subtype'] in constants.SUBTYPE_TAGS:

        if any('participant' in key.lower() for key, value in att.items()):

            if any(value == 'yes' for key, value in att.items() if 'participant:vehicle' in key.lower()):
                relevant = True
            else:
                relevant = False

        else:
            relevant = True

    else:
        relevant = False

    return relevant


def derive_abs_geom(ll, map_ll, direction, d_id):
    # Function to derive the geometry of an atomic behavior space from a lanelet
    # atm only long. boundaries

    fid, bid, map_ll = create_long_bdrs(ll, map_ll, direction, d_id)

    return fid, bid, map_ll


def create_long_bdrs(ll, map_ll, direction, d_id):
    # Determining longitudinal boundaries of a lanelet by first checking conditions for which a creation
    # of a new linestring is not necessary. The IDs of both linestrings are returned by the function for
    # saving them in the behavior space relation

    def create_long_ls(map_ll, pt_left, pt_right, side):

        id_linestring = []

        if not direction == side and direction:
            # Use previously created linestring
            id_linestring = d_id
        elif pt_left == pt_right:
            # No longitudinal boundary exists
            print(ll.id)        
        else:
            # Check for existing lineStrings (e.g. stop_line)
            id_line, length_ok = find_line(pt_left, pt_right, map_ll.lineStringLayer)
            if length_ok:
                id_linestring = id_line

            # Create new line, if necessary
            else:
                    pt_pairs = [map_ll.pointLayer[pt_left], map_ll.pointLayer[pt_right]]
                    ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'Boundary'})
                    map_ll.add(ls)
                    id_linestring = ls.id

        return map_ll, id_linestring

    # Create vectors with point-IDs for lateral boundaries
    lB = [pt.id for pt in iter(ll.leftBound)]
    rB = [pt.id for pt in iter(ll.rightBound)]

    # Call function to determine long. boundary for both sides of the lanelet
    map_ll, fid = create_long_ls(map_ll, lB[-1], rB[-1], 'fwd')
    map_ll, bid = create_long_ls(map_ll, lB[0], rB[0], 'bwd')

    return fid, bid, map_ll


def find_line(pt1, pt2, lsLayer):
    # Find a linestring that contains two given points and return its ID and whether it has a length of 2
    # Todo: Optimize search (use lanelet2 functions) and check for multiple linestrings

    line_id = []
    len_ok = False
    for line in lsLayer:
        if any(pt.id == pt1 for pt in iter(line)) and any(pt.id == pt2 for pt in iter(line)):
                # print(line.id, len(line))
                line_id = line.id
                if len(line) == 2:
                    len_ok = True

    return line_id, len_ok


def create_placeholder(ll):
    # Function that calls code to create empty placeholders for all objects that are necessary for
    # a behavior space in the BSSD.
    assert ll


if __name__ == '__main__':
    framework()
