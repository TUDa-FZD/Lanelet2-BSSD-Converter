import lanelet2
import os
import time
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BasicPoint2d
from lanelet2.projection import UtmProjector

import BSSD_elements
from BSSD_elements import create_placeholder
import constants
import io_data


def main():
    # Process Lanelet2 map and derive behavior spaces

    # ----------- INPUT --------------
    # Load example file from lanelet2
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "res/mapping_example.osm")
    map_ll, graph = io_data.load_map(example_file)

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()
    ll_rel = [item.id for item in map_ll.laneletLayer if ll_relevant(item.attributes)]

    BSSD = BSSD_elements.bssdClass()
    end = time.perf_counter()
    print(f"Preprocessing done. Elapsed time: {round(end - start, 2)}")

    # ----------- PROCESSING --------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    while ll_rel:
        # print('start')
        BSSD, ll_rel = ll_recursion(map_ll.laneletLayer[ll_rel[0]], ll_rel, graph, map_ll, BSSD)
    end = time.perf_counter()
    print(f"Processing done. Elapsed time: {round(end - start, 2)}")

    # ----------- OUTPUT --------------
    # Save edited .osm-map to desired file
    start = time.perf_counter()
    file1 = "map_ll.osm"
    file2 = "map_bssd.osm"
    io_data.save_map(map_ll, file1)

    io_data.save_map(BSSD, "bssd_test.osm")

    io_data.write_bssd_elements(BSSD, file2)
    io_data.merge_files(file1, file2)
    end = time.perf_counter()
    print(f"BSSD file is written. Elapsed time: {round(end - start, 2)}")


def ll_recursion(ll, list_relevant_ll, graph, map_ll, map_bssd, direction=None, ls=None):

    if ll.id in list_relevant_ll:
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        list_relevant_ll.remove(ll.id)

        # Perform derivation of behavior space from current lanelet
        # atm only long. boundary
        fwd_ls, bwd_ls, map_ll = derive_abs_geom(ll, map_ll, direction, ls)
        ## derive abs behavior
        new_bs = create_placeholder(ll, fwd_ls, bwd_ls)
        map_bssd.add(new_bs)
        # Call function in itself for the succeeding and preceding lanelet and hand over information
        # about already created boundaries
        for successor in graph.following(ll):
            map_bssd, list_relevant_ll = ll_recursion(successor, list_relevant_ll, graph, map_ll, map_bssd, 'fwd', fwd_ls)
        for predecessor in graph.previous(ll):
            map_bssd, list_relevant_ll = ll_recursion(predecessor,  list_relevant_ll, graph, map_ll, map_bssd, 'bwd', bwd_ls)
            
    return map_bssd, list_relevant_ll


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

        ls = None

        if not direction == side and direction:
            # Use previously created linestring
            ls = d_id
        elif pt_left == pt_right:
            # No longitudinal boundary exists
            # print(ll.id)
            pass
        else:
            # Check for existing lineStrings (e.g. stop_line)
            id_line, length_ok = find_line(pt_left, pt_right, map_ll.lineStringLayer)
            if length_ok:
                ls = map_ll.lineStringLayer[id_line]

            # Create new line, if necessary
            else:
                pt_pairs = [map_ll.pointLayer[pt_left], map_ll.pointLayer[pt_right]]
                ls = LineString3d(getId(), pt_pairs, {'type': 'BSSD', 'subtype': 'Boundary'})
                map_ll.add(ls)

        return map_ll, ls

    # Call function to determine long. boundary for both sides of the lanelet
    map_ll, fid = create_long_ls(map_ll, ll.leftBound[-1].id, ll.rightBound[-1].id, 'fwd')
    map_ll, bid = create_long_ls(map_ll, ll.leftBound[0].id, ll.rightBound[0].id, 'bwd')

    return fid, bid, map_ll


def find_line(pt1, pt2, lsLayer):
    # Find a linestring that contains two given points and return its ID and whether it has a length of 2
    # Todo: Optimize search (use lanelet2 functions) and check for multiple linestrings
    # Todo: Distinguish better between different lines. Also lines longer than two points can be a long Bdr
    line_id = []
    len_ok = False
    for line in lsLayer:
        if any(pt.id == pt1 for pt in iter(line)) and any(pt.id == pt2 for pt in iter(line)):
                # print(line.id, len(line))
                line_id = line.id
                if len(line) == 2:
                    len_ok = True

    return line_id, len_ok


if __name__ == '__main__':
    main()
