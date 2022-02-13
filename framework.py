import lanelet2
import os
import time
from lanelet2.core import AttributeMap, TrafficLight, Lanelet, LineString3d, Point2d, Point3d, getId, \
    LaneletMap, BoundingBox2d, BoundingBox3d, BasicPoint2d, BasicPoint3d
import BSSD_elements
from BSSD_elements import create_placeholder
import constants
import io_data
from behavior_derivation import derive_behavior
from geometry_derivation import get_long_bdr
from copy import deepcopy


def main():
    # Process Lanelet2 map and derive behavior spaces

    # ----------- INPUT --------------
    # Load example file from lanelet2
    filename = "res/DA_Nieder-Ramst-Mühlstr-Hochstr.osm"
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    pos_ids_file_path = "/home/jannik/Documents/WS_Lanelet2/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")
    map_ll = io_data.load_map(example_file)

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()
    ll_rel = [item.id for item in map_ll.laneletLayer if ll_relevant(item.attributes)]
    ll_rel, ll_bicycle = get_relevant_bicycle_lls(map_ll.laneletLayer, ll_rel)

    graph = io_data.get_RoutingGraph(map_ll)

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

    io_data.write_bssd_elements(BSSD, file2)
    io_data.merge_files(file1, file2, filename[3:])
    end = time.perf_counter()
    print(f"BSSD file is written. Elapsed time: {round(end - start, 2)}")


def ll_recursion(ll, list_relevant_ll, graph, map_ll, map_bssd, direction=None, ls=None):

    if ll.id in list_relevant_ll:
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        list_relevant_ll.remove(ll.id)

        # Perform derivation of behavior space from current lanelet
        # atm only long. boundary
        # fwd_ls, bwd_ls, map_ll = derive_abs_geom(ll, map_ll, direction, ls)
        map_ll, fwd_ls, long_ref = get_long_bdr(map_ll, ll.leftBound[-1], ll.rightBound[-1], 'fwd', direction, ls)
        map_ll, bwd_ls, long_ref = get_long_bdr(map_ll, ll.leftBound[0], ll.rightBound[0], 'bwd', direction, ls)
        # derive abs behavior
        new_bs = create_placeholder(ll, fwd_ls, bwd_ls)
        new_bs = derive_behavior(new_bs, ll, map_ll)
        map_bssd.add(new_bs)
        #print(new_bs.alongBehavior.leftBound.tags['crossing'])
        # for id_obj, bssd_object in map_bssd.BoundaryLatLayer.items():
        #     print(bssd_object.tags['crossing'])
        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries
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


def ls_of_pbl(att):
    if ('line' in att['type'] and att['subtype'] == 'dashed') or att['type'] == 'virtual':
        return True
    else:
        return False


def get_map_all_vehicle(map_original):

    map_new = deepcopy(map_original)

    for ll in map_new.laneletLayer:
        ll.attributes['participant:vehicle'] = 'yes'

    return map_new


def get_relevant_bicycle_lls(ll_layer, list_relevant):

    list_bicycle = [item for item in ll_layer if item.attributes['subtype'] == 'bicycle_lane']

    for ll in list_bicycle:
        nbrs_left = ll_layer.findUsages(ll.leftBound)
        nbrs_left.remove(ll)
        nbrs_right = ll_layer.findUsages(ll.rightBound)
        nbrs_right.remove(ll)

        if len(nbrs_left) > 1 or len(nbrs_right) > 1:
            print(f"For bicycle_lane with ID {ll.id}: Multiple neighbors have been found")

        if (nbrs_left and ls_of_pbl(ll.leftBound.attributes) and ll_relevant(nbrs_left[0].attributes)) or \
                (nbrs_right and ls_of_pbl(ll.rightBound.attributes) and ll_relevant(nbrs_right[0].attributes)):
            ll.attributes['subtype_alt'] = 'bicycle_lane_protected'
            ll.attributes['participant:vehicle'] = 'yes'
            ll.attributes['participant:bicycle'] = 'yes'
            list_relevant.append(ll.id)
            list_bicycle.remove(ll)

    return list_relevant, list_bicycle


if __name__ == '__main__':
    main()
