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
from preprocessing import ll_relevant, get_relevant_bicycle_lls
import logging
import argparse


logging.basicConfig(filename='derivation.log',
                    level=logging.DEBUG,
                    filemode='w',
                    format='[%(asctime)s] %(levelname)s %(message)s')
logger = logging.getLogger(__name__)
stream = logging.StreamHandler()
stream.setLevel(logging.INFO)
streamformat = logging.Formatter("%(levelname)s:%(message)s")
stream.setFormatter(streamformat)

logger.addHandler(stream)


def main():
    parser = argparse.ArgumentParser(description="Run BSSD-derivation framework")
    parser.add_argument("-in", help="Lanelet2 map file", dest="filename", type=str, required=True)
    parser.set_defaults(func=framework)
    args = parser.parse_args()
    args.func(args)


def framework():
    # Process Lanelet2 map and derive behavior spaces

    # ----------- INPUT --------------
    # Load example file from lanelet2
    # filename = args.filename
    filename = "res/aseag.osm"
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    pos_ids_file_path = "/home/jannik/Documents/WS_Lanelet2/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")
    map_ll = io_data.load_map(example_file)

    logger.info(f'File {filename} loaded successfully')

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()

    logger.info(f'Start preprocessing. Finding relevant lanelets and distinguishing bicycle_lanes')
    ll_rel = [item.id for item in map_ll.laneletLayer if ll_relevant(item.attributes)]
    ll_rel, ll_bicycle = get_relevant_bicycle_lls(map_ll.laneletLayer, ll_rel)

    logger.info(f'Determined relevant lanelets')

    graph = io_data.get_RoutingGraph(map_ll)
    graph_all = io_data.get_RoutingGraph_all(map_ll)

    logger.info(f'RoutingGraph for all lanelets created')

    BSSD = BSSD_elements.bssdClass()
    end = time.perf_counter()
    logger.info(f"Preprocessing done. Elapsed time: {round(end - start, 2)}")

    # ----------- PROCESSING --------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    logger.info(f'Start recursive loop through relevant lanelets')
    while ll_rel:
        # print('start')
        BSSD, ll_rel = ll_recursion(map_ll.laneletLayer[ll_rel[0]], ll_rel, graph_all, map_ll, BSSD)
    end = time.perf_counter()
    logger.info(f"Loop for relevant lanelets completed. Elapsed time: {round(end - start, 2)}")

    # ----------- OUTPUT --------------
    # Save edited .osm-map to desired file
    start = time.perf_counter()
    file1 = "map_ll.osm"
    file2 = "map_bssd.osm"
    io_data.save_map(map_ll, file1)

    io_data.write_bssd_elements(BSSD, file2)
    io_data.merge_files(file1, file2, filename[3:])
    end = time.perf_counter()
    logger.info(f'Saved map {filename[3:]} with BSSD extension in output directory\
                Elapsed time: {round(end - start, 2)}')


def ll_recursion(ll, list_relevant_ll, graph, map_ll, map_bssd, direction=None, ls=None):

    if ll.id in list_relevant_ll:
        # Remove current lanelet from list of relevant lanelets to keep track which lanelets still have to be done
        list_relevant_ll.remove(ll.id)

        logger.debug(f'-----------------------------------------------')
        logger.debug(f'Derivation for Lanelet {ll.id}')

        # Perform derivation of behavior space from current lanelet
        # atm only long. boundary
        map_ll, fwd_ls, long_ref_f = get_long_bdr(map_ll, ll.leftBound[0], ll.rightBound[0], 'fwd' == direction, ls)
        map_ll, bwd_ls, long_ref_b = get_long_bdr(map_ll, ll.leftBound[-1], ll.rightBound[-1], 'bwd' == direction, ls)
        # derive abs behavior
        new_bs = create_placeholder(ll, fwd_ls, bwd_ls)
        logger.debug(f'Created Behavior Space {new_bs.id}')
        new_bs.alongBehavior.assign_long_ref(long_ref_f, 'along')
        new_bs.againstBehavior.assign_long_ref(long_ref_b, 'against')
        new_bs = derive_behavior(new_bs, ll, map_ll, graph)
        #new_bs = derive_speed_limit(new_bs, ll, map_ll, graph)
        map_bssd.add(new_bs)

        # Call function in itself for the succeeding and preceding lanelet(s) and hand over information
        # about already derived boundaries
        for successor in graph.following(ll):
            map_bssd, list_relevant_ll = ll_recursion(successor, list_relevant_ll, graph, map_ll, map_bssd, 'fwd', bwd_ls)
        for predecessor in graph.previous(ll):
            map_bssd, list_relevant_ll = ll_recursion(predecessor,  list_relevant_ll, graph, map_ll, map_bssd, 'bwd', fwd_ls)
            
    return map_bssd, list_relevant_ll


if __name__ == '__main__':
    framework()
