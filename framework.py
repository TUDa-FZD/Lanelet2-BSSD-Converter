import os
import time
import argparse

from io_handler import IoHandler
from data_handler import DataHandler
from preprocessing import Preprocessing
from util import edit_log_file, setup_logger


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
    filename = "res/lanelet2_DA_Alexanderstr_Hügelstr.osm"

    logger, log_file = setup_logger(filename)

    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    pos_ids_file_path = "/home/jannik/Documents/WS_Lanelet2/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")

    io = IoHandler(example_file)
    map_ll = io.load_map()
    orig_nr_ls = len(map_ll.lineStringLayer)

    logger.info(f'File {filename} loaded successfully')

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()
    logger.info(f'Start preprocessing. Finding relevant lanelets and distinguishing bicycle_lanes')

    # Perform preprocessing steps: Create RoutingGraph and find relevant lanelets
    preprocessor = Preprocessing(map_ll)
    relevant_lanelets = preprocessor.find_relevant_lanelets()
    routing_graph = preprocessor.get_routing_graph_all()

    # Setup main data handler to perform behavior space derivation for the given Lanelet2 map
    handler = DataHandler(preprocessor.map_lanelet, relevant_lanelets, routing_graph)
    end = time.perf_counter()
    logger.info(f"Preprocessing completed, relevant lanelets detected and RoutingGraph created."
                f"\nElapsed time: {round(end - start, 2)}")

    # ----------- PROCESSING --------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    logger.info(f'Start recursive loop through relevant lanelets')
    while handler.relevant_lanelets:
        handler.recursive_loop(handler.relevant_lanelets[0])
    end = time.perf_counter()
    logger.info(f"Loop for relevant lanelets completed.\nElapsed time: {round(end - start, 2)}")

    # ----------- OUTPUT --------------
    # Save edited .osm-map to desired file
    start = time.perf_counter()

    io.save_map(handler.map_lanelet)
    io.write_bssd_elements(handler.map_bssd)
    io.merge_files(filename[3:])
    end = time.perf_counter()
    logger.info(f'Saved map {filename[3:]} with BSSD extension in output directory. '
                f'\nElapsed time: {round(end - start, 2)}')
    lc = logger.handlers[0].levelcount
    logger.info(f"\n------ Statistics ------"
                f"\nBehavior Spaces: {len(handler.map_bssd.BehaviorSpaceLayer)}"
                f"\nBehaviors:       {len(handler.map_bssd.BehaviorLayer)}"
                f"\nBoundary Lat:    {len(handler.map_bssd.BoundaryLatLayer)}"
                f"\nBoundary Long:   {len(handler.map_bssd.BoundaryLongLayer)}"
                f"\nReservations:    {len(handler.map_bssd.ReservationLayer)}"
                f"\nNew Linestrings: {len(handler.map_lanelet.lineStringLayer) - orig_nr_ls}"
                f"\nWarnings:        {lc['WARNING']}"
                f"\nCritical Logs:   {lc['CRITICAL']}"
                f"\nErrors:          {lc['ERROR']}"
                f"\n------------------------")

    edit_log_file(log_file)


if __name__ == '__main__':
    framework()
