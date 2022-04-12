import os
import time
import argparse

from BSSD_derivation_for_Lanelet2.io_handler import IoHandler
from BSSD_derivation_for_Lanelet2.data_handler import DataHandler
from BSSD_derivation_for_Lanelet2.preprocessing import Preprocessing
from BSSD_derivation_for_Lanelet2.util import edit_log_file, setup_logger

'''
This framework automatically derives the BSSD extension for Lanelet2 maps. In this module, the submodules are called to
run through the necessary steps of the BSSD derivation.
1. Loading a given Lanelet2 map
2. Preprocessing of the map which includes identification of relevant lanelets and creation of a RoutingGraph
3. Loop through all the detected relevant lanelets, create BSSD elements and derive the behavioral demand.
4. Save the Lanelet2 and BSSD elements to a new output map file.
'''


def main():
    parser = argparse.ArgumentParser(description="Run BSSD-derivation framework")
    parser.add_argument("-m", "--map", help="Lanelet2 map file", dest="filename", type=str, required=True)
    parser.add_argument("-c", "--coordinates", help="origin coordinates for projection", dest="origin_coordinates", type=str, required=False)
    parser.set_defaults(func=framework)
    args = parser.parse_args()
    args.func(args)


def framework(args):
    # Process Lanelet2 map and derive behavior spaces

    # --------------------------------
    # ----------- INPUT --------------
    # --------------------------------
    # Load example file from lanelet2
    filename = args.filename

    # Setup the logging module
    logger, log_file = setup_logger(filename)

    #example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    example_file = filename

    # Run a script that makes sure every ID in a map is positive
    pos_ids_file_path = "/home/jannik/Documents/WS_Lanelet2/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")

    # Load the Lanelet2 map using the IO module
    io = IoHandler(example_file)
    map_ll = io.load_map()

    # Save the amount of linestrings existing in this map to determine the number of newly created linestrings at
    # the end of the framework for statistical purposes
    orig_nr_ls = len(map_ll.lineStringLayer)

    logger.info(f'File {filename} loaded successfully')

    # ------------------------------------
    # ---------- PREPROCESSING -----------
    # ------------------------------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()
    logger.info(f'Start preprocessing. Finding relevant lanelets and distinguishing bicycle_lanes')

    # Perform preprocessing steps using the Preprocessing module: Create RoutingGraph and find relevant lanelets
    preprocessor = Preprocessing(map_ll)
    relevant_lanelets = preprocessor.find_relevant_lanelets()
    routing_graph = preprocessor.get_routing_graph_all()

    # Setup main data handler to perform behavior space derivation for the given Lanelet2 map
    handler = DataHandler(preprocessor.map_lanelet, relevant_lanelets, routing_graph)
    end = time.perf_counter()
    logger.info(f"Preprocessing completed, relevant lanelets detected and RoutingGraph created."
                f"\nElapsed time: {round(end - start, 2)}")

    # -------------------------------------
    # ----------- PROCESSING --------------
    # -------------------------------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    logger.info(f'Start recursive loop through relevant lanelets')
    while handler.relevant_lanelets:
        handler.recursive_loop(handler.relevant_lanelets[0])
    end = time.perf_counter()
    logger.info(f"Loop for relevant lanelets completed.\nElapsed time: {round(end - start, 2)}")

    # ---------------------------------
    # ----------- OUTPUT --------------
    # ---------------------------------
    # Save edited .osm-map to desired filepath
    start = time.perf_counter()

    # Save the Lanelet2 elements to an osm-file
    io.save_map(handler.map_lanelet)
    # Save the BSSD elements to an osm-file
    io.write_bssd_elements(handler.map_bssd)
    # Merge the above created osm-files to one output file
    io.merge_files(filename)
    end = time.perf_counter()
    logger.info(f'Saved map {filename} with BSSD extension in output directory. '
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

    # Edit the log-file so that the statistics (see above) are placed at the beginning of the file
    edit_log_file(log_file)


if __name__ == '__main__':
    main()
