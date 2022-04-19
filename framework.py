import time
import argparse

from BSSD_derivation_for_Lanelet2.io_handler import IoHandler
from BSSD_derivation_for_Lanelet2.data_handler import DataHandler
from BSSD_derivation_for_Lanelet2.preprocessing import Preprocessing
from BSSD_derivation_for_Lanelet2.util import edit_log_file, setup_logger

"""
This framework automatically derives the BSSD extension for Lanelet2 maps. In this module, the submodules are called to
run through the necessary steps of the BSSD derivation.
1. Loading a given Lanelet2 map
2. Preprocessing of the map which includes identification of relevant lanelets and creation of a RoutingGraph
3. Loop through all the detected relevant lanelets, create BSSD elements and derive the behavioral demand.
4. Save the Lanelet2 and BSSD elements to a new output map file.
"""


def main():
    parser = argparse.ArgumentParser(description="Run BSSD-derivation framework")
    parser.add_argument("-m", "--map", help="Lanelet2 map file", dest="filepath", type=str, required=True)
    parser.add_argument("-lat", "--latitude_coordinate", help="latitude origin coordinate for projection",
                        dest="latitude", type=float, required=False)
    parser.add_argument("-lon", "--longitude_coordinate", help="longitude origin coordinate for projection",
                        dest="longitude", type=float, required=False)
    parser.set_defaults(func=framework)
    args = parser.parse_args()
    args.func(args)


def framework(args):
    # Process Lanelet2 map and derive behavior spaces

    # --------------------------------
    # ----------- INPUT --------------
    # --------------------------------
    # Load desired file with lanelet2 map
    file = args.filepath

    # Setup the logging module
    logger, log_file = setup_logger(file)

    # Load the Lanelet2 map using the IO module
    if args.latitude and args.longitude:
        io = IoHandler(file, [args.latitude, args.longitude])
    else:
        io = IoHandler(file)
    map_lanelet = io.load_map()

    # Save the amount of linestrings existing in this map to determine the number of newly created linestrings at
    # the end of the framework for statistical purposes
    orig_nr_ls = len(map_lanelet.lineStringLayer)

    logger.info(f'File {file} loaded successfully')

    # ------------------------------------
    # ---------- PREPROCESSING -----------
    # ------------------------------------
    # Make list with all IDs of lanelets that are relevant
    start_preprocessing = time.perf_counter()
    logger.info(f'Start preprocessing. Finding relevant lanelets and distinguishing bicycle_lanes')

    # Perform preprocessing steps using the Preprocessing module: Create RoutingGraph and find relevant lanelets
    preprocessor = Preprocessing(map_lanelet)
    relevant_lanelets = preprocessor.find_relevant_lanelets()
    routing_graph = preprocessor.get_routing_graph_all()

    # Setup main data handler to perform behavior space derivation for the given Lanelet2 map
    data_handler = DataHandler(preprocessor.map_lanelet, relevant_lanelets, routing_graph)
    end_preprocessing = time.perf_counter()
    logger.info(f"Preprocessing completed, relevant lanelets detected and RoutingGraph created."
                f"\nElapsed time: {round(end_preprocessing - start_preprocessing, 2)}")

    # -------------------------------------
    # ----------- PROCESSING --------------
    # -------------------------------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start_processing = time.perf_counter()
    logger.info(f'Start recursive loop through relevant lanelets')
    while data_handler.relevant_lanelets:
        data_handler.recursive_loop(data_handler.relevant_lanelets[0])
    end_processing = time.perf_counter()
    logger.info(f"Loop for relevant lanelets completed.\nElapsed time: {round(end_processing - start_processing, 2)}")

    # ---------------------------------
    # ----------- OUTPUT --------------
    # ---------------------------------
    # Save edited .osm-map to desired filepath
    start_output = time.perf_counter()

    # Save the Lanelet2 elements to an osm-file
    io.save_map(data_handler.map_lanelet)
    # Save the BSSD elements to an osm-file
    io.write_bssd_elements(data_handler.map_bssd)
    # Merge the above created osm-files to one output file
    io.merge_files(file)
    end_output = time.perf_counter()
    logger.info(f'Saved map {file} with BSSD extension in output directory. '
                f'\nElapsed time: {round(end_output - start_output, 2)}')
    lc = logger.handlers[0].levelcount
    logger.info(f"\n------ Statistics ------"
                f"\nBehavior Spaces: {len(data_handler.map_bssd.BehaviorSpaceLayer)}"
                f"\nBehaviors:       {len(data_handler.map_bssd.BehaviorLayer)}"
                f"\nBoundary Lat:    {len(data_handler.map_bssd.BoundaryLatLayer)}"
                f"\nBoundary Long:   {len(data_handler.map_bssd.BoundaryLongLayer)}"
                f"\nReservations:    {len(data_handler.map_bssd.ReservationLayer)}"
                f"\nNew Linestrings: {len(data_handler.map_lanelet.lineStringLayer) - orig_nr_ls}"
                f"\nWarnings:        {lc['WARNING']}"
                f"\nCritical Logs:   {lc['CRITICAL']}"
                f"\nErrors:          {lc['ERROR']}"
                f"\n------------------------")

    # Edit the log-file so that the statistics (see above) are placed at the beginning of the file
    edit_log_file(log_file)


if __name__ == '__main__':
    main()
