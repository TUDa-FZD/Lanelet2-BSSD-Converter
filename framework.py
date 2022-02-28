import os
import time
import io_data
from data_handler import data_handler
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
    filename = "res/DA_Nieder-Ramst-Mühlstr-Hochstr.osm"
    example_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), filename)
    pos_ids_file_path = "/home/jannik/Documents/WS_Lanelet2/src/lanelet2/lanelet2_python/scripts/make_ids_positive.py"
    os.system(f"python3 {pos_ids_file_path} -i {example_file}")
    map_ll = io_data.load_map(example_file)

    logger.info(f'File {filename} loaded successfully')

    # ------- PREPROCESSING --------------
    # Make list with all IDs of lanelets that are relevant
    start = time.perf_counter()

    logger.info(f'Start preprocessing. Finding relevant lanelets and distinguishing bicycle_lanes')
    handler = data_handler(map_ll)
    handler.find_relevant_lanelets()

    logger.info(f'Determined relevant lanelets')

    handler.get_RoutingGraph_all()

    logger.info(f'RoutingGraph for all lanelets created')

    end = time.perf_counter()
    logger.info(f"Preprocessing done. Elapsed time: {round(end - start, 2)}")

    # ----------- PROCESSING --------------
    # Recursively loop through all lanelets to perform desired actions for each (e.g. derive long. boundary)
    start = time.perf_counter()
    logger.info(f'Start recursive loop through relevant lanelets')
    while handler.relevant_lanelets:
        handler.ll_recursion(handler.relevant_lanelets[0])
    end = time.perf_counter()
    logger.info(f"Loop for relevant lanelets completed. Elapsed time: {round(end - start, 2)}")

    # ----------- OUTPUT --------------
    # Save edited .osm-map to desired file
    start = time.perf_counter()
    file1 = "map_ll.osm"
    file2 = "map_bssd.osm"
    io_data.save_map(handler.map_lanelet, file1)

    io_data.write_bssd_elements(handler.map_bssd, file2)
    io_data.merge_files(file1, file2, filename[3:])
    end = time.perf_counter()
    logger.info(f'Saved map {filename[3:]} with BSSD extension in output directory\
                Elapsed time: {round(end - start, 2)}')


if __name__ == '__main__':
    framework()
