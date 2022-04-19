from BSSD_derivation_for_Lanelet2 import io_handler


def test_autodetect_coordinates():
    """
    Check, if the autodetect function correctly detects the first coordinates in a map.
    """
    io = io_handler.IoHandler('test/DA_Nieder-Ramst-Mühlstr-Hochstr.osm')
    map_lanelet = io.load_map()

    assert io.origin_coordinates[0] == 49.86963758435
    assert io.origin_coordinates[1] == 8.65871449566