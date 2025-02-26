# Architecture

The tool is divided into different modules that are responsible for different tasks.
The main executable script is framework.py which uses the modules to derive the BSSD for a given Lanelet2 map.

## Modules
The following modules are included:
- **io_handler**: Provides functions to load and save maps. Stores information about the file location, origin coordinates
and the projector of Lanelet2. Furthermore, functions for automatic detection of origin coordinates and reversing the
changes made to the Lanelet2 map are included. Loading a Lanelet2 map includes a step to make the IDs of that map
positive.
- **preprocessing**: A class that uses a loaded Lanelet2 map from the io_handler to perform certain preprocessing steps.
Within these steps mainly a RoutingGraph for every lanelet of a map (instead of only one class of traffic participants)
is being created and lanelets are distinguished by their relevance for behavior space derivations.
- **BSSD_elements**: Module that contains classes for each BSSD element as well as a class that serves as a container for
every BSSD element. The latter includes methods to create placeholder objects for behavior spaces.
- **data_handler**: This is the main module for the actual processing and BSSD derivation for a Lanelet2 map. Using the
list of relevant lanelets and the RoutingGraph for all lanelets, an algorithm loops through every relevant lanelet of
the map and creates new behavior space objects, determines longitudinal boundaries and derives behavioral demands.
Static methods for geometry derivation and behavior derivation are partially moved to the modules geometry_derivation
and behavior_derivation to improve the overview in the data_handler class. Most of the methods are included in the
DataHandler class, because they need access to attributes like the Lanelet2 map or the RoutingGraph. 
- **geometry_derivation**: Additional functions for derivation of the geometry (currently the longitudinal boundary) of
behavior spaces.
- **behavior_derivation**: Additional functions for derivation of the behavioral demands of
behavior spaces.
- **util**: Additional functions that are used all across the framework.
- **constants**: Constant lists and dictionaries that are used within the framework, e.g. to initialize certain objects.

## Processing of Lanelet2 Maps

1. Using the io_handler module, a Lanelet2 map is loaded.
2. Using the preprocessing module, every relevant lanelet is being identified based on the currently used conditions.
3. An instance of the DataHandler is created which stores the list of relevant lanelets, 
4. A while loop starts and runs while there are lanelets left in the list of relevant lanelets
   1. The while loop calls the function 'recursive_loop' in the DataHandler class which processes lanelets one by one
   and recursively moving on to every predecessor and successor. The function starts at a random lanelet in the list.
   Each processed lanelet will be removed from the list of relevant lanelets. As soon as no lanelet in the current paths
   can be reached anymore, the while loop calls the function again for one of the remaining relevant lanelets.
   2. During the processing of a lanelet the function 'recursive_loop' calls functions that
      1. identifies the longitudinal boundaries of both sides of the lanelet (which is currently covering the same space
      as a behavior space),
      2. creates a behavior space object including all the elements that are necessary for that,
      3. assigns the longitudinal boundaries to the respective behavior objects,
      4. calls the function 'derive_behavior' in the DataHandler class which itself calls multiple functions
      that derive behavioral demands for the newly created behavior space object.
5. The io_handler module saves Lanelet2 and BSSD elements to separate files and merges those files to
eventually achieve a united map-file of a Lanelet2 map with the generated BSSD extension.

## Behavior Derivation and Extendability
As mentioned in the previous section, the function 'derive_behavior' calls multiple sub functions that derive
behavioral properties of the behavioral attributes of BSSD. Since this framework is not deriving the entire behavioral
demand of a scenery in Lanelet2 yet, further integrations will be necessary to derive more behavioral
attributes and their properties. To achieve this extension, more functions can be integrated in the method
'derive_behavior' in the DataHandler class. The current derivations are spread over different behavioral
attributes and can be seen as a demonstration how a derivation may be possible. At the moment the following derivations
are included:
- Derivation of the **CrossingType of lateral boundaries** based on their type and subtype. Furthermore, for parking
areas next to a lateral boundary of a lanelet, the property 'parking_only' is derived. The following functions are used
for this derivation:
  - DataHandler.derive_behavior_boundary_lateral
  - behavior_derivation.derive_crossing_type_for_lat_boundary
- Derivation of the property **'no_stagnant_traffic' at longitudinal boundaries** that are lying at a zebra crossing. The 
following functions are used for this derivation:
  - DataHandler.derive_boundary_long_behavior
- Derivation of the **speed limit along and against reference direction** of a behavior space. The 
following functions are used for this derivation:
  - DataHandler.derive_segment_speed_limit
  - DataHandler.find_adjacent
  - DataHandler.assign_speed_limit_along
  - DataHandler.find_one_sided_neighbors
  - DataHandler.neighbor_next_to_area
  - DataHandler.filter_for_segment_membership
  - DataHandler.are_linestrings_orthogonal
  - DataHandler.find_neighbor_areas
- Derivation of the property **ReservationType at zebra crossings**. The ReservationType at affected behavior spaces
is set to 'externally' and furthermore, reservation links are derived. The following functions are used
for this derivation:
  - DataHandler.derive_conflicts
  - DataHandler.find_neighbor_areas
  - behavior_derivation.is_zebra_and_intersecting