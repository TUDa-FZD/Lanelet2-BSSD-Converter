# Framework for the Automated Generation of the BSSD Extension for Lanelet2 Maps

This framework generates the BSSD extension for Lanelet2 maps. Behavior spaces are therefore mapped
on lanelets. For each lanelet of a map that can be reached by a motorized vehicle a behavior
space is created and some properties of its behavior attributes are already derived.

## Requirements

- Python (implemented with 3.8)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [BSSD Core](https://git.rwth-aachen.de/fzd/unicar-agil/sonstiges/bssd/core) (included as submodule)
- packages
  - numpy
  - pyosmium
  
## Installation

1. Install Lanelet2 following this [guide](doc/Lanelet2 installation guide.md).

2. Go to the directory of your choice and clone the repository (with HTTPS or SSH) including all submodules
   1. HTTPS:  
   <code>git clone --recurse-submodules https://git.rwth-aachen.de/fzd/unicar-agil/sonstiges/bssd/lanelet2_bssd_converter.git </code>
   or 
   2. SSH:  
   <code>git clone --recurse-submodules git@git.rwth-aachen.de:fzd/unicar-agil/sonstiges/bssd/lanelet2_bssd_converter.git </code>
3. In the same terminal, go into the directory lanelet2_bssd_converter with <code> cd lanelet2_bssd_converter </code>
4. Install package and dependencies by invoking the following in the same terminal    
   <code> pip install -e . </code>
5. Install BSSD Core
   1. Still in the same terminal, go to directory libraries/core with <code> cd libraries/core </code>
   4. Install the BSSD Core with <code> pip install -e . </code>
   
(Create & activate a virtual environment if you want to install the tool inside a virtual environment)

## Usage

1. Get the path to the Lanelet2 map that you wish to derive the BSSD extension for.
2. To start the tool, open a terminal window in the directory of the tool and invoke 
<code> python3 framework.py -m path_to_Lanelet2_map </code>
3. The tool will automatically execute and show some information about the current status in the terminal.
4. After successful execution, the modified Lanelet2 map will be stored in the same directory as the original map
with "_BSSD" at the end of the filename.
5. Furthermore, a derivation-log-file is saved into the same directory.

## Architecture

To get an overview of how this framework is build, read [this](doc/architecture.md).

## Tests

To run the tests that are included with pytest, open a terminal in the directory in which the repository
is installed and invoke <code>pytest test</code>.
