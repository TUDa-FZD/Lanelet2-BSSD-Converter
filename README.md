# Development of a Framework for the Automated Generation of the BSSD Extension for Lanelet2 Maps

This framework generates the BSSD extension for Lanelet2 maps. Behavior spaces are therefore mapped
on lanelets. For each lanelet of a map that can be reached by a motorized vehicle a behavior
space is created some properties of its behavior attributes are derived.

## Requirements

- Python (implemented with 3.8)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- [BSSD Core](https://git.rwth-aachen.de/fzd/unicar-agil/sonstiges/bssd/core)
- packages
  - numpy
  - pyosmium
  
## Installation

1. Install Lanelet2 following this [guide](doc/Lanelet2 installation guide.md).

2. Clone the Repository (with HTTPS or SSH)
   1. HTTPS:  
   <code>git clone https://git.rwth-aachen.de/fzd/unicar-agil/studentische-arbeiten/math-837/-/tree/develop </code>
   or 
   2. SSH:  
   <code>git clone git@git.rwth-aachen.de/fzd/unicar-agil/studentische-arbeiten/math-837/-/tree/develop</code>
3. Install package and dependencies by invoking the following in a terminal inside the directory of this framework  
   <code> pip install -e .</code>
4. Install BSSD core
   1. In folder 'libraries' create a new folder <code>bssd</code>
   2. invoke <code>git clone https://git.rwth-aachen.de/fzd/unicar-agil/sonstiges/bssd/core.git@develop </code>
   3. <code> pip install -e path/to/bssd </code>
   
(Create & activate a virtual environment if you want to install the tool inside a virtual environment)

## Usage

1. Get the path to the Lanelet2 map that you wish to derive the BSSD extension for.
2. To start the tool, open a terminal window in the directory of the tool and invoke 
   1. <code>source venv/bin/activate</code> to activate the virtual environment
   2. <code> python3 framework.py -m path_to_Lanelet2_map </code>
3. The tool will automatically execute and show some information about the current status in the terminal.
4. After successful execution, the modified Lanelet2 map will be stored in the same directory as the original map
with "_BSSD" at the end of the filename.
5. Furthermore, a derivation-log-file is saved into the same directory.

## Architecture

To get an overview of how this framework is build, read [this](doc/architecture.md).

## Tests

To run the tests that are included with pytest, open a terminal in the directory in which the repository
is installed and invoke <code>pytest test</code>.