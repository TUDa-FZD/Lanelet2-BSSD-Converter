# math-837

Entwicklung eines Frameworks zur automatisierten Generierung der BSSD-Erweiterung für lanelet2-Karten

# Requirements

- Python (implemented with 3.8)
- Lanelet2
- BSSD Core -> link
- packages
  - numpy
  - pyosmium
  
# Installation

1. Install Lanelet2 following this [guide](doc/Lanelet2 installation guide.md)

2. Clone the Repository (with HTTPS or SSH)
   1. HTTPS:  
   <code>git clone https://git.rwth-aachen.de/fzd/unicar-agil/studentische-arbeiten/math-837/-/tree/develop </code>
   or 
   2. SSH:  
   <code>git clone git@git.rwth-aachen.de/fzd/unicar-agil/studentische-arbeiten/math-837/-/tree/develop</code>
3. Install package and dependencies with  
   <code> pip install -e .</code>
4. Install bssd core
   1. In folder libraries create a new folder <code>bssd</code>
   2. invoke <code>git clone https://git.rwth-aachen.de/fzd/unicar-agil/sonstiges/bssd/bssdhelper.git@develop </code>
   3. <code> pip install -e path/to/bssd </code>  
      (Create & activate a virtual environment if you want to install the tool inside a virtual environment)

Open a terminal window in the folder math-837 and install
the required packages as well as the tool itself with the command


# Usage

1. Get the path to the Lanelet2 map that you wish to use.
2. To start the tool, open a terminal window in the directory of the tool and type in 
   1. <code>source venv/bin/activate</code> to activate the virtual environment
   2. <code> python .\main.py "path_to_the_file" </code>
3. The tool will automatically execute and show some information about the current status in the terminal
4. After successful execution, the modified Lanelet2 map will be stored in the subfolder /Output, which will be created (if not already existing)  in the same folder as the original osm-map.
5. Furthermore, a derivation-log-file is saved into the Output folder