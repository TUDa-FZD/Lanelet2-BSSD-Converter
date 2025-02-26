# Lanelet2 BSSD Converter

> **IMPORTANT NOTE** This repository is part of the _Behavior-Semantic Scenery Description (BSSD)_ framework. Check out our BSSD documentation and overview repository in your git of choice:
[![GitLab](https://img.shields.io/badge/GitLab-330F63?style=flat&logo=gitlab&logoColor=white)](https://gitlab.com/tuda-fzd/scenery-representations-and-maps/behavior-semantic-scenery-description)
[![GitHub](https://img.shields.io/badge/GitHub-181717?style=flat&logo=github&logoColor=white)](https://github.com/TUDa-FZD/Behavior-Semantic-Scenery-Description)

This tool generates the BSSD extension for Lanelet2 maps. Behavior spaces are therefore mapped
on lanelets. For each lanelet of a map that can be reached by a motorized vehicle a behavior
space is created and some properties of its behavior attributes are already derived.

## Requirements

- Python (implemented with 3.8)
- [Lanelet2](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)
- packages
  - numpy >= 1.22.3,
  - osmium >= 3.2.0,
  - [bssd-core](https://pypi.org/project/bssd-core/) >= 0.1.0,

## Installation

After installing Lanelet2 (see the provided [guide](/doc/Lanelet2%20installation%20guide.md) if you need help) you can install the _Lanelet2 BSSD Converter_ either using pip or manually from the source code.

### Using pip
```bash
pip install lanelet2-bssd-converter
```
This will install the latest version of the Lanelet2 BSSD Converter available in [PyPI](https://pypi.org/project/lanelet2-bssd-converter/) to your environment.

### Manual Installation

Clone the source code to a directory of your choice (```path/to/lanelet2-bssd-converter-src/```).

If you are using virtual environments, make sure to activate the correct environment to install the library into e.g:

```bash
source /<path-to-my-project>/.venv/bin/activate
```

Install the library:
```bash
pip install -e path/to/lanelet2-bssd-converter-src/
```


## Usage

1. Get the path to the Lanelet2 map that you wish to derive the BSSD extension for.
2. To run the converter, use the command:
   ```bash
   lanelet2-bssd-converter -m </path/to/Lanelet2_map>
   ```
3. The tool will automatically execute and show some information about the current status in the terminal.
4. After successful execution, the modified Lanelet2 map will be stored in the same directory as the original map
with "_BSSD" at the end of the filename.
5. Furthermore, a derivation-log-file is saved into the same directory.

> Note: use ```lanelet2-bssd-converter -h``` to see all the available options for the tool.


## Architecture

To get an overview of how this tool is built, read [this](/doc/architecture.md).

## Tests

To run the tests that are included with pytest, open a terminal in the directory in which the repository
is installed and invoke
```bash
pytest test
```
