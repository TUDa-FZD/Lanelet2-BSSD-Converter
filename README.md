# Framework for the Automated Generation of the BSSD Extension for Lanelet2 Maps

This framework generates the BSSD extension for Lanelet2 maps. Behavior spaces are therefore mapped
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

After installing lanelet2 (See the provided [guide](/doc/Lanelet2%20installation%20guide.md) for help) you can install the lanelet2-bssd-converter either using pip or manually from the source code.

### Using pip
```bash
pip install lanelet2-bssd-converter
```

### Manual installation

1. Go to the directory of your choice and clone the repository (with HTTPS or SSH)
   - HTTPS:
      ```bash
        git clone https://gitlab.com/tuda-fzd/scenery-representations-and-maps/lanelet2-bssd-converter.git
        ```

   - SSH:
      ```bash
      git clone git@gitlab.com:tuda-fzd/scenery-representations-and-maps/lanelet2-bssd-converter.git
      ```
2. In the same terminal, go into the directory lanelet2_bssd_converter with
    ```bash
    cd lanelet2_bssd_converter
    ```

3. Install package and dependencies by invoking the following in the same terminal
    ```bash
    pip install -e .
    ```

  (Create & activate a virtual environment if you want to install the tool inside a virtual environment)

## Usage

1. Get the path to the Lanelet2 map that you wish to derive the BSSD extension for.
2. To run the converter, use the command:
    > Note: You need to replace </path/to/Lanelet2/map> in the following command with the actual path to the targeted map file.
    ```bash
    lanelet2-bssd-converter -m </path/to/Lanelet2/map>
    ```
3. The tool will automatically execute and show some information about the current status in the terminal.
4. After successful execution, the modified Lanelet2 map will be stored in the same directory as the original map
with "_BSSD" at the end of the filename.
5. Furthermore, a derivation-log-file is saved into the same directory.

> Note: use ```lanelet2-bssd-converter -h``` to see all the available options for the tool.


## Architecture

To get an overview of how this framework is build, read [this](/doc/architecture.md).

## Tests

To run the tests that are included with pytest, open a terminal in the directory in which the repository
is installed and invoke
```bash
pytest test
```
