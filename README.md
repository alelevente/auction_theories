# **SIA-based Parking Assignment**

In this repository, you can find source codes for the article:

*L. Alekszejenko and T. Dobrowiecki*, "Attitude-driven Simultaneous Online Auctions for Parking Spaces", Infocommunications Journal, XVII(3), pp. 73-83, 2025. DOI: 10.36244/ICJ.2025.3.9

Please cite the article as:
```bibtex
@article{aldt25,
    author = {L. Alekszejenkó and T. Dobrowiecki},
    title = {Attitude-driven Simultaneous Online Auctions for Parking Spaces},
    journal = {Infocommunications Journal},
    volume = {XVII},
    number = {3},
    pages = {73-83},
    doi = {10.36244/ICJ.2025.3.9},
    year = {2025}
}
```

## Content and Usage

- `01_simulation` contains inputs for simulating a small town's parking.
- `02_src` contains the source codes, and measurement scripts
- `03_results` will contain the files created by the scipts

To check the numeric simulations, see the notebook of `02_src/serial_auctions.ipynb`.

## Running the simulation of the small town:

**Prerequisites**

Install [Eclipse SUMO](https://eclipse.dev/sumo/).

**Simulation steps**

0. You can check whether or not the Eclipse SUMO is available on your compter by `02_src/traci_test.ipynb`
1. Run the cells of the `02_src/setup.ipynb` notebook.
2. Run the measurement scripts in `02_src` folder: `meas_beta10.sh`, `meas_beta50.sh`, `meas_beta90.sh`, `meas_betaMIX.sh`.
3. Analyze the results by `02_src/results.ipynb`.
