
[![Linux](https://img.shields.io/badge/os-linux-blue.svg)](https://www.linux.org/)
[![Python 3.10](https://img.shields.io/badge/python-3.10-blue.svg)](https://www.python.org/downloads/release/python-3100/) [![Python 3.9](https://img.shields.io/badge/python-3.9-blue.svg)](https://www.python.org/downloads/release/python-390/)


# 🚸 Occlusion-aware Trajectory Assessment 🚴

<details>
<summary> <h2> 🔧 Requirements & Pre-installation Steps </h2> </summary>

### Requirements
The software is  developed and tested on recent versions of Linux. We strongly recommend to use [Ubuntu 22.04](https://ubuntu.com/download/desktop) or higher.
For the python installation, we suggest the usage of Virtual Environment with Python 3.10 or Python 3.9
For the development IDE we suggest [PyCharm](http://www.jetbrains.com/pycharm/).

To execute this example, we recommend using the [Frenetix Motion Planning Algorithm](https://github.com/TUM-AVS/Frenetix-Motion-Planner). 
An appropriate configuration is already included in this package.


### Pre-installation Steps
1. Make sure that the following **dependencies** are installed on your system for the C++ implementation of Frenetix:
   * [Eigen3](https://eigen.tuxfamily.org/dox/) 
     * On Ubuntu: `sudo apt-get install libeigen3-dev`
   * [Boost](https://www.boost.org/)
     * On Ubuntu: `sudo apt-get install libboost-all-dev`
   * [OpenMP](https://www.openmp.org/) 
     * On Ubuntu: `sudo apt-get install libomp-dev`
   * [python3.10-full](https://packages.ubuntu.com/jammy/python3.10-full) 
        * On Ubuntu: `sudo apt-get install python3.10-full` and `sudo apt-get install python3.10-dev`

2. **Clone** this repository & create a new virtual environment `python3.10 -m venv venv`

3. **Install** the package:
    * Source & Install the package via pip: `source venv/bin/activate` & `pip install .`
    * Everything should install automatically. If not please write [korbinian.moller@tum.de](mailto:korbinian.moller@tum.de).
4. **Optional**: Download additional Scenarios [here](https://gitlab.lrz.de/tum-cps/commonroad-scenarios.git)


</details>


<details>
<summary> <h2> 📋 Step-by-Step Instructions to execute Frenetix-Occlusion </h2> </summary>

To execute the main with an example Frenet Planning algorithm from [here](https://github.com/TUM-AVS/Frenetix-Motion-Planner), do the following steps: 

1. Install Frenetix-Motion-Planner with `pip install frenetix-motion-planner`
2. **OPTIONAL:** Configure the Occlusion Module parameters or Planner settings in the `configurations` folder.
   * Parameters of Frenetix-Occlusion can be set in `simulation/occlusion.yaml`
       * activate/deactivate the Module (`use_occlusion_module: True`) 
       * activate/deactivate metrics (`activated_metrics`)
       * set metric thresholds for trajectory assessment (`metric_thresholds`)
       * configure the internal sensor model (`sensor_model`)
   * further parameters, e.g. costs can be adapted in the other `*.yaml` files

3. Select a scenario (default is an intersection where a left turn is performed)
4. Execute `python3 main.py` or use your IDE GUI

**OPTIONAL:**
To incorporate custom obstacles into the scenario, this can be achieved by modifying the `simulation/occlusion.yaml` file under the `agents` section. 
To add a custom obstacle, you must specify its position, type, velocity, timestep, and horizon.
E.g.:

    agents:

        position: [16, 2.8]

        velocity: 8

        agent_type: "Truck"

        timestep: 0

        horizon: 5.0


</details>




<details>
<summary> <h2> 📈 Test Data </h2> </summary>

Additional scenarios can be found [here](https://commonroad.in.tum.de/scenarios).

</details>


<details>
<summary> <h2> 📇 Contact Info </h2> </summary>

[Korbinian Moller](mailto:korbinian.moller@tum.de),
Professorship Autonomous Vehicle Systems,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

[Rainer Trauth](mailto:rainer.trauth@tum.de),
Institute of Automotive Technology,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

[Johannes Betz](mailto:johannes.betz@tum.de),
Professorship Autonomous Vehicle Systems,
School of Engineering and Design,
Technical University of Munich,
85748 Garching,
Germany

</details>

<details>
<summary> <h2> 📃 Citation </h2> </summary>
   
If you use this repository for any academic work, please cite our code:
- [Occlusion-aware Planning](https://arxiv.org/abs/2402.01507)

```bibtex
@misc{moller2024overcoming,
      title={Overcoming Blind Spots: Occlusion Considerations for Improved Autonomous Driving Safety}, 
      author={Korbinian Moller and Rainer Trauth and Johannes Betz},
      year={2024},
      eprint={2402.01507},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```
</details>
