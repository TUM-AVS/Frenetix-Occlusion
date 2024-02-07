import os
import sys
import csv
import shutil
from datetime import datetime
import traceback
from cr_scenario_handler.simulation.simulation import Simulation
from cr_scenario_handler.utils.configuration_builder import ConfigurationBuilder
from cr_scenario_handler.utils.general import get_scenario_list
import warnings
warnings.filterwarnings("ignore")


def start_simulation(scenario_name, scenario_folder, mod_path, logs_path, use_cpp, start_multiagent=False, count=0):
    log_path = os.path.join(logs_path, scenario_name)
    config_sim = ConfigurationBuilder.build_sim_configuration(scenario_name=scenario_name,
                                                              scenario_folder=scenario_folder,
                                                              root_path=mod_path)
    config_sim.simulation.use_multiagent = start_multiagent

    config_planner = ConfigurationBuilder.build_frenetplanner_configuration(scenario_name=scenario_name,
                                                                            root_path=mod_path)
    config_planner.debug.use_cpp = use_cpp

    simulation = None

    try:
        simulation = Simulation(config_sim, config_planner)
        simulation.run_simulation()

    except Exception as e:
        try:
            simulation.close_processes()
        except:
            pass
        error_traceback = traceback.format_exc()  # This gets the entire error traceback
        with open(os.path.join(logs_path, 'log_failures.csv'), 'a', newline='') as f:
            writer = csv.writer(f)
            current_time = datetime.now().strftime('%H:%M:%S')
            # Check if simulation is not None before trying to access current_timestep
            current_timestep = str(simulation.global_timestep) if simulation else "N/A"
            writer.writerow(["Scenario Name: " + log_path.split("/")[-1] + "\n" +
                             "Error time: " + str(current_time) + "\n" +
                             "In Scenario Timestep: " + current_timestep + "\n" +
                             "CODE ERROR: " + str(e) + error_traceback + "\n\n\n\n"])
            print(error_traceback)


def main():
    if sys.platform == "darwin":
        os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

    mod_path = os.path.dirname(os.path.abspath(__file__))

    logs_path = os.path.join(mod_path, "logs")
    scenario_folder = os.path.join(mod_path, "example_scenarios")
    # *************************
    # Set Python or C++ Planner
    # *************************
    use_cpp = True

    # **********************************************************************
    # If the previous are set to "False", please specify a specific scenario
    # **********************************************************************
    scenario_list = ['scenario1', 'scenario2', 'scenario3']
    scenario_name = scenario_list[0]  # do not add .xml format to the name

    scenario_files = get_scenario_list(scenario_name, scenario_folder, False, None, False)

    # ***************************************************
    # Delete former logs & Create new score overview file
    # ***************************************************
    delete_former_logs = False
    if delete_former_logs:
        shutil.rmtree(logs_path, ignore_errors=True)
    os.makedirs(logs_path, exist_ok=True)
    if not os.path.exists(os.path.join(logs_path, "score_overview.csv")):
        with open(os.path.join(logs_path, "score_overview.csv"), 'a') as file:
            line = "scenario;agent;timestep;status;message\n"
            file.write(line)

    start_simulation(scenario_files[0], scenario_folder, mod_path, logs_path, use_cpp, False)


if __name__ == '__main__':
    main()
