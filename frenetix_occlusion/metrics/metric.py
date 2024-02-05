__author__ = "Korbinian Moller,"
__copyright__ = "TUM Professorship Autonomous Vehicle Systems"
__version__ = "1.0"
__maintainer__ = "Korbinian Moller"
__email__ = "korbinian.moller@tum.de"
__status__ = "Beta"

# imports
from frenetix_occlusion.metrics.hr import HR  # harm and risk metric
from frenetix_occlusion.metrics.cp import CP  # collision probability metric
from frenetix_occlusion.metrics.dce import DCE  # distance to closest encounter metric
from frenetix_occlusion.metrics.ttc import TTC  # time to collision
from frenetix_occlusion.metrics.ttce import TTCE  # distance to closest encounter metric
from frenetix_occlusion.metrics.wttc import WTTC  # worst time to collision metric
from frenetix_occlusion.metrics.be import BE  # break evaluation metric


class Metric:
    """
    Base class for handling metrics for oap calculations.
    """
    def __init__(self, config, vehicle_params, agent_manager):
        """
        Initialize the Metric class with a list of metric names.
        :param config: metric configuration dict
        """
        self.config = config
        self.metric_thresholds = config["metric_thresholds"]
        self.vehicle_params = vehicle_params
        self.agent_manager = agent_manager
        self.metrics = self._initialize_metrics(self.config['activated_metrics'])

        return

    def evaluate_metrics(self, trajectory):
        """
        Evaluate all configured metrics.
        :param trajectory: The ego trajectory to be used for evaluating the metrics.
        :return: Dictionary of metric evaluations.
        """
        results = {}

        # return if no agents area available
        if not self.agent_manager.phantom_agents or not self.metrics:
            return results, True

        for name, metric in self.metrics.items():
                results[name] = metric.evaluate(trajectory, results)

        # evaluate metrics and perform the safety check
        safety_check = True

        # perform trajectory safety assessment with given thresholds
        # break threat number threshold (be)
        if 'be' in results and self.metric_thresholds['be'] is not None:
            be = results['be']
            for key in be:
                if be[key]['break_threat_number'] > self.metric_thresholds['be']:
                    # print('btn too high')
                    safety_check = False
                    break

        # harm threshold (hr)
        if 'hr' in results and self.metric_thresholds['harm'] is not None:
            harm = results['hr']['max_obst_harm_with_cp_all']
            if harm > self.metric_thresholds['harm']:
                # print('trajectory harm too high')
                safety_check = False

        # risk threshold (hr)
        if 'hr' in results and self.metric_thresholds['risk'] is not None:
            risk = results['hr']['max_obst_risk_all']
            if risk > self.metric_thresholds['risk']:
                # print('trajectory risk too high')
                safety_check = False

        # collision probability threshold (cp)
        if 'hr' in results and self.metric_thresholds['cp'] is not None:
            cp = results['hr']['max_collision_probability_all']
            if cp > self.metric_thresholds['cp']:
                # print('collision probability too high')
                safety_check = False

        # time to collision (ttc)
        if 'ttc' in results and self.metric_thresholds['ttc'] is not None:
            ttc = results['ttc']
            if min(ttc.values()) < self.metric_thresholds['ttc']:
                # print('ttc too close')
                safety_check = False

        # distance to closest encounter (dce)
        if 'dce' in results and self.metric_thresholds['dce'] is not None:
            dce = results['dce']
            for key in dce:
                if dce[key]['dce'] < self.metric_thresholds['dce']:
                    # print('dce to small')
                    safety_check = False
                    break

        return results, safety_check

    def _initialize_metrics(self, metric_names):
        """
        Initialize metrics based on the provided names.
        :param metric_names: List of metric names.
        :return: Dictionary of metric instances.
        """
        # available classes
        metric_classes = {
            'dce': DCE(self.vehicle_params, self.agent_manager),
            'cp': CP(self.vehicle_params, self.agent_manager),
            'ttc': TTC(self.agent_manager),
            'ttce': TTCE(self.agent_manager),
            'wttc': WTTC,
            'be': BE(self.vehicle_params, self.agent_manager),
            'hr': HR(self.vehicle_params, self.agent_manager)
        }

        metric_names = self._check_required_metrics(metric_names)

        # return specified metrics (instantiate object if needed)
        return {name: metric_classes[name] if type(metric_classes[name]) is not type else metric_classes[name]()
                for name in metric_names if name in metric_classes}

    @staticmethod
    def _check_required_metrics(metric_names):
        # check required metrics

        # if wttc shall be calculated, ttc has to be calculated first
        if 'wttc' in metric_names:
            if 'ttc' in metric_names:
                metric_names.remove('ttc')
            metric_names.insert(0, 'ttc')

        # if ttc or ttce shall be calculated, dce has to be calculated first
        if 'ttc' in metric_names or 'ttce' in metric_names or 'be' in metric_names:
            if 'dce' in metric_names:
                metric_names.remove('dce')
            metric_names.insert(0, 'dce')

        # if harm and risk (hr) metric shall be calculated, the collision probability must be available
        if 'hr' in metric_names:
            if 'cp' in metric_names:
                metric_names.remove('cp')
            metric_names.insert(0, 'cp')

        return metric_names
