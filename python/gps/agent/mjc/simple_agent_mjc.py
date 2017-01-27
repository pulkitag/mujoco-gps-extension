import copy
import numpy as np
import mjcpy
import os
from os import path as osp

from gps.agent.agent import Agent
from gps.agent.config import AGENT_MUJOCO

class SimpleAgentMuJoCo(object):
    """
    All communication between the algorithms and MuJoCo is done through
    this class.
    """
    def __init__(self, hyperparams):
        config = copy.deepcopy(AGENT_MUJOCO)
        config.update(hyperparams)
        self._hyperparams = config
        #Agent.__init__(self, config)
        self._setup_world(hyperparams['filename'])

    def _setup_world(self, filename):
        """
        Helper method for handling setup of the MuJoCo world.
        Args:
            filename: Path to XML file containing the world information.
        """
        self._world = []
        self._model = []

        world = mjcpy.MJCWorld(filename)
        self._world = [world]
        self._model = [self._world[0].get_model().copy()]
        self._joint_idx = list(range(self._model[0]['nq']))
        self._vel_idx = [i + self._model[0]['nq'] for i in self._joint_idx]

        cam_pos = self._hyperparams['camera_pos']
        self._world[0].init_viewer(AGENT_MUJOCO['image_width'],
                                   AGENT_MUJOCO['image_height'],
                                   0, 0, cam_pos[2],
                                   cam_pos[3], cam_pos[4], cam_pos[5])

    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
      pass


def simple_test():
  hyperparams = {}
  DATA_DIR    = osp.join(os.getenv('HOME'), 'code', 'gps', 'mjc_models') 
  hyperparams['filename'] = osp.join(DATA_DIR, 'reacher.xml')
  return SimpleAgentMuJoCo(hyperparams) 
