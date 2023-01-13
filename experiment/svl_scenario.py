import lgsvl
from lgsvl.geometry import Transform, Vector
from tqdm import tqdm
import os
import random
import yaml

class svl_scenario(object):
    def __init__(self, cfg_path):
        # load cfg file        
        with open(cfg_path, 'r') as f:
            self.cfg = yaml.load(f, Loader=yaml.FullLoader)

        self.sim = lgsvl.Simulator(
		    address=self.cfg['simulator']['address'],
		    port=self.cfg['simulator']['port'])
        
        self.origin = Transform()                
        self.is_collapsed = False
        self.collison_info = [] 
        self.u_forward = Vector(0,0)
        self.u_right = Vector(0,0)
        self.start_flag = False

        self.reset()

    def reset(self):
        # Reset scene
        target_scene = self.cfg['simulator']['scene']
        if self.sim.current_scene == target_scene:
            self.sim.reset()
        else:
            self.sim.load(target_scene)

        # Reset origin
        spawns = self.sim.get_spawn()
        self.origin = Transform(
			spawns[0].position,
			spawns[0].rotation)
        self.origin.position.x += self.cfg['origin']['offset']['x']
        self.origin.position.y += self.cfg['origin']['offset']['y']
        self.origin.position.z += self.cfg['origin']['offset']['z']
        self.origin.rotation.y += self.cfg['origin']['offset']['r']

        self.is_collapsed = False        
        self.collaped_position = []

        self.u_forward = lgsvl.utils.transform_to_forward(self.origin)
        self.u_right = lgsvl.utils.transform_to_right(self.origin)

        return

    def create_ego(self):
        ego_state = lgsvl.AgentState()
        ego_state.transform = Transform(self.origin.position, self.origin.rotation)
        ego = self.sim.add_agent(self.cfg['ego']['asset-id'], lgsvl.AgentType.EGO, ego_state)
        ego.connect_bridge(self.cfg['lgsvl_bridge']['address'], self.cfg['lgsvl_bridge']['port'])

        def ego_collision(agent1, agent2, contact):
            self.collaped_position = [ego_state.position.x, ego_state.position.z]
            self.is_collapsed = True
            return
        
        ego.on_collision(ego_collision)

        return

    def create_npc(self):
        if not 'npc' in self.cfg: return

        for i in range(len(self.cfg['npc'])):
            npc_state = lgsvl.AgentState()

            npc_state.transform = \
				Transform(lgsvl.Vector(0,0,0), self.origin.rotation)

            npc_state.transform.position += \
				self.cfg['npc'][i]['offset']['forward'] * self.u_forward

            npc_state.transform.position += \
				self.cfg['npc'][i]['offset']['right'] * self.u_right

            npc_state.transform.rotation = \
				self.origin.rotation + lgsvl.Vector(0, self.cfg['npc'][i]['offset']['rotation'], 0)			
            
            npc = self.sim.add_agent(self.cfg['npc'][i]['type'], lgsvl.AgentType.NPC, npc_state)

        return

    def init(self):
        self.sim.reset()
        self.create_ego()
        self.create_npc()

    def run(self, timeout, is_init=False, label='None'):
        if is_init: self.sim.run(timeout)
        else:
            pbar = tqdm(range(timeout))
            for _ in pbar:
                pbar.set_description('Duration: ' + label)
                self.sim.run(1)
                if self.is_collapsed: break
                
        return self.is_collapsed, self.collaped_position
