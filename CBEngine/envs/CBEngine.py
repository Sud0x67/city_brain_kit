import gym
import citypb
import os
import gc
import numpy as np
class CBEngine(gym.Env):
    def __init__(self,simulator_cfg_file,thread_num,gym_dict,metric_period):
        self.simulator_cfg_file = simulator_cfg_file
        # with open(cfg_file,'r') as f:
        #     lines = f.readlines()
        #     for line in lines:
        #         line = line.rstrip('\n').split(' ')
        #         if(line[0] == 'simulator_cfg'):
        #             self.simulator_cfg_file = line[-1]
        #         if (line[0] == 'observation_features'):
        #             self.observation_features = line[2:]
        self.observation_features = gym_dict['observation_features']
        self.thread_num = thread_num
        self.eng = citypb.Engine(self.simulator_cfg_file,thread_num)
        self.action_space = gym.spaces.Discrete(9)
        self.num_per_action = 10
        self.vehicles = {}
        self.metric_period = metric_period

        # CFG FILE MUST HAVE SPACE
        with open(self.simulator_cfg_file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                line = line.rstrip('\n').split(' ')
                if(line[0] == 'start_time_epoch'):
                    self.now_step = int(line[-1])
                if(line[0] == 'max_time_epoch'):
                    self.max_step = int(line[-1])
                if(line[0] == 'road_file_addr'):
                    self.roadnet_file = line[-1]
                if(line[0] == 'report_log_rate'):
                    self.log_interval = int(line[-1])
                if(line[0] == 'report_log_addr'):
                    self.log_path = line[-1]



        # here agent is those intersections with signals
        self.intersections = {}
        self.roads = {}
        self.agents = {}
        self.lane_vehicle_state = {}
        self.log_enable = 1
        self.warning_enable = 1
        self.ui_enable = 1
        self.info_enable = 1
        with open(self.roadnet_file,'r') as f:
            lines = f.readlines()
            cnt = 0
            pre_road = 0
            is_obverse = 0
            for line in lines:
                line = line.rstrip('\n').split(' ')
                if('' in line):
                    line.remove('')
                if(len(line) == 1):
                    if(cnt == 0):
                        self.agent_num = int(line[0])
                        cnt+=1
                    elif(cnt == 1):
                        self.road_num = int(line[0])*2
                        cnt +=1
                    elif(cnt == 2):
                        self.signal_num = int(line[0])
                        cnt+=1
                else:
                    if(cnt == 1):
                        self.intersections[int(line[2])] = {
                            'latitude':float(line[0]),
                            'longitude':float(line[1]),
                            'have_signal':int(line[3]),
                            'end_roads':[],
                            'start_roads':[]
                        }
                    elif(cnt == 2):
                        if(len(line)!=8):
                            road_id = pre_road[is_obverse]
                            self.roads[road_id]['lanes'] = {}
                            for i in range(self.roads[road_id]['num_lanes']):
                                self.roads[road_id]['lanes'][road_id*100+i] = list(map(int,line[i*3:i*3+3]))
                                self.lane_vehicle_state[road_id*100+i] = set()
                            is_obverse ^= 1
                        else:
                            self.roads[int(line[-2])]={
                                'start_inter':int(line[0]),
                                'end_inter':int(line[1]),
                                'length':float(line[2]),
                                'speed_limit':float(line[3]),
                                'num_lanes':int(line[4]),
                                'inverse_road':int(line[-1])
                            }
                            self.roads[int(line[-1])] = {
                                'start_inter': int(line[1]),
                                'end_inter': int(line[0]),
                                'length': float(line[2]),
                                'speed_limit': float(line[3]),
                                'num_lanes': int(line[5]),
                                'inverse_road':int(line[-2])
                            }
                            self.intersections[int(line[0])]['end_roads'].append(int(line[-1]))
                            self.intersections[int(line[1])]['end_roads'].append(int(line[-2]))
                            self.intersections[int(line[0])]['start_roads'].append(int(line[-2]))
                            self.intersections[int(line[1])]['start_roads'].append(int(line[-1]))
                            pre_road = (int(line[-2]),int(line[-1]))
                    else:
                        # 4 out-roads
                        signal_road_order = list(map(int,line[1:]))
                        now_agent = int(line[0])
                        in_roads = []
                        for road in signal_road_order:
                            if(road != -1):
                                in_roads.append(self.roads[road]['inverse_road'])
                            else:
                                in_roads.append(-1)
                        in_roads += signal_road_order
                        self.agents[now_agent] = in_roads

                        # 4 in-roads
                        # self.agents[int(line[0])] = self.intersections[int(line[0])]['end_roads']
                        # 4 in-roads plus 4 out-roads
                        # self.agents[int(line[0])] += self.intersections[int(line[0])]['start_roads']
        for agent,agent_roads in self.agents.items():
            self.intersections[agent]['lanes'] = []
            for road in agent_roads:
                ## here we treat road -1 have 3 lanes
                if(road == -1):
                    for i in range(3):
                        self.intersections[agent]['lanes'].append(-1)
                else:
                    for lane in self.roads[road]['lanes'].keys():
                        self.intersections[agent]['lanes'].append(lane)



        # add 1 dimension to give current step for fixed time agent
        # features = ['get_lane_vehicle_count','get_lane_speed']

        ob_space = {}
        for agent in self.agents.keys():
            for feature in self.observation_features:
                ob_space["{}_{}".format(agent,feature)] = gym.spaces.Box(low=-1e10, high=1e10, shape=(len(self.intersections[agent]['lanes']) + 1,))
        self.observation_space = gym.spaces.Dict(ob_space)

    def set_log(self,flg):
        self.log_enable = flg

    def set_warning(self,flg):
        self.warning_enable = flg

    def set_ui(self,flg):
        self.ui_enable = flg

    def set_info(self,flg):
        self.info_enable = flg

    def step(self, action):
        # here action is a dict {agent_id:phase}
        for agent_id,phase in action.items():
            result = self.eng.set_ttl_phase(agent_id,phase)
            if(result == -1 and self.warning_enable):
                print('Warnning: at step {} , agent {} switch to phase {} . Maybe empty road'.format(self.now_step,agent_id,phase))
        for cur in range(self.num_per_action):
            self.eng.next_step()
            self.now_step+=1




            if((self.now_step +1)% self.log_interval == 0 and self.ui_enable==1):
                self.eng.log_info(os.path.join(self.log_path,'time{}.json'.format(self.now_step//self.log_interval)))

            if((self.now_step+1) % self.log_interval ==0 and self.log_enable == 1):
                # replay file
                # vehicle info file
                vlist = self.eng.get_vehicles()
                for vehicle in vlist:
                    if(vehicle not in self.vehicles.keys()):
                        self.vehicles[vehicle] = {}
                    for k,v in self.eng.get_vehicle_info(vehicle).items():
                        self.vehicles[vehicle][k] = v
                        self.vehicles[vehicle]['step'] = [self.now_step]
            if((self.now_step + 1) % self.metric_period == 0 and self.log_enable == 1):
                with open(os.path.join(self.log_path,'info_step {}.log'.format(self.now_step)),'w+') as f:
                    f.write("{}\n".format(self.eng.get_vehicle_count()))
                    for vehicle in self.vehicles.keys():
                        # if(self.vehicles[vehicle]['step'][0] <= self.now_step - self.metric_period):
                        #     continue
                        f.write("for vehicle {}\n".format(vehicle))
                        for k,v in self.vehicles[vehicle].items():
                            # f.write("{}:{}\n".format(k,v))
                            if(k != 'step'):
                                f.write("{} :".format(k))
                                for val in v:
                                    f.write(" {}".format(val))
                                f.write("\n")
                        f.write('step :')
                        for val in self.vehicles[vehicle]['step']:
                            f.write(" {}".format(val))
                        f.write("\n")
                        f.write("-----------------\n")
            # if((self.now_step+1) % self.log_interval ==0 and self.log_enable == 1):
            # # replay file
            # self.eng.log_info(os.path.join(self.log_path,'time{}.json'.format(self.now_step//self.log_interval)))
            # # vehicle info file
            # with open(os.path.join(self.log_path,'info_step {}.log'.format(self.now_step)),'w+') as f:
            #     f.write("{}\n".format(self.eng.get_vehicle_count()))
            #     for vehicle in self.eng.get_vehicles():
            #         f.write("for vehicle {}\n".format(vehicle))
            #         for k,v in self.eng.get_vehicle_info(vehicle).items():
            #             # f.write("{}:{}\n".format(k,v))
            #             f.write("{} :".format(k))
            #             for val in v:
            #                 f.write(" {}".format(val))
            #             f.write("\n")
            #         f.write("-----------------\n")

        reward = self._get_reward()
        dones = self._get_dones()
        obs = self._get_observations()
        info = self._get_info()
        return obs, reward, dones , info

    def reset(self):
        del self.eng
        gc.collect()
        self.eng = citypb.Engine(self.simulator_cfg_file, self.thread_num)
        self.now_step = 0
        self.vehicles.clear()
        return self._get_observations(),self._get_info()
    def _get_info(self):
        info = {}
        if(self.info_enable == 0):
            return info
        else:
            v_list = self.eng.get_vehicles()
            for vehicle in v_list:
                info[vehicle] = self.eng.get_vehicle_info(vehicle)
            return info
    def _get_reward(self):

        def get_diff(pre,sub):
            in_num = 0
            out_num = 0
            for vehicle in pre:
                if(vehicle not in sub):
                    out_num +=1
            for vehicle in sub:
                if(vehicle not in pre):
                    in_num += 1
            return in_num,out_num

        rwds = {}
        # return every
        lane_vehicle = self.eng.get_lane_vehicles()

        for agent_id, roads in self.agents.items():
            rwds[agent_id] = []
            for lane in self.intersections[agent_id]['lanes']:
                # -1 indicates empty roads in 'signal' of roadnet file
                if (lane == -1):
                    rwds[agent_id].append(-1)
                else:
                    if(lane not in lane_vehicle.keys()):
                        lane_vehicle[lane] = set()
                    rwds[agent_id].append(get_diff(self.lane_vehicle_state[lane],lane_vehicle[lane]))
                    self.lane_vehicle_state[lane] = lane_vehicle[lane]

        return rwds



    def _get_observations(self):
        # return self.eng.get_lane_vehicle_count()
        obs = {}
        lane_vehicle = self.eng.get_lane_vehicles()
        vehicle_speed = self.eng.get_vehicle_speed()

        features = self.observation_features

        # add 1 dimension to give current step for fixed time agent
        for agent_id, roads in self.agents.items():
            for feature in features:
                now_key = "{}_{}".format(agent_id,feature)
                obs[now_key] = [self.now_step]
                if(feature == 'lane_speed'):
                    for lane in self.intersections[agent_id]['lanes']:
                        # -1 indicates empty roads in 'signal' of roadnet file
                        if(lane == -1):
                            obs[now_key].append(-1)
                        else:
                            # -2 indicates there's no vehicle on this lane
                            if(lane not in lane_vehicle.keys()):
                                obs[now_key].append(-2)
                            else:
                                # the average speed of this lane
                                speed_total = 0.0
                                for vehicle in lane_vehicle[lane]:
                                    speed_total += vehicle_speed[vehicle]
                                obs[now_key].append(speed_total / len(lane_vehicle[lane]))

                if(feature == 'lane_vehicle_num'):
                    for lane in self.intersections[agent_id]['lanes']:
                        # -1 indicates empty roads in 'signal' of roadnet file
                        if(lane == -1):
                            obs[now_key].append(-1)
                        else:
                            # -2 indicates there's no vehicle on this lane
                            if(lane not in lane_vehicle.keys()):
                                obs[now_key].append(0)
                            else:
                                # the vehicle number of this lane
                                obs[now_key].append(len(lane_vehicle[lane]))
        return obs

    def _get_dones(self):
        #
        dones = {}
        for agent_id in self.agents.keys():
            dones[agent_id] = self.now_step >= self.max_step

        return dones