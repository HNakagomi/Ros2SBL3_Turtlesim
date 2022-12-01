# for turtlesim client
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty

# for ros2
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

# for gym
import gym
import numpy as np

import time

# for stable_baselines
from stable_baselines3 import PPO
from stable_baselines3 import DQN
#from stable_baselines3.common.callbacks import CheckpointCallback

class MyEnv(Node, gym.Env):
    def __init__(self):

        rclpy.init()

        # ros2 service ##############################
        super().__init__('rl_test_client')

        # generation of clients
        self.cli = self.create_client(Spawn, 'spawn')
        self.cli_kill = self.create_client(Kill, 'kill')
        self.cli_reset = self.create_client(Empty, 'reset')

        # check availability of services
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # generation of a request instance. Request data is stored in this.
        self.req = Spawn.Request()
        self.req_kill = Kill.Request()
        self.req_reset = Empty.Request()

        # set ros2 subscriber ###############################
        self.subscription = self.create_subscription(
            Pose,
            '/kame1/pose',
            self.subscribe_source_pos_callback,
            1)
        self.subscription = self.create_subscription(
            Pose,
            '/kame2/pose',
            self.subscribe_target_pos_callback,
            1)
        self.subscription  # prevent unused variable warning

        # set ros2 publisher ###################################
        self.publisher_ = self.create_publisher(
            Twist, 
            '/kame1/cmd_vel',
             1)

        # gym #####################################
        self.WINDOW_SIZE = 12 # turtlesim window size

        # set action map
        self.ACTION_MAP = np.array([[1, 0], [1, 3.14], [1, -3.14]])
        ACTION_NUM = 3
        self.action_space = gym.spaces.Discrete(ACTION_NUM)

        # reach of judging the goal
        self.GOAL_RANGE = 1

        # Define the range of states
        LOW = np.array([-np.pi])
        #LOW = np.array([-np.pi,0])
        HIGH = np.array([np.pi])
        #HIGH = np.array([np.pi,12])
        self.observation_space = gym.spaces.Box(low=LOW, high=HIGH)

        # executing reset 
        self.reset()

    def send_request_reset(self):
        self.future = self.cli_reset.call_async(self.req_reset)

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Response reset')
                break

    def send_request_kill(self,Name=None):

        self.req_kill.name = Name
        self.future = self.cli_kill.call_async(self.req_kill)

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Response kill[%s]' % Name)
                break
 

    def send_request_respawn(self,position=None,Theta=None,Name=None):

        self.req.x = float(position[0])
        self.req.y = float(position[1])
        self.req.theta = float(Theta)
        self.req.name = Name
        self.future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if self.future.done():
                try:
                    response = self.future.result()
                except Exception as e:
                    self.get_logger().info(
                        'Service call failed %r' % (e,))
                else:
                    self.get_logger().info('Response respawn[%s]' % response.name)
                break

    def subscribe_source_pos_callback(self,msg):
        self.source_position = msg

    def subscribe_target_pos_callback(self,msg):
        self.target_position = msg

    def publish_twist(self,action=None):
        msg = Twist()
        msg.linear.x = float(action[0])
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(action[1])
        self.publisher_.publish(msg)

    def conv_to_local(self,source_position=None,target_position=None,source_theta=None):
        conv_target_x = np.cos(-source_theta) * (target_position[0] - source_position[0]) - np.sin(-source_theta) * (target_position[1] - source_position[1])
        conv_target_y = np.sin(-source_theta) * (target_position[0] - source_position[0]) + np.cos(-source_theta) * (target_position[1] - source_position[1])
        return np.array([conv_target_x,conv_target_y])

    def reset(self):

        # ros
        # init source and target position
        source_position = np.array([np.random.randint(0, self.WINDOW_SIZE), np.random.randint(0, self.WINDOW_SIZE)])
        source_theta = np.random.randint(0, 3.1425*2)
        target_position = np.array([np.random.randint(0, self.WINDOW_SIZE), np.random.randint(0, self.WINDOW_SIZE)])

        # initialize request to turtlesim server
        self.send_request_reset() # reset simulator
        self.send_request_kill(Name="turtle1")
        self.send_request_respawn(position=source_position,Theta=source_theta,Name="kame1")
        self.send_request_respawn(position=target_position,Theta=0.0,Name="kame2")

        # get observation
        conv_target_position = self.conv_to_local(source_position=source_position,target_position=target_position,source_theta=source_theta)
        direction = np.arctan2(conv_target_position[1],conv_target_position[0])
        observation = np.array([direction])
        self.get_logger().info('step: direction: "%f"' % direction)

        distance = np.linalg.norm(source_position - target_position) # distance
        self.before_distance = distance

        return observation


    def step(self, action_index):

        action = self.ACTION_MAP[action_index]

        # execute action
        self.publish_twist(action=action)

        # wait 
        time.sleep(0.02)

        # get observation
        rclpy.spin_once(self)
        rclpy.spin_once(self)

        source_position = np.array([self.source_position.x,self.source_position.y])
        theta = self.source_position.theta
        target_position = np.array([self.target_position.x,self.target_position.y])
        conv_target_position = self.conv_to_local(source_position=source_position,target_position=target_position,source_theta=theta)

        direction = np.arctan2(conv_target_position[1],conv_target_position[0])
        observation = np.array([direction])
        self.get_logger().info('step: direction: "%f"' % direction)

        # get reward value
        distance = np.linalg.norm(source_position - target_position) # distance
        reward = self.before_distance - distance # reward
        self.get_logger().info('step: distance: "%f"' % distance)
        self.get_logger().info('step: reward: "%f"' % reward)

        # done judgement
        done = False
        if distance < self.GOAL_RANGE:
           done = True
        # range out
        if source_position[0] < 0.2 or source_position[0] > 10.8:
            done = True
        if source_position[1] < 0.2 or source_position[1] > 10.8:
            done = True

        self.before_distance = distance

        return observation, reward, done, {}


    def destroy(self):
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):

    env = MyEnv()

    model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="log")
    #model = DQN("MlpPolicy", env, verbose=1, tensorboard_log="log")

    # train
    #model.learn(total_timesteps=10000)
    #model.save('./save_weights/rl_model_final_steps.zip')

    # test
    model = PPO.load("./save_weights/rl_model_final_steps.zip")
    #model = DQN.load("./save_weights/rl_model_final_steps.zip")
    for i in range(100):
        obs = env.reset()
        while True:
            action, _states = model.predict(obs)
            obs, rewards, dones, info = env.step(action)
            if dones:
                break

    env.destroy()

if __name__ == '__main__':
    main()
