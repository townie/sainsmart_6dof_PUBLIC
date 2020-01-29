import os
import sys
import signal
import argparse

from models.oneshot import OneShotMeta
from models.oneshotv2 import OneShotMetaV2

from arm_env import ArmEnv
from models.common import logger

do_exit = False


class ModelEnvWrapper(object):
    def __init__(self, env, agent, **kwargs):
        self.env = env
        self.agent = agent
        self.state = None
        self.allargs = kwargs

    def reset(self):
        self.agent.explore_noise.reset()
        return self.env.reset_env()

    def run_epoch(self, test=True):
        self.state = self.reset()
        R = 0  # return
        t = 1
        terminate = False
        while not terminate:
            action = self.agent.get_action(self.state, with_noise=not test)
            state_n, reward, terminate = self.env.step_forward(action)
            if not test:
                self.agent.feedback(self.state, action, reward, term, state_n)
            self.state = state_n
            t += 1
            R += reward
            if do_exit:
                print("step:", t, ", reward:", reward, ", total_reward:", R)
                sys.exit(-1)
        return R, t


def create_dir(dir_name):
    if not os.path.isdir(dir_name):
        os.makedirs(dir_name)
    return dir_name


def signal_handler(signum, frame):
    global do_exit
    do_exit = True

def model_importer(name):
    components = name.split('.')
    mod = __import__(components[0])
    for comp in components[1:]:
        mod = getattr(mod, comp)
    return mod

def run(args):
    state_dim = 5
    action_dim = 6
    train_dir = create_dir('./result')

    # make a model
    if not args.model_class:
        agent = OneShotMeta(state_dim, action_dim, train_dir=train_dir)

    else:

        ModelClass = model_importer(args.model_class)
        agent = ModelClass(state_dim, action_dim, train_dir=train_dir, gamma=args.gamma)

    if hasattr(agent, 'explore_noise'):
        agent.explore_noise.theta = 1.0
        agent.explore_noise.sigma = 2.0
    
    
    env = ArmEnv(image_shape=agent.image_size, max_move_step=args.tmax, gamma=args.gamma)

    t_train, t_test = 0, 0
    modelwrapper = ModelEnvWrapper(env, agent, args.tmax)
    while True:
        # test
        T = t_test
        R = []
        while t_test - T < args.test:
            r, t = modelwrapper.run_epoch(test=True)
            R.append(r)
            t_test += t
        if len(R) > 0:
            avr = sum(R) / len(R)
        # train
        T = t_train
        R = []
        while t_train - T < args.train:
            r, t = modelwrapper.run_epoch(test=False)
            R.append(r)
            t_train += t
        if len(R) > 0:
            avr = sum(R) / len(R)
        
            logger.info('loss {}  steps {} '.format(avr, t_train))


def parser_argument():
    parse = argparse.ArgumentParser()
    parse.add_argument("--train", type=int, default=5000, help="train time step")
    parse.add_argument("--test", type=int, default=400, help="test time step")
    parse.add_argument("--tmax", type=int, default=200, help="time step max")
    parse.add_argument("--gamma", type=float, default=0.99, help="gamma")
    parse.add_argument("--model_class", type=str, default="models.OneShotMeta", help="model class")

    args = parse.parse_args()
    return args


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    run(parser_argument())
