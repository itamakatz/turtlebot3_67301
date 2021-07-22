#!/usr/bin/env python
import rospy
import sys
import select
from collections import defaultdict

class Module(object):

    def __init__(self, agent_id, name):
        self.cli_cmds = []
        self.agent_id = agent_id
        self.base_topic = '/tb3_' + str(agent_id)
        self.name = name
        self.verbose_name = self.name + '.' + str(self.agent_id) + ' | '

    def update(self): pass
    def cli(self, cmd): pass

    def print_v(self, msg): 
        '''verbose print'''
        print(self.verbose_name + msg)
    
    def get_topic(self, topic): 
        self.print_v("composed topic: '" + self.base_topic + topic + "'")
        return self.base_topic + topic

class Modular:

    def __init__(self, module_list):
        self.modules = module_list
        self.cli_cmds = defaultdict(list)
        for module in self.modules:
            for cmd in module.cli_cmds:
                self.cli_cmds[cmd].append(module)
    
    def run(self):
        # Update the modules
        while (not rospy.is_shutdown()):
            for module in self.modules: module.update()
            # Parse cli commands
            has_input, _, _ = select.select( [sys.stdin], [], [], 0.1 ) 
            if (has_input):
                cli_str = sys.stdin.readline().strip().lower()
                print("got cli command: '" + cli_str + "'")
                if cli_str in ['k', 'kill']: 
                    print("killing program")
                    sys.exit()
                for module in self.cli_cmds[cli_str]: 
                    module.cli(cli_str)