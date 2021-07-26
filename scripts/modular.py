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

    def update(self): return None
    def cli(self, cmd): pass

    def print_v(self, msg): 
        '''verbose print'''
        print(self.verbose_name + msg)
    
    def get_topic(self, topic): 
        # self.print_v("composed topic: '" + self.base_topic + topic + "'")
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
            self.run_single_round()   

    # Needed for the parallels runs:
    def run_single_round(self):
        for module in self.modules: 
            optional_msg = module.update()
            if(optional_msg is not None): self.parse_cli(optional_msg)

        # Parse cli commands
        has_input, _, _ = select.select( [sys.stdin], [], [], 0.1 )

        if (has_input):
            cli_str = sys.stdin.readline().strip().lower()
            print("got cli command: '" + cli_str + "'")
            self.parse_cli(cli_str)

    def parse_cli(self, msg):
        if msg in ['k', 'kill']: 
            print("killing program")
            sys.exit()
        for module in self.cli_cmds[msg]: 
            module.cli(msg)