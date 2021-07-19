#!/usr/bin/env python
import rospy
import sys
import select
from collections import defaultdict

class Module:
    def __init__(self, agent_id):
        self.cli_cmds = []

    def update(self):
        pass

    def cli(self, cmd):
        pass


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
            for module in self.modules:
                module.update()
            # Parse cli commands
            has_input, _, _ = select.select( [sys.stdin], [], [], 0.1 ) 
            if (has_input):
                cli_str = sys.stdin.readline().strip()
                cli_str = cli_str.lower()
                for module in self.cli_cmds[cli_str]:
                    module.cli(cli_str)
                if cli_str in ['k', 'kill']:
                    sys.exit()
                
                

