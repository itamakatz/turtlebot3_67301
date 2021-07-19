#!/usr/bin/env python
import rospy
import sys
import multi_move_base 
from modular import Modular
from visited_map_module import VisitedMapModule

def vacuum_cleaning(agent_id):
       
    #multi_move_base.move(0,1,0.5)
    x = 0
    y = 1   
    print('cleaning (%d,%d)'%(x,y))
    result = multi_move_base.move(agent_id, x,y)
    
    print('moving agent %d'%agent_id)
    x = 1
    y = 0   
    print('cleaning (%d,%d)'%(x,y))
    result = multi_move_base.move(agent_id, x,y)
    
    
    #multi_move_base.move(1,2,0.5)
    #multi_move_base.move(x,y)
    #raise NotImplementedError

def inspection():
    print('start inspection')
    mod0 = Modular([
        VisitedMapModule(0)
    ])
    print("Running modular robot 0")
    mod0.run()


# If the python node is executed as main process (sourced directly)
if __name__ == '__main__':

    # Initializes a rospy node to let the SimpleActionClient publish and subscribe
    rospy.init_node('assignment_2')

    exec_mode = sys.argv[1] 
    print('exec_mode:' + exec_mode)        

    if exec_mode == 'cleaning':        
        agent_id = int(sys.argv[2])
        vacuum_cleaning(agent_id)
        print('agent id:' + sys.argv[2])        
    elif exec_mode == 'inspection':
        inspection()
    else:
        print("Code not found")
        raise NotImplementedError