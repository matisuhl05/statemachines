from transitions.extensions import HierarchicalMachine
from transitions.extensions import MachineFactory as factory
from transitions import Machine


class HexapodSM():
    def __init__(self):
        #Init state change conditions
        self.retract = False
        self.prep = False
        #self.push = False
        self.hexapodState = "retract"

        #Sends pressure output to controls
        self.currentPressure = 0
        self.commandedPressure = 0

        ## Hexapod States ##
        ## push           ##
        ## prepretract   ##
        ## retract        ##
        ## preppush      ##

        #When on all the pressures get sent to 0 and state sends deflate
        self.launchStandFlag = True

        #Possible states
        self.states = ['Abort',{'name':'Idle',  'children':
                                    {'name':'Manual',
                                    'children' : ['prepretract', 'retract', 'preppush','push']
                                    }
                                },
                               {'name': 'Nominal', 'children': 
                                    {'name':'Autonomous',
                                        'children' : ['prepretract', 'retract', 'preppush','push']
                                    }
                                }
                      ]
        #What the hexapod should be for current state 
        self.stateToHexapod =  {'prepretract' :{'retract':0,},
                                 'preppush'    :{'retract':1}, 
                                 'push'       :{'retract':0},                    
                                 'retract'    :{'retract':1 }}
        self.transitions = [
            {'trigger': 'to_abort', 'source': '*', 'dest': 'Abort'},
            {'trigger': 'to_idle', 'source': 'Abort', 'dest': 'Idle'},
            {'trigger': 'to_nominal', 'source': 'Idle', 'dest': 'Nominal'},
    
            #Nominal to Manual and Autonomous
            {'trigger': 'to_manual', 'source': 'Idle', 'dest': 'Idle_Manual_prepretract'},
            {'trigger': 'to_autonomous', 'source': 'Nominal', 'dest': 'Nominal_Autonomous_prepretract','after': 'stateHexapodOutput'},

            {'trigger': 'to_retract', 'source': 'Nominal_Autonomous_prepretract', 'dest': 'Nominal_Autonomous_retract','after': 'stateHexapodOutput'},
            {'trigger': 'to_preppush', 'source': 'Nominal_Autonomous_retract', 'dest': 'Nominal_Autonomous_preppush', 'conditions': 'is_preppush_conditions_met','after': 'stateHexapodOutput'},
            {'trigger': 'to_push', 'source': 'Nominal_Autonomous_preppush', 'dest': 'Nominal_Autonomous_push', 'conditions': 'is_push_conditions_met','after': 'stateHexapodOutput'},
            {'trigger': 'to_prepretract', 'source': 'Nominal_Autonomous_push', 'dest': 'Nominal_Autonomous_prepretract', 'conditions': 'is_prepretract_conditions_met','after': 'stateHexapodOutput'},

            #Manual Switching
            {'trigger': 'to_manual_retract', 'source': '*', 'dest': 'Idle_Manual_retract','conditions': 'is_idle_state','after': 'stateHexapodOutput'},
            {'trigger': 'to_manual_preppush', 'source': '*', 'dest': 'Idle_Manual_preppush', 'conditions': 'is_idle_state','after': 'stateHexapodOutput'},
            {'trigger': 'to_manual_push', 'source': '*', 'dest': 'Idle_Manual_push', 'conditions': 'is_idle_state','after': 'stateHexapodOutput'},
            {'trigger': 'to_manual_prepretract', 'source': '*', 'dest': 'Idle_Manual_prepretract', 'conditions': 'is_idle_state','after': 'stateHexapodOutput'},
        ]
         
        self.machine = HierarchicalMachine(model=self, states=self.states, transitions=self.transitions, initial='Abort', ignore_invalid_triggers=True)
        self.GraphHSM = factory.get_predefined(nested=True, graph=True)
        self.machine = self.GraphHSM(model=self, states=self.states, transitions=self.transitions, initial='Abort', ignore_invalid_triggers=True)

    def generate_graph(self, filename='hexapod.png'):
        graph = self.machine.get_graph()
        graph.draw(filename, prog='dot')

    def is_retract_conditions_met(self):
        #return self.retract == 0 and self.push == 1 and self.hexapodState == "prepretract"
        #return self.retract == 0 and self.hexapodState == "prepretract"
        return self.retract == 1 and self.prep == 1;

    def is_preppush_conditions_met(self):
        #return self.retract == 1 and self.push == 0 and self.hexapodState == "retract"
        #return self.retract == 1 and self.hexapodState == "retract"
        return self.retract == 1 and self.prep == 0;

    def is_push_conditions_met(self):
        #return self.retract == 0 and self.push == 1 and self.hexapodState == "preppush"
        #return self.retract == 1 and self.hexapodState == "preppush"
        return self.retract == 0 and self.prep == 1;
    
    def is_prepretract_conditions_met(self):
        #return self.retract == 0 and self.push == 1 and self.hexapodState == "push"
        #return self.retract == 0 and self.hexapodState == "push"
        return self.retract == 0 and self.prep == 0;
    
    def processHexapodStates(self, retract, prep):
        #Updates bexapod states
        self.retract = retract
        self.prep = prep

        #Checks to see if hexapod is already in correct states
        if self.state == 'Nominal_Autonomous_retract' and self.is_retract_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_preppush' and self.is_preppush_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_push' and self.is_push_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_prepretract' and self.is_prepretract_conditions_met():
            return 'Already in Correct State'
        
        #Checks to see if hexapod state need to be changed
        elif self.state == 'Nominal_Autonomous_retract' and self.is_preppush_conditions_met():
            self.to_preppush()

        elif self.state == 'Nominal_Autonomous_preppush' and self.is_push_conditions_met():
            self.to_push()

        elif self.state == 'Nominal_Autonomous_push' and self.is_prepretract_conditions_met():
            self.to_prepretract()

        elif self.state == 'Nominal_Autonomous_prepretract' and self.is_retract_conditions_met():
            self.to_retract()
        else:
            return 'Hexapod State ERROR!'
                   
        return "Moved States Successfully!"
    
        
    def stateHexapodOutput(self):
        if 'Manual' in self.state or 'Autonomous' in self.state:
            for key in self.stateToHexapod:
                if self.state.endswith(key):
                    return self.stateToHexapod[key]
        else:
            return "Cannot return hexapod output because not in Manual or Autonomous\n"
        
prop = HexapodSM()
prop.generate_graph()



assert prop.state == "Abort", f"Expected state: Abort, but got: {prop.state}"
assert prop.stateHexapodOutput() == "Cannot return hexapod output because not in Manual or Autonomous\n", f"Unexpected output for state: {prop.state}"

prop.to_idle()
assert prop.state == "Idle", f"Expected state: Abort, but got: {prop.state}"
assert prop.stateHexapodOutput() == "Cannot return hexapod output because not in Manual or Autonomous\n", f"Unexpected output for state: {prop.state}"

prop.to_nominal()
assert prop.state == "Nominal", f"Expected state: Abort, but got: {prop.state}"
assert prop.stateHexapodOutput() == "Cannot return hexapod output because not in Manual or Autonomous\n", f"Unexpected hexapod output for state: {prop.state}"

#Testing Autonomous Mode
prop.to_autonomous()

#retract
assert prop.state == "Nominal_Autonomous_prepretract", f"Expected state: Nominal_Autonomous_prepretract, but got: {prop.state}"

result = prop.processHexapodStates(True, True)
assert prop.state == "Nominal_Autonomous_retract", f"Expected state: Nominal_Autonomous_retract, but got: {prop.state}"

#preppush
result = prop.processHexapodStates(True, False)
assert prop.state == "Nominal_Autonomous_preppush", f"Expected state: Nominal_Autonomous_preppush, but got: {prop.state}"

#push
result = prop.processHexapodStates(False, True)
assert prop.state == "Nominal_Autonomous_push", f"Expected state: Nominal_Autonomous_push, but got: {prop.state}"
#assert result == "Moved States Successfully!", f"Unexpected result: {result}"

#prepretract
result = prop.processHexapodStates(False, False)
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

prop.generate_graph()
