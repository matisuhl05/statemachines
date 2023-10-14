from transitions.extensions import HierarchicalMachine
from transitions.extensions import MachineFactory as factory
from enum import Enum

class Pressure(Enum):
        INFLATED = True
        DEFLATED = False

class TunnelSupportgripSM():
    def __init__(self):
        #Init sensor readings
        self.FrontGripper = False
        self.BackGripper = False
        self.hexapodState = "" 

        #Sends pressure output to controls
        self.currentPressure = 0
        self.commandedPressure = 0

        ## Hexapod States ##
        ## push           ##
        ## prep retract   ##
        ## retract        ##
        ## prep push      ##

        #When on all the pressures get sent to 0 and state sends deflate
        self.launchStandFlag = True

        #Possible states
        self.states = ['Abort',{'name':'Idle',  'children':
                                    {'name':'Manual',
                                    'children' : ['BackHold', 'PrepFrontHold', 'FrontHold','PrepBackHold', 'AllDeflate']
                                    }
                                },
                               {'name': 'Nominal', 'children': 
                                    {'name':'Autonomous',
                                        'children' : ['BackHold', 'PrepFrontHold', 'FrontHold','PrepBackHold']
                                    }
                                }
                      ]
        #What actuator outputs should be for current state 
        self.stateToGripper = { 'BackHold'      :{'gripperFront':0, 'gripperBack':1}, 
                                'PrepFrontHold'     :{'gripperFront':1, 'gripperBack':1}, 
                                'FrontHold'    :{'gripperFront':1, 'gripperBack':0}, 
                                'PrepBackHold'     :{'gripperFront':1, 'gripperBack':1}, 
                                'AllDeflate':{'gripperFront':0, 'gripperBack':0}
                                }

            # (two proj pressures, inflate or deflate, two current pressures)
            # add on launch stand state
            # check if sevice is blocking
        
        self.transitions = [
            #Abort,Idel, Nominal
            {'trigger': 'to_abort', 'source': '*', 'dest': 'Abort'},
            {'trigger': 'to_idle', 'source': 'Abort', 'dest': 'Idle'},
            {'trigger': 'to_nominal', 'source': 'Idle', 'dest': 'Nominal'},
    
            #Nominal to Manual and Autonomous
            {'trigger': 'to_manual', 'source': 'Idle', 'dest': 'Idle_Manual_BackHold'},
            {'trigger': 'to_autonomous', 'source': 'Nominal', 'dest': 'Nominal_Autonomous_BackHold','after': 'stateActuatorOutput'},

            #Autonomous BackHold, PrepFrontHold, FrontHold, PrepBackHold
            {'trigger': 'to_BackHold', 'source': ['Nominal_Autonomous_PrepBackHold','Nominal',], 'dest': 'Nominal_Autonomous_BackHold','after': 'stateActuatorOutput'},
            {'trigger': 'to_PrepFrontHold', 'source': 'Nominal_Autonomous_BackHold', 'dest': 'Nominal_Autonomous_PrepFrontHold', 'conditions': 'is_PrepFrontHold_conditions_met','after': 'stateActuatorOutput'},
            {'trigger': 'to_FrontHold', 'source': 'Nominal_Autonomous_PrepFrontHold', 'dest': 'Nominal_Autonomous_FrontHold', 'conditions': 'is_FrontHold_conditions_met','after': 'stateActuatorOutput'},
            {'trigger': 'to_PrepBackHold', 'source': 'Nominal_Autonomous_FrontHold', 'dest': 'Nominal_Autonomous_PrepBackHold', 'conditions': 'is_PrepBackHold_conditions_met','after': 'stateActuatorOutput'},

            #Manual Switching
            {'trigger': 'to_manual_BackHold', 'source': '*', 'dest': 'Idle_Manual_BackHold', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_PrepFrontHold', 'source': '*', 'dest': 'Idle_Manual_PrepFrontHold', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_FrontHold', 'source': '*', 'dest': 'Idle_Manual_FrontHold', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_PrepBackHold', 'source': '*', 'dest': 'Idle_Manual_PrepBackHold', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_AllDeflate', 'source': '*', 'dest': 'Idle_Manual_AllDeflate', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
        ]

        #Init the graph drawing of the state machine
        self.machine = HierarchicalMachine(model=self, states=self.states, transitions=self.transitions, initial='Abort', ignore_invalid_triggers=True)
        self.GraphHSM = factory.get_predefined(nested=True, graph=True)
        self.machine = self.GraphHSM(model=self, states=self.states, transitions=self.transitions, initial='Abort', ignore_invalid_triggers=True)

    def generate_graph(self, filename='state_machine.png'):
        graph = self.machine.get_graph()
        graph.draw(filename, prog='dot')

    def is_idle_state(self):
        return self.state.startswith('Idle')

    def is_PrepFrontHold_conditions_met(self):
        return self.FrontGripper == Pressure.DEFLATED and self.BackGripper == Pressure.INFLATED and self.hexapodState == "prep retract"

    def is_FrontHold_conditions_met(self):
        return self.FrontGripper == Pressure.INFLATED and self.BackGripper == Pressure.INFLATED and  self.hexapodState == "prep retract"

    def is_PrepBackHold_conditions_met(self):
        return self.FrontGripper == Pressure.INFLATED and self.BackGripper == Pressure.DEFLATED and self.hexapodState=="prep push"
    
    def is_BackHold_conditions_met(self):
        return self.FrontGripper == Pressure.INFLATED and self.BackGripper == Pressure.INFLATED and self.hexapodState == "prep push"

    def processSensors(self, FrontGripper, BackGripper,hexapod):
        #Updates sensor inpugrip
        self.hexapodState = hexapod
        self.FrontGripper = FrontGripper
        self.BackGripper = BackGripper

        #Checks to see if already in correct state
        if self.state == 'Nominal_Autonomous_BackHold' and self.is_BackHold_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_PrepFrontHold' and self.is_PrepFrontHold_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_FrontHold' and self.is_FrontHold_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_PrepBackHold' and self.is_PrepBackHold_conditions_met():
            return 'Already in Correct State'
        
        #Checks to see if state needs to be changed
        elif self.state == 'Nominal_Autonomous_PrepBackHold' and self.is_BackHold_conditions_met():
            self.to_BackHold()
            #SEND ROS COMMAND TO GO TO RETRACT FOR HEXAPOD WHEN THIS TIRE CHANGE IS DONE

        elif self.state == 'Nominal_Autonomous_BackHold' and self.is_PrepFrontHold_conditions_met():
            self.to_PrepFrontHold()

        elif self.state == 'Nominal_Autonomous_PrepFrontHold' and self.is_FrontHold_conditions_met():
            self.to_FrontHold()
            #SEND ROS COMMAND TO GO TO RETRACT FOR HEXAPOD WHEN THIS TIRE CHANGE IS DONE

        elif self.state == 'Nominal_Autonomous_FrontHold' and self.is_PrepBackHold_conditions_met():
            self.to_PrepBackHold()
        #Edge case 
        else:
            return 'GRIPPER SENSORS ERROR'
                   
        return "Moved States Successfully!"

    def stateActuatorOutput(self):
        if 'Manual' in self.state or 'Autonomous' in self.state:
            for key in self.stateToGripper:
                if self.state.endswith(key):
                    return self.stateToGripper[key]
        else:
            return "Cannot return actuator output because not in Manual or Autonomous\n"


grip = TunnelSupportgripSM()
grip.generate_graph()
assert grip.state == "Abort", f"Expected state: Abort, but got: {grip.state}"
assert grip.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected output for state: {grip.state}"

grip.to_idle()
assert grip.state == "Idle", f"Expected state: Abort, but got: {grip.state}"
assert grip.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected output for state: {grip.state}"

grip.to_nominal()
assert grip.state == "Nominal", f"Expected state: Abort, but got: {grip.state}"
assert grip.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected actuator output for state: {grip.state}"

#Testing Autonomous Mode
grip.to_autonomous()

#PrepFrontHold
assert grip.state == "Nominal_Autonomous_BackHold", f"Expected state: Nominal_Autonomous_BackHold, but got: {grip.state}"

result = grip.processSensors(Pressure.DEFLATED, Pressure.INFLATED, 'prep retract')
assert grip.state == "Nominal_Autonomous_PrepFrontHold", f"Expected state: Nominal_Autonomous_PrepFrontHold, but got: {grip.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = grip.processSensors(False, True, 'prep push')
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = grip.processSensors(False, False, 'prep push')
assert result == "gripULSION ERROR!", f"Unexpected result: {result}"

#FrontHold
result = grip.processSensors(True, True, 'prep retract')
assert grip.state == "Nominal_Autonomous_FrontHold", f"Expected state: Nominal_Autonomous_FrontHold, but got: {grip.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = grip.processSensors(True, True, 'prep retract')
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = grip.processSensors(False, False, 'prep retract')
assert result == "gripULSION ERROR!", f"Unexpected result: {result}"

#PrepBackHold
result = grip.processSensors(True, False, 'prep push')
assert grip.state == "Nominal_Autonomous_PrepBackHold", f"Expected state: Nominal_Autonomous_PrepBackHold, but got: {grip.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = grip.processSensors(True, False, 'prep push')
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = grip.processSensors(False, True, 'prep push')
assert grip.state == "Nominal_Autonomous_PrepBackHold", f"Expected state: Nominal_Autonomous_PrepBackHold, but got: {grip.state}"
assert result == "gripULSION ERROR!", f"Unexpected result: {result}"

#BackHold
result = grip.processSensors(True, True, 'prep push')
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = grip.processSensors(True, True, 'prep push')
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = grip.processSensors(True, False, 'prep push')
assert result == "gripULSION ERROR!", f"Unexpected result: {result}"

grip.generate_graph()