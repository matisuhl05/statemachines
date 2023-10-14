from transitions.extensions import HierarchicalMachine
from transitions.extensions import MachineFactory as factory

class TunnelSupportSM():
    def __init__(self):
        #Init sensor readings
        self.loadSensorT = False
        self.loadSensorB = False
        self.endSensor = False

        #Possible states
        self.states = ['Abort',{'name':'Idle',  'children':
                                    {'name':'Manual',
                                    'children' : ['Load', 'Hold1', 'Unload', 'Hold2', 'AllRetract']
                                    }
                                }, 
                               {'name': 'Nominal', 'children': 
                                    {'name':'Autonomous',
                                        'children' : ['Load', 'Hold1', 'Unload', 'Hold2']
                                    }
                                }
                    ]
        #What actuator outputs should be for current state 
        self.stateToActuator = {'Load'      :{'actuatorL':0, 'actuatorR':1}, 
                                'Hold1'     :{'actuatorL':1, 'actuatorR':1}, 
                                'Unload'    :{'actuatorL':1, 'actuatorR':0}, 
                                'Hold2'     :{'actuatorL':1, 'actuatorR':1}, 
                                'AllRetract':{'actuatorL':0, 'actuatorR':0}}

        self.transitions = [
            #Abort,Idel, Nominal
            {'trigger': 'to_abort', 'source': '*', 'dest': 'Abort'},
            {'trigger': 'to_idle', 'source': 'Abort', 'dest': 'Idle'},
            {'trigger': 'to_nominal', 'source': 'Idle', 'dest': 'Nominal'},
    
            #Nominal to Manual and Autonomous
            {'trigger': 'to_manual', 'source': 'Idle', 'dest': 'Idle_Manual_Load'},
            {'trigger': 'to_autonomous', 'source': 'Nominal', 'dest': 'Nominal_Autonomous_Load','after': 'stateActuatorOutput'},

            #Autonomous Load, Hold1, Unload, Hold2
            {'trigger': 'to_load', 'source': ['Nominal_Autonomous_Hold2','Nominal',], 'dest': 'Nominal_Autonomous_Load','after': 'stateActuatorOutput'},
            {'trigger': 'to_hold1', 'source': 'Nominal_Autonomous_Load', 'dest': 'Nominal_Autonomous_Hold1', 'conditions': 'is_hold1_conditions_met','after': 'stateActuatorOutput'},
            {'trigger': 'to_unload', 'source': 'Nominal_Autonomous_Hold1', 'dest': 'Nominal_Autonomous_Unload', 'conditions': 'is_unload_conditions_met','after': 'stateActuatorOutput'},
            {'trigger': 'to_hold2', 'source': 'Nominal_Autonomous_Unload', 'dest': 'Nominal_Autonomous_Hold2', 'conditions': 'is_hold2_conditions_met','after': 'stateActuatorOutput'},

            #Manual Switching
            {'trigger': 'to_manual_Load', 'source': '*', 'dest': 'Idle_Manual_Load', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_Hold1', 'source': '*', 'dest': 'Idle_Manual_Hold1', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_Unload', 'source': '*', 'dest': 'Idle_Manual_Unload', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_Hold2', 'source': '*', 'dest': 'Idle_Manual_Hold2', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_AllRetract', 'source': '*', 'dest': 'Idle_Manual_AllRetract', 'conditions': 'is_idle_state','after': 'stateActuatorOutput'},
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

    def is_hold1_conditions_met(self):
        return self.loadSensorT == 1 and self.loadSensorB == 1 and self.endSensor == 0

    def is_unload_conditions_met(self):
        return self.loadSensorT == 1 and self.loadSensorB == 1 and self.endSensor == 1

    def is_hold2_conditions_met(self):
        return self.loadSensorT == 0 and self.loadSensorB == 0 and self.endSensor == 0

    def processSensors(self, loadSensorT, loadSensorB, endSensor):
        #Updates sensor inputs
        self.loadSensorT = loadSensorT
        self.loadSensorB = loadSensorB
        self.endSensor = endSensor

        #Checks to see if already in correct state
        if self.state == 'Nominal_Autonomous_Load' and self.is_hold2_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_Hold1' and self.is_hold1_conditions_met():
            return 'Already in Correct State'
        
        elif self.state == 'Nominal_Autonomous_Unload' and (self.is_hold1_conditions_met() or self.is_unload_conditions_met()):
            return 'Already in Correct State'
        
        #Checks to see if state needs to be changed
        elif (self.state == 'Nominal_Autonomous_Hold2'):
            self.to_load()

        elif self.state == 'Nominal_Autonomous_Load' and self.is_hold1_conditions_met():
            self.to_hold1()

        elif self.state == 'Nominal_Autonomous_Hold1' and self.is_unload_conditions_met():
            self.to_unload()

        elif self.state == 'Nominal_Autonomous_Unload' and self.is_hold2_conditions_met():
            self.to_hold2()
        #Edge case for when sensors fuck up
        else:
            return 'TUNNEL SUPPORT SENSORS ERROR!'
                   
        return "Moved States Successfully!"

    def stateActuatorOutput(self):
        if 'Manual' in self.state or 'Autonomous' in self.state:
            for key in self.stateToActuator:
                if self.state.endswith(key):
                    return self.stateToActuator[key]
        else:
            return "Cannot return actuator output because not in Manual or Autonomous\n"

ts = TunnelSupportSM()
ts.generate_graph()

assert ts.state == "Abort", f"Expected state: Abort, but got: {ts.state}"
assert ts.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected output for state: {ts.state}"

ts.to_idle()
assert ts.state == "Idle", f"Expected state: Abort, but got: {ts.state}"
assert ts.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected output for state: {ts.state}"

ts.to_nominal()
assert ts.state == "Nominal", f"Expected state: Abort, but got: {ts.state}"
assert ts.stateActuatorOutput() == "Cannot return actuator output because not in Manual or Autonomous\n", f"Unexpected actuator output for state: {ts.state}"

#Testing Autonomous Mode
ts.to_autonomous()

#Hold1
assert ts.state == "Nominal_Autonomous_Load", f"Expected state: Nominal_Autonomous_Load, but got: {ts.state}"

result = ts.processSensors(True, True, False)
assert ts.state == "Nominal_Autonomous_Hold1", f"Expected state: Nominal_Autonomous_Hold1, but got: {ts.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = ts.processSensors(True, True, False)
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = ts.processSensors(False, True, False)
assert result == "TUNNEL SUPPORT SENSORS ERROR!", f"Unexpected result: {result}"

#Unload
result = ts.processSensors(True, True, True)
assert ts.state == "Nominal_Autonomous_Unload", f"Expected state: Nominal_Autonomous_Unload, but got: {ts.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = ts.processSensors(True, True, True)
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = ts.processSensors(False, True, False)
assert result == "TUNNEL SUPPORT SENSORS ERROR!", f"Unexpected result: {result}"

#Hold2
result = ts.processSensors(False, False, False)
assert ts.state == "Nominal_Autonomous_Hold2", f"Expected state: Nominal_Autonomous_Hold2, but got: {ts.state}"
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

#Load
result = ts.processSensors(False, False, False)
assert result == "Moved States Successfully!", f"Unexpected result: {result}"

result = ts.processSensors(False, False, False)
assert result == "Already in Correct State", f"Unexpected result: {result}"

result = ts.processSensors(False, True, False)
assert result == "TUNNEL SUPPORT SENSORS ERROR!", f"Unexpected result: {result}"

ts.generate_graph()