from transitions.extensions import HierarchicalMachine
from transitions.extensions import MachineFactory as factory

class HexapodSM:
    def __init__(self):
        self.sensor = False
        self.states = ['Abort', 'Idle', {'name': 'Nominal', 'children': 
                            [
                                {'name':'Manual',
                                    'children' : ['Extend','Retract']}, 

                                {'name':'Autonomous',
                                    'children' : ['Extend','Retract']}, 
                            ]
                        }
                    ]
        
        self.stateToActuator = {'Extend'  :{'actuator':1}, 
                                'Retract' :{'actuator':0}}
        
        self.transitions = [
            # Abort, Idle, Nominal
            {'trigger': 'to_abort', 'source': '*', 'dest': 'Abort'},
            {'trigger': 'to_idle', 'source': 'Abort', 'dest': 'Idle'},
            {'trigger': 'to_nominal', 'source': 'Idle', 'dest': 'Nominal'},

            # Nominal to Manual and Autonomous
            {'trigger': 'to_manual', 'source': 'Nominal', 'dest': 'Nominal_Manual_Retract'},
            {'trigger': 'to_autonomous', 'source': 'Nominal', 'dest': 'Nominal_Autonomous_Retract', 'after': 'stateActuatorOutput'},

            # Autonomous Load, Hold1, Unload, Hold2
            {'trigger': 'to_Retract', 'source': 'Nominal_Autonomous_Extend', 'dest': 'Nominal_Autonomous_Retract', 'conditions': 'is_retract_conditions_met', 'after': 'stateActuatorOutput'},
            {'trigger': 'to_Extend', 'source': 'Nominal_Autonomous_Retract', 'dest': 'Nominal_Autonomous_Extend', 'conditions': 'is_extend_conditions_met', 'after': 'stateActuatorOutput'},

            # Manual Switching
            {'trigger': 'to_manual_Retract', 'source': '*', 'dest': 'Nominal_Manual_Retract', 'conditions': 'is_nominal_state', 'after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_Extend', 'source': '*', 'dest': 'Nominal_Manual_Extend', 'conditions': 'is_nominal_state', 'after': 'stateActuatorOutput'},

            # Transitions Manual to Autonomous state
            {'trigger': 'to_autonomous_from_manual_Retract', 'source': 'Nominal_Manual_Retract', 'dest': 'Nominal_Autonomous_Retract', 'after': 'stateActuatorOutput'},
            {'trigger': 'to_autonomous_from_manual_Extend', 'source': 'Nominal_Manual_Extend', 'dest': 'Nominal_Autonomous_Extend', 'after': 'stateActuatorOutput'},
            
            # Transitions Autonomous to Manual state
            {'trigger': 'to_manual_from_autonomous_Retract', 'source': 'Nominal_Autonomous_Retract', 'dest': 'Nominal_Manual_Retract', 'after': 'stateActuatorOutput'},
            {'trigger': 'to_manual_from_autonomous_Extend', 'source': 'Nominal_Autonomous_Extend', 'dest': 'Nominal_Manual_Extend', 'after': 'stateActuatorOutput'},
        ]

        self.currentSensorReadings = {'Nominal_Autonomous_Retract': {'sensor':0,},
                                      'Nominal_Autonomous_Extend': {'sensor':1}
                                     }

        self.machine = HierarchicalMachine(model=self, states=self.states, transitions=self.transitions, initial='Abort', ignore_invalid_triggers=True)
        self.GraphHSM = factory.get_predefined(nested=True, graph=True)

    def is_extend_conditions_met(self):
        return self.sensor == 1

    def is_retract_conditions_met(self):
        return self.sensor == 0

    def is_nominal_state(self):
        return self.state.startswith('Nominal_Autonomous') or self.state.startswith('Nominal_Manual')

    def generate_graph(self, filename='hexapod_state_machine.png'):
        graph = self.machine.get_graph()
        graph.draw(filename, prog='dot')

    def processSensors(self, sensor):
        self.sensor = sensor

        # Check if current sensor readings match the expected readings for the current state
        expected_sensor_readings = self.currentSensorReadings.get(self.state)
        if expected_sensor_readings:
            if (expected_sensor_readings['sensor'] == self.sensor):
                return 'Already in Correct State'

        # Handle state transitions based on current state and sensor readings
        return self.handleStateTransition()

    def handleStateTransition(self):
        if (self.state == 'Nominal_Autonomous_Extend' or self.state == 'Nominal') and self.is_retract_conditions_met():
            self.to_Retract()
        elif self.state == 'Nominal_Autonomous_Retract' and self.is_extend_conditions_met():
            self.to_Extend()

        else:
            return ("||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n"
                    "|||                    TUNNEL SUPPORT SENSORS ERROR!                   |||\n"
                    "|||  The Sensors are in an Undefined State Possible TS Sensor failing  |||\n"
                    "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||\n")
        
        return "Moved States Successfully!\n"

    def stateActuatorOutput(self):
        if self.state.startswith('Nominal_Manual') or self.state.startswith('Nominal_Autonomous'):
            for key in self.stateToActuator:
                if self.state.endswith(key):
                    print(self.state)
                    print(self.stateToActuator[key],'\n')
                    return self.stateToActuator[key]
        else:
            return "Cannot return actuator output because not in Manual or Autonomous\n"

ts = HexapodSM()
ts.generate_graph()
