class SwitchRapidLanding():

    def __init__(self, transition_z):
        self.transition_z = transition_z

    def check_for_switch(self, data):
        assert len(data) == 1, 'ERROR: data passed into check_for_switch is incorrect size'
        