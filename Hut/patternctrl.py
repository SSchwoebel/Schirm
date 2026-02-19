@Pyro4.expose
class PatternCtrl(object):
    def __init__(self, pattern_list):
        self.pattern_list = pattern_list
        self.CurrentPatternNr = 0
        self.CurrentPattern = self.pattern_list[self.CurrentPatternNr]

    @property
    def Nr(self):             # exposed as 'proxy.attr' remote attribute
        return self.CurrentPatternNr
    @Nr.setter
    def Nr(self, value):      # exposed as 'proxy.attr' writable
        self.CurrentPatternNr = value
        self.CurrentPattern = self.pattern_list[self.CurrentPatternNr]

    @property
    def Parameters(self):
        return self.CurrentPattern.parameters
    @Parameters.setter
    def Parameters(self, value):
        self.CurrentPattern.parameters = value
    
    @property
    def ParameterDescriptions(self):
        return self.CurrentPattern.parameterdescriptions