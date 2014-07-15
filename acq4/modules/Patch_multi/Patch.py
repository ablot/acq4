# -*- coding: utf-8 -*-
from acq4.modules.Module import *
from PatchWindow import *
import os
from PyQt4 import QtGui
from collections import OrderedDict


class Patch_multi(Module):

    defaults = {
        'mode': 'vc',
        'rate': 400000,
        'downsample': 10,
        'cycleTime': .2,
        'recordTime': 0.1,
        'delayTime': 0.01,
        'pulseTime': 0.01,
        'icPulse': -30e-12,
        'vcPulse': -10e-3,
        'icHolding': 0,
        'vcHolding': -60e-3,
        'icHoldingEnabled': False,
        'icPulseEnabled': True,
        'vcHoldingEnabled': False,
        'vcPulseEnabled': True,
        'drawFit': True,
        'average': 1,
        }

    defaultModes = OrderedDict([
        ('Bath', dict(
            mode='vc',
            vcPulseEnabled=True, 
            vcHoldingEnabled=False,
            cycleTime=0.2,
            pulseTime=10e-3,
            delayTime=10e-3,
            average=1
            )),
        ('Patch', dict(
            mode='vc',
            vcPulseEnabled=True, 
            vcHoldingEnabled=True,
            cycleTime=0.2,
            pulseTime=10e-3,
            delayTime=10e-3,
            average=1
            )),
        ('Cell', dict(
            mode='ic',
            vcPulseEnabled=True, 
            cycleTime=250e-3,
            pulseTime=150e-3,
            delayTime=30e-3,
            average=1
            )),
        ('Monitor', dict(
            cycleTime=40.,
            average=5
            )),
        ])
    
    defaultDisplay = {'patchCurveColor': (200, 200, 200),
                      'commandCurveColor': (200, 200, 200),
                      'analysisCurveColor': (200, 200, 200),
                      'fitCurveColor': (0, 100, 200)}
    
    def monitorMode(self):
        self.ui.cycleTimeSpin.setValue(40)
        self.ui.averageSpin.setValue(5)
    
    def __init__(self, manager, name, config):
        Module.__init__(self, manager, name, config)
        self.config = config
        
        # Read mode configurations from config file
        modes = config.get('modes', self.defaultModes)
        self.defaults.update(modes.get('default', {}))
        modes['default'] = self.defaults
        for modeName, mode in modes.items():
            for param, val in mode.items():
                if param not in self.defaults:
                    print 'Ignoring unknown parameter in config file: "%s".' % param
                    continue
                typ = type(self.defaults[param])
                if not isinstance(val, typ):
                    print "Value for parameter '%s' should have type %s; ignoring." % (param, typ)
                    continue
                    
        self.devNames = manager.listInterfaces('clamp')
        self.ui = PatchWindow(manager, name, self.devNames, modes)
        
        display = config.get('display', self.defaultDisplay)
        for param, val in display.items():
            if isinstance(val, OrderedDict):
                dev = param
                param, val = val.items()[0]
            else:
                dev = None
            self.ui.changeDisplay(dev, param, val)
             
        self.ui.sigWindowClosed.connect(self.quit)
        mp = os.path.dirname(__file__)
        self.ui.setWindowIcon(QtGui.QIcon(os.path.join(mp, 'icon.png')))
    
    def window(self):
        return self.ui

    def quit(self):
        self.ui.quit()
        Module.quit(self)