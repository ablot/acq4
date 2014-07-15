# -*- coding: utf-8 -*-
from __future__ import with_statement
from PatchMultiTemplate import *
from devicePageWidget import DevicePageWidget
from PyQt4 import QtGui, QtCore
#from PPatchWindow.pyyQt4 import Qwt5 as Qwt
from acq4.pyqtgraph import WidgetGroup
from acq4.pyqtgraph import PlotWidget
from acq4.util.metaarray import *
from acq4.util.Mutex import Mutex, MutexLocker
import traceback, sys, time
from numpy import *
import scipy.optimize
from acq4.util.debug import *
from acq4.pyqtgraph import siFormat
import acq4.Manager as Manager
import acq4.util.ptime as ptime
#from acq4.LogWindow import LogButton
from acq4.util.StatusBar import StatusBar

COLORS = [(255,217,47),
          (141,160,203),
          (252,141,98),
          (166,216,84),
          (229,196,148),
          (179,179,179),
          (231,138,195),
          (102,194,165)]

class PatchWindow(QtGui.QMainWindow):
    
    sigWindowClosed = QtCore.Signal(object)
    
    def __init__(self, dm, name, devNames, modes):
        QtGui.QMainWindow.__init__(self)
        self.setWindowTitle(name)
        self.startTime = None
        self.redrawCommand = 1
        
        self.analysisItems = {
            'inputResistance': u'Ω', 
            'accessResistance': u'Ω',
            'capacitance': 'F',
            'restingPotential': 'V', 
            'restingCurrent': 'A', 
            'fitError': ''
        }
        
        self.params = modes.pop('default')
        self.modes = modes
        self.stylesheet = {}
        self.devPages = dict()
        self.paramLock = Mutex(QtCore.QMutex.Recursive)
        self.storageFile = {}
        
        self.manager = dm
        self.name= name.replace(' ', '_')
        # self.clampName = clampName
        
        self.thread = PatchThread(self)
        self.cw = QtGui.QWidget()
        self.setCentralWidget(self.cw)
        self.ui = Ui_Form()
        self.ui.setupUi(self.cw)
        self.setStatusBar(StatusBar())
        
        self.devNames = sorted(devNames)
        self.colors = {}
        for i, dev in enumerate(sorted(devNames)):
            self.addDevicePage(dev)
            self.colors[dev] = QtGui.QColor(*COLORS[i%8])
                    
        # Create one button for each configured mode
        row = None
        self.modeRows = []
        rowLen = 0
        def mkModeCallback(name):
            return lambda: self.setMode(name)
        
        for modeName, mode in modes.items():
            if modeName == 'default':
                continue
            if row is None:
                row = QtGui.QWidget()
                layout = QtGui.QHBoxLayout()
                row.setLayout(layout)
                self.ui.modeLayout.addWidget(row)
                self.modeRows.append(row)
            btn = QtGui.QPushButton(modeName)
            layout.addWidget(btn)
            rowLen += btn.sizeHint().width()
            if rowLen > 200:
                row = None
                rowLen = 0
            btn.clicked.connect(mkModeCallback(modeName))
            
        self.stateFile = os.path.join('modules', self.name + '_ui.cfg')
        uiState = Manager.getManager().readConfigFile(self.stateFile)
        if 'geometry' in uiState:
            geom = QtCore.QRect(*uiState['geometry'])
            self.setGeometry(geom)
        if 'window' in uiState:
            ws = QtCore.QByteArray.fromPercentEncoding(uiState['window'])
            self.restoreState(ws)
            
        self.ui.splitter_2.setSizes([self.width()/5., self.width()*4./5.]) 
        self.ui.splitter_3.setSizes([self.width()/5., self.width()*4./5.]) 
        self.ui.splitter_4.setSizes([self.width()*3./4, self.width()/4.]) 
        
        
        self.plots = {}
        for k in self.analysisItems:
            p = PlotWidget()
            p.setLabel('left', text=k, units=self.analysisItems[k])
            self.ui.plotLayout.addWidget(p)
            self.plots[k] = p
        self.ui.cycleTimeSpin.setOpts(dec=True, step=1, minStep=1e-6, bounds=[0,None], siPrefix=True, suffix='s')
        self.ui.pulseTimeSpin.setOpts(dec=True, step=1, minStep=1e-6, bounds=[0,1.], siPrefix=True, suffix='s')
        self.ui.delayTimeSpin.setOpts(dec=True, step=1, minStep=1e-6, bounds=[0,1.], siPrefix=True, suffix='s')
        
        
        self.stateGroup = WidgetGroup([
        #     (self.ui.icPulseSpin, 'icPulse'),
        #     (self.ui.vcPulseSpin, 'vcPulse'),
        #     (self.ui.icHoldSpin, 'icHolding'),
        #     (self.ui.vcHoldSpin, 'vcHolding'),
        #     (self.ui.icPulseCheck, 'icPulseEnabled'),
        #     (self.ui.vcPulseCheck, 'vcPulseEnabled'),
        #     (self.ui.icHoldCheck, 'icHoldingEnabled'),
        #     (self.ui.vcHoldCheck, 'vcHoldingEnabled'),
             (self.ui.cycleTimeSpin, 'cycleTime'),
             (self.ui.pulseTimeSpin, 'pulseTime'),
             (self.ui.delayTimeSpin, 'delayTime'),
             (self.ui.drawFitCheck, 'drawFit'),
             (self.ui.averageSpin, 'average'),
         ])
        self.stateGroup.setState(self.params)
            
        
        self.ui.patchPlot.setLabel('left', text='Primary', units='A')
        self.patchCurve = {}
        self.patchFitCurve = {}
        self.commandCurve = {}
        pen = QtGui.QPen(QtGui.QColor(0,100,200))
        # pen.setWidth(1)
        pen.setStyle(QtCore.Qt.DotLine)
        for dev in devNames:
            self.patchCurve[dev] = self.ui.patchPlot.plot(pen=QtGui.QPen(QtGui.QColor(200,200,200)))
            self.patchFitCurve[dev] = self.ui.patchPlot.plot(pen=pen)
            self.commandCurve[dev] = self.ui.commandPlot.plot(pen=QtGui.QPen(QtGui.QColor(200,200,200)))
        self.ui.commandPlot.setLabel('left', text='Command', units='V')

        
        self.ui.startBtn.clicked.connect(self.startClicked)
        self.ui.recordBtn.clicked.connect(self.recordClicked)
        self.ui.resetBtn.clicked.connect(self.resetClicked)
        self.thread.finished.connect(self.threadStopped)
        self.thread.sigNewFrame.connect(self.handleNewFrame)
        self.stateGroup.sigChanged.connect(self.updateParams)
                
        ## Configure analysis plots, curves, and data arrays
        self.analysisCurves = dict([(dev, {}) for dev in self.devNames])
        self.analysisData = dict([(dev, {'time': []}) for dev in self.devNames])
        for n in self.analysisItems:
            w = getattr(self.ui, n+'Check')
            w.clicked.connect(self.showPlots)
            p = self.plots[n]
            for devN in devNames:
                self.analysisCurves[devN][n] = p.plot(pen=QtGui.QPen(QtGui.QColor(200,200,200)))
                for suf in ['', 'Std']:
                    self.analysisData[devN][n+suf] = []
                
        for devN in self.devNames:
            # print 'here: %s voila'%str(self.colors[i].getRgb()[:-1])
            self.changeDisplay(devN, 'all', self.colors[devN].getRgb()[:-1])
            self.devPages[devN]['stateGroup'].setState(self.params)
            
            
        self.showPlots()
        self.updateParams()
        self.show()
    
    def quit(self):
        #print "Stopping patch thread.."
        geom = self.geometry()
        uiState = {'window': str(self.saveState().toPercentEncoding()), 'geometry': [geom.x(), geom.y(), geom.width(), geom.height()]}
        Manager.getManager().writeConfigFile(uiState, self.stateFile)
        
        self.thread.stop(block=True)
        #print "Patch thread exited; module quitting."
        
    def closeEvent(self, ev):
        self.quit()
        self.sigWindowClosed.emit(self)
    
    def showPlots(self):
        """Show/hide analysis plot widgets"""
        for n in self.analysisItems:
            w = getattr(self.ui, n+'Check')
            p = self.plots[n]
            if w.isChecked():
                p.show()
            else:
                p.hide()
        self.updateAnalysisPlots()
        
    def setMode(self, modeName):
        """Activate a mode as defined in the configuration"""
        mode = self.modes[modeName].copy()
        clampMode = mode.pop('mode', None)
            
        state = self.stateGroup.state()
        state.update(mode)
        self.stateGroup.setState(state)
        for dev, actif in self.activeDevices().items():
            if not actif:
                continue
            state = self.devPages[dev]['stateGroup'].state()
            state.update(mode)
            self.devPages[dev]['stateGroup'].setState(state)
            if clampMode == 'ic':
                self.devPages[dev]['ui'].ui.icModeRadio.setChecked(True)
            elif clampMode == 'vc':
                self.devPages[dev]['ui'].ui.vcModeRadio.setChecked(True)
    
    def activeDevices(self):
        return dict([(devName, not devPage['ui'].ui.offModeRadio.isChecked()) for devName, devPage in self.devPages.items()])
    
    def updateParams(self, *args):
        with self.paramLock:
            state = self.stateGroup.state()
            for p in self.params:
                if p in state:
                    self.params[p] = state[p]
            self.params['recordTime'] = self.params['delayTime'] *2.0 + self.params['pulseTime']
            devParams = {}
            for devN, actif in self.activeDevices().items():
                if not actif:
                    self.patchCurve[devN].hide()
                    self.commandCurve[devN].hide()
                    self.patchFitCurve[devN].hide()
                    for n in self.analysisItems:
                        self.analysisCurves[devN][n].hide()
                    continue
                self.patchCurve[devN].show()
                self.commandCurve[devN].show()
                for n in self.analysisItems:
                    self.analysisCurves[devN][n].show()
                ui =self.devPages[devN]['ui'].ui
                if ui.icModeRadio.isChecked():
                    mode = 'ic'
                else:
                    mode = 'vc'
                devParams[devN] =dict(mode = mode)
                state = self.devPages[devN]['stateGroup'].state()
                for p in self.params:
                    if p in state:
                        devParams[devN][p] = state[p]
            self.params['devParams'] = devParams
        self.thread.updateParams()
        self.redrawCommand = 2   ## may need to redraw twice to make sure the update has gone through
        
    def recordClicked(self):
        if self.ui.recordBtn.isChecked():
            data = self.makeAnalysisArray()
            for devN, d in data.items():
                if d.shape[0] == 0:  ## no data yet; don't start the file
                    self.storageFile = {}
                    return
                self.newFile(devN, d)
        else:
            self.storageFile = {}
            
    def newFile(self, deviceName, data):
        sd = self.storageDir()
        self.storageFile[deviceName] =  sd.writeFile(data, self.name + '_' + deviceName, autoIncrement=True, appendAxis='Time', newFile=True)
        if self.startTime is not None:
            self.storageFile[deviceName].setInfo({'startTime': self.startTime})
                
    def storageDir(self):
        return self.manager.getCurrentDir().getDir('Patch', create=True)
                
    #def storageFile(self):
        #sd = self.storageDir()
        #return sd.getFile(self.clampName, create=True)
            
        
    def resetClicked(self):
        print 'RESET'
        self.ui.recordBtn.setChecked(False)
        self.recordClicked()
        for d in self.devNames:
            for n in self.analysisData[d]:
                self.analysisData[d][n] = []
        self.startTime = None
        
    def handleNewFrame(self, frame):
        prof = Profiler('PatchWindow.handleNewFrame', disabled=True)
        with self.paramLock:
            mode = self.params['mode']
        # clampName = str(self.ui.device_comboBox.currentText())
        
        if mode == 'vc':
            self.ui.patchPlot.setLabel('left', units='A')
        else:
            self.ui.patchPlot.setLabel('left', units='V')
        prof.mark('1')
        validDev = [devN for devN, act in self.activeDevices().items() if act]
        for devN in validDev:
            data = frame['data'][devN]
            self.patchCurve[devN].setData(data.xvals('Time'), data['primary'])
            
            prof.mark('2')
            if self.redrawCommand > 0:
                self.redrawCommand -= 1
                #print "set command curve"
                self.commandCurve[devN].setData(data.xvals('Time'), data['command'])
                if mode == 'vc':
                    self.ui.commandPlot.setLabel('left', units='V')
                else:
                    self.ui.commandPlot.setLabel('left', units='A')
            prof.mark('3')
            #self.ui.patchPlot.replot()
            #self.ui.commandPlot.replot()
            # for i in range(len(self.devNames)):
            if frame['analysis'][devN]['fitTrace'] is not None:
                self.patchFitCurve[devN].show()
                self.patchFitCurve[devN].setData(data.xvals('Time'), frame['analysis'][devN]['fitTrace'])
            else:
                self.patchFitCurve[devN].hide()
            prof.mark('4')
            
            for k in self.analysisItems:
                if k in frame['analysis'][devN]:
                    self.analysisData[devN][k].append(frame['analysis'][devN][k])
        prof.mark('5')
                
        for r in ['input', 'access']:
            res = r+'Resistance'
            label = getattr(self.ui, res+'Label')
            text = []
            for devN in validDev:
                resistance = frame['analysis'][devN][res]
                text.append('<font color="%s">'%(self.colors[devN].name())+siFormat(resistance) + u'Ω</font>')
            label.setText(' | '.join(text))
            
            
        prof.mark('6')
        textRP = []
        textRC = []
        textC = []
        textfERR = []
        for devN in validDev:
            v= siFormat(frame['analysis'][devN]['restingPotential'], error=frame['analysis'][devN]['restingPotentialStd'], suffix='V')
            textRP.append('<font color="%s">'%(self.colors[devN].name())+ v +'</font>')
            v = siFormat(frame['analysis'][devN]['restingCurrent'], error=frame['analysis'][devN]['restingCurrentStd'], suffix='A')
            textRC.append('<font color="%s">'%(self.colors[devN].name())+ v +'</font>')
            v = '%sF' % siFormat(frame['analysis'][devN]['capacitance'])
            textC.append('<font color="%s">'%(self.colors[devN].name())+ v +'</font>')
            v = '%7.2g' % frame['analysis'][devN]['fitError']
            textfERR.append('<font color="%s">'%(self.colors[devN].name())+ v +'</font>')
        self.ui.restingPotentialLabel.setText(' | '.join(textRP))
        self.ui.restingCurrentLabel.setText(' | '.join(textRC))
        self.ui.capacitanceLabel.setText(' | '.join(textC))
        self.ui.fitErrorLabel.setText(' | '.join(textfERR))
        prof.mark('7')
        
        start = data._info[-1]['DAQ']['command']['startTime']
        if self.startTime is None:
            self.startTime = start
            if self.ui.recordBtn.isChecked(): 
                for devN in validDev:
                    if devN in self.storageFile:
                        self.storageFile[devN].setInfo({'startTime': self.startTime})
        for devN in validDev:
            self.analysisData[devN]['time'].append(start - self.startTime)
        prof.mark('8')
        self.updateAnalysisPlots()
        prof.mark('9')
        
        ## Record to disk if requested.
        if self.ui.recordBtn.isChecked():
            arr = self.makeAnalysisArray(lastOnly=True)
            #print "appending array", arr.shape
            for devN in validDev:
                if devN not in self.storageFile:
                    self.newFile(devN, arr[devN])
            else:
                arr[devN].write(self.storageFile[devN].name(), appendAxis='Time')
        prof.mark('10')
        prof.finish()
        
    def makeAnalysisArray(self, lastOnly=False):
        ## Determine how much of the data to include in this array
        if lastOnly:
            sl = slice(-1, None)
        else:
            sl = slice(None)
        validDev = [devN for devN, act in self.activeDevices().items() if act]
        data ={}
        for devN in validDev:
            ## Generate the meta-info structure
            info = [
                {'name': 'Time', 'values': self.analysisData[devN]['time'][sl], 'units': 's'},
                {'name': 'Value', 'cols': []}
            ]
            for k in self.analysisItems:
                for s in ['', 'Std']:
                    if len(self.analysisData[devN][k+s]) < 1:
                        continue
                    info[1]['cols'].append({'name': k+s, 'units': self.analysisItems[k]})
                
            ## Create the blank MetaArray
            data[devN] = MetaArray(
                (len(info[0]['values']), len(info[1]['cols'])), 
                dtype=float,
                info=info
            )
            ## Fill with data
            for k in self.analysisItems:
                for s in ['', 'Std']:
                    if len(self.analysisData[devN][k+s]) < 1:
                        continue
                    try:
                        data[devN][:, k+s] = self.analysisData[devN][k+s][sl]
                    except:
                        print data[devN].shape, data[devN][:, k+s].shape, len(self.analysisData[devN][k+s][sl])
                        raise
                
        return data
            
            
        
    def updateAnalysisPlots(self):
        for n in self.analysisItems:
            p = self.plots[n]
            if p.isVisible():
                for devN in self.analysisData:
                    self.analysisCurves[devN][n].setData(self.analysisData[devN]['time'], self.analysisData[devN][n])
                #if len(self.analysisData[n+'Std']) > 0:
                    #self.analysisCurves[p+'Std'].setData(self.analysisData['time'], self.analysisData[n+'Std'])
                #p.replot()
    
    def startClicked(self):
        if self.ui.startBtn.isChecked():
            if not self.thread.isRunning():
                self.thread.start()
                Manager.logMsg("Patch module started.")
            self.ui.startBtn.setText('Stop')
            for devN, p in self.devPages.items():
                p['ui'].ui.offModeRadio.setEnabled(False)

        else:
            self.ui.startBtn.setEnabled(False)
            self.thread.stop()
            Manager.logMsg("Patch module stopped.")
            
    def threadStopped(self):
        self.ui.startBtn.setText('Start')
        self.ui.startBtn.setEnabled(True)
        self.ui.startBtn.setChecked(False)
        for devN, p in self.devPages.items():
            p['ui'].ui.offModeRadio.setEnabled(True)

    def changeDisplay(self, devN, param, val):
        if devN in self.devNames:
            found = 0
            if param in ['traceCurveColor', 'all', 'traces']:
                self.patchCurve[devN].setPen(QtGui.QPen(QtGui.QColor(*val)))
                found = 1
            if param in ['commandCurveColor', 'all', 'traces']:
                self.commandCurve[devN].setPen(QtGui.QPen(QtGui.QColor(*val)))
                found = 1
            if param in ['fitCurveColor', 'all', 'traces']:
                pen = QtGui.QPen(QtGui.QColor(*val))
                pen.setStyle(QtCore.Qt.DotLine)
                self.patchFitCurve[devN].setPen(pen)
                found = 1
            if param in ['analysisCurveColor', 'all', 'traces']:
                for curve in self.analysisCurves[devN].values():
                    curve.setPen(QtGui.QPen(QtGui.QColor(*val)))
                found = 1
            if param in ['textColor', 'all']:
                # self.stylesheet['QWidget#%s_page{color: rgb%s}'%(dev,val)] = None
                # stl = ';\n'.join(self.stylesheet.keys())
                self.colors[devN] = QtGui.QColor(*val)
                self.devPages[devN]['w'].setStyleSheet('color: rgb'+str(val))
                found = 1
            if not found:
                print 'Unknown display param: %s for dev %s'%(param, dev)
                return
        # if param == 'traceCurveColor':
        #     self.patchCurve.setPen(QtGui.QPen(QtGui.QColor(*val)))
        # elif param == 'commandCurveColor':
        #     self.commandCurve.setPen(QtGui.QPen(QtGui.QColor(*val)))
        # elif param == 'fitCurveColor':
        #     self.patchFitCurve.setPen(QtGui.QPen(QtGui.QColor(*val)))
        # elif param == 'analysisCurveColor':
        #     for curve in self.analysisCurves.values():
        #         curve.setPen(QtGui.QPen(QtGui.QColor(*val)))
        elif param == 'textColor':
            self.stylesheet['color:rgb'+str(val)] = None
            stl = ';\n'.join(self.stylesheet.keys())
            self.cw.setStyleSheet(stl)
        elif param == 'backgroundColor':
            self.stylesheet['background-color:rgb'+str(val)] = None
            stl = ';\n'.join(self.stylesheet.keys())
            self.cw.setStyleSheet(stl)
        else:
            print 'Unknown display param: %s'%param
            return
            
        
    def addDevicePage(self, devName):
        if devName in self.devPages.keys():
            raise ValueError('%s is already in the tabBox'%devName)
        
        # Create page
        self.devPages[devName] = dict(w = QtGui.QWidget())
        self.devPages[devName]['ui'] = DevicePageWidget(self.devPages[devName]['w'])
        self.ui.tabWidget.addTab(self.devPages[devName]['w'], devName)
        self.devPages[devName]['ui'].setObjectName('%s_page'%devName)
                
        ui = self.devPages[devName]['ui'].ui
        # Connect it
        ui.vcModeRadio.toggled.connect(self.updateParams)
        ui.offModeRadio.toggled.connect(self.updateParams)
        ui.icModeRadio.toggled.connect(self.updateParams)
        ui.icPulseSpin.setOpts(dec=True, step=1, minStep=1e-12, bounds=[None,None], siPrefix=True, suffix='A')
        ui.vcPulseSpin.setOpts(dec=True, step=1, minStep=1e-3, bounds=[None,None], siPrefix=True, suffix='V')
        ui.icHoldSpin.setOpts(dec=True, step=1, minStep=1e-12, bounds=[None,None], siPrefix=True, suffix='A')
        ui.vcHoldSpin.setOpts(dec=True, step=1, minStep=1e-3, bounds=[None,None], siPrefix=True, suffix='V')
        grp = WidgetGroup([
            (ui.icPulseSpin, 'icPulse'),
            (ui.vcPulseSpin, 'vcPulse'),
            (ui.icHoldSpin, 'icHolding'),
            (ui.vcHoldSpin, 'vcHolding'),
            (ui.icPulseCheck, 'icPulseEnabled'),
            (ui.vcPulseCheck, 'vcPulseEnabled'),
            (ui.icHoldCheck, 'icHoldingEnabled'),
            (ui.vcHoldCheck, 'vcHoldingEnabled'),
    ])
        grp.sigChanged.connect(self.updateParams)
        self.devPages[devName]['stateGroup'] = grp



        
class PatchThread(QtCore.QThread):
    
    sigNewFrame = QtCore.Signal(object)
    
    def __init__(self, ui):
        self.ui = ui
        self.manager = ui.manager
        QtCore.QThread.__init__(self)
        self.lock = Mutex(QtCore.QMutex.Recursive)
        self.stopThread = True
        self.paramsUpdated = True
    
    def updateParams(self):
        with self.lock:
            self.paramsUpdated = True
    
    def run(self):
        """Main loop for patch thread. This is where protocols are executed and data collected."""
        try:
            with MutexLocker(self.lock) as l:
                self.stopThread = False
                # daqNames = clamp.listChannels().values()[0]['device']  ## Just guess the DAQ by checking one of the clamp's channels
                # clampName = self.ui.clampName
                self.paramsUpdated = True
                l.unlock()
                
                lastTime = None
                while True:
                    ## copy in parameters from GUI
                    updateCommand = False
                    l.relock()
                    if self.paramsUpdated:
                        with self.ui.paramLock:
                            params = self.ui.params.copy()
                            self.paramsUpdated = False
                            clampNames = [devN for devN, act in self.ui.activeDevices().items() if act]
                            if not len(clampNames):
                                print 'No device selected, stopping module'
                                break
                            clamps = [self.manager.getDevice(cl) for cl in clampNames]
                            daqNames = {}
                            for cl, devN in zip(clamps, clampNames):
                                for ch in cl.listChannels().values():
                                    daqNames[ch['device']] = None
                        updateCommand = True
                    l.unlock()
                    
                    ## run protocol and analysis
                    try:
                        self.runOnce(params, l, clamps, daqNames, clampNames)
                    except:
                        printExc("Error running/analyzing patch protocol")
                    
                    
                    
                    lastTime = ptime.time()-params['recordTime'] ## This is not a proper 'cycle time', but instead enforces a minimum interval between cycles (but this can be very important for performance)
                    
                    ## sleep until it is time for the next run
                    c = 0
                    stop = False
                    while True:
                        ## check for stop button every 100ms
                        if c % 10 == 0:
                            l.relock()
                            if self.stopThread:
                                l.unlock()
                                stop = True
                                break
                            l.unlock()
                        now = ptime.time()
                        if now >= (lastTime+params['cycleTime']):
                            break
                        
                        time.sleep(10e-3) ## Wake up every 10ms
                        c += 1
                    if stop:
                        break
        except:
            printExc("Error in patch acquisition thread, exiting.")
        #self.emit(QtCore.SIGNAL('threadStopped'))
        
    def runOnce(self, params, l, clamps, daqNames, clampNames):
        prof = Profiler('PatchThread.run', disabled=True)
        #lastTime = time.clock()   ## moved to after the command run
        
        
        ## Regenerate command signal if parameters have changed
        numPts = int(float(params['recordTime']) * params['rate'])
        start = int(params['delayTime'] * params['rate'])
        stop = start + int(params['pulseTime'] * params['rate'])
        cmdData = empty((len(clamps), numPts))
        holding = empty(len(clamps))
        for i, devN in enumerate(clampNames):
            p = params['devParams'][devN]
            mode = p['mode']
            if p[mode+'HoldingEnabled']:
                holding[i] = p[mode+'Holding']
            else:
                holding[i] = 0.
            if p[mode+'PulseEnabled']:
                amplitude = p[mode+'Pulse']
            else:
                amplitude = 0.
            cmdData[i,:] = holding[i]
            cmdData[i,start:stop] = holding[i] + amplitude
        #cmdData[-1] = holding
            
        cmd = {'protocol': {'duration': params['recordTime'], 'leadTime': 0.02}}
        for d in daqNames:
            cmd[d] = {'rate': params['rate'], 'numPts': numPts, 'downsample': params['downsample']}
        for i, (devN, cl) in enumerate(zip(clampNames, clamps)):
            cmd[devN] = {'mode': params['devParams'][devN]['mode'],
                        'command': cmdData[i],
                        'holding': holding[i]
                    }
          
        
        prof.mark('build command')
        
        ## Create and execute task.
        ## the try/except block is just to catch errors that come up during multiclamp auto pipette offset procedure.
        results = []
        for i in range(params['average']):
            exc = False
            count = 0
            while not exc:
                count += 1
                try:
                    ## Create task
                    task = self.manager.createTask(cmd)
                    ## Execute task
                    task.execute()
                    exc = True
                except:
                    err = sys.exc_info()[1].args
                    #print err
                    if count < 5 and len(err) > 1 and err[1] == 'ExtCmdSensOff':  ## external cmd sensitivity is off, wait to see if it comes back..
                        time.sleep(1.0)
                        continue
                    else:
                        raise
            #print cmd
            
            ## analyze trace 
            result = task.getResult()
            #print result
            results.append(result)
            
        prof.mark('execute')
            
        ## average together results if we collected more than 1
        if len(results) == 1:
            result = results[0]
            avg = [result[devN] for devN in clampNames]
        else:
            avg = [concatenate([res[devN].view(ndarray)[newaxis, ...] for res in results], axis=0).mean(axis=0) for devN in clampNames]
            avg = [MetaArray(av, info=results[0][devN].infoCopy()) for av, devN in zip(avg, clampNames)]
            result = results[0]
            for av, devN in zip(avg, clampNames):
                result[devN] = av
        #print result[clampName]['primary'].max(), result[clampName]['primary'].min()
        
        #print result[clampName]
        try:
            analysis = {}
            for i, devN in enumerate(clampNames):
                p =params.copy()
                p.update(params['devParams'][devN])
                analysis[devN] = self.analyze(avg[i], p)
            frame = {'data': result, 'analysis': analysis}
            prof.mark('analyze')
            
            #self.emit(QtCore.SIGNAL('newFrame'), frame)
            self.sigNewFrame.emit(frame)
        except:
            printExc('Error in patch analysis:')
        finally:
            prof.finish()
            
    def analyze(self, data, params):
        #print "\n\nAnalysis parameters:", params
        ## Extract specific time segments
        nudge = 50e-6
        base = data['Time': 0.0:(params['delayTime']-nudge)]
        pulse = data['Time': params['delayTime']+nudge:params['delayTime']+params['pulseTime']-nudge]
        pulseEnd = data['Time': params['delayTime']+(params['pulseTime']*2./3.):params['delayTime']+params['pulseTime']-nudge]
        end = data['Time':params['delayTime']+params['pulseTime']+nudge:]
        #print "time ranges:", pulse.xvals('Time').min(),pulse.xvals('Time').max(),end.xvals('Time').min(),end.xvals('Time').max()
        ## Exponential fit
        #  v[0] is offset to start of exp
        #  v[1] is amplitude of exp
        #  v[2] is tau
        def expFn(v, t):
            return (v[0]-v[1]) + v[1] * exp(-t / v[2])
        # predictions
        ar = 10e6
        ir = 200e6
        if params['mode'] == 'vc':
            ari = params['vcPulse'] / ar
            iri = params['vcPulse'] / ir
            pred1 = [ari, ari-iri, 1e-3]
            pred2 = [iri-ari, iri-ari, 1e-3]
        else:
            #clamp = self.manager.getDevice(self.clampName)
            try:
                bridge = data._info[-1]['ClampState']['ClampParams']['BridgeBalResist']
                bridgeOn = data._info[-1]['ClampState']['ClampParams']['BridgeBalEnabled']
                #bridge = float(clamp.getParam('BridgeBalResist'))  ## pull this from the data instead.
                #bridgeOn = clamp.getParam('BridgeBalEnable')
                if not bridgeOn:
                    bridge = 0.0
            except:
                bridge = 0.0
            #print "bridge:", bridge
            arv = params['icPulse'] * ar - bridge
            irv = params['icPulse'] * ir
            pred1 = [arv, -irv, 10e-3]
            pred2 = [irv, irv, 50e-3]
            
        # Fit exponential to pulse and post-pulse traces
        tVals1 = pulse.xvals('Time')-params['delayTime']
        #tVals2 = end.xvals('Time')-end.xvals('Time').min()
        
        baseMean = base['primary'].mean()
        fit1 = scipy.optimize.leastsq(
            lambda v, t, y: y - expFn(v, t), pred1, 
            args=(tVals1, pulse['primary'].view(np.ndarray) - baseMean),
            maxfev=200, full_output=1)
        
        ## fit again using shorter data
        ## this should help to avoid fitting against h-currents
        tau4 = fit1[0][2]*10
        t0 = pulse.xvals('Time')[0]
        shortPulse = pulse['Time': t0:t0+tau4]
        if shortPulse.shape[0] > 10:  ## but only if we can get enough samples from this
            tVals2 = shortPulse.xvals('Time')-params['delayTime']
            fit1 = scipy.optimize.leastsq(
                lambda v, t, y: y - expFn(v, t), pred1, 
                args=(tVals2, shortPulse['primary'].view(np.ndarray) - baseMean),
                maxfev=200, full_output=1)
        
        
        #fit2 = scipy.optimize.leastsq(
            #lambda v, t, y: y - expFn(v, t), pred2, 
            #args=(tVals2, end['primary'] - baseMean),
            #maxfev=200, full_output=1, warning=False)
            
        
        #err = max(abs(fit1[2]['fvec']).sum(), abs(fit2[2]['fvec']).sum())
        err = abs(fit1[2]['fvec']).sum()
        
        
        # Average fit1 with fit2 (needs massaging since fits have different starting points)
        #print fit1
        fit1 = fit1[0]
        #fit2 = fit2[0]
        #fitAvg = [   ## Let's just not do this.
            #0.5 * (fit1[0] - (fit2[0] - (fit1[0] - fit1[1]))),
            #0.5 * (fit1[1] - fit2[1]),
            #0.5 * (fit1[2] + fit2[2])            
        #]
        fitAvg = fit1

        (fitOffset, fitAmp, fitTau) = fit1
        #print fit1
        
        fitTrace = empty(len(data))
        
        ## Handle analysis differently depenting on clamp mode
        if params['mode'] == 'vc':
            #global iBase, iPulse, iPulseEnd
            iBase = base['Channel': 'primary'].asarray()
            iPulse = pulse['Channel': 'primary']
            iPulseEnd = pulseEnd['Channel': 'primary'] 
            vBase = base['Channel': 'command'].asarray()
            vPulse = pulse['Channel': 'command'] 
            vStep = vPulse.mean() - vBase.mean()
            sign = [-1, 1][vStep > 0]

            iBaseMean = iBase.mean()
            iPulseEndMean = iPulseEnd.asarray().mean()
            iStep = sign * max(1e-15, sign * (iPulseEndMean - iBaseMean))
            iRes = vStep / iStep
            
            ## From Santos-Sacchi 1993
            
            ## 1. compute charge transfered during the charging phase 
            pTimes = pulse.xvals('Time')
            iCapEnd = pTimes[-1]
            iCap = iPulse['Time':pTimes[0]:iCapEnd] - iPulseEndMean
            #self.iCap1 = iCap
            ## Instead, we will use the fit to guess how much charge transfer there would have been 
            ## if the charging curve had gone all the way back to the beginning of the pulse
            iCap = expFn((fit1[1],fit1[1],fit1[2]), np.linspace(0, iCapEnd-pTimes[0], iCap.shape[0]))
            #self.iCap2 = iCap
            Q = sum(iCap) * (iCapEnd - pTimes[0]) / iCap.shape[0]
            
            
            Rin = iRes
            Vc = vStep
            Rs_denom = (Q * Rin + fitTau * Vc)
            if Rs_denom != 0.0:
                Rs = (Rin * fitTau * Vc) / Rs_denom
                Rm = Rin - Rs
                Cm = (Rin**2 * Q) / (Rm**2 * Vc)
            else:
                Rs = 0
                Rm = 0
                Cm = 0
            aRes = Rs
            cap = Cm
            
        if params['mode'] == 'ic':
            iBase = base['Channel': 'command'].asarray()
            iPulse = pulse['Channel': 'command'] 
            vBase = base['Channel': 'primary'].asarray()
            vPulse = pulse['Channel': 'primary'] 
            vPulseEnd = pulseEnd['Channel': 'primary'] 
            iStep = iPulse.mean() - iBase.mean()
            
            if iStep >= 0:
                vStep = max(1e-5, -fitAmp)
            else:
                vStep = min(-1e-5, -fitAmp)
            #sign = [-1, 1][iStep >= 0]
            #vStep = sign * max(1e-5, sign * (vPulseEnd.mean() - vBase.mean()))
            #vStep = sign * max(1e-5, sign * fitAmp)
            if iStep == 0:
                iStep = 1e-14
            iRes = (vStep / iStep)
            #print iRes, vStep, iStep, bridge
            #print "current step:", iStep
            #print "bridge:", bridge
            aRes = (fitOffset / iStep) + bridge
            #iRes = (-fitAvg[1] / iStep) + bridge
            cap = fitTau / iRes
            
            
        rmp = vBase.mean()
        rmps = vBase.std()
        rmc = iBase.mean()
        rmcs = iBase.std()
        #print rmp, rmc
        
        ## Compute values for fit trace to be plotted over raw data
        if params['drawFit']:
            fitTrace = MetaArray((data.shape[1],), info=[{'name': 'Time', 'values': data.xvals('Time')}])
            if params['mode'] == 'vc':
                fitTrace[:] = rmc
            else:
                fitTrace[:] = rmp
    #        print i1, i2, len(tVals1), len(tVals2), len(expFn(fit1, tVals2)), len(fitTrace[i2:])
            ## slices from fitTrace must exactly match slices from data at the beginning of the function.
            fitTrace['Time': params['delayTime']+nudge:params['delayTime']+params['pulseTime']-nudge] = expFn(fit1, tVals1)+baseMean
            #fitTrace['Time':params['delayTime']+params['pulseTime']+nudge:] = expFn(fit2, tVals2)+baseMean
        else:
            fitTrace = None
        
        
        
            
        return {
            'inputResistance': iRes, 
            'accessResistance': aRes,
            'capacitance': cap,
            'restingPotential': rmp, 'restingPotentialStd': rmps,
            'restingCurrent': rmc, 'restingCurrentStd': rmcs,
            'fitError': err,
            'fitTrace': fitTrace
        }
            
    def stop(self, block=False):
        with self.lock:
            self.stopThread = True
        if block:
            if not self.wait(10000):
                raise Exception("Timed out while waiting for patch thread exit!")
                

