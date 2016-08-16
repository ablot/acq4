# -*- coding: utf-8 -*-
from TaskTemplate import Ui_Form
from acq4.devices.Device import TaskGui
from PyQt4 import QtCore, QtGui
from acq4.Manager import getManager, logMsg, logExc
import random
import numpy as np
from acq4.util.debug import Profiler
import optimize ## for determining random scan patterns
import os, sys
import acq4.pyqtgraph as pg
from acq4.util.HelpfulException import HelpfulException
import acq4.pyqtgraph.parametertree.parameterTypes as pTypes
from acq4.pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType


### Error IDs:
###  1: Could not find spot size from calibration. (from ScannerTaskGui.pointSize)


class PositionCtrlGroup(pTypes.GroupParameter):
    sigAddNewRequested = QtCore.Signal(object, object)
    def __init__(self):
        opts = {
            'name': 'Position Controls',
            'type': 'group',
            'addText': "Add Control..",
            'addList': ['Point', 'Grid', 'Occlusion'],

        }
        pTypes.GroupParameter.__init__(self, **opts)
    
    def addNew(self, typ):
        self.sigAddNewRequested.emit(self, typ)

class ProgramCtrlGroup(pTypes.GroupParameter):
    sigAddNewRequested = QtCore.Signal(object, object)
    def __init__(self):
        opts = {
            'name': 'Program Controls',
            'type': 'group',
            'addText': "Add Control..",
            'addList': ['lineScan', 'multipleLineScan', 'rectangleScan'],
            'autoIncrementName': True,
        }
        pTypes.GroupParameter.__init__(self, **opts)
    
    def addNew(self, typ):
        self.sigAddNewRequested.emit(self, typ)


class ScannerTaskGui(TaskGui):
    
    #sigSequenceChanged = QtCore.Signal(object)  ## inherited from Device
    
    def __init__(self, dev, taskRunner):
        TaskGui.__init__(self, dev, taskRunner)
        self.ui = Ui_Form()
        self.ui.setupUi(self)
        dm = getManager()
        self.targets = None
        self.items = {}
        self.haveCalibration = True   ## whether there is a calibration for the current combination of laser/optics
        self.currentOpticState = None
        self.currentCamMod = None
        self.programCtrls = []
        self.displaySize = {}  ## maps (laser,opticState) : display size
                               ## since this setting is remembered for each objective.
        
        ## Populate module/device lists, auto-select based on device defaults 
        self.defCam = None
        if 'defaultCamera' in self.dev.config:
            self.defCam = self.dev.config['defaultCamera']
        defLaser = None
        if 'defaultLaser' in self.dev.config:
            defLaser = self.dev.config['defaultLaser']

        self.ui.cameraCombo.setTypes(['cameraModule'])
        self.ui.laserCombo.setTypes(['laser'])
        
        self.positionCtrlGroup = PositionCtrlGroup()
        self.positionCtrlGroup.sigAddNewRequested.connect(self.addPositionCtrl)
        self.ui.itemTree.setParameters(self.positionCtrlGroup, showTop=False)
        self.positionCtrlGroup.sigChildRemoved.connect(self.positionCtrlRemoved)
        
        self.programCtrlGroup = ProgramCtrlGroup()
        self.programCtrlGroup.sigAddNewRequested.connect(self.addProgramCtrl)
        self.ui.programTree.setParameters(self.programCtrlGroup, showTop=False)
        self.programCtrlGroup.sigChildRemoved.connect(self.programCtrlRemoved)

        ## Set up SpinBoxes
        self.ui.minTimeSpin.setOpts(dec=True, step=1, minStep=1e-3, siPrefix=True, suffix='s', bounds=[0, 50])
        self.ui.minDistSpin.setOpts(dec=True, step=1, minStep=1e-6, siPrefix=True, suffix='m', bounds=[0, 10e-3])
        self.ui.sizeSpin.setOpts(dec=True, step=1, minStep=1e-6, siPrefix=True, suffix='m', bounds=[1e-9, 1e-3])
        ## Create state group for saving/restoring state
        self.stateGroup = pg.WidgetGroup([
            (self.ui.cameraCombo,),
            (self.ui.laserCombo,),
            (self.ui.minTimeSpin, 'minTime'),
            (self.ui.minDistSpin, 'minDist'),
            (self.ui.simulateShutterCheck, 'simulateShutter'),
            (self.ui.sizeSpin, 'spotSize'),
        ])
        self.stateGroup.setState({'minTime': 10, 'minDist': 500e-6, 'sizeSpin':100e-6})
        self.tdPlot = self.ui.tdPlotWidget.plotItem
        self.tdPlot.setLabel('bottom', text="Distance", units='m')
        self.tdPlot.setLabel('left', text="Wait time", units='s')

        ## Note we use lambda functions for all these clicks to strip out the arg sent with the signal
        
        self.ui.hideCheck.toggled.connect(self.showInterface)
        self.ui.hideMarkerBtn.clicked.connect(self.hideSpotMarker)
        self.ui.cameraCombo.currentIndexChanged.connect(self.camModChanged)
        self.ui.laserCombo.currentIndexChanged.connect(self.laserDevChanged)
        self.ui.sizeFromCalibrationRadio.toggled.connect(self.updateSpotSizes)
        self.ui.sizeSpin.valueChanged.connect(self.sizeSpinEdited)
        self.ui.minTimeSpin.valueChanged.connect(self.sequenceChanged)
        self.ui.minDistSpin.valueChanged.connect(self.sequenceChanged)
        self.ui.recomputeBtn.clicked.connect(self.recomputeClicked)
        self.ui.loadConfigBtn.clicked.connect(self.loadConfiguration)
        
        self.dev.sigGlobalSubdeviceChanged.connect(self.opticStateChanged)
        
        self.testTarget = TargetPoint(name="Test", ptSize=100e-6)
        self.testTarget.setPen(QtGui.QPen(QtGui.QColor(255, 200, 200)))
        self.spotMarker = TargetPoint(name="Last", ptSize=100e-6, movable=False)
        self.spotMarker.setPen(pg.mkPen(color=(255,255,255), width = 2))

        self.spotMarker.hide()
        self.updateSpotSizes()

        self.camModChanged()
        self.updateTDPlot()
        
            
        #self.ui.simulateShutterCheck.setChecked(False)
        if 'offVoltage' not in self.dev.config: ## we don't have a voltage for virtual shuttering
            self.ui.simulateShutterCheck.setChecked(False)
            self.ui.simulateShutterCheck.setEnabled(False)
            
    def setHaveCalibration(self, have):
        self.haveCalibration = have
        self.updateVisibility()
        
    def showInterface(self, b):
        self.updateVisibility()
        
    def updateVisibility(self):
        b = self.haveCalibration and not self.ui.hideCheck.isChecked()
        for k in self.items:
            self.items[k].setVisible(b)
        self.testTarget.setVisible(b)
        
    def camModChanged(self):
        camMod = self.cameraModule()
        if self.currentCamMod is not None:
            self.currentCamMod.ui.removeItem(self.testTarget)
            self.currentCamMod.ui.removeItem(self.spotMarker)
            
        self.currentCamMod = camMod
        if camMod is None:
            return
        self.currentCamMod = camMod
            
        camMod.ui.addItem(self.testTarget, None, [1,1], 1010)
        camMod.ui.addItem(self.spotMarker, None, [1,1], 1010)
        
        self.opticStateChanged()
        
    def getLaser(self):
        return self.ui.laserCombo.currentText()
    
    def opticStateChanged(self):
        opticState = self.dev.getDeviceStateKey()
        laser = self.getLaser()
        if self.currentOpticState != opticState:
            self.currentOpticState = opticState
            
            ## recall display size settings for this objective
            dispSize = self.displaySize.get((laser,opticState), None)
            if dispSize is None:
                self.ui.sizeFromCalibrationRadio.setChecked(True)
            else:
                self.ui.sizeSpin.setValue(dispSize)
            
            for i in self.items.values():
                active = (i.opticState == opticState)
                i.parameters().setValue(active)
                    

    def laserDevChanged(self):
        ## called when laser device combo is changed
        ## need to update spot size
        self.updateSpotSizes()
        
    def sizeSpinEdited(self):
        self.ui.sizeCustomRadio.setChecked(True)
        self.updateSpotSizes()
        
      
    def updateSpotSizes(self):
        try:
            size, display = self.pointSize()
            for i in self.items.values():
                i.setPointSize(display, size)
            self.testTarget.setPointSize(size)
            self.spotMarker.setPointSize(size)
            
            self.setHaveCalibration(True)
        except HelpfulException as exc:
            if exc.kwargs.get('errId', None) == 1:
                self.setHaveCalibration(False)
            else:
                raise

    def pointSize(self):
        ## returns (calibrated spot size, requested display size)
        try:
            camMod = self.cameraModule()
            if camMod is None:
                return (1,1)
            cam = camMod.config['camDev']
            laser = self.getLaser()
            cal = self.dev.getCalibration(laser)
            ss = cal['spot'][1]
            
            
        except:
            #logMsg("Could not find spot size from calibration.", msgType='error') ### This should turn into a HelpfulException.
            exc = sys.exc_info()
            raise HelpfulException("Could not find spot size from calibration. ", exc=exc, reasons=["Correct camera and/or laser device are not selected.", "There is no calibration file for selected camera and laser."], errId=1)
            
        if self.ui.sizeFromCalibrationRadio.isChecked():
            displaySize = ss
            ## reconnecting before this to get around reload errors, breaks the disconnect
            #try:
                #self.ui.sizeSpin.valueChanged.disconnect(self.sizeSpinEdited)
            #except TypeError:
                #logExc("A TypeError was caught in ScannerTaskGui.pointSize(). It was probably caused by a reload.", msgType='status', importance=0)
            self.stateGroup.setState({'spotSize':ss})
            #self.ui.sizeSpin.valueChanged.connect(self.sizeSpinEdited)
            self.displaySize[(laser, self.currentOpticState)] = None
        elif self.ui.sizeCustomRadio.isChecked():
            displaySize = self.ui.sizeSpin.value()
            self.displaySize[(laser, self.currentOpticState)] = displaySize
            
        return (ss, displaySize)
        #return (0.0001, packing)
        
    def cameraModule(self):
        mod = self.ui.cameraCombo.getSelectedObj()
        if mod is None:
            return None
        return mod
        
    def saveState(self, saveItems=False):
        state = self.stateGroup.state()
        if saveItems:
            state['items'] = [param.item.saveState() for param in self.positionCtrlGroup.children()]
        return state
        
    def restoreState(self, state):
        self.stateGroup.setState(state)
        if 'items' in state:
            for itemState in state['items']:
                typ = itemState['type']
                self.addItem(typ, itemState)
    
    def storeConfiguration(self):
        state = self.saveState(saveItems=True)
        fileName = os.path.join(self.dev.configDir(), 'lastConfig')
        self.dev.dm.writeConfigFile(state, fileName)

    def loadConfiguration(self):
        fileName = os.path.join(self.dev.configDir(), 'lastConfig')
        state = self.dev.dm.readConfigFile(fileName)
        self.restoreState(state)
        
    def listSequence(self):
        #items = self.activeItems()
        targets = self.getTargetList()
        if targets > 0:
            return {'targets': targets}
        else:
            return {}
        
    def generateTask(self, params=None):
        if self.cameraModule() is None:
            raise Exception('No camera module selected, can not build task.')
        
        if params is None or 'targets' not in params:
            target = self.testTarget.listPoints()[0]
            delay = 0
        else:
            if self.targets is None:
                self.generateTargets()
            #print "targets:", len(self.targets), params['targets']
            (target, delay) = self.targets[params['targets']]
            
        if len(self.programCtrls) == 0: # doing regular position mapping
            task = {
                'position': target, 
                'minWaitTime': delay,
                #'camera': self.cameraModule().config['camDev'], 
                'laser': self.ui.laserCombo.currentText(),
              #  'simulateShutter': self.ui.simulateShutterCheck.isChecked(), ## was commented out... 
                'duration': self.taskRunner.getParam('duration')
            }
        else: # doing programmed scans
            daqName = self.dev.getDaqName()
            task = {
               # 'position': target, 
                'minWaitTime': delay,
                #'camera': self.cameraModule().config['camDev'], 
                'laser': self.ui.laserCombo.currentText(),
                'simulateShutter': self.ui.simulateShutterCheck.isChecked(),
                'duration': self.taskRunner.getParam('duration'),
                'numPts': self.taskRunner.getDevice(daqName).currentState()['numPts'],
                'program': [],
                   #('step', 0.0, None),           ## start with step to "off" position 
                   #('step', 0.2, (1.3e-6, 4e-6)), ## step to the given location after 200ms
                   #('line', (0.2, 0.205), (1.3e-6, 4e-6))  ## 5ms sweep to the new position 
                   #('step', 0.205, None),           ## finish step to "off" position at 205ms
               #]
            }
            for ctrl in self.programCtrls:
                if ctrl.isActive():
                    task['program'].append(ctrl.generateTask())
        return task
    
    def hideSpotMarker(self):
        self.spotMarker.hide()
        
        
    def handleResult(self, result, params):
        if not self.spotMarker.isVisible():
            self.spotMarker.show()
        #print 'ScannerTaskGui.handleResult() result:', result
        if 'position' in result:
            pos = result['position']
            ss = result['spotSize']
            self.spotMarker.setPos((pos[0]-ss*0.5, pos[1]-ss*0.5))
        #print 'handleResult'
        getManager().scanResult=result
    
    def addPositionCtrl(self, param, typ):
        ## called when "Add Control.." combo is changed
        self.addItem(typ)

    def positionCtrlRemoved(self, param, ctrl):
        item = ctrl.item
        item.scene().removeItem(item)
        try:
            item.parameters().sigValueChanged.disconnect(self.itemActivationChanged)
        except TypeError:
            pass
        del self.items[item.name]
        #self.updateGridLinkCombos()
        self.itemChanged()
        
    def addProgramCtrl(self, param, itemType):
        ## called when "Add Control.." combo is changed
        cls = {'lineScan': ProgramLineScan, 
               'multipleLineScan': ProgramMultipleLineScan, 
               'rectangleScan': ProgramRectScan}[itemType]
        state = {}
        ctrl = cls(**state)
        self.programCtrlGroup.addChild(ctrl.parameters())
        self.programCtrls.append(ctrl)
        camMod = self.cameraModule()
        if camMod is None:
            raise HelpfulException("Cannot add control items until a camera module is available to display them.")
            return False
        for item in ctrl.getGraphicsItems():
            camMod.ui.addItem(item, None, [1, 1], 1000)

    def programCtrlRemoved(self, parent, param):
        ctrl = param.ctrl
        for item in ctrl.getGraphicsItems():
            if item.scene() is not None:
                item.scene().removeItem(item)
        self.programCtrls.remove(ctrl)
        
    def getNextItemName(self, base):
        ## Return the next available item name starting with base
        names = [item.name for item in self.items.values()]
        num = 1
        while True:
            name = base + str(num)
            if name not in names:
                return name
            num += 1
        
    #def addProgram(self, name=None): 
        #camMod = self.cameraModule()
        #if camMod is None:
            #return False
        #self.ui.programControlsLayout.setEnabled(True)
        #item = TargetProgram()
        #if name is None:
            #name = 'Program' + str(self.nextId)
        #self.nextId += 1 
        #item.name = name
        #item.objective = self.currentObjective
        #self.items[name] = item
        #treeitem = QtGui.QTreeWidgetItem(QtCore.QStringList(name))
        #treeitem.setCheckState(0, QtCore.Qt.Checked)
        #self.ui.itemTree.addTopLevelItem(treeitem)
        #self.updateItemColor(treeitem)
        #camMod.ui.addItem(item.origin, None, [1,1], 1000)
        #item.connect(QtCore.SIGNAL('regionChangeFinished'), self.itemMoved)
        #item.connect(QtCore.SIGNAL('regionChanged'), self.getTargetList)
        #item.connect(QtCore.SIGNAL('pointsChanged'), self.itemChanged)
        #self.itemChanged(item)
        #self.updateDeviceTargetList(item)
        
    def addItem(self, itemType, state=None):
        
        if state is None:
            state = {}
            
        if state.get('name', None) is None:
            state['name'] = self.getNextItemName(itemType)
            
        try:
            ptSize, dispSize = self.pointSize()
        except HelpfulException as ex:
            exc = sys.exc_info()
            if ex.kwargs.get('errId', None) == 1:
                raise HelpfulException('Cannot add items: %s is not calibrated for %s.' %(str(self.ui.laserCombo.currentText()), self.currentOpticState), exc=exc)
            else:
                raise
            
        state['ptSize'] = dispSize
        
        cls = {'Grid': Grid, 'Point': TargetPoint, 'Occlusion': TargetOcclusion}[itemType]
        item = cls(**state)
        
        camMod = self.cameraModule()
        if camMod is None:
            raise HelpfulException("Cannot add control items until a camera module is available to display them.")
            return False

        item.opticState = self.currentOpticState
        self.items[item.name] = item
        
        pos = state.get('pos', None)  ## if no position is given, the camera will automatically place the item in the middle fo the view
        camMod.ui.addItem(item, pos, [1, 1], 1000)
            
        self.positionCtrlGroup.addChild(item.parameters())
        
        item.sigStateChanged.connect(self.itemChanged)
        item.parameters().sigValueChanged.connect(self.itemActivationChanged)
        
        self.itemChanged(item)
        self.storeConfiguration()
        
    def itemMoved(self, item):
        self.itemChanged()
       

    def itemChanged(self, item=None):
        self.targets = None
        self.sequenceChanged()
        self.storeConfiguration()
        
    def itemActivationChanged(self, param, val):
        i = param.item
        if val:
            i.setOpacity(1.0)  ## have to hide this way since we still want the children to be visible
            for h in i.handles:
                h['item'].setOpacity(1.0)
        else:
            i.setOpacity(0.0)
            for h in i.handles:
                h['item'].setOpacity(0.0)
        self.cameraModule().ui.update()
            
        self.sequenceChanged()
    
    def getTargetList(self):  ## should probably do some caching here.
        items = self.activeItems()
        locations = []
        occArea = QtGui.QPainterPath()
        for i in items:
            if isinstance(i, TargetOcclusion):
                occArea |= i.mapToView(i.shape())
            
        for i in items:
            pts = i.listPoints()
            for j in range(len(pts)):
                p = pts[j]
                point = QtCore.QPointF(p[0], p[1])
                if occArea.contains(point):
                    i.setTargetPen(j, QtGui.QPen(QtGui.QColor(0,0,0,160)))
                else:
                    locations.append(p)
                    i.setTargetPen(j, None)
        return locations

    
    def sequenceChanged(self):
        self.targets = None
        self.sigSequenceChanged.emit(self.dev.name())
        self.updateTDPlot()
        
    def updateTDPlot(self):
        self.tdPlot.clear()
        state = self.stateGroup.state()
        minTime = state['minTime']
        minDist = state['minDist']
        
        dist = np.arange(0, 0.001, 0.00002)
        cost = optimize.costFn(dist, minTime, minDist)
        
        self.tdPlot.plot(dist, cost)

    def recomputeClicked(self):
        try:
            self.ui.recomputeBtn.setEnabled(False)
            self.generateTargets()
        finally:
            self.ui.recomputeBtn.setEnabled(True)

    def generateTargets(self):
        #items = self.activeItems()
        #prof= Profiler('ScanerTaskGui.generateTargets()')
        self.targets = []
        locations = self.getTargetList()
        
        bestTime = None
        bestSolution = None

        nTries = np.clip(int(10 - len(locations)/20), 1, 10)
        
        ## About to compute order/timing of targets; display a progress dialog
        #prof.mark('setup')
        #progressDlg = QtGui.QProgressDialog("Computing pseudo-optimal target sequence...", 0, 1000)
        #progressDlg.setWindowModality(QtCore.Qt.WindowModal)
        #progressDlg.setMinimumDuration(500)
        #prof.mark('progressDlg')
        deadTime = self.taskRunner.getParam('duration')

        state = self.stateGroup.state()
        minTime = state['minTime']
        minDist = state['minDist']

        #try:
        with pg.ProgressDialog("Computing random target sequence...", 0, 1000, busyCursor=True) as dlg:
            #times=[]
            for i in range(nTries):
                #prof.mark('attempt: %i' %i)
                
                ## Run in a remote process for a little speedup
                for n, m in optimize.opt2(locations, minTime, minDist, deadTime, greed=1.0):
                    ## we can update the progress dialog here.
                    if m is None:
                        solution = n
                    else:
                        prg = int(((i/float(nTries)) + ((n/float(m))/float(nTries))) * 1000)
                        #print n,m, prg
                        dlg.setValue(prg)
                        #print i
                        if dlg.wasCanceled():
                            raise Exception("Target sequence computation canceled by user.")
                #prof.mark('foundSolution')
                time = sum([l[1] for l in solution])
                if bestTime is None or time < bestTime:
                    #print "  new best time:", time
                    bestTime = time
                    bestSolution = solution[:]
                    #print "new best:", len(bestSolution), minTime
                #prof.mark('check time')
        #except:
            #raise
        #finally:
            ## close progress dialog no matter what happens
            #print "Times: ", times
            #progressDlg.setValue(1000)
        
        self.targets = bestSolution
        #print "Solution:"
        #for t in self.targets:
            #print "  ", t
        self.ui.timeLabel.setText('Total time: %0.1f sec'% bestTime)
        #prof.mark('Done.')
        #prof.finish()
        
    def activeItems(self):
        return [self.items[i] for i in self.items if self.items[i].isActive()]
    
    def taskStarted(self, params):
        """Task has started; color the current and previous targets"""
        pass
    
    def quit(self):
        s = self.testTarget.scene()
        if s is not None:
            for item in self.items.values():
                item.close()
            s.removeItem(self.testTarget)
            s.removeItem(self.spotMarker)
            for ctrl in self.programCtrls:
                for item in ctrl.getGraphicsItems():
                    s.removeItem(item)


class TargetPoint(pg.EllipseROI):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self, name, ptSize, **args):
        self.name = name
        #if 'host' in args:
            #self.host = args.pop('host')
        
        pg.ROI.__init__(self, (0,0), [ptSize] * 2, movable=args.get('movable', True))
        self.aspectLocked = True
        self.overPen = None
        self.underPen = self.pen
        self.setFlag(QtGui.QGraphicsItem.ItemIgnoresParentOpacity, True)
        self.params = pTypes.SimpleParameter(name=self.name, type='bool', value=args.get('active', True), removable=True, renamable=True, children=[
        ])
        self.params.item = self

    def isActive(self):
        return self.params.value()

    def parameters(self):
        return self.params
        
    def setPointSize(self, displaySize, realSize=None):
        s = displaySize / self.state['size'][0]
        self.scale(s, [0.5, 0.5])
        
    def listPoints(self):
        p = self.mapToView(self.boundingRect().center())
        return [(p.x(), p.y())]
        
    def setPen(self, pen):
        self.underPen = pen
        pg.EllipseROI.setPen(self, pen)
    
    def setTargetPen(self, index, pen):
        self.overPen = pen
        if pen is None:
            pen = self.underPen
        pg.EllipseROI.setPen(self, pen)
        
    def resetParents(self):
        pass
    
    def saveState(self):
        pos = self.listPoints()[0]
        return {'type': 'Point', 'pos': pos, 'active': self.params.value()}

    def close(self):
        if self.scene() is not None:
            self.scene().removeItem(self)


class Grid(pg.CrosshairROI):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self, name, ptSize, **args):
        pg.CrosshairROI.__init__(self, pos=(0,0), size=args.get('size', [ptSize*4]*2), angle=args.get('angle', 0), **args)

        self.name = name

        self.params = pTypes.SimpleParameter(name=self.name, type='bool', value=True, removable=True, renamable=True, children=[
            dict(name='layout', type='list', value=args.get('layout', 'Hexagonal'), values=['Square', 'Hexagonal']),
            dict(name='spacing', type='float', value=args.get('spacing', ptSize), suffix='m', siPrefix=True, bounds=[1e-9, None], step=10e-6),
            dict(name='Active Regions', type='group', addText="Add region...", addList=['Rectangle', 'Polygon'])
        ])
        
        self.params.item = self
        self.params.param('Active Regions').addNew = self.addActiveRegion
        self.rgns = []
        self.pointSize = ptSize
        self._points = []
        self._scene = None 
        self._scatter = pg.ScatterPlotItem(pxMode=False, brush=None, antialias=True)
        self._scatter.setParentItem(self)
        self.params.param('layout').sigStateChanged.connect(self.invalidatePoints)
        self.params.param('spacing').sigStateChanged.connect(self.invalidatePoints)
        self.params.param('Active Regions').sigChildRemoved.connect(self.rgnRemoved)
        
        self.sigRegionChanged.connect(self.invalidatePoints)
        self.params.sigValueChanged.connect(self.toggled)
        self.params.sigRemoved.connect(self.removed)
        
    def isActive(self):
        return self.params.value()
    
    def parameters(self):
        return self.params
    
    def setTargetPen(self, *args):
        pass
    
    def toggled(self, b):
        if b:
            self.show()
            for r in self.rgns:
                r.item.show()
        else:
            self.hide()
            for r in self.rgns:
                r.item.hide()

    def setVisible(self, vis):
        super(Grid, self).setVisible(vis)
        for rgn in self.rgns:
            rgn.item.setVisible(vis)
                
    def removed(self, child):
        #print "Grid.removed called.", self, child
        self.close()

    def close(self):
        for r in self.rgns:
            self.rgnRemoved(self, r)
        if self.scene() is not None:
            self.scene().removeItem(self)
    
    def addActiveRegion(self, rgnType):
        rgn = pTypes.SimpleParameter(name=rgnType, autoIncrementName=True,
                                     type='bool', value=True, 
                                     removable=True, renamable=True)
        self.params.param('Active Regions').addChild(rgn)
        pos = self.getViewBox().viewRect().center() 
        size = self.params.param('spacing').value()*4
        
        if rgnType == 'Rectangle':
            roi = pg.ROI(pos=pos, size=size, angle=self.angle())
            roi.addScaleHandle([0, 0], [1, 1])
            roi.addScaleHandle([1, 1], [0, 0])
            roi.addRotateHandle([0, 1], [0.5, 0.5])
            roi.addRotateHandle([1, 0], [0.5, 0.5])
        elif rgnType == 'Polygon':
            roi = pg.PolyLineROI((pos, pos+pg.Point(0,1)*size, pos+pg.Point(1,0)*size), closed=True)
        else:
            raise Exception('Not sure how to add region of type:%s' % rgnType)
        
        rgn.item = roi
        self.rgns.append(rgn)
        self.getViewBox().addItem(roi)
        roi.sigRegionChanged.connect(self.invalidatePoints)
        roi.sigRegionChangeFinished.connect(self.stateChangeFinished)
        rgn.sigValueChanged.connect(self.rgnToggled)
        self.invalidatePoints()
        self.stateChangeFinished()
        
    def rgnToggled(self, rgn, b):
        if b:
            rgn.item.setVisible(True)
        else:
            rgn.item.setVisible(False)
        self.invalidatePoints()
        self.stateChangeFinished()
            
    def rgnRemoved(self, grp, rgn):
        roi = rgn.item
        if roi.scene() is not None:
            roi.scene().removeItem(roi)
        self.invalidatePoints()
        self.stateChangeFinished()
        
    def localPoints(self):
        """Return active points in local coordinate system"""
        if self._points is None:
            self._points = self.generatePoints()
        return self._points
            
    def listPoints(self):
        pts = self.localPoints()
        tr = self.viewTransform()
        if tr is None:
            return []
        return pg.transformCoordinates(tr, pts, transpose=True)
    
    def setPointSize(self, displaySize, realSize):
        self.pointSize = displaySize
        self.update()
    
    def invalidatePoints(self):
        self._points = None
        self.update()
        
    def stateChangeFinished(self):
        self.sigStateChanged.emit(self)

    def generatePoints(self):
        layout = self.params['layout']

        # get x/y point spacing
        sepx = sepy = self.params['spacing']
        if layout == 'Hexagonal':
            sepy *= 3 ** 0.5

        # find grid points inside each active region
        points = []
        for rgn in self.rgns:
            if not rgn.value():
                continue
            roi = rgn.item
            rect = self.mapFromItem(roi, roi.boundingRect()).boundingRect()

            ## find 'snap' position of first spot
            newPos = self.getSnapPosition((rect.x(), rect.y()))
            x = newPos.x() - sepx
            y = newPos.y() - sepy
            w = rect.width() + 2*sepx
            h = rect.height() + 2*sepy

            if layout == "Hexagonal":
                # make every other row of the grid starting from top
                points.append(self.generateGrid([x, y], [x+w, y+h], [sepx, sepy]))
                # make every other row of the grid starting with 2nd row
                points.append(self.generateGrid([x+0.5*sepx, y+0.5*sepy], [x+w, y+h], [sepx, sepy]))
            elif layout == "Square":
                points.append(self.generateGrid([x, y], [x+w, y+h], [sepx, sepx]))
        
        if len(points) == 0:
            return np.empty((0,2), dtype=float)
        
        # stack all point arrays together
        points = np.ascontiguousarray(np.vstack(points))
        # do some rounding to make it easier to detect duplicate points
        dec = int(-np.log10(sepx) + 4)
        points = np.round(points, dec)
        # keep only unique points
        points = np.unique(points.view(dtype=[('x', float), ('y', float)]))        
        # convert back to normal array
        points = points.view(dtype=float).reshape(len(points), 2)
        
        # get shape of total active region
        activeArea = QtGui.QPainterPath()
        for rgn in self.rgns:
            if rgn.value():
                roi = rgn.item
                activeArea |= self.mapFromItem(roi, roi.shape())
            
        # filter for all points within active region
        mask = np.array([activeArea.contains(pg.Point(*pt)) for pt in points], dtype=bool)

        return points[mask]

    @staticmethod
    def generateGrid(start, stop, sep):
        # 2-dimensional range(); generates point locations filling a rectangular grid
        nx = int((stop[0] - start[0]) / sep[0]) + 1
        ny = int((stop[1] - start[1]) / sep[1]) + 1
        pts = np.mgrid[0:nx,0:ny].reshape(2, nx*ny).transpose()
        pts = pts * np.array(sep).reshape(1, 2)
        pts += np.array(start).reshape(1, 2)
        return pts
            
    def getSnapPosition(self, pos):
        ## Given that pos has been requested, return the nearest snap-to position
        ## optionally, snap may be passed in to specify a rectangular snap grid.
        ## override this function for more interesting snap functionality..
    
        layout = self.params['layout']
        spacing = self.params['spacing']
        
        if layout == 'Square':
            snap = pg.Point(spacing, spacing)
            w = round(pos[0] / snap[0]) * snap[0]
            h = round(pos[1] / snap[1]) * snap[1]
            return pg.Point(w, h)
        
        elif layout == 'Hexagonal':
            snap1 = pg.Point(spacing, spacing*3.0**0.5)
            dx = 0.5*snap1[0]
            dy = 0.5*snap1[1]
            w1 = round(pos[0] / snap1[0]) * snap1[0]
            h1 = round(pos[1] / snap1[1]) * snap1[1]
            w2 = round((pos[0]-dx) / snap1[0]) * snap1[0] + dx
            h2 = round((pos[1]-dy) / snap1[1]) * snap1[1] + dy
            if (pg.Point(w1, h1)-pos).length() < (pg.Point(w2,h2) - pos).length():
                return pg.Point(w1, h1)
            else:
                return pg.Point(w2, h2)
    
    def parentChanged(self):
        # Called when grid gets a new parent or scene. 
        super(Grid, self).parentChanged()
        if self._scene is not None:
            try:
                self._scene.sigPrepareForPaint.disconnect(self.prepareForPaint)
            except TypeError:
                pass
        self._scene = self.scene()
        if self._scene is not None:
            self._scene.sigPrepareForPaint.connect(self.prepareForPaint)

    def prepareForPaint(self):
        # Update points in scatter plot item
        pts = self.localPoints()
        self._scatter.setData(x=pts[:,0], y=pts[:,1], size=self.pointSize)

        
class TargetOcclusion(pg.PolygonROI):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self, name, ptSize, **args):
        self.name = name
        points = args.get('points', ([0,0], [0,ptSize*3], [ptSize*3,0]))
        pos = (0,0)
        pg.PolygonROI.__init__(self, points, pos)
        self.setZValue(10000000)
        self.params = pTypes.SimpleParameter(name=self.name, type='bool', value=True, removable=True, renamable=True, children=[
        ])
        self.params.item = self
        self.sigRegionChanged.connect(self.rgnChanged)

    def isActive(self):
        return self.params.value()

    def rgnChanged(self):
        self.sigStateChanged.emit(self)

    def parameters(self):
        return self.params
        
    def setPointSize(self, size, realSize):
        pass
    
    def resetParents(self):
        pass
    
    def saveState(self):
        return {'type':'Occlusion', 'pos': (self.pos().x(), self.pos().y()), 'points': [(p.x(), p.y()) for p in self.listPoints()]}
    
    def listPoints(self):
        return []

    def close(self):
        if self.scene() is not None:
            self.scene().removeItem(self)
    
class ProgramLineScan(QtCore.QObject):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self):
        QtCore.QObject.__init__(self)
        self.name = 'lineScan'
        ### These need to be initialized before the ROI is initialized because they are included in stateCopy(), which is called by ROI initialization.
        
        self.params = pTypes.SimpleParameter(name=self.name, autoIncrementName = True, type='bool', value=True, removable=True, renamable=True, children=[
            dict(name='length', type='float', value=1e-5, suffix='m', siPrefix=True, bounds=[1e-6, None], step=1e-6),
            dict(name='startTime', type='float', value=5e-2, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2),
            dict(name='sweepDuration', type='float', value=4e-3, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2),
            dict(name='retraceDuration', type='float', value=1e-3, suffix='s', siPrefix=True, bounds=[0., None], step=1e-3),
            dict(name='nScans', type='int', value=100, bounds=[1, None]),
            dict(name='endTime', type='float', value=5.5e-1, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2, readonly=True),
        ])
        self.params.ctrl = self        
        self.roi = pg.LineSegmentROI([[0.0, 0.0], [self.params['length'], self.params['length']]])
 #       print dir(self.roi)
        self.params.sigTreeStateChanged.connect(self.update)
        self.roi.sigRegionChangeFinished.connect(self.updateFromROI)
        
    def getGraphicsItems(self):
        return [self.roi]

    def isActive(self):
        return self.params.value()

    def setVisible(self, vis):
        if vis:
            self.roi.setOpacity(1.0)  ## have to hide this way since we still want the children to be visible
            for h in self.roi.handles:
                h['item'].setOpacity(1.0)
        else:
            self.roi.setOpacity(0.0)
            for h in self.roi.handles:
                h['item'].setOpacity(0.0)
        
    def parameters(self):
        return self.params
    
    def update(self):
        self.params['endTime'] = self.params['startTime']+self.params['nScans']*(self.params['sweepDuration'] + self.params['retraceDuration'])
        self.setVisible(self.params.value())
            
    def updateFromROI(self):
        p =self.roi.listPoints()
        dist = (pg.Point(p[0])-pg.Point(p[1])).length()
        self.params['length'] = dist
        
    def generateTask(self):
        points = self.roi.listPoints() # in local coordinates local to roi.
        points = [self.roi.mapToView(p) for p in points] # convert to view points (as needed for scanner)
        return {'type': 'lineScan', 'active': self.isActive(), 'points': points, 'startTime': self.params['startTime'], 'sweepDuration': self.params['sweepDuration'], 
                'endTime': self.params['endTime'], 'retraceDuration': self.params['retraceDuration'], 'nScans': self.params['nScans']}


class MultiLineScanROI(pg.PolyLineROI):
    """ custom class over ROI polyline to allow alternate coloring of different segments
    """
    def addSegment(self, *args, **kwds):
        pg.PolyLineROI.addSegment(self, *args, **kwds)
        self.recolor()
    
    def removeSegment(self, *args, **kwds):
        pg.PolyLineROI.removeSegment(self, *args, **kwds)
        self.recolor()
    
    def recolor(self):
        for i, s in enumerate(self.segments):
            if i % 2 == 0:
                s.setPen(self.pen)
            else:
                s.setPen(pg.mkPen([75, 200, 75]))


class ProgramMultipleLineScan(QtCore.QObject):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self):
        QtCore.QObject.__init__(self)
        self.name = 'multipleLineScan'
        ### These need to be initialized before the ROI is initialized because they are included in stateCopy(), which is called by ROI initialization.
        
        self.params = pTypes.SimpleParameter(name=self.name, type='bool', value=True, removable=True, renamable=True, children=[
            dict(name='Length', type='float', value=1e-5, suffix='m', siPrefix=True, bounds=[1e-6, None], step=1e-6),
            dict(name='startTime', type='float', value=5e-2, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2),
            dict(name='sweepSpeed', type='float', value=1e-6, suffix='m/ms', siPrefix=True, bounds=[1e-8, None], minStep=5e-7, step=0.5, dec=True),
            dict(name='interSweepSpeed', type='float', value=5e-6, suffix='m/ms', siPrefix=True, bounds=[1e-8, None], minStep=5e-7, step=0.5, dec=True),
            dict(name='nScans', type='int', value=100, bounds=[1, None]),
            dict(name='endTime', type='float', value=5.5e-1, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2, readonly=True),
        ])
        self.params.ctrl = self        
        self.roi = MultiLineScanROI([[0.0, 0.0], [self.params['Length'], self.params['Length']]])
        self.roi.sigRegionChangeFinished.connect(self.updateFromROI)
        self.params.sigTreeStateChanged.connect(self.update)
        
    def getGraphicsItems(self):
        return [self.roi]

    def isActive(self):
        return self.params.value()
    
    def setVisible(self, vis):
        if vis:
            self.roi.setOpacity(1.0)  ## have to hide this way since we still want the children to be visible
            for h in self.roi.handles:
                h['item'].setOpacity(1.0)
        else:
            self.roi.setOpacity(0.0)
            for h in self.roi.handles:
                h['item'].setOpacity(0.0)
                
    def parameters(self):
        return self.params
    
    def update(self):
        pts = self.roi.listPoints()
        scanTime = 0.
        interScanFlag = False
        for k in xrange(len(pts)): # loop through the list of points
            k2 = k + 1
            if k2 > len(pts)-1:
                k2 = 0
            dist = (pts[k]-pts[k2]).length()
            if interScanFlag is False:
                scanTime += dist/(self.params['sweepSpeed']*1000.)
            else:
                scanTime += dist/(self.params['interSweepSpeed']*1000.)
            interScanFlag = not interScanFlag
        self.params['endTime'] = self.params['startTime']+(self.params['nScans']*scanTime)
        self.setVisible(self.params.value())
    
    def updateFromROI(self):
        self.update()
    #p =self.roi.listPoints()
        #dist = (pg.Point(p[0])-pg.Point(p[1])).length()
        #self.params['length'] = dist
        
    def generateTask(self):
        points=self.roi.listPoints() # in local coordinates local to roi.
        points = [self.roi.mapToView(p) for p in points] # convert to view points (as needed for scanner)
        points = [(p.x(), p.y()) for p in points]   ## make sure we can write this data to HDF5 eventually..
        return {'type': 'multipleLineScan', 'active': self.isActive(), 'points': points, 'startTime': self.params['startTime'], 'sweepSpeed': self.params['sweepSpeed'], 
                'endTime': self.params['endTime'], 'interSweepSpeed': self.params['interSweepSpeed'], 'nScans': self.params['nScans']}
                
    
class ProgramRectScan(QtCore.QObject):
    
    sigStateChanged = QtCore.Signal(object)
    
    def __init__(self):
        QtCore.QObject.__init__(self)
        self.name = 'rectScan'
        ### These need to be initialized before the ROI is initialized because they are included in stateCopy(), which is called by ROI initialization.
        
        self.params = pTypes.SimpleParameter(name=self.name, type='bool', value=True, removable=True, renamable=True, children=[
            dict(name='width', type='float', value=2e-5, suffix='m', siPrefix=True, bounds=[1e-6, None], step=1e-6),
            dict(name='height', type='float', value=1e-5, suffix='m', siPrefix=True, bounds=[1e-6, None], step=1e-6),
            dict(name='overScan', type='float', value=70., suffix='%', siPrefix=False, bounds=[0, 200.], step = 1),
            dict(name='pixelSize', type='float', value=4e-7, suffix='m', siPrefix=True, bounds=[2e-7, None], step=2e-7),
            dict(name='startTime', type='float', value=1e-2, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2),
            dict(name='duration', type='float', value=5e-1, suffix='s', siPrefix=True, bounds=[0., None], step=1e-2),
            dict(name='nScans', type='int', value=10, bounds=[1, None]),
        ])
        self.params.ctrl = self
        self.roi = pg.ROI(size=[self.params['width'], self.params['height']], pos=[0.0, 0.0])
        self.roi.addScaleHandle([1,1], [0.5, 0.5])
        self.roi.addRotateHandle([0,0], [0.5, 0.5])
        self.params.sigTreeStateChanged.connect(self.update)
        self.roi.sigRegionChangeFinished.connect(self.updateFromROI)
        
    def getGraphicsItems(self):
        return [self.roi]

    def isActive(self):
        return self.params.value()
 
    def setVisible(self, vis):
        if vis:
            self.roi.setOpacity(1.0)  ## have to hide this way since we still want the children to be visible
            for h in self.roi.handles:
                h['item'].setOpacity(1.0)
        else:
            self.roi.setOpacity(0.0)
            for h in self.roi.handles:
                h['item'].setOpacity(0.0)
                
    def parameters(self):
        return self.params

    def update(self):
        self.setVisible(self.params.value())
    
    def updateFromROI(self):
        """ read the ROI rectangle width and height and repost
        in the parameter tree """
        state = self.roi.getState()
        w, h = state['size']
        self.params['width'] = w
        self.params['height'] = h
        
    def generateTask(self):
        state = self.roi.getState()
        w, h = state['size']
        p0 = pg.Point(0,0)
        p1 = pg.Point(w,0)
        p2 = pg.Point(0, h)
        points = [p0, p1, p2]
        points = [pg.Point(self.roi.mapToView(p)) for p in points] # convert to view points (as needed for scanner)
        return {'type': self.name, 'active': self.isActive(), 'points': points, 'startTime': self.params['startTime'], 
                'endTime': self.params['duration']+self.params['startTime'], 'duration': self.params['duration'],
                'nScans': self.params['nScans'],
                'pixelSize': self.params['pixelSize'], 'overScan': self.params['overScan'],
                }
        


