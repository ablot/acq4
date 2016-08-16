# -*- coding: utf-8 -*-
from PyQt4 import QtCore, QtGui
import modules
import acq4.pyqtgraph.dockarea as dockarea
import acq4.Manager
#from acq4.LogWindow import LogButton
from acq4.util.StatusBar import StatusBar

class AnalysisHost(QtGui.QMainWindow):
    """Window for hosting analysis widgets.
    Provides:
     - File / DB access for module
     - 
    
    
    """
    
    def __init__(self, dataManager=None, dataModel=None, module=None):
        QtGui.QMainWindow.__init__(self)
        self.dm = dataManager
        self.dataModel = dataModel
        self.mod = None
        self.dockArea = dockarea.DockArea()
        self.setCentralWidget(self.dockArea)
        
        #self.logBtn = LogButton('Log')
        #self.statusBar().addPermanentWidget(self.logBtn)
        self.setStatusBar(StatusBar())
        
        if module is not None:
            self.loadModule(module)
            
        self.show()
        
    def dataManager(self):
        return self.dm
        
    def loadModule(self, modName):
        if self.mod is not None:
            raise Exception("No fair loading extra modules in one host.")
        self.mod = modules.load(modName, self)
        
        elems = self.mod.listElements()
        for name, el in elems.iteritems():
            w = self.mod.getElement(name, create=True)
            d = dockarea.Dock(name=name, size=el.size())
            if w is not None:
                d.addWidget(w)
            pos = el.pos()
            if pos is None:
                pos = ()
            #print d, pos
            if isinstance(pos, basestring):
                pos = (pos,)
            self.dockArea.addDock(d, *pos)
        self.elements = elems
        
        self.setWindowTitle(modName)
        
        acq4.Manager.getManager().declareInterface(modName, 'analysisMod', self.mod)
        
    def closeEvent(self, ev):
        if self.quit():
            ev.accept()
        
    def quit(self):
        return self.mod.quit()
