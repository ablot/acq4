# -*- coding: utf-8 -*-
from acq4.modules.Module import *
from ManagerTemplate import Ui_MainWindow
from PyQt4 import QtCore, QtGui
import sys, os
import acq4.util.configfile as configfile
from acq4.util.debug import *
from acq4 import modules


class Manager(Module):
    moduleDisplayName = "Manager"
    moduleCategory = None

    def __init__(self, manager, name, config):
        Module.__init__(self, manager, name, config)
        self.win = QtGui.QMainWindow()
        mp = os.path.dirname(__file__)
        self.win.setWindowIcon(QtGui.QIcon(os.path.join(mp, 'icon.png')))
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self.win)
        self.stateFile = os.path.join('modules', self.name + '_ui.cfg')
        firstDock = None
        
        self.modGroupOrder = ['Acquisition', 'Analysis', 'Utilities']

        self.devRackDocks = {}
        for d in self.manager.listDevices():
            try:
                dw = self.manager.getDevice(d).deviceInterface(self)
                if dw is None:
                    continue
                dock = QtGui.QDockWidget(d)
                dock.setFeatures(dock.DockWidgetMovable | dock.DockWidgetFloatable)
                dock.setObjectName(d)
                dock.setWidget(dw)
                
                self.devRackDocks[d] = dock
                self.win.addDockWidget(QtCore.Qt.RightDockWidgetArea, dock)
                
                # By default, we stack all docks
                if firstDock is None:
                    firstDock = dock
                else:
                    self.win.tabifyDockWidget(firstDock, dock)
            except:
                self.showMessage("Error creating dock for device '%s', see console for details." % d, 10000)
                printExc("Error while creating dock for device '%s':" % d)

        self.updateModList()
        self.updateConfList()

        self.ui.loadConfigBtn.clicked.connect(self.loadConfig)
        self.ui.loadModuleBtn.clicked.connect(self.loadSelectedModule)
        self.ui.reloadModuleBtn.clicked.connect(self.reloadAll)
        self.ui.configList.itemDoubleClicked.connect(self.loadConfig)
        self.ui.moduleList.itemDoubleClicked.connect(self.loadSelectedModule)
        self.ui.quitBtn.clicked.connect(self.requestQuit)

        state = self.manager.readConfigFile(self.stateFile)
        # restore window position
        if 'geometry' in state:
            geom = QtCore.QRect(*state['geometry'])
            self.win.setGeometry(geom)
        # restore dock configuration
        if 'window' in state:
            ws = QtCore.QByteArray.fromPercentEncoding(state['window'])
            self.win.restoreState(ws)

        self.win.show()
        
    def showMessage(self, *args):
        self.ui.statusBar.showMessage(*args)
        
    def updateModList(self):
        # Fill the list of modules.

        # clear tree and create top-level items in default order
        self.ui.moduleList.clear()
        self._modGrpItems = {}
        for n in self.modGroupOrder:
            self._mkModGrpItem(n)

        # load defined configurations first
        confMods = []
        for name, conf in self.manager.listDefinedModules().items():
            cls = modules.getModuleClass(conf['module'])
            confMods.append(cls)
            root = self._mkModGrpItem(cls.moduleCategory)
            item = QtGui.QTreeWidgetItem([name])
            font = item.font(0)
            font.setBold(True)
            item.setFont(0, font)
            item.confModName = name
            root.addChild(item)
        
        # if a module has no defined configurations, then just give it a default entry without configuration.
        for name,cls in modules.getModuleClasses().items():
            if cls is Manager or cls in confMods:
                continue
            root = self._mkModGrpItem(cls.moduleCategory)
            dispName = cls.moduleDisplayName or cls.__name__
            item = QtGui.QTreeWidgetItem([dispName])
            item.modName = name
            root.addChild(item)
    
    def _mkModGrpItem(self, name):
        if name is None:
            name = "Other"
        if name in self._modGrpItems:
            return self._modGrpItems[name]
        parts = name.split('.')
        if len(parts) > 1:
            root = self._mkModGrpItem('.'.join(parts[:-1]))
        else:
            root = self.ui.moduleList.invisibleRootItem()
        item = QtGui.QTreeWidgetItem([parts[-1]])
        root.addChild(item)
        item.setExpanded(True)
        self._modGrpItems[name] = item
        return item

    def updateConfList(self):
        self.ui.configList.clear()
        for m in self.manager.listConfigurations():
            self.ui.configList.addItem(m)
        
    def show(self):
        self.win.show()

    def requestQuit(self):
        self.manager.quit()

    def loadSelectedModule(self):
        item = self.ui.moduleList.currentItem()
        if hasattr(item, 'confModName'):
            self.loadConfiguredModule(item.confModName)
        elif hasattr(item, 'modName'):
            self.loadModule(item.modName)

    def loadConfiguredModule(self, mod):
        try:
            QtGui.QApplication.setOverrideCursor(QtGui.QCursor(QtCore.Qt.WaitCursor))
            self.manager.loadDefinedModule(mod)
            self.showMessage("Loaded module configuration '%s'." % mod, 10000)
        finally:
            QtGui.QApplication.restoreOverrideCursor()
        
    def loadModule(self, mod):
        try:
            QtGui.QApplication.setOverrideCursor(QtGui.QCursor(QtCore.Qt.WaitCursor))
            self.manager.loadModule(mod)
            self.showMessage("Loaded module '%s'." % mod, 10000)
        finally:
            QtGui.QApplication.restoreOverrideCursor()
        
    def reloadAll(self):
        self.manager.reloadAll()
        #mod = str(self.ui.moduleList.currentItem().text())
        #self.manager.loadDefinedModule(mod, forceReload=True)
        #self.showMessage("Loaded module '%s'." % mod, 10000)
        
    def loadConfig(self):
        #print "LOAD CONFIG"
        cfg = str(self.ui.configList.currentItem().text())
        self.manager.loadDefinedConfig(cfg)
        self.updateModList()
        self.showMessage("Loaded configuration '%s'." % cfg, 10000)

    def quit(self):
        ## save ui configuration
        geom = self.win.geometry()
        state = {'window': str(self.win.saveState().toPercentEncoding()), 'geometry': [geom.x(), geom.y(), geom.width(), geom.height()]}
        self.manager.writeConfigFile(state, self.stateFile)
        Module.quit(self)
