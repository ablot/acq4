# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './acq4/analysis/modules/IVCurve/ctrlTemplate.ui'
#
# Created: Tue Dec 24 01:49:13 2013
#      by: PyQt4 UI code generator 4.10
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(326, 475)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.MinimumExpanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        self.gridLayout = QtGui.QGridLayout(Form)
        self.gridLayout.setMargin(0)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.groupBox = QtGui.QGroupBox(Form)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Ignored, QtGui.QSizePolicy.Ignored)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.gridLayout_3 = QtGui.QGridLayout(self.groupBox)
        self.gridLayout_3.setMargin(5)
        self.gridLayout_3.setHorizontalSpacing(10)
        self.gridLayout_3.setVerticalSpacing(1)
        self.gridLayout_3.setObjectName(_fromUtf8("gridLayout_3"))
        self.IVCurve_ssTStop = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_ssTStop.setFont(font)
        self.IVCurve_ssTStop.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_ssTStop.setMinimum(-5000.0)
        self.IVCurve_ssTStop.setMaximum(50000.0)
        self.IVCurve_ssTStop.setObjectName(_fromUtf8("IVCurve_ssTStop"))
        self.gridLayout_3.addWidget(self.IVCurve_ssTStop, 5, 1, 1, 2)
        self.IVCurve_pkTStart = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_pkTStart.setFont(font)
        self.IVCurve_pkTStart.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_pkTStart.setMinimum(-5000.0)
        self.IVCurve_pkTStart.setMaximum(50000.0)
        self.IVCurve_pkTStart.setObjectName(_fromUtf8("IVCurve_pkTStart"))
        self.gridLayout_3.addWidget(self.IVCurve_pkTStart, 7, 1, 1, 2)
        self.IVCurve_tau2TStart = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_tau2TStart.setFont(font)
        self.IVCurve_tau2TStart.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_tau2TStart.setDecimals(2)
        self.IVCurve_tau2TStart.setMaximum(5000.0)
        self.IVCurve_tau2TStart.setObjectName(_fromUtf8("IVCurve_tau2TStart"))
        self.gridLayout_3.addWidget(self.IVCurve_tau2TStart, 9, 1, 1, 2)
        self.label_10 = QtGui.QLabel(self.groupBox)
        self.label_10.setObjectName(_fromUtf8("label_10"))
        self.gridLayout_3.addWidget(self.label_10, 14, 0, 1, 1)
        self.IVCurve_PrintResults = QtGui.QPushButton(self.groupBox)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.MinimumExpanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.IVCurve_PrintResults.sizePolicy().hasHeightForWidth())
        self.IVCurve_PrintResults.setSizePolicy(sizePolicy)
        self.IVCurve_PrintResults.setObjectName(_fromUtf8("IVCurve_PrintResults"))
        self.gridLayout_3.addWidget(self.IVCurve_PrintResults, 14, 5, 1, 1)
        self.IVCurve_Update = QtGui.QPushButton(self.groupBox)
        self.IVCurve_Update.setObjectName(_fromUtf8("IVCurve_Update"))
        self.gridLayout_3.addWidget(self.IVCurve_Update, 14, 2, 1, 1)
        self.IVCurve_SpikeThreshold = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_SpikeThreshold.setFont(font)
        self.IVCurve_SpikeThreshold.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_SpikeThreshold.setDecimals(1)
        self.IVCurve_SpikeThreshold.setMinimum(-100.0)
        self.IVCurve_SpikeThreshold.setObjectName(_fromUtf8("IVCurve_SpikeThreshold"))
        self.gridLayout_3.addWidget(self.IVCurve_SpikeThreshold, 10, 1, 1, 2)
        self.label_4 = QtGui.QLabel(self.groupBox)
        self.label_4.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName(_fromUtf8("label_4"))
        self.gridLayout_3.addWidget(self.label_4, 10, 0, 1, 1)
        self.IVCurve_tauh_Commands = QtGui.QComboBox(self.groupBox)
        self.IVCurve_tauh_Commands.setObjectName(_fromUtf8("IVCurve_tauh_Commands"))
        self.IVCurve_tauh_Commands.addItem(_fromUtf8(""))
        self.gridLayout_3.addWidget(self.IVCurve_tauh_Commands, 10, 4, 1, 2)
        self.label_16 = QtGui.QLabel(self.groupBox)
        self.label_16.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.gridLayout_3.addWidget(self.label_16, 10, 3, 1, 1)
        self.IVCurve_showHide_lrss = QtGui.QCheckBox(self.groupBox)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.IVCurve_showHide_lrss.setFont(font)
        self.IVCurve_showHide_lrss.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.IVCurve_showHide_lrss.setChecked(True)
        self.IVCurve_showHide_lrss.setObjectName(_fromUtf8("IVCurve_showHide_lrss"))
        self.gridLayout_3.addWidget(self.IVCurve_showHide_lrss, 5, 0, 1, 1)
        self.IVCurve_showHide_lrpk = QtGui.QCheckBox(self.groupBox)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.IVCurve_showHide_lrpk.setFont(font)
        self.IVCurve_showHide_lrpk.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.IVCurve_showHide_lrpk.setChecked(True)
        self.IVCurve_showHide_lrpk.setObjectName(_fromUtf8("IVCurve_showHide_lrpk"))
        self.gridLayout_3.addWidget(self.IVCurve_showHide_lrpk, 7, 0, 1, 1)
        self.IVCurve_showHide_lrtau = QtGui.QCheckBox(self.groupBox)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.IVCurve_showHide_lrtau.setFont(font)
        self.IVCurve_showHide_lrtau.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.IVCurve_showHide_lrtau.setAutoFillBackground(False)
        self.IVCurve_showHide_lrtau.setObjectName(_fromUtf8("IVCurve_showHide_lrtau"))
        self.gridLayout_3.addWidget(self.IVCurve_showHide_lrtau, 9, 0, 1, 1)
        self.label = QtGui.QLabel(self.groupBox)
        self.label.setAlignment(QtCore.Qt.AlignCenter)
        self.label.setObjectName(_fromUtf8("label"))
        self.gridLayout_3.addWidget(self.label, 2, 2, 1, 1)
        self.IVCurve_showHide_lrrmp = QtGui.QCheckBox(self.groupBox)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.IVCurve_showHide_lrrmp.setFont(font)
        self.IVCurve_showHide_lrrmp.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.IVCurve_showHide_lrrmp.setObjectName(_fromUtf8("IVCurve_showHide_lrrmp"))
        self.gridLayout_3.addWidget(self.IVCurve_showHide_lrrmp, 3, 0, 1, 1)
        self.IVCurve_rmpTStart = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_rmpTStart.setFont(font)
        self.IVCurve_rmpTStart.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_rmpTStart.setDecimals(2)
        self.IVCurve_rmpTStart.setMaximum(10000.0)
        self.IVCurve_rmpTStart.setObjectName(_fromUtf8("IVCurve_rmpTStart"))
        self.gridLayout_3.addWidget(self.IVCurve_rmpTStart, 3, 1, 1, 2)
        self.dbStoreBtn = QtGui.QPushButton(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.dbStoreBtn.setFont(font)
        self.dbStoreBtn.setObjectName(_fromUtf8("dbStoreBtn"))
        self.gridLayout_3.addWidget(self.dbStoreBtn, 14, 3, 1, 1)
        self.line = QtGui.QFrame(self.groupBox)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Expanding, QtGui.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.line.sizePolicy().hasHeightForWidth())
        self.line.setSizePolicy(sizePolicy)
        self.line.setFrameShape(QtGui.QFrame.HLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.gridLayout_3.addWidget(self.line, 12, 0, 1, 6)
        self.IVCurve_vrmp = QtGui.QLineEdit(self.groupBox)
        self.IVCurve_vrmp.setObjectName(_fromUtf8("IVCurve_vrmp"))
        self.gridLayout_3.addWidget(self.IVCurve_vrmp, 15, 2, 1, 1)
        self.label_11 = QtGui.QLabel(self.groupBox)
        self.label_11.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName(_fromUtf8("label_11"))
        self.gridLayout_3.addWidget(self.label_11, 19, 0, 1, 1)
        self.IVCurve_Rin = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_Rin.setFont(font)
        self.IVCurve_Rin.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_Rin.setObjectName(_fromUtf8("IVCurve_Rin"))
        self.gridLayout_3.addWidget(self.IVCurve_Rin, 16, 2, 1, 1)
        self.label_8 = QtGui.QLabel(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.label_8.setFont(font)
        self.label_8.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName(_fromUtf8("label_8"))
        self.gridLayout_3.addWidget(self.label_8, 15, 3, 1, 1)
        self.label_7 = QtGui.QLabel(self.groupBox)
        self.label_7.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_7.setObjectName(_fromUtf8("label_7"))
        self.gridLayout_3.addWidget(self.label_7, 15, 0, 1, 1)
        self.IVCurve_Tau = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_Tau.setFont(font)
        self.IVCurve_Tau.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_Tau.setObjectName(_fromUtf8("IVCurve_Tau"))
        self.gridLayout_3.addWidget(self.IVCurve_Tau, 18, 2, 1, 1)
        self.label_15 = QtGui.QLabel(self.groupBox)
        self.label_15.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.gridLayout_3.addWidget(self.label_15, 16, 3, 1, 1)
        self.label_2 = QtGui.QLabel(self.groupBox)
        self.label_2.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName(_fromUtf8("label_2"))
        self.gridLayout_3.addWidget(self.label_2, 16, 0, 1, 1)
        self.label_9 = QtGui.QLabel(self.groupBox)
        self.label_9.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_9.setObjectName(_fromUtf8("label_9"))
        self.gridLayout_3.addWidget(self.label_9, 18, 0, 1, 1)
        self.label_17 = QtGui.QLabel(self.groupBox)
        self.label_17.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.gridLayout_3.addWidget(self.label_17, 20, 0, 1, 1)
        self.IVCurve_AR = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_AR.setFont(font)
        self.IVCurve_AR.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_AR.setObjectName(_fromUtf8("IVCurve_AR"))
        self.gridLayout_3.addWidget(self.IVCurve_AR, 19, 2, 1, 1)
        self.IVCurve_FOType = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_FOType.setFont(font)
        self.IVCurve_FOType.setObjectName(_fromUtf8("IVCurve_FOType"))
        self.gridLayout_3.addWidget(self.IVCurve_FOType, 19, 5, 1, 1)
        self.IVCurve_Ih_ba = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_Ih_ba.setFont(font)
        self.IVCurve_Ih_ba.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_Ih_ba.setObjectName(_fromUtf8("IVCurve_Ih_ba"))
        self.gridLayout_3.addWidget(self.IVCurve_Ih_ba, 18, 5, 1, 1)
        self.label_12 = QtGui.QLabel(self.groupBox)
        self.label_12.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_12.setObjectName(_fromUtf8("label_12"))
        self.gridLayout_3.addWidget(self.label_12, 20, 3, 1, 1)
        self.IVCurve_Tauh = QtGui.QLineEdit(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_Tauh.setFont(font)
        self.IVCurve_Tauh.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_Tauh.setObjectName(_fromUtf8("IVCurve_Tauh"))
        self.gridLayout_3.addWidget(self.IVCurve_Tauh, 16, 5, 1, 1)
        self.IVCurve_pkAmp = QtGui.QLineEdit(self.groupBox)
        self.IVCurve_pkAmp.setObjectName(_fromUtf8("IVCurve_pkAmp"))
        self.gridLayout_3.addWidget(self.IVCurve_pkAmp, 20, 2, 1, 1)
        self.IVCurve_ssAmp = QtGui.QLineEdit(self.groupBox)
        self.IVCurve_ssAmp.setObjectName(_fromUtf8("IVCurve_ssAmp"))
        self.gridLayout_3.addWidget(self.IVCurve_ssAmp, 20, 5, 1, 1)
        self.label_6 = QtGui.QLabel(self.groupBox)
        self.label_6.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName(_fromUtf8("label_6"))
        self.gridLayout_3.addWidget(self.label_6, 19, 3, 1, 1)
        self.label_5 = QtGui.QLabel(self.groupBox)
        self.label_5.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName(_fromUtf8("label_5"))
        self.gridLayout_3.addWidget(self.label_5, 18, 3, 1, 1)
        self.IVCurve_Gh = QtGui.QLineEdit(self.groupBox)
        self.IVCurve_Gh.setObjectName(_fromUtf8("IVCurve_Gh"))
        self.gridLayout_3.addWidget(self.IVCurve_Gh, 15, 5, 1, 1)
        self.IVCurve_MPLExport = QtGui.QPushButton(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.IVCurve_MPLExport.setFont(font)
        self.IVCurve_MPLExport.setObjectName(_fromUtf8("IVCurve_MPLExport"))
        self.gridLayout_3.addWidget(self.IVCurve_MPLExport, 21, 5, 1, 1)
        self.IVCurve_subLeak = QtGui.QCheckBox(self.groupBox)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.IVCurve_subLeak.setFont(font)
        self.IVCurve_subLeak.setLayoutDirection(QtCore.Qt.RightToLeft)
        self.IVCurve_subLeak.setObjectName(_fromUtf8("IVCurve_subLeak"))
        self.gridLayout_3.addWidget(self.IVCurve_subLeak, 4, 0, 1, 1)
        self.IVCurve_LeakMin = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_LeakMin.setFont(font)
        self.IVCurve_LeakMin.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_LeakMin.setDecimals(1)
        self.IVCurve_LeakMin.setMinimum(-200.0)
        self.IVCurve_LeakMin.setMaximum(200.0)
        self.IVCurve_LeakMin.setProperty("value", -5.0)
        self.IVCurve_LeakMin.setObjectName(_fromUtf8("IVCurve_LeakMin"))
        self.gridLayout_3.addWidget(self.IVCurve_LeakMin, 4, 1, 1, 2)
        self.pushButton = QtGui.QPushButton(self.groupBox)
        self.pushButton.setObjectName(_fromUtf8("pushButton"))
        self.gridLayout_3.addWidget(self.pushButton, 21, 3, 1, 1)
        self.IVCurve_KeepT = QtGui.QCheckBox(self.groupBox)
        self.IVCurve_KeepT.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.IVCurve_KeepT.setObjectName(_fromUtf8("IVCurve_KeepT"))
        self.gridLayout_3.addWidget(self.IVCurve_KeepT, 21, 0, 1, 1)
        self.IVCurve_KeepAnalysis = QtGui.QCheckBox(self.groupBox)
        self.IVCurve_KeepAnalysis.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.IVCurve_KeepAnalysis.setObjectName(_fromUtf8("IVCurve_KeepAnalysis"))
        self.gridLayout_3.addWidget(self.IVCurve_KeepAnalysis, 21, 2, 1, 1)
        self.IVCurve_IVLimitMax = QtGui.QDoubleSpinBox(self.groupBox)
        self.IVCurve_IVLimitMax.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_IVLimitMax.setDecimals(1)
        self.IVCurve_IVLimitMax.setMinimum(-2000.0)
        self.IVCurve_IVLimitMax.setMaximum(2000.0)
        self.IVCurve_IVLimitMax.setSingleStep(5.0)
        self.IVCurve_IVLimitMax.setProperty("value", 100.0)
        self.IVCurve_IVLimitMax.setObjectName(_fromUtf8("IVCurve_IVLimitMax"))
        self.gridLayout_3.addWidget(self.IVCurve_IVLimitMax, 0, 5, 1, 1)
        self.IVCurve_IVLimitMin = QtGui.QDoubleSpinBox(self.groupBox)
        self.IVCurve_IVLimitMin.setDecimals(1)
        self.IVCurve_IVLimitMin.setMinimum(-2000.0)
        self.IVCurve_IVLimitMin.setMaximum(2000.0)
        self.IVCurve_IVLimitMin.setSingleStep(5.0)
        self.IVCurve_IVLimitMin.setProperty("value", -160.0)
        self.IVCurve_IVLimitMin.setObjectName(_fromUtf8("IVCurve_IVLimitMin"))
        self.gridLayout_3.addWidget(self.IVCurve_IVLimitMin, 0, 3, 1, 1)
        self.IVCurve_IVLimits = QtGui.QCheckBox(self.groupBox)
        self.IVCurve_IVLimits.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.IVCurve_IVLimits.setObjectName(_fromUtf8("IVCurve_IVLimits"))
        self.gridLayout_3.addWidget(self.IVCurve_IVLimits, 0, 2, 1, 1)
        self.label_13 = QtGui.QLabel(self.groupBox)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.gridLayout_3.addWidget(self.label_13, 0, 0, 1, 1)
        self.IVCurve_Sequence1 = QtGui.QComboBox(self.groupBox)
        self.IVCurve_Sequence1.setObjectName(_fromUtf8("IVCurve_Sequence1"))
        self.IVCurve_Sequence1.addItem(_fromUtf8(""))
        self.IVCurve_Sequence1.addItem(_fromUtf8(""))
        self.IVCurve_Sequence1.addItem(_fromUtf8(""))
        self.IVCurve_Sequence1.addItem(_fromUtf8(""))
        self.IVCurve_Sequence1.addItem(_fromUtf8(""))
        self.gridLayout_3.addWidget(self.IVCurve_Sequence1, 1, 2, 1, 1)
        self.label_14 = QtGui.QLabel(self.groupBox)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.gridLayout_3.addWidget(self.label_14, 1, 0, 1, 1)
        self.IVCurve_tau2TStop = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_tau2TStop.setFont(font)
        self.IVCurve_tau2TStop.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_tau2TStop.setMaximum(5000.0)
        self.IVCurve_tau2TStop.setObjectName(_fromUtf8("IVCurve_tau2TStop"))
        self.gridLayout_3.addWidget(self.IVCurve_tau2TStop, 9, 3, 1, 1)
        self.IVCurve_pkTStop = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_pkTStop.setFont(font)
        self.IVCurve_pkTStop.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_pkTStop.setMinimum(-5000.0)
        self.IVCurve_pkTStop.setMaximum(50000.0)
        self.IVCurve_pkTStop.setObjectName(_fromUtf8("IVCurve_pkTStop"))
        self.gridLayout_3.addWidget(self.IVCurve_pkTStop, 7, 3, 1, 1)
        self.IVCurve_ssTStart = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_ssTStart.setFont(font)
        self.IVCurve_ssTStart.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_ssTStart.setMinimum(-5000.0)
        self.IVCurve_ssTStart.setMaximum(50000.0)
        self.IVCurve_ssTStart.setObjectName(_fromUtf8("IVCurve_ssTStart"))
        self.gridLayout_3.addWidget(self.IVCurve_ssTStart, 5, 3, 1, 1)
        self.IVCurve_LeakMax = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_LeakMax.setFont(font)
        self.IVCurve_LeakMax.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_LeakMax.setDecimals(1)
        self.IVCurve_LeakMax.setMinimum(-200.0)
        self.IVCurve_LeakMax.setMaximum(203.0)
        self.IVCurve_LeakMax.setProperty("value", 5.0)
        self.IVCurve_LeakMax.setObjectName(_fromUtf8("IVCurve_LeakMax"))
        self.gridLayout_3.addWidget(self.IVCurve_LeakMax, 4, 3, 1, 1)
        self.IVCurve_rmpTStop = QtGui.QDoubleSpinBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_rmpTStop.setFont(font)
        self.IVCurve_rmpTStop.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IVCurve_rmpTStop.setMaximum(10000.0)
        self.IVCurve_rmpTStop.setObjectName(_fromUtf8("IVCurve_rmpTStop"))
        self.gridLayout_3.addWidget(self.IVCurve_rmpTStop, 3, 3, 1, 1)
        self.label_3 = QtGui.QLabel(self.groupBox)
        self.label_3.setAlignment(QtCore.Qt.AlignCenter)
        self.label_3.setObjectName(_fromUtf8("label_3"))
        self.gridLayout_3.addWidget(self.label_3, 2, 3, 1, 1)
        self.IVCurve_SubRMP = QtGui.QCheckBox(self.groupBox)
        self.IVCurve_SubRMP.setObjectName(_fromUtf8("IVCurve_SubRMP"))
        self.gridLayout_3.addWidget(self.IVCurve_SubRMP, 3, 5, 1, 1)
        self.label_18 = QtGui.QLabel(self.groupBox)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.gridLayout_3.addWidget(self.label_18, 4, 5, 1, 1)
        self.IVCurve_RMPMode = QtGui.QComboBox(self.groupBox)
        font = QtGui.QFont()
        font.setPointSize(12)
        self.IVCurve_RMPMode.setFont(font)
        self.IVCurve_RMPMode.setObjectName(_fromUtf8("IVCurve_RMPMode"))
        self.IVCurve_RMPMode.addItem(_fromUtf8(""))
        self.IVCurve_RMPMode.addItem(_fromUtf8(""))
        self.IVCurve_RMPMode.addItem(_fromUtf8(""))
        self.gridLayout_3.addWidget(self.IVCurve_RMPMode, 5, 5, 1, 1)
        self.label_19 = QtGui.QLabel(self.groupBox)
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.gridLayout_3.addWidget(self.label_19, 1, 3, 1, 1)
        self.IVCurve_Sequence2 = QtGui.QComboBox(self.groupBox)
        self.IVCurve_Sequence2.setObjectName(_fromUtf8("IVCurve_Sequence2"))
        self.IVCurve_Sequence2.addItem(_fromUtf8(""))
        self.gridLayout_3.addWidget(self.IVCurve_Sequence2, 1, 5, 1, 1)
        self.IVCurve_dataMode = QtGui.QLabel(self.groupBox)
        self.IVCurve_dataMode.setObjectName(_fromUtf8("IVCurve_dataMode"))
        self.gridLayout_3.addWidget(self.IVCurve_dataMode, 2, 0, 1, 1)
        self.IVCurve_getFileInfo = QtGui.QPushButton(self.groupBox)
        self.IVCurve_getFileInfo.setObjectName(_fromUtf8("IVCurve_getFileInfo"))
        self.gridLayout_3.addWidget(self.IVCurve_getFileInfo, 2, 5, 1, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(_translate("Form", "Form", None))
        self.groupBox.setTitle(_translate("Form", "IV Analysis-V1.0", None))
        self.label_10.setText(_translate("Form", "Results", None))
        self.IVCurve_PrintResults.setText(_translate("Form", "Print", None))
        self.IVCurve_Update.setText(_translate("Form", "Update", None))
        self.IVCurve_SpikeThreshold.setSuffix(_translate("Form", " mV", None))
        self.label_4.setText(_translate("Form", "Spike Thr", None))
        self.IVCurve_tauh_Commands.setItemText(0, _translate("Form", "-0.6", None))
        self.label_16.setText(_translate("Form", "Command", None))
        self.IVCurve_showHide_lrss.setText(_translate("Form", "IV:SS", None))
        self.IVCurve_showHide_lrpk.setText(_translate("Form", "IV:Peak", None))
        self.IVCurve_showHide_lrtau.setText(_translate("Form", "Ih tool", None))
        self.label.setText(_translate("Form", "T Start", None))
        self.IVCurve_showHide_lrrmp.setText(_translate("Form", "IV:RMP", None))
        self.dbStoreBtn.setText(_translate("Form", "-> db", None))
        self.label_11.setText(_translate("Form", "Adapt \n"
"Ratio", None))
        self.label_8.setText(_translate("Form", "gH", None))
        self.label_7.setText(_translate("Form", "RMP/I<sub>0</sub>", None))
        self.label_15.setText(_translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:12pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">&tau;<span style=\" vertical-align:sub;\">h</span> (ms)</p></body></html>", None))
        self.label_2.setText(_translate("Form", "R<sub>in</sub>", None))
        self.label_9.setText(_translate("Form", "&tau;<sub>m</sub> (ms)", None))
        self.label_17.setText(_translate("Form", "Pk Amp", None))
        self.label_12.setText(_translate("Form", "SS Amp", None))
        self.label_6.setText(_translate("Form", "F&O Type", None))
        self.label_5.setText(_translate("Form", "b/a (%)", None))
        self.IVCurve_MPLExport.setText(_translate("Form", "MPL Export", None))
        self.IVCurve_subLeak.setText(_translate("Form", "IV:Leak", None))
        self.pushButton.setText(_translate("Form", "Reset", None))
        self.IVCurve_KeepT.setText(_translate("Form", "Keep\n"
"Times", None))
        self.IVCurve_KeepAnalysis.setText(_translate("Form", "Keep \n"
"Analysis", None))
        self.IVCurve_IVLimits.setText(_translate("Form", "Limits", None))
        self.label_13.setText(_translate("Form", "IV Limits", None))
        self.IVCurve_Sequence1.setItemText(0, _translate("Form", "None", None))
        self.IVCurve_Sequence1.setItemText(1, _translate("Form", "001", None))
        self.IVCurve_Sequence1.setItemText(2, _translate("Form", "002", None))
        self.IVCurve_Sequence1.setItemText(3, _translate("Form", "003", None))
        self.IVCurve_Sequence1.setItemText(4, _translate("Form", "004", None))
        self.label_14.setText(_translate("Form", "Seq #1", None))
        self.label_3.setText(_translate("Form", "T Stop", None))
        self.IVCurve_SubRMP.setText(_translate("Form", "Sub RMP", None))
        self.label_18.setText(_translate("Form", "Abcissa", None))
        self.IVCurve_RMPMode.setItemText(0, _translate("Form", "T (s)", None))
        self.IVCurve_RMPMode.setItemText(1, _translate("Form", "I (pA)", None))
        self.IVCurve_RMPMode.setItemText(2, _translate("Form", "Sp (#/s)", None))
        self.label_19.setText(_translate("Form", "Seq #2", None))
        self.IVCurve_Sequence2.setItemText(0, _translate("Form", "None", None))
        self.IVCurve_dataMode.setText(_translate("Form", "DataMode", None))
        self.IVCurve_getFileInfo.setText(_translate("Form", "FileInfo", None))

