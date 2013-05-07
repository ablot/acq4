# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file './lib/analysis/modules/pbm_ImageAnalysis/ctrlTemplateAnalysis.ui'
#
# Created: Tue Jan 15 15:29:34 2013
#      by: PyQt4 UI code generator 4.9.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    _fromUtf8 = lambda s: s

class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName(_fromUtf8("Form"))
        Form.resize(400, 410)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(12)
        Form.setFont(font)
        self.gridLayout = QtGui.QGridLayout(Form)
        self.gridLayout.setMargin(0)
        self.gridLayout.setHorizontalSpacing(10)
        self.gridLayout.setVerticalSpacing(0)
        self.gridLayout.setObjectName(_fromUtf8("gridLayout"))
        self.groupBox = QtGui.QGroupBox(Form)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy)
        self.groupBox.setMinimumSize(QtCore.QSize(0, 380))
        self.groupBox.setAlignment(QtCore.Qt.AlignCenter)
        self.groupBox.setObjectName(_fromUtf8("groupBox"))
        self.gridLayout_2 = QtGui.QGridLayout(self.groupBox)
        self.gridLayout_2.setSizeConstraint(QtGui.QLayout.SetNoConstraint)
        self.gridLayout_2.setMargin(0)
        self.gridLayout_2.setHorizontalSpacing(5)
        self.gridLayout_2.setVerticalSpacing(0)
        self.gridLayout_2.setObjectName(_fromUtf8("gridLayout_2"))
        self.line = QtGui.QFrame(self.groupBox)
        self.line.setFrameShape(QtGui.QFrame.VLine)
        self.line.setFrameShadow(QtGui.QFrame.Sunken)
        self.line.setObjectName(_fromUtf8("line"))
        self.gridLayout_2.addWidget(self.line, 1, 1, 2, 1)
        self.line_2 = QtGui.QFrame(self.groupBox)
        self.line_2.setFrameShape(QtGui.QFrame.HLine)
        self.line_2.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_2.setObjectName(_fromUtf8("line_2"))
        self.gridLayout_2.addWidget(self.line_2, 0, 0, 1, 1)
        self.IAFuncs_widget = QtGui.QWidget(self.groupBox)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.IAFuncs_widget.sizePolicy().hasHeightForWidth())
        self.IAFuncs_widget.setSizePolicy(sizePolicy)
        self.IAFuncs_widget.setObjectName(_fromUtf8("IAFuncs_widget"))
        self.IAFuncs_checkbox_TraceLabels = QtGui.QCheckBox(self.IAFuncs_widget)
        self.IAFuncs_checkbox_TraceLabels.setGeometry(QtCore.QRect(10, 360, 83, 18))
        font = QtGui.QFont()
        font.setPointSize(10)
        self.IAFuncs_checkbox_TraceLabels.setFont(font)
        self.IAFuncs_checkbox_TraceLabels.setChecked(True)
        self.IAFuncs_checkbox_TraceLabels.setObjectName(_fromUtf8("IAFuncs_checkbox_TraceLabels"))
        self.widget = QtGui.QWidget(self.IAFuncs_widget)
        self.widget.setGeometry(QtCore.QRect(180, 0, 146, 371))
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)
        self.widget.setObjectName(_fromUtf8("widget"))
        self.line_4 = QtGui.QFrame(self.widget)
        self.line_4.setGeometry(QtCore.QRect(10, 245, 131, 20))
        self.line_4.setLineWidth(2)
        self.line_4.setFrameShape(QtGui.QFrame.HLine)
        self.line_4.setFrameShadow(QtGui.QFrame.Sunken)
        self.line_4.setObjectName(_fromUtf8("line_4"))
        self.IAFuncs_Analysis_AFFT_Individual = QtGui.QPushButton(self.widget)
        self.IAFuncs_Analysis_AFFT_Individual.setGeometry(QtCore.QRect(25, 300, 101, 32))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_AFFT_Individual.setFont(font)
        self.IAFuncs_Analysis_AFFT_Individual.setObjectName(_fromUtf8("IAFuncs_Analysis_AFFT_Individual"))
        self.IAFuncs_Analysis_FourierMap = QtGui.QPushButton(self.widget)
        self.IAFuncs_Analysis_FourierMap.setGeometry(QtCore.QRect(35, 330, 81, 32))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_FourierMap.setFont(font)
        self.IAFuncs_Analysis_FourierMap.setObjectName(_fromUtf8("IAFuncs_Analysis_FourierMap"))
        self.IAFuncs_Analysis_AFFT = QtGui.QPushButton(self.widget)
        self.IAFuncs_Analysis_AFFT.setGeometry(QtCore.QRect(30, 270, 91, 32))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_AFFT.setFont(font)
        self.IAFuncs_Analysis_AFFT.setText(_fromUtf8("Avg FFT"))
        self.IAFuncs_Analysis_AFFT.setObjectName(_fromUtf8("IAFuncs_Analysis_AFFT"))
        self.label_15 = QtGui.QLabel(self.widget)
        self.label_15.setGeometry(QtCore.QRect(10, 10, 136, 16))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_15.setFont(font)
        self.label_15.setObjectName(_fromUtf8("label_15"))
        self.label_13 = QtGui.QLabel(self.widget)
        self.label_13.setGeometry(QtCore.QRect(30, 255, 96, 21))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        font.setBold(True)
        font.setWeight(75)
        self.label_13.setFont(font)
        self.label_13.setObjectName(_fromUtf8("label_13"))
        self.IAFuncs_Analysis_smcAnalyze = QtGui.QPushButton(self.widget)
        self.IAFuncs_Analysis_smcAnalyze.setGeometry(QtCore.QRect(35, 175, 71, 32))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_smcAnalyze.setFont(font)
        self.IAFuncs_Analysis_smcAnalyze.setObjectName(_fromUtf8("IAFuncs_Analysis_smcAnalyze"))
        self.layoutWidget = QtGui.QWidget(self.widget)
        self.layoutWidget.setGeometry(QtCore.QRect(11, 30, 126, 140))
        self.layoutWidget.setObjectName(_fromUtf8("layoutWidget"))
        self.formLayout = QtGui.QFormLayout(self.layoutWidget)
        self.formLayout.setMargin(0)
        self.formLayout.setObjectName(_fromUtf8("formLayout"))
        self.horizontalLayout_2 = QtGui.QHBoxLayout()
        self.horizontalLayout_2.setObjectName(_fromUtf8("horizontalLayout_2"))
        self.label_18 = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.label_18.setFont(font)
        self.label_18.setObjectName(_fromUtf8("label_18"))
        self.horizontalLayout_2.addWidget(self.label_18)
        self.smc_Kd = QtGui.QDoubleSpinBox(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.smc_Kd.setFont(font)
        self.smc_Kd.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.smc_Kd.setDecimals(0)
        self.smc_Kd.setMinimum(0.0)
        self.smc_Kd.setMaximum(5000.0)
        self.smc_Kd.setProperty("value", 345.0)
        self.smc_Kd.setObjectName(_fromUtf8("smc_Kd"))
        self.horizontalLayout_2.addWidget(self.smc_Kd)
        self.formLayout.setLayout(1, QtGui.QFormLayout.SpanningRole, self.horizontalLayout_2)
        self.horizontalLayout_3 = QtGui.QHBoxLayout()
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.label_19 = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.label_19.setFont(font)
        self.label_19.setObjectName(_fromUtf8("label_19"))
        self.horizontalLayout_3.addWidget(self.label_19)
        self.smc_TCa = QtGui.QDoubleSpinBox(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.smc_TCa.setFont(font)
        self.smc_TCa.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.smc_TCa.setDecimals(3)
        self.smc_TCa.setMinimum(0.001)
        self.smc_TCa.setMaximum(5.0)
        self.smc_TCa.setSingleStep(0.01)
        self.smc_TCa.setProperty("value", 0.025)
        self.smc_TCa.setObjectName(_fromUtf8("smc_TCa"))
        self.horizontalLayout_3.addWidget(self.smc_TCa)
        self.formLayout.setLayout(2, QtGui.QFormLayout.SpanningRole, self.horizontalLayout_3)
        self.horizontalLayout_4 = QtGui.QHBoxLayout()
        self.horizontalLayout_4.setObjectName(_fromUtf8("horizontalLayout_4"))
        self.label_17 = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.label_17.setFont(font)
        self.label_17.setObjectName(_fromUtf8("label_17"))
        self.horizontalLayout_4.addWidget(self.label_17)
        self.smc_C0 = QtGui.QDoubleSpinBox(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.smc_C0.setFont(font)
        self.smc_C0.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.smc_C0.setDecimals(3)
        self.smc_C0.setMinimum(0.01)
        self.smc_C0.setMaximum(1000.0)
        self.smc_C0.setObjectName(_fromUtf8("smc_C0"))
        self.horizontalLayout_4.addWidget(self.smc_C0)
        self.formLayout.setLayout(3, QtGui.QFormLayout.SpanningRole, self.horizontalLayout_4)
        self.horizontalLayout = QtGui.QHBoxLayout()
        self.horizontalLayout.setObjectName(_fromUtf8("horizontalLayout"))
        self.label_16 = QtGui.QLabel(self.layoutWidget)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.label_16.setFont(font)
        self.label_16.setObjectName(_fromUtf8("label_16"))
        self.horizontalLayout.addWidget(self.label_16)
        self.smc_Amplitude = QtGui.QDoubleSpinBox(self.layoutWidget)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.smc_Amplitude.setFont(font)
        self.smc_Amplitude.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.smc_Amplitude.setMaximum(1000.0)
        self.smc_Amplitude.setSingleStep(0.1)
        self.smc_Amplitude.setProperty("value", 1.0)
        self.smc_Amplitude.setObjectName(_fromUtf8("smc_Amplitude"))
        self.horizontalLayout.addWidget(self.smc_Amplitude)
        self.formLayout.setLayout(0, QtGui.QFormLayout.SpanningRole, self.horizontalLayout)
        self.IAFuncs_Analysis_SpikeXCorr = QtGui.QPushButton(self.widget)
        self.IAFuncs_Analysis_SpikeXCorr.setGeometry(QtCore.QRect(10, 200, 126, 32))
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_SpikeXCorr.setFont(font)
        self.IAFuncs_Analysis_SpikeXCorr.setObjectName(_fromUtf8("IAFuncs_Analysis_SpikeXCorr"))
        self.widget1 = QtGui.QWidget(self.IAFuncs_widget)
        self.widget1.setGeometry(QtCore.QRect(10, 110, 143, 221))
        self.widget1.setObjectName(_fromUtf8("widget1"))
        self.formLayout_4 = QtGui.QFormLayout(self.widget1)
        self.formLayout_4.setFieldGrowthPolicy(QtGui.QFormLayout.FieldsStayAtSizeHint)
        self.formLayout_4.setMargin(0)
        self.formLayout_4.setSpacing(0)
        self.formLayout_4.setObjectName(_fromUtf8("formLayout_4"))
        self.IAFuncs_Analysis_AXCorr_Individual = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_AXCorr_Individual.setFont(font)
        self.IAFuncs_Analysis_AXCorr_Individual.setObjectName(_fromUtf8("IAFuncs_Analysis_AXCorr_Individual"))
        self.formLayout_4.setWidget(1, QtGui.QFormLayout.SpanningRole, self.IAFuncs_Analysis_AXCorr_Individual)
        self.IAFuncs_Analysis_UnbiasedXC = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_UnbiasedXC.setFont(font)
        self.IAFuncs_Analysis_UnbiasedXC.setObjectName(_fromUtf8("IAFuncs_Analysis_UnbiasedXC"))
        self.formLayout_4.setWidget(2, QtGui.QFormLayout.SpanningRole, self.IAFuncs_Analysis_UnbiasedXC)
        self.IAFuncs_Distance = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Distance.setFont(font)
        self.IAFuncs_Distance.setObjectName(_fromUtf8("IAFuncs_Distance"))
        self.formLayout_4.setWidget(3, QtGui.QFormLayout.SpanningRole, self.IAFuncs_Distance)
        self.IAFuncs_DistanceStrength = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_DistanceStrength.setFont(font)
        self.IAFuncs_DistanceStrength.setObjectName(_fromUtf8("IAFuncs_DistanceStrength"))
        self.formLayout_4.setWidget(4, QtGui.QFormLayout.SpanningRole, self.IAFuncs_DistanceStrength)
        self.formLayout_3 = QtGui.QFormLayout()
        self.formLayout_3.setObjectName(_fromUtf8("formLayout_3"))
        self.label = QtGui.QLabel(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))
        self.formLayout_3.setWidget(0, QtGui.QFormLayout.LabelRole, self.label)
        self.IAFuncs_XCorrThreshold = QtGui.QDoubleSpinBox(self.widget1)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.IAFuncs_XCorrThreshold.setFont(font)
        self.IAFuncs_XCorrThreshold.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.IAFuncs_XCorrThreshold.setMaximum(1.0)
        self.IAFuncs_XCorrThreshold.setSingleStep(0.02)
        self.IAFuncs_XCorrThreshold.setProperty("value", 0.25)
        self.IAFuncs_XCorrThreshold.setObjectName(_fromUtf8("IAFuncs_XCorrThreshold"))
        self.formLayout_3.setWidget(0, QtGui.QFormLayout.FieldRole, self.IAFuncs_XCorrThreshold)
        self.formLayout_4.setLayout(5, QtGui.QFormLayout.SpanningRole, self.formLayout_3)
        self.IAFuncs_NetworkGraph = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_NetworkGraph.setFont(font)
        self.IAFuncs_NetworkGraph.setObjectName(_fromUtf8("IAFuncs_NetworkGraph"))
        self.formLayout_4.setWidget(6, QtGui.QFormLayout.SpanningRole, self.IAFuncs_NetworkGraph)
        self.IAFuncs_DistanceStrengthPrint = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_DistanceStrengthPrint.setFont(font)
        self.IAFuncs_DistanceStrengthPrint.setObjectName(_fromUtf8("IAFuncs_DistanceStrengthPrint"))
        self.formLayout_4.setWidget(7, QtGui.QFormLayout.SpanningRole, self.IAFuncs_DistanceStrengthPrint)
        self.IAFuncs_MatplotlibCheckBox = QtGui.QCheckBox(self.widget1)
        font = QtGui.QFont()
        font.setPointSize(11)
        self.IAFuncs_MatplotlibCheckBox.setFont(font)
        self.IAFuncs_MatplotlibCheckBox.setObjectName(_fromUtf8("IAFuncs_MatplotlibCheckBox"))
        self.formLayout_4.setWidget(8, QtGui.QFormLayout.FieldRole, self.IAFuncs_MatplotlibCheckBox)
        self.IAFuncs_Analysis_AXCorr = QtGui.QPushButton(self.widget1)
        font = QtGui.QFont()
        font.setFamily(_fromUtf8("Arial"))
        font.setPointSize(11)
        self.IAFuncs_Analysis_AXCorr.setFont(font)
        self.IAFuncs_Analysis_AXCorr.setObjectName(_fromUtf8("IAFuncs_Analysis_AXCorr"))
        self.formLayout_4.setWidget(0, QtGui.QFormLayout.SpanningRole, self.IAFuncs_Analysis_AXCorr)
        self.widget2 = QtGui.QWidget(self.IAFuncs_widget)
        self.widget2.setGeometry(QtCore.QRect(5, 12, 146, 92))
        self.widget2.setObjectName(_fromUtf8("widget2"))
        self.formLayout_2 = QtGui.QFormLayout(self.widget2)
        self.formLayout_2.setMargin(0)
        self.formLayout_2.setObjectName(_fromUtf8("formLayout_2"))
        self.label_14 = QtGui.QLabel(self.widget2)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.label_14.setFont(font)
        self.label_14.setObjectName(_fromUtf8("label_14"))
        self.formLayout_2.setWidget(0, QtGui.QFormLayout.LabelRole, self.label_14)
        self.IAFuncs_DigitalRadioBtn = QtGui.QRadioButton(self.widget2)
        self.IAFuncs_DigitalRadioBtn.setObjectName(_fromUtf8("IAFuncs_DigitalRadioBtn"))
        self.formLayout_2.setWidget(2, QtGui.QFormLayout.LabelRole, self.IAFuncs_DigitalRadioBtn)
        self.IAFuncs_AnalogRadioBtn = QtGui.QRadioButton(self.widget2)
        self.IAFuncs_AnalogRadioBtn.setChecked(True)
        self.IAFuncs_AnalogRadioBtn.setObjectName(_fromUtf8("IAFuncs_AnalogRadioBtn"))
        self.formLayout_2.setWidget(1, QtGui.QFormLayout.LabelRole, self.IAFuncs_AnalogRadioBtn)
        self.IAFuncs_GetCSVFile = QtGui.QPushButton(self.widget2)
        self.IAFuncs_GetCSVFile.setObjectName(_fromUtf8("IAFuncs_GetCSVFile"))
        self.formLayout_2.setWidget(3, QtGui.QFormLayout.LabelRole, self.IAFuncs_GetCSVFile)
        self.gridLayout_2.addWidget(self.IAFuncs_widget, 1, 0, 2, 1)
        self.gridLayout.addWidget(self.groupBox, 0, 0, 1, 1)

        self.retranslateUi(Form)
        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        Form.setWindowTitle(QtGui.QApplication.translate("Form", "Form", None, QtGui.QApplication.UnicodeUTF8))
        self.groupBox.setTitle(QtGui.QApplication.translate("Form", "Imaging Analysis Functions", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_checkbox_TraceLabels.setText(QtGui.QApplication.translate("Form", "Trace Labels", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AFFT_Individual.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">For Intrinsic Imaging: Compute the FFT of each image.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AFFT_Individual.setText(QtGui.QApplication.translate("Form", "Individual FFT", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_FourierMap.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">For Intrinsic Imaging: Compute the Fourier Phase map relative the the stimulus (Valatsky et al., 2003)</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_FourierMap.setText(QtGui.QApplication.translate("Form", "Fourier Map", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AFFT.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">For Intrinsic Imaging: Compute the average FFT of images. </p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.label_15.setText(QtGui.QApplication.translate("Form", "SMC-Calcium Event Detection", None, QtGui.QApplication.UnicodeUTF8))
        self.label_13.setText(QtGui.QApplication.translate("Form", "Intrinsic Imaging", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_smcAnalyze.setText(QtGui.QApplication.translate("Form", "SMC", None, QtGui.QApplication.UnicodeUTF8))
        self.label_18.setText(QtGui.QApplication.translate("Form", "Kd (nM)", None, QtGui.QApplication.UnicodeUTF8))
        self.label_19.setText(QtGui.QApplication.translate("Form", "TCa  (sec)", None, QtGui.QApplication.UnicodeUTF8))
        self.label_17.setText(QtGui.QApplication.translate("Form", "C0 (uM)", None, QtGui.QApplication.UnicodeUTF8))
        self.label_16.setText(QtGui.QApplication.translate("Form", "A (uM)", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_SpikeXCorr.setText(QtGui.QApplication.translate("Form", "Digital Xcorr", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AXCorr_Individual.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Compute the cross-correlation between each pair of ROI\'s. The computation is done on the analog waveform.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AXCorr_Individual.setText(QtGui.QApplication.translate("Form", "Individual XCorr", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_UnbiasedXC.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Compute the cross-correlation between each pair of ROI\'s. The computation is done on the analog waveform.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_UnbiasedXC.setText(QtGui.QApplication.translate("Form", "Unbiased XCorr (slow)", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Distance.setText(QtGui.QApplication.translate("Form", "ROI Distances", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_DistanceStrength.setText(QtGui.QApplication.translate("Form", "Distance-Strength", None, QtGui.QApplication.UnicodeUTF8))
        self.label.setText(QtGui.QApplication.translate("Form", "XCorr Thr", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_NetworkGraph.setText(QtGui.QApplication.translate("Form", "Network Graph", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_DistanceStrengthPrint.setText(QtGui.QApplication.translate("Form", "Print Dist-Strength", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_MatplotlibCheckBox.setText(QtGui.QApplication.translate("Form", "MatPlotLib (slower)", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AXCorr.setToolTip(QtGui.QApplication.translate("Form", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:\'Lucida Grande\'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">Compute the average cross-correlation across all pairs of ROIs in the field over time. The computation is done on the analog waveform.</p></body></html>", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_Analysis_AXCorr.setText(QtGui.QApplication.translate("Form", "Average XCorr", None, QtGui.QApplication.UnicodeUTF8))
        self.label_14.setText(QtGui.QApplication.translate("Form", "Correlation Mode", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_DigitalRadioBtn.setText(QtGui.QApplication.translate("Form", "CSV (digital)", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_AnalogRadioBtn.setText(QtGui.QApplication.translate("Form", "Analog", None, QtGui.QApplication.UnicodeUTF8))
        self.IAFuncs_GetCSVFile.setText(QtGui.QApplication.translate("Form", "Select CSV File", None, QtGui.QApplication.UnicodeUTF8))

