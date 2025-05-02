# FourBar_app.py

#region imports
from FourBar_GUI import Ui_Form
from FourBarLinkage_MVC import FourBarLinkage_Controller
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw
import sys
#endregion

class MainWindow(Ui_Form, qtw.QWidget):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # core linkage setup
        widgets = [
            self.gv_Main,
            self.nud_InputAngle,
            self.lbl_OutputAngle_Val,
            self.nud_Link1Length,
            self.nud_Link3Length,
            self.spnd_Zoom
        ]
        self.FBL_C = FourBarLinkage_Controller(widgets)
        self.FBL_C.setupGraphics()
        self.gv_Main.setMouseTracking(True)
        self.setMouseTracking(True)
        self.FBL_C.buildScene()
        self.lbl_OutputAngle_Val.setText(f"{self.FBL_C.FBL_M.OutputLink.AngleDeg():.3f}")
        self.nud_Link1Length.setValue(self.FBL_C.FBL_M.InputLink.length)
        self.nud_Link3Length.setValue(self.FBL_C.FBL_M.OutputLink.length)

        # ── angle‐limit controls ──
        self.lbl_MinAngle = qtw.QLabel("Min Angle (°)")
        self.nud_MinAngle = qtw.QDoubleSpinBox()
        self.nud_MinAngle.setRange(0, 360)
        self.nud_MinAngle.setValue(0)
        self.lbl_MaxAngle = qtw.QLabel("Max Angle (°)")
        self.nud_MaxAngle = qtw.QDoubleSpinBox()
        self.nud_MaxAngle.setRange(0, 360)
        self.nud_MaxAngle.setValue(360)

        for w in (self.lbl_MinAngle, self.nud_MinAngle, self.lbl_MaxAngle, self.nud_MaxAngle):
            self.horizontalLayout.addWidget(w)

        # ── simulation parameter controls ──
        self.lbl_MassInput = qtw.QLabel("Mass (kg)")
        self.nud_MassInput = qtw.QDoubleSpinBox()
        self.nud_MassInput.setRange(0.1, 100)
        self.nud_MassInput.setValue(10)
        self.lbl_SpringK = qtw.QLabel("Spring k (N/m)")
        self.nud_SpringK = qtw.QDoubleSpinBox()
        self.nud_SpringK.setRange(0, 1e3)
        self.nud_SpringK.setValue(self.FBL_C.FBL_M.Spring.k)
        self.lbl_DampC = qtw.QLabel("Damp c (N·s/m)")
        self.nud_DampC = qtw.QDoubleSpinBox()
        self.nud_DampC.setRange(0, 1e3)
        self.nud_DampC.setValue(self.FBL_C.FBL_M.DashPot.c)
        self.btn_RunSim = qtw.QPushButton("Run Simulation")

        for w in (
            self.lbl_MassInput, self.nud_MassInput,
            self.lbl_SpringK,   self.nud_SpringK,
            self.lbl_DampC,     self.nud_DampC,
            self.btn_RunSim
        ):
            self.horizontalLayout.addWidget(w)

        # ── signals/slots ──
        self.spnd_Zoom.valueChanged.connect(self.setZoom)
        self.nud_Link1Length.valueChanged.connect(self.setInputLinkLength)
        self.nud_Link3Length.valueChanged.connect(self.setOutputLinkLength)

        self.nud_MinAngle.valueChanged.connect(
            lambda _: self.FBL_C.setAngleLimits(self.nud_MinAngle.value(), self.nud_MaxAngle.value())
        )
        self.nud_MaxAngle.valueChanged.connect(
            lambda _: self.FBL_C.setAngleLimits(self.nud_MinAngle.value(), self.nud_MaxAngle.value())
        )

        self.btn_RunSim.clicked.connect(
            lambda: self.FBL_C.runSimulation(
                massInput = self.nud_MassInput.value(),
                k_spring  = self.nud_SpringK.value(),
                c_damper  = self.nud_DampC.value()
            )
        )

        self.FBL_C.FBL_V.scene.installEventFilter(self)
        self.mouseDown = False
        self.show()

    def setInputLinkLength(self):
        self.FBL_C.setInputLinkLength()

    def setOutputLinkLength(self):
        self.FBL_C.setOutputLinkLength()

    def eventFilter(self, obj, event):
        if obj == self.FBL_C.FBL_V.scene:
            et = event.type()
            if et == qtc.QEvent.GraphicsSceneMouseMove and self.mouseDown:
                self.FBL_C.moveLinkage(event.scenePos())
            elif et == qtc.QEvent.GraphicsSceneMousePress and event.button()==qtc.Qt.LeftButton:
                self.mouseDown = True
            elif et == qtc.QEvent.GraphicsSceneMouseRelease:
                self.mouseDown = False
        return super().eventFilter(obj, event)

    def setZoom(self):
        self.gv_Main.resetTransform()
        self.gv_Main.scale(self.spnd_Zoom.value(), self.spnd_Zoom.value())

if __name__ == '__main__':
    app = qtw.QApplication(sys.argv)
    mw = MainWindow()
    mw.setWindowTitle('Four Bar Linkage')
    sys.exit(app.exec())
