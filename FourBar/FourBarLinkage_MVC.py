# FourBarLinkage_MVC.py
#region imports
import PyQt5.QtGui as qtg, PyQt5.QtCore as qtc, PyQt5.QtWidgets as qtw
import math, time, numpy as np
from scipy import optimize, integrate
from copy import deepcopy as dc
#endregion

# ── (RigidLink, RigidPivotPoint, Tracer, LinearSpring unchanged) ──

class DashPot(qtw.QGraphicsItem):
    def __init__(self, ptSt=qtc.QPointF(0,0), ptEn=qtc.QPointF(1,1), dpWidth=10, dpLength=30,
                 parent=None, pen=None, name='Dashpot', label=None, c=10):
        super().__init__(parent)
        self.stPt, self.enPt = ptSt, ptEn
        self.freeLength = self.getLength()
        self.Width, self.Length = dpWidth, dpLength
        self.conn1Len = (self.freeLength - self.Length)/2
        self.pen, self.name, self.c = pen, name, c
        self.transformation = qtg.QTransform()

    def setc(self, c=None):
        if c is not None: self.c = c

    def getLength(self):
        p = self.enPt - self.stPt
        self.length = math.hypot(p.x(), p.y())
        return self.length

    def getForce(self):
        L = self.getLength()
        now = time.time()
        if hasattr(self, 'prev_length'):
            dt = now - self.prev_time
            vel = (L - self.prev_length)/dt if dt>1e-6 else 0.0
        else:
            vel = 0.0
        self.force = self.c * vel
        self.prev_length, self.prev_time = L, now
        return self.force

    def paint(self, painter, option, widget=None):
        # … existing dashpot drawing code …
        # then overlay c & F:
        painter.setFont(qtg.QFont("Arial", 6))
        txt = f"c={self.c:.1f}, F={getattr(self,'force',0):.2f} N"
        painter.drawText(self.boundingRect(), qtc.Qt.AlignCenter, txt)
        self.transformation.reset()

# ── FourBarLinkage_Model (unchanged) ──

class FourBarLinkage_Controller():
    def __init__(self, widgets):
        self.gv_Main, self.nud_InA, self.lbl_OutA, self.nud_L1, self.nud_L3, self.spnd_Z = widgets
        self.FBL_M = FourBarLinkage_Model()
        self.FBL_V = FourBarLinkage_View(self.gv_Main)
        self.minAngle = 0.0
        self.maxAngle = 2*math.pi

    def setAngleLimits(self, amin_deg, amax_deg):
        self.minAngle = math.radians(amin_deg)
        self.maxAngle = math.radians(amax_deg)

    def setupGraphics(self):
        self.FBL_V.setupGraphics()

    def buildScene(self):
        self.FBL_V.BuildScene(self.FBL_M)

    def setInputLinkLength(self):
        self.FBL_M.setInputLength(self.nud_L1.value())
        self.moveLinkage(self.FBL_M.InputLink.enPt)

    def setOutputLinkLength(self):
        self.FBL_M.setOutputLength(self.nud_L3.value())
        self.moveLinkage(self.FBL_M.InputLink.enPt)

    def moveLinkage(self, scenePos):
        self.FBL_M.moveLinkage(scenePos)
        # clamp input angle
        θ = self.FBL_M.InputLink.angle
        if θ < self.minAngle or θ > self.maxAngle:
            θ = max(self.minAngle, min(self.maxAngle, θ))
            L = self.FBL_M.InputLink.length
            x0, y0 = self.FBL_M.InputLink.stPt.x(), self.FBL_M.InputLink.stPt.y()
            pt = qtc.QPointF(x0 + math.cos(θ)*L, y0 - math.sin(θ)*L)
            self.FBL_M.moveLinkage(pt)

        self.FBL_V.scene.update()
        self.nud_InA.setValue(self.FBL_M.InputLink.AngleDeg())
        self.lbl_OutA.setText(f"{self.FBL_M.OutputLink.AngleDeg():.2f}")

    def runSimulation(self, massInput, k_spring, c_damper):
        # set params
        self.FBL_M.InputLink.mass = massInput
        self.FBL_M.Spring.setk(k_spring)
        self.FBL_M.DashPot.setc(c_damper)

        J  = massInput * self.FBL_M.InputLink.length**2 / 3
        θ0 = math.radians(self.nud_InA.value())
        y0 = [θ0, 0.0]
        t_end = 5.0
        t_eval = np.linspace(0, t_end, 200)

        def ode(t, y):
            th, w = y
            return [ w,
                    -(c_damper/J)*w - (k_spring/J)*(th - math.pi/2)
                   ]

        sol = integrate.solve_ivp(ode, [0, t_end], y0, t_eval=t_eval)

        self.sim_t, self.sim_th = sol.t, sol.y[0]
        self.sim_i = 0
        self.tm = qtc.QTimer()
        interval = int(1000*(t_end/len(self.sim_t)))
        self.tm.setInterval(interval)
        self.tm.timeout.connect(self._advanceSim)
        self.tm.start()

    def _advanceSim(self):
        if self.sim_i >= len(self.sim_t):
            self.tm.stop(); return
        θ = self.sim_th[self.sim_i]
        L, x0, y0 = self.FBL_M.InputLink.length, self.FBL_M.InputLink.stPt.x(), self.FBL_M.InputLink.stPt.y()
        pt = qtc.QPointF(x0 + math.cos(θ)*L, y0 - math.sin(θ)*L)
        self.moveLinkage(pt)
        self.sim_i += 1
