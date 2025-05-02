# FourBarLinkage_MVC.py

#region imports
import math
import time
import numpy as np
from copy import deepcopy as dc
from scipy import integrate
import PyQt5.QtGui as qtg
import PyQt5.QtCore as qtc
import PyQt5.QtWidgets as qtw
#endregion

# ── DATA STRUCTURE FOR A LINK ──
class Link:
    def __init__(self, stPt, length, angle=0.0):
        # stPt: QPointF, length in pixels, angle in radians
        self.stPt = qtc.QPointF(stPt)
        self.enPt = qtc.QPointF(stPt)
        self.length = length
        self.angle = angle

    def AngleDeg(self):
        # return angle in [0,360)
        return math.degrees(self.angle) % 360

# ── SPRING GRAPHIC ITEM ──
class Spring(qtw.QGraphicsItem):
    def __init__(self, ptSt, ptEn, k=50, parent=None):
        super().__init__(parent)
        self.stPt = qtc.QPointF(ptSt)
        self.enPt = qtc.QPointF(ptEn)
        self.k = k
        self.freeLength = self.getLength()
        self.force = 0.0

    def setk(self, k=None):
        if k is not None:
            self.k = k

    def getLength(self):
        p = self.enPt - self.stPt
        return math.hypot(p.x(), p.y())

    def getForce(self):
        L = self.getLength()
        self.force = self.k * (L - self.freeLength)
        return self.force

    def boundingRect(self):
        # bounding box around the two endpoints, with padding for text
        x0 = min(self.stPt.x(), self.enPt.x())
        y0 = min(self.stPt.y(), self.enPt.y())
        w  = abs(self.enPt.x() - self.stPt.x())
        h  = abs(self.enPt.y() - self.stPt.y())
        pad = 20
        return qtc.QRectF(x0 - pad, y0 - pad, w + 2*pad, h + 2*pad)

    def paint(self, painter, option, widget=None):
        painter.setPen(qtg.QPen(qtc.Qt.blue, 2))
        painter.drawLine(self.stPt, self.enPt)
        self.getForce()
        painter.setFont(qtg.QFont("Arial", 6))
        painter.drawText(self.boundingRect(), qtc.Qt.AlignCenter,
                         f"k={self.k:.1f}, F={self.force:.1f}N")

# ── DASHPOT GRAPHIC ITEM ──
class DashPot(qtw.QGraphicsItem):
    def __init__(self, ptSt, ptEn, c=10, parent=None):
        super().__init__(parent)
        self.stPt = qtc.QPointF(ptSt)
        self.enPt = qtc.QPointF(ptEn)
        self.c = c
        self.freeLength = self.getLength()
        self.prev_length = self.freeLength
        self.prev_time = time.time()
        self.force = 0.0

    def setc(self, c=None):
        if c is not None:
            self.c = c

    def getLength(self):
        p = self.enPt - self.stPt
        return math.hypot(p.x(), p.y())

    def getForce(self):
        L = self.getLength()
        now = time.time()
        dt = now - self.prev_time
        if dt > 1e-6:
            vel = (L - self.prev_length) / dt
        else:
            vel = 0.0
        self.force = self.c * vel
        self.prev_length = L
        self.prev_time = now
        return self.force

    def boundingRect(self):
        x0 = min(self.stPt.x(), self.enPt.x())
        y0 = min(self.stPt.y(), self.enPt.y())
        w  = abs(self.enPt.x() - self.stPt.x())
        h  = abs(self.enPt.y() - self.stPt.y())
        pad = 20
        return qtc.QRectF(x0 - pad, y0 - pad, w + 2*pad, h + 2*pad)

    def paint(self, painter, option, widget=None):
        painter.setPen(qtg.QPen(qtc.Qt.red, 2))
        painter.drawLine(self.stPt, self.enPt)
        self.getForce()
        painter.setFont(qtg.QFont("Arial", 6))
        painter.drawText(self.boundingRect(), qtc.Qt.AlignCenter,
                         f"c={self.c:.1f}, F={self.force:.1f}N")

# ── FOUR-BAR LINKAGE MODEL ──
class FourBarLinkage_Model:
    def __init__(self):
        # fixed pivots (in scene coordinates)
        self.InputLink  = Link(stPt=qtc.QPointF(200, 200), length=100, angle=math.pi/2)
        self.OutputLink = Link(stPt=qtc.QPointF(400, 200), length=100, angle=0)
        self.CouplerLink = Link(stPt=self.InputLink.stPt, length=150, angle=0)

        # spring & dashpot between link midpoints
        self.Spring  = Spring(self.InputLink.stPt, self.OutputLink.stPt, k=50)
        self.DashPot = DashPot(self.InputLink.stPt, self.OutputLink.stPt, c=10)

        # for branch‐selection in kinematics
        self.prevP3 = None

        # compute all the rest of the geometry
        self.computePositions()

    def computePositions(self):
        # 1) Input link end P2
        x0, y0 = self.InputLink.stPt.x(), self.InputLink.stPt.y()
        th1 = self.InputLink.angle
        P2 = qtc.QPointF(
            x0 + math.cos(th1)*self.InputLink.length,
            y0 - math.sin(th1)*self.InputLink.length
        )
        self.InputLink.enPt = P2

        # 2) Solve circle‐circle for coupler‐output joint P3
        P4 = self.OutputLink.stPt
        L2, L3 = self.CouplerLink.length, self.OutputLink.length
        dx, dy = P4.x() - P2.x(), P4.y() - P2.y()
        d = math.hypot(dx, dy)

        if d == 0 or d > (L2 + L3) or d < abs(L2 - L3):
            # degenerate — hold last
            P3 = self.prevP3 or qtc.QPointF(P2)
        else:
            a = (L2**2 - L3**2 + d**2)/(2*d)
            h = math.sqrt(max(0, L2**2 - a**2))
            xm, ym = P2.x() + a*(dx/d), P2.y() + a*(dy/d)
            rx, ry = -dy*(h/d), dx*(h/d)
            I1 = qtc.QPointF(xm+rx, ym+ry)
            I2 = qtc.QPointF(xm-rx, ym-ry)
            if self.prevP3 is None:
                P3 = I1
            else:
                # choose branch nearest last
                d1 = (I1.x()-self.prevP3.x())**2 + (I1.y()-self.prevP3.y())**2
                d2 = (I2.x()-self.prevP3.x())**2 + (I2.y()-self.prevP3.y())**2
                P3 = I1 if d1<d2 else I2

        self.prevP3 = dc(P3)
        self.CouplerLink.stPt, self.CouplerLink.enPt = P2, P3
        self.CouplerLink.angle = math.atan2(P3.y()-P2.y(), P3.x()-P2.x())

        # 3) Output link
        self.OutputLink.enPt = P3
        self.OutputLink.angle = math.atan2(P3.y()-P4.y(), P3.x()-P4.x())

        # 4) Spring & dashpot attach at the midpoints of those two links
        mid2 = qtc.QPointF((P2.x()+P3.x())/2, (P2.y()+P3.y())/2)
        mid4 = qtc.QPointF((P4.x()+P3.x())/2, (P4.y()+P3.y())/2)
        self.Spring.stPt,  self.Spring.enPt  = mid2, mid4
        self.DashPot.stPt, self.DashPot.enPt = mid2, mid4

    def moveLinkage(self, scenePos):
        # derive new input angle from mouse position
        x0, y0 = self.InputLink.stPt.x(), self.InputLink.stPt.y()
        dx = scenePos.x() - x0
        dy = y0 - scenePos.y()  # invert because we did y0 - sin(theta)
        self.InputLink.angle = math.atan2(dy, dx)
        self.computePositions()

    def setInputLength(self, L):
        self.InputLink.length = L
        self.computePositions()

    def setOutputLength(self, L):
        self.OutputLink.length = L
        self.computePositions()

# ── FOUR-BAR LINKAGE VIEW ──
class FourBarLinkage_View:
    def __init__(self, graphicsView):
        self.view  = graphicsView
        self.scene = qtw.QGraphicsScene()
        self.view.setScene(self.scene)

    def setupGraphics(self):
        self.scene.clear()
        # three rigid‐body links as plain QGraphicsLineItems
        self.inputLine   = qtw.QGraphicsLineItem()
        self.couplerLine = qtw.QGraphicsLineItem()
        self.outputLine  = qtw.QGraphicsLineItem()
        for L in (self.inputLine, self.couplerLine, self.outputLine):
            L.setPen(qtg.QPen(qtc.Qt.black, 2))
            self.scene.addItem(L)

    def BuildScene(self, model):
        # add the spring & dashpot items (they are QGraphicsItem subclasses)
        self.scene.addItem(model.Spring)
        self.scene.addItem(model.DashPot)
        # initialize lines & spring/dashpot positions
        self.update(model)

    def update(self, model):
        # redraw link lines
        p1, p2 = model.InputLink.stPt, model.InputLink.enPt
        self.inputLine.setLine(p1.x(), p1.y(), p2.x(), p2.y())
        p2, p3 = model.CouplerLink.stPt, model.CouplerLink.enPt
        self.couplerLine.setLine(p2.x(), p2.y(), p3.x(), p3.y())
        p4, p3 = model.OutputLink.stPt, model.OutputLink.enPt
        self.outputLine.setLine(p4.x(), p4.y(), p3.x(), p3.y())

        # tell spring/dashpot their bounding boxes changed
        model.Spring.prepareGeometryChange()
        model.DashPot.prepareGeometryChange()

        # trigger a redraw
        self.scene.update()

# ── FOUR-BAR LINKAGE CONTROLLER ──
class FourBarLinkage_Controller:
    def __init__(self, widgets):
        # widgets = [gv_Main, nud_InA, lbl_OutA, nud_L1, nud_L3, spnd_Z]
        (self.gv_Main, self.nud_InA, self.lbl_OutA,
         self.nud_L1,   self.nud_L3,   self.spnd_Z) = widgets

        # model & view
        self.FBL_M = FourBarLinkage_Model()
        self.FBL_V = FourBarLinkage_View(self.gv_Main)

        # default angle limits
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
        # redraw at new position
        self.moveLinkage(self.FBL_M.InputLink.enPt)

    def setOutputLinkLength(self):
        self.FBL_M.setOutputLength(self.nud_L3.value())
        self.moveLinkage(self.FBL_M.InputLink.enPt)

    def moveLinkage(self, scenePos):
        # interactive drag
        self.FBL_M.moveLinkage(scenePos)
        theta = self.FBL_M.InputLink.angle
        # clamp to limits
        if theta < self.minAngle or theta > self.maxAngle:
            theta = max(self.minAngle, min(self.maxAngle, theta))
            self.FBL_M.InputLink.angle = theta
            self.FBL_M.computePositions()

        # redraw everything
        self.FBL_V.update(self.FBL_M)
        # update UI labels
        self.nud_InA.setValue(self.FBL_M.InputLink.AngleDeg())
        self.lbl_OutA.setText(f"{self.FBL_M.OutputLink.AngleDeg():.2f}")

    def runSimulation(self, massInput, k_spring, c_damper):
        # set parameters
        self.FBL_M.InputLink.mass = massInput
        self.FBL_M.Spring.setk(k_spring)
        self.FBL_M.DashPot.setc(c_damper)

        # moment of inertia about pivot (rod ~ uniform)
        J = massInput * self.FBL_M.InputLink.length**2 / 3

        # initial cond: release from current angle, zero angular vel
        theta0 = math.radians(self.nud_InA.value())
        y0 = [theta0, 0.0]

        # time vector
        t_end  = 5.0
        t_eval = np.linspace(0, t_end, 200)

        # ODE: θ̈ + (c/J) θ̇ + (k/J)(θ - π/2) = 0
        def ode(t, y):
            th, w = y
            return [w,
                    -(c_damper/J)*w - (k_spring/J)*(th - math.pi/2)]

        sol = integrate.solve_ivp(ode, [0, t_end], y0, t_eval=t_eval)
        self.sim_t, self.sim_th = sol.t, sol.y[0]
        self.sim_i = 0

        # timer to step through animation
        self.tm = qtc.QTimer()
        interval = int(1000 * (t_end / len(self.sim_t)))
        self.tm.setInterval(interval)
        self.tm.timeout.connect(self._advanceSim)
        self.tm.start()

    def _advanceSim(self):
        if self.sim_i >= len(self.sim_t):
            self.tm.stop()
            return
        th = self.sim_th[self.sim_i]
        L  = self.FBL_M.InputLink.length
        x0, y0 = (self.FBL_M.InputLink.stPt.x(),
                  self.FBL_M.InputLink.stPt.y())
        # compute new input‐link end
        pt = qtc.QPointF(x0 + math.cos(th)*L,
                         y0 - math.sin(th)*L)
        self.moveLinkage(pt)
        self.sim_i += 1
