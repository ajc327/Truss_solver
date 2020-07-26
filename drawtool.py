
import sys 
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow,QMenuBar, QMenu , QAction, QPushButton
from PyQt5.QtCore import Qt, QPoint, pyqtSlot
from PyQt5.QtGui import QIcon , QImage, QBrush, QPen,QPainter, QColor 
from PyQt5 import QtCore

# we need to define a main window in which the app runs

 
class Window(QMainWindow):
    def __init__(self): 
        super().__init__()
        top = 400 
        left = 400
        width =1920
        height = 1080
        self.setWindowTitle('Tool') 
        self.members = []
        self.joints = []
        self.setGeometry(top,left, width, height) 
        self.begin = QPoint() 
        #initialise the three fixed vertices 
        self.left_vert = QPoint(400,500) 
        self.right_vert = QPoint(1200,500)
        self.mid_vert = QPoint(800,500)
        self.verts ={'0':self.left_vert, '1':self.right_vert, '2':self.mid_vert}
        self.outverts = {}
        self.snapflag =False
        self.snapto = ''
        self.tempsnap = ''
        self.outmembers = []
        self.outjoints = {}
        #snapflag is to indicate whether the mouse is next to an existing vertex, if this is true then we dont create new vertex.
        self.end = QPoint()
        button = QPushButton("Save the truss configuration",self)
        button.move(1400,800)
        button.resize(200,50)
        button.clicked.connect(self.on_click)
        # saveAction = QAction(QIcon("icons/save.png"), "Save", self) 

    def paintEvent(self, event): 
        #the method to paint the lines and vertices
        qp = QPainter(self) 
        br = QBrush(QColor(100,10,10,40))
        qp.setBrush(br) 
        qp.drawLine(self.begin,self.end)
        pen= QPen(Qt.darkGreen,5)
        qp.setPen(pen)
        
        for item in self.verts:
            # we draw the vertices
            qp.drawEllipse(self.verts[item],7,7)
        for member in self.members:
            #and we draw the members (here we unpacked the tuple) 
            qp.drawLine(*member)
        if self.snapflag ==True: 
            qp.drawEllipse(self.verts[self.snapto],5,5)
    def mousePressEvent(self,event):
        #the method for when the left mouse button is pressed 
        
        if event.button() == Qt.LeftButton: 
            #we update the event position and store the new vertex if we need to create one 
            self.begin = event.pos()
            for vert in self.verts:
                    if (self.verts[vert]-event.pos()).manhattanLength()<20:
                        self.snapflag = True 
                        self.snapto = vert
                        break 
                    else: 
                        self.snapflag = False 
            
            if self.snapflag== False:
                self.verts[str(len(self.verts))] = event.pos()
                self.tempsnap = str(len(self.verts))
            else: 
                self.begin = self.verts[self.snapto]
                self.tempsnap = self.snapto
            self.end = event.pos()
            self.update()
            
    def mouseMoveEvent(self, event): 
        for vert in self.verts:
            if (self.verts[vert]-event.pos()).manhattanLength()<20:
                self.snapflag = True 
                self.snapto = vert
                break 
            else: 
                self.snapflag = False 
        if event.buttons() & Qt.LeftButton:
            #during mouse click we update the position and draw 
            self.end = event.pos()
            self.update()
            
    def mouseReleaseEvent(self,event): 
        
        if event.button() == Qt.LeftButton:
            # when released we record the final position, add this to the members list and draw. if we didnt snap, add a new vertex.
            self.end = event.pos() 

            if self.snapflag == False : 
                temp=str(len(self.verts))

                self.verts[str(len(self.verts))] = event.pos()
            else: 
                self.end = self.verts[self.snapto]
                temp = self.snapto
            if (self.begin-self.end).manhattanLength()>30:    
                self.members.append((self.begin,self.end))
                self.outmembers.append(self.tempsnap+temp)
            self.begin= event.pos()
            self.update() 
        
    def on_click(self):
        for i in range(len(self.outmembers)):
                my_member = self.outmembers[i]
                if my_member[0]>my_member[1]:
                    self.outmembers[i] = self.outmembers[i][::-1]
        for item in self.verts:
            self.outjoints[item]=[]
            self.outverts[item] = (self.verts[item].x(), self.verts[item].y())
        for member in self.outmembers:
            vert = member[0]
            self.outjoints[vert].append(member[1]) 
            self.outjoints[member[1]].append(member[0])
        
        self.close()
        
    
    
    
if __name__ == '__main__':
    # Create the example of QApplication 
    pass