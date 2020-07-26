# -*- coding: utf-8 -*-
"""
Created on Thu Jul 23 17:09:18 2020

@author: Andy
"""
from drawtool import *
import numpy as np
from sklearn import preprocessing


import sys 
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow,QMenuBar, QMenu , QAction, QPushButton
from PyQt5.QtCore import Qt, QPoint, pyqtSlot
from PyQt5.QtGui import QIcon , QImage, QBrush, QPen,QPainter, QColor 
from PyQt5 import QtCore





class joint(object):
    def __init__(self, joint_name,joint_targets, external_force=[0,0]):
        self.targets = joint_targets
        self.name = joint_name
        self.external_force = preprocessing.normalize(np.array(external_force).reshape(-1,1),axis=0)
        self.solvedness = False
        self.members = [self.name+x for x in self.targets]
                # I will assume the members given is in the format 'nm' where m>n
                # This fixes the joint members list if they are not in this order
        for i in range(len(self.members)):
            my_member = self.members[i]
            if my_member[0]>my_member[1]:
                self.members[i] = self.members[i][::-1]
        self.unsolved_members = self.members

        
   
    def update(self):
        if len(self.unsolved_members) ==0: 
            self.solvedness = True
            
            
class truss(object):
    def __init__(self,verts,joints):
        self.members =[]
        self.joints =joints

        for joint in self.joints: 
            for item in joint.members:
                if item not in self.members: 
                    self.members.append(item)
                
        self.verts = verts 
        self.member_forces = {}
        self.max_force = []
        self.solvedness = False
        self.unsolved_joints = joints
        self.member_lengths={}
        for member in self.members: 
            self.member_forces[member] = 'na'
            mylength = np.linalg.norm(np.array(self.verts[member[0]])-np.array(self.verts[member[1]]))
            self.member_lengths[member] = mylength
            
        self.total_length = sum(self.member_lengths.values())
    def update(self):
        if len(self.unsolved_joints)==0:
            self.solvedness = True 
            max_val = max(self.member_forces.values())
            for item in self.member_forces:
                if self.member_forces == max_val:
                    self.max_force = (item, max_val)
                    break 
            
                        
    def solve(self):
        # This will use the method of joints to calculate the force at each joint
        while len(self.unsolved_joints)>0: 
            #if there is still unsolved joints, we pick a joint to solve
            min_unsolved_members = 100
            for my_joint in self.unsolved_joints: 
                if len(my_joint.unsolved_members)<=min_unsolved_members:
                    joint = my_joint
                    min_unsolved_members = len(my_joint.unsolved_members)
                
            
            desti_joints= joint.targets
            
            rhs = -joint.external_force
            a_matrix =np.array([[],[]])
            unknown_members = []
            for i in range(len(joint.members)):
                # iterate through all the members attached to the particular 
                # particular joint. 
                # if the value of the thing is known, we want to add it to the 
                # force vector. 
                member = joint.members[i]
                desti= desti_joints[i]
                #find the direction of that member
                member_dir = preprocessing.normalize(
                (np.array(self.verts[desti])-np.array(self.verts[joint.name])).reshape(-1,1),axis=0)
                
                
                
                if self.member_forces[member] == 'na':
                    #if the member force is not yet determined we add it to the unknown 
                    # and append the direction to the a matrix
                    unknown_members.append(member)
                    a_matrix = np.column_stack((a_matrix, member_dir)) 
                    
                else: 
                    # if the force is known we compute the resultant and subtract from  b
                    rhs = rhs - self.member_forces[member]*member_dir
                    
            # a has the wrong shape so we transpose it then solve linear equations
            try: 
                solved_forces = np.linalg.solve(a_matrix,rhs)
            except:
                if rhs[0] !=0:
                    solved_forces = rhs[0]/a_matrix[0]
                else: 
                    solved_forces = rhs[1]/a_matrix[1]

            for i in range(len(unknown_members)):
                #we update the member forces 
                self.member_forces[unknown_members[i]]= float(solved_forces[i])
            joint.solvedness = True
            self.unsolved_joints.remove(joint)
            for joint in self.unsolved_joints:
                joint.unsolved_members = [i for i in joint.unsolved_members if i not in unknown_members] 
                joint.update()
                
        self.update()
    
    def draw(self):
        pass



class truss_generator(object):
    def __init__(self, my_joints, verts,force_loc):
        self.force_loc = force_loc
        self.verts = verts
        joints = []
        for item in my_joints: 
              my_joint = joint(item,my_joints[item])
              if my_joint.name in force_loc:
                  my_joint.external_force= np.array([[0],[-0.5]])
              elif my_joint.name == '1':
                  my_joint.external_force = np.array([[0],[1]])
              joints.append(my_joint)
              
        self.joints = joints
        
    def gen_truss(self):
        return truss(self.verts,self.joints)        

    


# we need to define a main window in which the app runs
class trussWindow(QMainWindow):
    
    def __init__(self,truss):
        super().__init__() 
        top = 400
        left = 400
        width = 1920
        height = 1080
        members = truss.members
        verts = truss.verts
        self.setWindowTitle('Truss Visualiser') 
        self.setGeometry(top,left, width, height) 
        self.member_forces = truss.member_forces
        self.members = {}
        self.verts = {}
        self.force_locs = {}
        
        scale = 1
        #offset = np.array([600, 600])
        offset = np.array([0,0])
        
        for item in members: 
            print(np.array(verts[item[0]])*scale)
            p1 = np.array(verts[item[0]])*scale+offset
            p2 = np.array(verts[item[1]])*scale+offset
            
            point1 = QPoint(*(np.array(verts[item[0]])*scale+offset))
            point2 = QPoint(*(np.array(verts[item[1]])*scale+offset))
            self.force_locs[item] = [(0.5*(p1+p2))[0],(0.5*(p1+p2))[1] - 30]
            self.members[item]=(point1,point2)
        for item in verts: 
            self.verts[item]= QPoint(*(np.array([verts[item][0],verts[item][1]])*scale+offset))
            
        
    def paintEvent(self, event): 
            #the method to paint the lines and vertices
          qp = QPainter(self) 
          br = QBrush(QColor(100,10,10,40))

          qp.setBrush(br) 
          
          for item in self.verts:
              # we draw the vertices
              qp.drawEllipse(self.verts[item],7,7)
          for member in self.members:
              #and we draw the members (here we unpacked the tuple)
              if self.member_forces[member]<0: 
                  
                  pen= QPen(Qt.red,5)
                  qp.setPen(pen)
              else: 
                  pen= QPen(Qt.darkGreen,5)
                  qp.setPen(pen)
        
              qp.drawLine(*self.members[member])
              qp.drawText(*self.force_locs[member],"%.2f" % self.member_forces[member])
            
         
    
    
    
if __name__ == '__main__':
    my_members = ['01','03','13','12','23']
    my_db = {'0': ['1','3'], '1':['0','2','3'], '2':['1','3'],'3':['0','1','2']}
    joints=[]
    my_verts = {'0': [0,0], '1':[400,0], '2':[800,0],'3':[400,-400]}
    my_gen = truss_generator(my_db,my_verts,['0','2'])
    test_truss= my_gen.gen_truss()
    test_truss.solve()
    
    app = QApplication(sys.argv) 
    # Create the window
    mywindow = trussWindow(test_truss)
    mywindow.show() 
    
    sys.exit(app.exec_())
    print ("the following")