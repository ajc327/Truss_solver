# -*- coding: utf-8 -*-
"""
Created on Thu Jul 23 17:09:18 2020

@author: Andy
"""
from drawtool import *
import numpy as np
from sklearn import preprocessing

import string
import sys 
import numpy as np
from PyQt5.QtWidgets import QApplication, QWidget, QMainWindow,QMenuBar, QMenu , QAction, QPushButton
from PyQt5.QtCore import Qt, QPoint, pyqtSlot
from PyQt5.QtGui import QIcon , QImage, QBrush, QPen,QPainter, QColor 
from PyQt5 import QtCore
from scipy.optimize import minimize,basinhopping,NonlinearConstraint,Bounds, differential_evolution, dual_annealing
import matplotlib.pyplot as plt




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
        # initialise the members
        for joint in self.joints: 
            for item in joint.members:
                if item not in self.members: 
                    self.members.append(item)
        # the verts are given in dictionaries {'a': [x,y]}
                    
        self.verts = verts 
        self.optimized_flag= False
        self.optx0=[]
        self.member_forces = {}
        self.max_force = []
        self.min_force =[]
        self.solvedness = False
        self.unsolved_joints = joints
        self.optimized_verts ={}
        # fixednum is the number of vertices to keep fixed and not participate in the optimisation
        self.fixednum= 3
        for i in range (self.fixednum):
            #initialise the optimized_verts dictionary
            self.optimized_verts[string.ascii_lowercase[i]] = self.verts[string.ascii_lowercase[i]]
        for i in range(len(self.verts)-self.fixednum):
            # setting up the x0 for scipy minimizer
            self.optx0.append(self.verts[string.ascii_lowercase[i+self.fixednum]][0])
            self.optx0.append(self.verts[string.ascii_lowercase[i+self.fixednum]][1])

        self.member_lengths={}
        
        for member in self.members: 
            # Initialises the member forces and lengths dictionaries
            self.member_forces[member] = 'na'
            mylength = np.linalg.norm(np.array(self.verts[member[0]])-np.array(self.verts[member[1]]))
            self.member_lengths[member] = mylength
            
        self.total_length = sum(self.member_lengths.values())
    def update(self):
        # the method to update the solved flag and the max_force locations
        if len(self.unsolved_joints)==0:
            self.solvedness = True 
            max_val = max(self.member_forces.values())
            min_val = min(self.member_forces.values())
            for item in self.member_forces:
                if self.member_forces[item] == max_val:
                    self.max_force = (item, max_val)
                    break 
                if self.member_forces[item] == min_val:
                    self.min_force = (item, min_val)
        self.total_length = sum(self.member_lengths.values())
            
                        
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
                if self.optimized_flag ==False:
                    member_dir = preprocessing.normalize(
                    (np.array(self.verts[desti])-np.array(self.verts[joint.name])).reshape(-1,1),axis=0)
                else: 
                    member_dir = preprocessing.normalize(
                    (np.array(self.optimized_verts[desti])-np.array(self.optimized_verts[joint.name])).reshape(-1,1),axis=0)
                    
                
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
    
    def linsolve(self):
        # this sets up a big A matrix using all the joint equations, then solves it 
        # with least squares
        
        # initialises the A and b in Ax = b
        rhs = np.zeros((2*len(self.joints),1))
        a_matrix =np.zeros((2*len(self.joints),len(self.members)))
        
        joint_counter = 0 
        
        for joint in self.joints:
            # loop through the joints to append equations from each joint into A
            
            desti_joints= joint.targets
            
            rhs[joint_counter*2] = -joint.external_force[0]
            rhs[joint_counter*2+1] = -joint.external_force[1]
            
            for i in range(len(joint.members)):
                # iterate through all the members attached to the particular 
                # particular joint. 
                # if the value of the thing is known, we want to add it to the 
                # force vector. 
                member = joint.members[i]
                member_index = self.members.index(member)
                desti= desti_joints[i]
                #find the direction of that member
                
                if self.optimized_flag ==False:
                    member_dir = preprocessing.normalize(
                    (np.array(self.verts[desti])-np.array(self.verts[joint.name])).reshape(-1,1),axis=0)
                else: 
                    member_dir = preprocessing.normalize(
                    (np.array(self.optimized_verts[desti])-np.array(self.optimized_verts[joint.name])).reshape(-1,1),axis=0)
                                       
                # and append the direction to the a matrix
                a_matrix[joint_counter*2,member_index] = member_dir[0]
                a_matrix[joint_counter*2+1,member_index] = member_dir[1]
                
            joint_counter +=1

            
        try: 
            # solve the Ax= b least squares problem 
            solved_forces = np.linalg.lstsq(a_matrix,rhs,rcond=None)
        except:
            print("cant solve the problem")
        for i in range(len(self.members)):
            # update the member force using the results of the lstsq solution
            self.member_forces[self.members[i]] = solved_forces[0][i]
        
        # we can set all to solved and update the min and max
        self.unsolved_joints=[]
        self.update()
        
    def objective(self,optverts):
        # this is the objective function to be used to in the optimisation part
        # optiverts is in the form [x1,y1,x2,y2...]
        for i in range(0,len(optverts),2): 
            #initialise verts dictionary again using the parameters optverts
            x = optverts[i]
            y = optverts[i+1]
            self.verts[string.ascii_lowercase[int(i/2)+self.fixednum]] = [x,y]
            
        # after updating the function we run linsolve again to calculate the member forces again
        self.linsolve()
        
        max_val = max(self.member_forces.values())
        min_val = min(self.member_forces.values())   
        diff  = max([np.abs(max_val),np.abs(min_val)])
        '''
        print("The objective function evaluates to %.2f" % diff)
        print("The total length is %.2f \n\n" % self.total_length)

        '''
        # the thing we want to change are the vertex locations.
        # need to define objective(x) where x are the vertext locations. 
        # this function then needs use linsolve to solve and return the resulting 
        # max force - min force. This should be a good indicator for performance
        return diff 
        
    def constraint(self,optverts):
        for i in range(0,len(optverts),2): 
            # again we update the self.verts dictionary first
            x = optverts[i]
            y = optverts[i+1]
            self.verts[string.ascii_lowercase[i+self.fixednum]] = [x,y]
        # the total length must be lower than 4000 mm 
        
        
        for member in self.members: 
            # recalculate the total member length
            mylength = np.linalg.norm(np.array(self.verts[member[0]])-np.array(self.verts[member[1]]))
            self.member_lengths[member] = mylength
            
        self.total_length = sum(self.member_lengths.values())
        return (self.total_length-4000)
    
    
    def optimize(self):
        # the optimizer itself.
        # first we define the constraints for the minimizer
        con1 = {'type': 'ineq', 'fun':self.constraint}
        cons = [con1]
        # These are for the differential evolution optimizer
        nlc = NonlinearConstraint(self.constraint, -np.inf, 0)
        lower =[]
        upper = []
        optrange = 100
        for item in self.optx0:
            lower.append(item-optrange)
            upper.append(item+optrange)
        
        bounds= Bounds(lower,upper)
        sol = differential_evolution(func=self.objective, bounds=bounds,strategy = 'best2exp',disp=True, 
                                     constraints=(nlc),popsize=30,tol =0.01,maxiter=300, mutation=(0.3,0.7))
        
        # we can choose which optimizer to use. 
        #sol = minimize(self.objective,self.optx0,constraints=cons,method= 'Powell' , tol = 0.00001, options={'disp':True})
        
        #sol = dual_annealing(self.objective,bounds = )
        optimized_sols = sol.x

        for i in range(0,len(optimized_sols),2):
            # use the results form the optimizer to set up the optimized verts dictionary
            self.optimized_verts[string.ascii_lowercase[int(i/2)+self.fixednum]] =[optimized_sols[i],optimized_sols[i+1]]
        self.optimized_flag = True
        #self.linsolve()
        # update the max min values and member length again 
        
        self.update()
        print("The function value from the optimiser is %.2f" % sol.fun)
        print("The final total length is %.2f" % self.total_length)
        max_val = max(self.member_forces.values())
        min_val = min(self.member_forces.values())   
        diff  = max([np.abs(max_val),np.abs(min_val)])
        print("The final objective function is %.2f" % diff)
  
        
class truss_generator(object):
    # this generates a truss, is probably not very necessary 
    def __init__(self, my_joints, verts,force_loc=['a','b']):
        # basically gets the joints and verts into a good form for the truss class
        # can probably just chuck the stuff below into the truss class 
        self.force_loc = force_loc
        self.verts = verts
        joints = []
        for item in my_joints: 
              my_joint = joint(item,my_joints[item])
              if my_joint.name in force_loc:
                  my_joint.external_force= np.array([[0],[-0.5]])
              elif my_joint.name == 'c':
                  my_joint.external_force = np.array([[0],[1]])
              joints.append(my_joint)
              
        self.joints = joints
        
    def gen_truss(self):
        return truss(self.verts,self.joints)        
        
    


# we need to define a main window in which the app runs
class trussWindow(QMainWindow):
    # ths is for visualising the truss and see its forces 
    def __init__(self,truss):
        super().__init__() 
        top = 400
        left = 400
        width = 1920
        height = 1080
        members = truss.members
        verts = truss.verts
        self.truss = truss
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
            # p1 and p2 are the two end points of the member, we store the centre 
            # with an offset so we can put the force there
            p1 = np.array(verts[item[0]])*scale+offset
            p2 = np.array(verts[item[1]])*scale+offset
            
            # point1 point2 are the QPoint objects we need to draw the member
            point1 = QPoint(*(np.array(verts[item[0]])*scale+offset))
            point2 = QPoint(*(np.array(verts[item[1]])*scale+offset))
            self.force_locs[item] = [(0.5*(p1+p2))[0],(0.5*(p1+p2))[1] - 30]
            self.members[item]=(point1,point2)
        for item in verts: 
            # we make the verts dictionary which has the q points
            self.verts[item]= QPoint(*(np.array([verts[item][0],verts[item][1]])*scale+offset))
            
        # the button for starting the optimisaiton process
        button = QPushButton("Optimise the truss",self)
        button.move(1400,800)
        button.resize(200,50)
        # connect the button to the on click method
        button.clicked.connect(self.on_click)    
        
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
            
    def update(self):
        # updates the members, verts, force dictionaries again, for when we finish optimisation 
        # and would like to display the results .
        members = self.truss.members
        verts = self.truss.optimized_verts
        self.member_forces = self.truss.member_forces
        self.members = {}
        
        self.verts = {}
        self.force_locs = {}
        
        scale = 1
        #offset = np.array([600, 600])
        offset = np.array([0,0])
        
        for item in members: 
            p1 = np.array(verts[item[0]])*scale+offset
            p2 = np.array(verts[item[1]])*scale+offset
            
            point1 = QPoint(*(np.array(verts[item[0]])*scale+offset))
            point2 = QPoint(*(np.array(verts[item[1]])*scale+offset))
            self.force_locs[item] = [(0.5*(p1+p2))[0],(0.5*(p1+p2))[1] - 30]
            self.members[item]=(point1,point2)
        for item in verts: 
            self.verts[item]= QPoint(*(np.array([verts[item][0],verts[item][1]])*scale+offset))
        
    def on_click(self):
        self.truss.optimize()
        self.update()        
        print("the truss is optimized")
        
    
    
if __name__ == '__main__':
    my_members = ['ac','ad','cd','ce','bc','be','de']
    my_db = {'a': ['c','d'], 'b':['c','e'], 'c':['a','b','d','e'],'d':['a','c','e'],'e':['d','c','b']}
    joints=[]
    my_verts = {'a': [400,700], 'b':[1200,700], 'c':[800,700],'d':[575,543],'e':[924,589]}
    my_gen = truss_generator(my_db,my_verts,['a','b'])
    test_truss= my_gen.gen_truss()
    test_truss.linsolve()
    
    app = QApplication(sys.argv) 
    # Create the window
    mywindow = trussWindow(test_truss)
    mywindow.show() 
    
    sys.exit(app.exec_())
    print ("the following")