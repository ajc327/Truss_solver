# -*- coding: utf-8 -*-
"""
Created on Thu Jul 23 17:09:18 2020

@author: Andy
"""
import numpy as np
from sklearn import preprocessing

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
    def __init__(self,members,verts,joints):
        self.members = members 
        self.verts = verts 
        self.member_forces = {}
        self.max_force = []
        self.joints =joints
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
        



class truss_generator(object):
    def __init__(self, members, my_joints, verts,force_loc):
        self.members = members 
        self.force_loc = force_loc
        joints = []
        for item in my_joints: 
              my_joint = joint(item,my_joints[item])
              if my_joint.name in force_loc:
                  my_joint.external_force= np.array([[0],[0.5]])
              joints.append(my_joint)
              
        self.joints = joints
        
        def gen_truss(self):
            return truss(self.members,self.verts,self.joints)        
    
    
    
    
    
    
if __name__ == '__main__':
    my_members = ['01','03','13','12','23']
    my_db = {'0': ['1','3'], '1':['0','2','3'], '2':['1','3'],'3':['0','1','2']}
    joints=[]
    my_verts = {'0': [0,0], '1':[1,0], '2':[2,0],'3':[1,1]}
    my_gen = truss_generator(my_members,my_db,my_verts,['0','2'])
    test_truss = truss(my_members,my_verts,joints)
    test_truss.solve()
    