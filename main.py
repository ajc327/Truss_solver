from classes import *
 

my_members = ['01','03','13','12','23']
my_db = {'0': ['1','3'], '1':['0','2','3'], '2':['1','3'],'3':['0','1','2']}
joints=[]
my_verts = {'0': [0,0], '1':[1,0], '2':[2,0],'3':[1,1]}
my_gen = truss_generator(my_members,my_db,my_verts,['0','2'])
test_truss = truss(my_members,my_verts,joints)
test_truss.solve()
    