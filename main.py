from classes import *
 
from drawtool import *


app = QApplication(sys.argv) 
# Create the window
window = Window()
window.show() 

if (sys.flags.interactive != 1) or not hasattr(QtCore, 'pyqt5'):
    QApplication.instance().exec_()
print("the following code will run")
my_joints = window.outjoints
my_verts = window.outverts
my_gen = truss_generator(my_joints,my_verts,['0','2'])
test_truss= my_gen.gen_truss()
test_truss.solve()
forces = test_truss.member_forces

results = trussWindow(test_truss)
results.show()
if (sys.flags.interactive != 1) or not hasattr(QtCore, 'pyqt5'):
    QApplication.instance().exec_()
print("success")