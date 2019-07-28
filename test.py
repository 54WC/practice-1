import numpy as np
import matplotlib.pyplot as plt
import random



print('Done')

    #with open("record.txt", "a+") as file2:
    #file2.write("%9.6s,"%str(t))
    #file2.write("%12.10s,"%str(XTD))
    #file2.write("%12.10s,"%str(tt))
    #file2.write("%12.10s,"%str(oo))
    #file2.write("%12.10s\n"%str(kk))
   #file2.close()


a=np.array([-5.410e+01,-5.060e+01,1.490e-01,1.231e+02,1.160e+01,-1.500e-01,1.400e-02])
b=[-5.410e+01,-5.060e+01,1.490e-01,1.231e+02,1.160e+01,-1.500e-01,1.400e-02]

if (a == b).all():
  print('same')
else:
  print('different')

#print( random.randint(0,2) )
