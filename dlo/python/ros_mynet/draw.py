import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
f = open('./screen0418.log')
loss = []
loss1 = []
loss2  = []
loss3 = [] 
for line in f:
  # if 'train' in line:  
  #   loss.append(line[16:24])
  #   loss1.append(line[31:38])
  #   loss2.append(line[47:55])
  #   loss3.append(line[62:70])
    # loss2.append(line[45:52])
    # loss3.append(line[59:66])
  if 'val' in line:  
    loss.append(line[14:22])
    loss1.append(line[29:37])
    loss2.append(line[45:53])
    loss3.append(line[60:68])
    # loss3.append(line[60:69])

loss = np.array(loss)
loss1 = np.array(loss1)
loss2 = np.array(loss2)
loss3 = np.array(loss3)

x = np.array(range(len(loss3)))
print(loss)
print(loss1)
print(loss2)
print(loss3)

# print x
# plt.scatter(loss,x)
# plt.show()
# data = {'loss':loss,'loss1':loss1,'loss2':loss2,'loss3':loss3}
data = {'loss2':loss2,'loss3':loss3}
# data = {'loss':loss,'loss1':loss1}
data = pd.DataFrame(data)
#print(data)


data = data.astype(float)
#print(data)

#print(data)
data.plot()
plt.show()
