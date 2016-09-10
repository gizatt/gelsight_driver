import matplotlib.pyplot as plt
import numpy as np

f = open('comparison_results.csv','r')

h = [] # histograms
er = [] # errors

for l in f:
  l = l[:-1]
  if len(l) > 20: # histogram row
    h.append([int(c) for c in l.split(' ')])
  else: # error row
    er.append(float(l))

hx = [2*float(i)/len(h[0])-1.0 for i in range(len(h[0]))]

hpos = []
for r in h:
  rpos = [0 for _ in range(len(r)/2 + 1)]
  rpos[0] = r[len(r)/2]
  for i in range(1,len(rpos)):
    rpos[i] = r[len(r)/2+i] + r[len(r)/2-i]
  hpos.append(rpos)

hxpos = [float(i)/(len(h[0])/2+1) for i in range(len(hpos[0]))]

#rpostotal = hpos[0]
#for rpos in hpos[1:]:
#  for i in range(len(rpostotal)):
#    rpostotal[i] = rpostotal[i] + rpos[i]
#
#hpos = [rpostotal]

for rpos in hpos:
  rpos = np.array([float(x) for x in rpos])
  rpos = rpos / sum(rpos)
  plt.plot(hxpos,np.cumsum(rpos))
#  plt.plot(hxpos, rpos)

for r in hpos:
  r = np.array([float(x) for x in r])
  r = r / sum(r)
#  plt.plot(hxpos,r)

plt.xlim([0,1])
#plt.xlim([-1,1])
plt.ylim([0,1])
plt.xlabel('|error|')
plt.ylabel('% pixels')
plt.title('Heightmap Reconstruction Error Rates within AOI')

plt.show()


