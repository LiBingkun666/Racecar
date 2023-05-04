import glob
import matplotlib.pyplot as plt

all_line = []
with open("./loss.txt","r",encoding="utf-8") as f:
    lline = f.readlines()
    for ll in lline:
        all_line.append(ll)
det = []
seg = []
for line in all_line:
    det.append(float(line.strip().split(",")[0]))
    seg.append(float(line.strip().split(",")[1]))

plt.title('detect loss')
plt.plot(list(range(0,len(seg))), det, color='green', label='detect loss')
plt.grid()
plt.xlabel('epoch')
plt.ylabel('loss')
plt.show()

plt.title('segment loss')

plt.plot(list(range(0,len(seg))), seg, color='green', label='segment loss')
plt.grid()
plt.xlabel('epoch')
plt.ylabel('loss')
plt.show()

