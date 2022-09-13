import matplotlib.pyplot as plt
import numpy as py
import scipy.stats as stats
import seaborn as sns

# 이 함수는 기록한 데이터를 바탕으로 분포 그래프 등을 plot하기 위한 py파일임

f = open('/home/vialab/Sonata_ws/cam2_output.txt', mode = 'r')

lines = f.readlines()

lidartime = []
cameratime = []

for line in lines:
    #print(line, end = '')
    lidartime.append(int(line.split('\t')[0]))
    cameratime.append(int(line.split('\t')[1]))
    #print(lidartime, end = '\n')
    #print(cameratime, end = '\n')
f.close()

#plt.plot(lidartime)
#plt.show()
#plt.plot(cameratime)
#plt.show()

time = 0

# 이 이하의 for구문은 기록된 lidarROSTIME과 cameraROSTIME간의 차이를 비교하기 위한 코드임
# 그리고 이 difftime을 기반으로 오차를 plot함
difftime = []
for i in range(0, len(cameratime) , 1):
    difftime.append((float(cameratime[i]) - float(lidartime[i]))/ 1000000)
    if(difftime[i] < 0):
            difftime[i] = 1000 + difftime[i]
    print(difftime[i])
    time = time + difftime[i]

time = time / len(difftime)
print(time)

#plt.subplots(2,1)

#plt.plot(difftime)
#plt.ylim(100000, 10000000)
#plt.xlabel('times')
#plt.ylabel('time difference (ms)')
#plt.grid(True)
#plt.show()

#difftime.sort()
#diff_mean = py.mean(difftime)
#diff_std = py.std(difftime)

#pdf = stats.norm.pdf(difftime, diff_mean, diff_std)
#plt.plot(difftime, pdf)
#plt.show()

# 이하의 구문은 계산된 difftime을 바탕으로 분포곡선을 plot하는 과정임
# 이때 nanosec을 기준으로 값이 저장되므로 10^6을 나누어 milisec으로 변환한 후 plot하였음
# 여기서 기록된 데이터는 첨부된 output22.txt를 기반으로 하며,
# diff_plot은 difftime 전체를 plot 한 것이고 diff_distplot은 difftime을 분포곡선으로 plot한 것임
sns.distplot(difftime)
plt.axis([0, 20, 0, 1])
plt.xlabel('(CameraRosTime(nsec) - LidarRosTime(nsec)) / 1000000')
plt.grid(True)
plt.show()

#sns.kdeplot(difftime)
#plt.title("Difference between Lidar time and Camera time")
#plt.xlim(-5000000, 5000000)
#plt.show()
