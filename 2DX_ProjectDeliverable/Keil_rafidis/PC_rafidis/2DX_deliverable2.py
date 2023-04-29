from cmath import cos, sin
from os import truncate
from turtle import color
import serial
import math
import numpy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
#Samer Rafidi, 400333524

prompt1 = input("Would you like to erase the previous graph (Start a whole new plot)?(yes/no)")
if(prompt1 == 'yes'):
    f = open('graph_data.txt','w')                                               #open file in write mode
    truncate                                                                    #delete everything
    f.close() 

s = serial.Serial('COM4', baudrate = 115200, timeout = 4)                       #begin to receive data from keil using COM4 (Found from device manager)
print("Opening: " + s.name)

s.reset_output_buffer()
s.reset_input_buffer()

fig = plt.figure(figsize=(10,10))                                               #Starts building the 3D graph
plot = fig.add_subplot(111, projection='3d')
#Set label plotis 
plot.set_xlabel('x')
plot.set_ylabel('y')  
plot.set_zlabel('z')

s.write('s'.encode())                                                           #Start collecting data
  
counter = 0
xline = 0
yline = 0
zline = 0
z = 0

x = [0]*64                                                                      #array of one rotation
f = open('graph_data.txt','a')                                                   #open in append mode
while (counter != 64):
    x[counter] = float(s.readline().decode())                                     #decode and put data in list
    print(x[counter], counter+1)
    f.write(str(x[counter]))                                                      #turn data to string and put in txt file
    f.write('\n')                                                               #next line
    counter+=1
f.close()

data = open("graph_data.txt","r")                                                #open text file in read (r) only mode
dataStr = data.read()                                                           #the text file is turned into a string
dataList = dataStr.split("\n")                                                  #spaces put into the file
for i in range(0,len(dataList),1):                                              
    dataList[i] = str(dataList[i])                                              #Put data into a list
data.close()
dataList.pop(len(dataList)-1)                                                   

Data = [float(i) for i in dataList]                                             #turned into type float to help with data calculation later on
number_of_lines = len(Data)                                                            #checks the number of measurements collected and puts that number into the variable number_of_lines

plot_data = input("Would you like to plot the current data collected?(yes/no)")   #First input to start the spinning of the motor
if(plot_data == 'yes'):                                                           # yes continues the operation

    counter = 0                                                                 #Counter used to track amount of steps taken
    while(counter != number_of_lines):                                                   #Checks for more data to graph

        if(counter%64==0 and counter != 0):                                      #checks for counter != 0 to make sure its always changing                     
            z+=0.1

        xline = [(Data[counter-1])*cos((counter-1)*0.09817477),(Data[counter])*cos(counter*0.09817477)]    #Line segment to find values for X
        yline = [(Data[counter-1])*sin((counter-1)*0.09817477),(Data[counter])*sin(counter*0.09817477)]    #Line segment to find values for y
        if(counter <  64):
            zline = [0,0]
        else:
            zline = [z,z] 
                                                                                #plot points in cartesian
        plot.scatter( (Data[counter])*cos(counter*0.09817477), (Data[counter])*sin(counter*0.09817477),z, c = "black", s = 1)

        if(counter<64):
            plot.plot(xline, yline, 0, color = 'black')                          #If counter is below 64, we know this is the first iteration of the code so z can be 0
        else: 
            plot.plot(xline, yline, zline, color = 'black')                     #If counter is above 64, we know this isn't the first iteration so we have to change the z value to account for the 30cm
        counter+=1

    #Hard coded for 10 iterations hence why it stops at 640 (64 x 10)
    counter = 0
    while(counter < number_of_lines-64):                                                 
        if(counter<64):
            z = 0
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0,0.1] #Hard coded z value to add 0.1 every iteration.
            plot.plot(xline, yline, zline, color = 'black') #Plot details
        elif(64<=counter<128):
            z = 0.1
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.1,0.2] #Hard coded z value to add 0.1 every iteration.
            plot.plot(xline, yline, zline, color = 'black')
        elif(128<=counter<192):
            z = 0.2
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.2,0.3]
            plot.plot(xline, yline, zline, color = 'black')
        elif(192<=counter<256):
            z = 0.3
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.3,0.4]
            plot.plot(xline, yline, zline, color = 'black')
        elif(256<=counter<320):
            z = 0.4
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.4,0.5]
            plot.plot(xline, yline, zline, color = 'black')
        elif(320<=counter<384):
            z = 0.5
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.5,0.6]
            plot.plot(xline, yline, zline, color = 'black')
        elif(384<=counter<448):
            z = 0.6
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.6,0.7]
            plot.plot(xline, yline, zline, color = 'black')
        elif(448<=counter<512):
            z = 0.7
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.7,0.8]
            plot.plot(xline, yline, zline, color = 'black')
        elif(512<=count<576):
            z = 0.8
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.8,0.9]
            plot.plot(xline, yline, zline, color = 'black')
        elif(576<=counter<640):
            z = 0.9
            xline = [(Data[counter])*cos((counter)*0.09817477),(Data[counter+64])*cos(counter*0.09817477)]
            yline = [(Data[counter])*sin((counter)*0.09817477),(Data[counter+64])*sin(counter*0.09817477)]
            zline = [0.9,1.0]
            plot.plot(xline, yline, zline, color = 'black')
        counter+=1
    plt.show()                                          #show graph created using axes3D

print("Closing: " + s.name )                            #close COM4 (found using device manager)
s.close()
