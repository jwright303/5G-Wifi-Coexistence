# 5G-Wifi-Coexistence
The following is a program I developed using Ns3 and Code developed by the Lena Project. The Lena Project added 5G specifications that are not currently present within Ns3.

This program creates wifi and 5g devices in an indoor enviornemnt transmitting on the same channel. Packets are monitored and staistics are printed out at the end to show the performance of the simulation. Execution of this program can be configured through the command line using the prompt below.

./waf --run "scratch/nr-wifi-int.cc --exampleParam=1 --exampleParam=100kbps"

In this, the number of Ues/Stas, Gnbs/Aps can be configured this way, as well as the rate of transmission, NR numerology, and listen before talk category. The default execution will run with 3 GnB/APs and 4 Ue/Stas. Note that the number of UEs configured will apply for both wifi and 5G so the defualt will create 4 wifi devices and 4 5G devices. The simulation will run in the 6Ghz on band 42 (6.615Ghz). An example of the end of execution is provided below

<img width="394" alt="Screen Shot 2021-11-10 at 5 16 18 PM" src="https://user-images.githubusercontent.com/41707123/141219468-3bd615dc-2d68-4af2-97c0-54109afa7ad0.png">

Similarly, programs were also written to parse the output of the execution and print out the location of each of the devices

Note:
In order to run this program a working copy of the Ns3 reposotory is needed as well as a copy of 5G repo. Links are provided below
https://www.nsnam.org/releases/ns-3-35/
https://5g-lena.cttc.es/download/
