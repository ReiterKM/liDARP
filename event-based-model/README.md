# Rolling-Horizon Algorithm for dynamic DARP

This is the C++ implementation of the rolling-horizon algorithm for the dynamic Dial-a-Ride Problem using Cplex. The algorithm was proposed in<br>

Gaul, Klamroth, and Stiglmayr, 2021. Solving the dynamic dial-a-ride problem using a rolling-horizon event-based graph.<br> 
21st Symposium on Algorithmic Approaches for Transportation Modelling, Optimization, and Systems (ATMOS 2021). Schloss Dagstuhl - Leibniz-Zentrum für Informatik.<br>
https://doi.org/10.4230/OASICS.ATMOS.2021.8.<br>

For details of the rolling-horizon algorithm and implementation please refer to the above reference. <br>
Any feedback is welcome, please send an email to gaul@math.uni-wuppertal.de.<br>

 ## Compilation

 Change the location of your Cplex directories in the Makefile first. Compile the project using "make". 
 

 ## Test instances 

 As test instances two different file formats are accepted:<br>
 - First format (this refers to instance_mode = 1): Test instance consists of one file.  <br>
    <pre>
    2 32 480 3 30
    0   0.000   0.000   0   0    0  480
    1  -1.198  -5.164   3   1    0 1440
    2   5.573   7.114   3   1    0 1440
    3  -6.614   0.072   3   1    0 1440
    4  -7.374  -1.107   3   1    0 1440
    5  -9.251   8.321   3   1    0 1440
    6   6.498  -6.036   3   1    0 1440
    7   0.861   6.903   3   1    0 1440
    8   3.904  -5.261   3   1    0 1440
    9   7.976  -9.000   3   1  276  291
    10  -2.610   0.039   3   1   32   47
    11   4.487   7.142   3   1  115  130
    12   8.938  -4.388   3   1   14   29
    13  -4.172  -9.096   3   1  198  213
    14   7.835  -9.269   3   1  160  175
    15   2.792  -7.944   3   1  180  195
    16   5.212   9.271   3   1  366  381
    17   6.687   6.731   3  -1  402  417
    18  -2.192  -9.210   3  -1  322  337
    19  -1.061   8.752   3  -1  179  194
    20   6.883   0.882   3  -1  138  153
    21   5.586  -1.554   3  -1   82   97
    22  -9.865   1.398   3  -1   49   64
    23  -9.800   5.697   3  -1  400  415
    24   1.271   1.018   3  -1  298  313
    25   4.404  -1.952   3  -1    0 1440
    26   0.673   6.283   3  -1    0 1440
    27   7.032   2.808   3  -1    0 1440
    28  -0.694  -7.098   3  -1    0 1440
    29   3.763  -7.269   3  -1    0 1440
    30   6.634  -7.426   3  -1    0 1440
    31  -9.450   3.792   3  -1    0 1440
    32  -8.819  -4.749   3  -1    0 1440
    33   0.000   0.000   0   0    0  48
    </pre>
    
    First line corresponds to: K 2n T Q L_i, where L_i = 30 for all i=1,...,n<br>
    Column 1: identifies nodes 0,...,2n+1<br>
    Column 2 and 3: x and y coordinates of the nodes<br>
    Column 4: service time <br>
    Column 5: load<br>
    Column 6 and 7: time window<br>


- Second format (this refers to instance_mode = 2): Test instance consists of two files.<br>
    1) One file containing the distance matrix (c_{i,j}), named "instance_name_c_a.txt". The file shoud be formatted like this:<br>
        <pre>
        c_{0,0}  c_{0,1} ... c{0,2n}
        c_{1,0}  c_{1,1} ... c{1,2n}
        .
        .
        .
        c_{2n,0} c_{2n,1} ... c_{2n,2n}
        </pre>
    2) The other file, "instance_name.txt", contains everything else. This file should have the following format: <br>

        <pre>
        8 508 840 3 0
        0 0.0 0 0.0 840.0 840.0
        1 0.75 1 126.0 151.0 22.33
        2 0.75 1 216.0 241.0 15.85
        3 0.75 1 290.0 315.0 18.46
        4 0.75 1 364.0 389.0 21.03
        5 0.75 1 381.0 406.0 20.83
        6 0.75 3 454.0 479.0 18.02
        7 0.75 1 494.0 519.0 22.49
        8 0.75 1 534.0 559.0 25.85
        9 0.75 1 546.0 571.0 16.32
        10 0.75 1 546.0 571.0 18.02
        ...
        500 0.75 -2 564.62 599.62 15.87
        501 0.75 -1 580.49 615.49 16.74
        502 0.75 -1 609.09 644.09 16.34
        503 0.75 -1 718.14 753.14 14.39
        504 0.75 -1 728.62 763.62 17.87
        505 0.75 -1 775.54 810.54 14.79
        506 0.75 -1 812.38 840.0 27.35
        507 0.75 -1 833.89 840.0 16.14
        508 0.75 -1 834.62 840.0 15.87
        509 0 0 0 840 840
        </pre>

        First line corresponds to: K 2n T Q 0<br>
        Column 1: identifies nodes 0,...,2n+1<br>
        Column 2: service time<br>
        Column 3: load<br>
        Column 4 and 5: time window<br>
        Column 6: ride time L_i<br>



This repository contains a test instance of the second type in the directory data/WSW. In this instance a vehicle capacity of Q=6 is assumed.


 ## Running an instance

There are two binaries <br>
 ./bin/darp_cplex_3<br>
 ./bin/darp_cplex_6<br>
 to choose between normal cabs (Q=3) and ridepooling cabs (Q=6). Usage: <br>

 ./bin/darp_cplex_6 instance_name e.g. ./bin/darp_milp_6 no_011_6_req<br>
 

 
 

 ## Acknowledgement

 This work was partially supported by the state of North Rhine-Westphalia (Germany) within the project “bergisch.smart.mobility”.<br>

 
 ## Authors

 The author of the code is Daniela Gaul (gaul@math.uni-wuppertal.de).
 It was developed at Bergische Universität Wuppertal with her PhD advisors Kathrin Klamroth and Michael Stiglmayr.
 
 ## License
 
<a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/"><img alt="Creative Commons License" style="border-width:0" src="https://i.creativecommons.org/l/by-nc-sa/4.0/88x31.png" /></a><br />This work is licensed under a <a rel="license" href="http://creativecommons.org/licenses/by-nc-sa/4.0/">Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License</a>.





