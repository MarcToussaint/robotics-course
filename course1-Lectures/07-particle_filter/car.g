
body world { X=<T t(0 0 .1)> type=0  size=[.01 .01 .01 0] fixed }
body transx { type=0  size=[.01 .01 .01 0] }
body transxy { type=0 size=[.01 .01 .01 0] }
body car {type=0  size=[0.2 0.5 .1 .0] color=[.8 0 0] }

joint (world transx) { type=0 }
joint (transx transxy) { type=0 }
joint (transxy car) { A=<T d(-90 0 1 0)> B=<T d(90 0 1 0)> type=0 }
