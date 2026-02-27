imu = binfile('imu.bin', 7);
gps = binfile('gps.bin', 15);
imuplot(imu(:,[2:7,1]))
gpsplot(gps(:,[3:5,2]))